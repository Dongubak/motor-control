"""
motor_safe.py - 위치 모니터링 기능이 추가된 안전한 모터 제어 라이브러리

변경 사항 (motor.py 대비):
1. 다축 위치 차이 모니터링 추가
2. 임계값 초과 시 모든 모터 긴급 정지
3. 동기화 오류 상태 공유
4. 안전 설정 옵션 추가 (max_sync_error_mm)
"""

import pysoem
import struct
import time
import multiprocessing as mp
from typing import List

# --- CiA 402 Controlword & 기본 상수 ---
CW_SHUTDOWN = 0x0006
CW_SWITCH_ON = 0x0007
CW_ENABLE_OPERATION = 0x000F
CW_DISABLE_VOLTAGE = 0x0000
CW_FAULT_RESET = 0x0080

PULSES_PER_REVOLUTION = 8388608
x_axis_mm_per_rev = 11.9993131404
z_axis_mm_per_rev = 5.99965657019

# --- 안전 관련 상수 ---
DEFAULT_MAX_SYNC_ERROR_MM = 0.5  # 기본 허용 동기화 오차 (mm)


def _ethercat_process_loop_safe(
    adapter_name, num_slaves, cycle_time_s,
    command_queue, shared_states, lock,
    max_sync_error_pulse  # 추가: 동기화 오차 임계값 (pulse)
):
    """
    [SAFE 버전] 별도의 프로세스에서 실행될 EtherCAT 통신 및 제어 루프.
    위치 차이 모니터링 및 긴급 정지 기능 포함.
    """
    master = pysoem.Master()

    local_motor_states = [{
        'target_pulse': 0,
        'offset': 0,
        'mode': 'csp',
        'axis': 'x',
        'trajectory': None
    } for _ in range(num_slaves)]

    is_running = True
    sync_error_detected = False  # 동기화 오류 플래그

    # 큐에서 설정 명령 읽기
    slave_configs = {i: {} for i in range(num_slaves)}
    while not command_queue.empty():
        slave_idx, cmd, value = command_queue.get_nowait()
        if cmd == 'SET_VELOCITY':
            slave_configs[slave_idx]['velocity'] = value
        elif cmd == 'SET_ACCEL':
            slave_configs[slave_idx]['accel'] = value
        elif cmd == 'SET_AXIS':
            local_motor_states[slave_idx]['axis'] = value
            print(f"[초기화] 모터 {slave_idx} 축을 '{value}'로 설정")

    # --- EtherCAT 초기화 ---
    max_init_retries = 3
    init_success = False

    for retry_count in range(max_init_retries):
        try:
            if retry_count > 0:
                print(f"\n[재시도 {retry_count}/{max_init_retries}] EtherCAT 초기화를 다시 시도합니다...")
                try:
                    master.close()
                except:
                    pass
                time.sleep(1.0)
                master = pysoem.Master()

            print("[초기화] EtherCAT 어댑터 열기...")
            master.open(adapter_name)

            print("[초기화] 슬레이브 검색 중...")
            found_slaves = master.config_init()
            if found_slaves < num_slaves:
                raise RuntimeError(f"슬레이브 {num_slaves}개를 찾을 수 없습니다. (발견: {found_slaves}개)")
            print(f"[성공] {found_slaves}개의 슬레이브를 찾았습니다.")

            for i in range(num_slaves):
                slave = master.slaves[i]
                print(f"[연결] Slave {i}: {slave.name}")

                slave.dc_sync(True, int(cycle_time_s * 1_000_000_000))
                _sdo_reset_fault(slave)

                if 'velocity' in slave_configs[i]:
                    print(f"  모터 {i}에 대한 SDO 설정 적용 중... (Velocity)")
                    velocity_pulse_per_sec = _rpm_to_pulse_per_sec(slave_configs[i]['velocity'])
                    slave.sdo_write(0x6081, 0, struct.pack("<I", velocity_pulse_per_sec))
                if 'accel' in slave_configs[i]:
                    print(f"  모터 {i}에 대한 SDO 설정 적용 중... (Accel/Decel)")
                    accel_val, decel_val = slave_configs[i]['accel']
                    slave.sdo_write(0x6083, 0, struct.pack("<I", accel_val))
                    slave.sdo_write(0x6084, 0, struct.pack("<I", decel_val))

                _configure_csp_pdos(slave)
                _setup_csp_mode(slave)

                try:
                    slave.sdo_write(0x6067, 0, struct.pack("<I", 200000000))
                    slave.sdo_write(0x6065, 0, struct.pack("<I", 200000000))
                    time.sleep(0.01)
                except Exception as e:
                    print(f"  [경고] SDO 윈도우 설정 실패 (계속 진행): {e}")

            print(f"[초기화] DC Sync 활성화 (Cycle: {cycle_time_s * 1000} ms)")
            pdo_size = master.config_map()
            print(f"[성공] PDO 맵핑 완료. Process data size: {pdo_size} bytes")

            # [안전 모드] 동기화 오차 임계값 출력
            print(f"[안전 모드] 동기화 오차 임계값: {max_sync_error_pulse} pulse")

            init_success = True
            break

        except Exception as e:
            print(f"[에러] EtherCAT 초기화 실패: {e}")
            if retry_count < max_init_retries - 1:
                print(f"  → 1.0초 후 재시도합니다...")
            else:
                print(f"  → {max_init_retries}회 시도 후 실패. 프로그램을 종료합니다.")
                try:
                    master.close()
                except:
                    pass
                return

    if not init_success:
        print("[에러] EtherCAT 초기화에 실패했습니다. 프로그램을 종료합니다.")
        return

    try:
        # --- OP 상태로 전환 ---
        max_op_retries = 3
        op_success = False

        for op_retry in range(max_op_retries):
            try:
                if op_retry > 0:
                    print(f"\n[재시도 {op_retry}/{max_op_retries}] OP 상태 전환을 다시 시도합니다...")
                    time.sleep(0.5)

                print("[초기화] OP 상태로 전환 중...")
                master.state = pysoem.OP_STATE
                master.write_state()

                is_op = False
                for _ in range(int(4.0 / cycle_time_s)):
                    master.send_processdata()
                    master.receive_processdata()
                    master.read_state()
                    if all(s.state == pysoem.OP_STATE for s in master.slaves):
                        is_op = True
                        break
                    time.sleep(cycle_time_s)

                if not is_op:
                    raise RuntimeError(f"모든 슬레이브가 OP 상태에 도달하지 못했습니다.")

                print("[성공] OP 상태 도달 성공! 주기 통신을 시작합니다.")

                for i in range(num_slaves):
                    current_pos = _read_actual_position(master.slaves[i])
                    local_motor_states[i]['target_pulse'] = current_pos
                    print(f"  [CSP 초기화] 모터 {i}: 현재 위치 {current_pos}로 target 설정")

                op_success = True
                break

            except Exception as e:
                print(f"[에러] OP 상태 전환 실패: {e}")
                if op_retry < max_op_retries - 1:
                    print(f"  → 0.5초 후 재시도합니다...")
                else:
                    raise

        if not op_success:
            raise RuntimeError("OP 상태 전환에 실패했습니다.")

        # --- 메인 제어 루프 ---
        cycle_counter = 0
        while is_running:
            loop_start_time = time.monotonic()
            cycle_counter += 1

            # 1. 명령 수신
            trajectory_commands = []

            while not command_queue.empty():
                slave_idx, cmd, value = command_queue.get_nowait()
                if cmd == 'STOP_ALL':
                    is_running = False
                    break
                elif cmd == 'SET_AXIS':
                    local_motor_states[slave_idx]['axis'] = value
                elif cmd == 'SET_ORIGIN':
                    current_pos = _read_actual_position(master.slaves[slave_idx])
                    local_motor_states[slave_idx]['offset'] = current_pos
                    local_motor_states[slave_idx]['target_pulse'] = current_pos
                    # 원점 설정 시 동기화 오류 플래그 리셋
                    sync_error_detected = False
                    print(f"[디버그] 모터 {slave_idx} 원점 설정: offset={current_pos}")
                elif cmd == 'MOVE_TO_MM':
                    # 동기화 오류 상태에서는 이동 명령 무시
                    if sync_error_detected:
                        print(f"[경고] 동기화 오류 상태! 모터 {slave_idx} 이동 명령 무시됨")
                    else:
                        trajectory_commands.append((slave_idx, value))
                elif cmd == 'RESET_SYNC_ERROR':
                    # 동기화 오류 리셋 명령
                    sync_error_detected = False
                    print("[안전] 동기화 오류 플래그 리셋")

            # 궤적 처리
            if trajectory_commands and not sync_error_detected:
                for slave_idx, _ in trajectory_commands:
                    if local_motor_states[slave_idx]['trajectory'] is not None:
                        print(f"[경고] 모터 {slave_idx}: 기존 궤적 취소 후 새 명령 처리")
                        local_motor_states[slave_idx]['trajectory'] = None

                common_start_time = time.monotonic()

                trajectory_data = []
                for slave_idx, target_mm in trajectory_commands:
                    state = local_motor_states[slave_idx]
                    mm_per_rev = x_axis_mm_per_rev if state['axis'] == 'x' else z_axis_mm_per_rev
                    revolutions = target_mm / mm_per_rev
                    target_pulse_relative = int(revolutions * PULSES_PER_REVOLUTION * 2)
                    target_pulse_absolute = target_pulse_relative + state['offset']

                    current_pos = _read_actual_position(master.slaves[slave_idx])
                    distance = abs(target_pulse_absolute - current_pos)

                    print(f"[계산] 모터 {slave_idx} 이동 명령: {target_mm:.2f}mm")

                    configured_velocity = slave_configs.get(slave_idx, {}).get('velocity', 60)
                    velocity_pulse_per_sec = (configured_velocity / 60.0) * PULSES_PER_REVOLUTION * 2
                    duration = distance / velocity_pulse_per_sec if velocity_pulse_per_sec > 0 else 1.0

                    trajectory_data.append({
                        'slave_idx': slave_idx,
                        'target_mm': target_mm,
                        'current_pos': current_pos,
                        'target_pos': target_pulse_absolute,
                        'distance': distance,
                        'duration': duration
                    })

                max_duration = max(max(traj['duration'] for traj in trajectory_data), 0.1)

                for traj_info in trajectory_data:
                    slave_idx = traj_info['slave_idx']
                    state = local_motor_states[slave_idx]
                    state['trajectory'] = {
                        'start': traj_info['current_pos'],
                        'end': traj_info['target_pos'],
                        'duration': max_duration,
                        'start_time': common_start_time
                    }
                    print(f"[궤적] 모터 {slave_idx}: {traj_info['target_mm']:.2f}mm, 시간: {max_duration:.2f}초")

            if not is_running:
                break

            # 2. PDO 통신
            master.send_processdata()
            master.receive_processdata()

            # ========================================
            # [안전 기능] 다축 위치 차이 모니터링
            # ========================================
            if num_slaves >= 2 and not sync_error_detected:
                # 모든 모터의 실제 위치 읽기
                positions = [_read_actual_position(master.slaves[i]) for i in range(num_slaves)]

                # 오프셋 보정된 상대 위치 계산
                relative_positions = [
                    positions[i] - local_motor_states[i]['offset']
                    for i in range(num_slaves)
                ]

                # 이동 중인 모터가 있는지 확인
                any_moving = any(
                    local_motor_states[i]['trajectory'] is not None
                    for i in range(num_slaves)
                )

                # 이동 중일 때만 위치 차이 확인
                if any_moving:
                    # 인접 모터 간 위치 차이 확인
                    for i in range(num_slaves - 1):
                        position_diff = abs(relative_positions[i] - relative_positions[i + 1])

                        if position_diff > max_sync_error_pulse:
                            # ========================================
                            # [긴급 정지] 동기화 오차 초과!
                            # ========================================
                            sync_error_detected = True

                            # mm 단위로 변환 (z축 기준)
                            diff_mm = position_diff / (PULSES_PER_REVOLUTION * 2) * z_axis_mm_per_rev

                            print(f"\n{'='*60}")
                            print(f"[긴급 정지] 동기화 오차 초과!")
                            print(f"  Motor {i} vs Motor {i+1}")
                            print(f"  위치 차이: {position_diff} pulse ({diff_mm:.3f} mm)")
                            print(f"  임계값: {max_sync_error_pulse} pulse")
                            print(f"{'='*60}")

                            # 모든 모터 즉시 정지
                            for j in range(num_slaves):
                                if local_motor_states[j]['trajectory'] is not None:
                                    local_motor_states[j]['trajectory'] = None
                                    current_pos = positions[j]
                                    local_motor_states[j]['target_pulse'] = current_pos
                                    print(f"  → 모터 {j} 긴급 정지: {current_pos} pulse에 고정")

                            break

                    # 디버그 출력 (매 50 사이클마다)
                    if cycle_counter % 50 == 0 and any_moving:
                        for i in range(num_slaves - 1):
                            diff = abs(relative_positions[i] - relative_positions[i + 1])
                            diff_mm = diff / (PULSES_PER_REVOLUTION * 2) * z_axis_mm_per_rev
                            print(f"[동기화] M{i}-M{i+1} 위치차: {diff} pulse ({diff_mm:.3f}mm)")

            # 3. Fault 확인
            fault_detected = False
            fault_motor_id = -1
            for i in range(num_slaves):
                status = _read_status_word(master.slaves[i])
                if (status & 0x0008) and local_motor_states[i]['trajectory'] is not None:
                    fault_detected = True
                    fault_motor_id = i
                    break

            if fault_detected:
                print(f"\n[긴급 정지] 모터 {fault_motor_id} Fault 감지!")
                for i in range(num_slaves):
                    if local_motor_states[i]['trajectory'] is not None:
                        local_motor_states[i]['trajectory'] = None
                        current_pos = _read_actual_position(master.slaves[i])
                        local_motor_states[i]['target_pulse'] = current_pos
                        print(f"  → 모터 {i} 정지: {current_pos} pulse에 고정")

            # 4. 각 슬레이브 제어
            for i in range(num_slaves):
                slave = master.slaves[i]
                state = local_motor_states[i]

                status = _read_status_word(slave)

                if 'last_status' not in state:
                    state['last_status'] = 0
                if state['last_status'] != status:
                    if state['trajectory'] is not None or (status & 0x0008):
                        print(f"[상태] 모터 {i}: 0x{state['last_status']:04X} → 0x{status:04X}")
                    state['last_status'] = status

                cw = 0
                if (status & 0x004F) == 0x0040:
                    cw = CW_SHUTDOWN
                elif (status & 0x006F) == 0x0021:
                    cw = CW_SWITCH_ON
                elif (status & 0x006F) == 0x0023:
                    cw = CW_ENABLE_OPERATION
                elif (status & 0x006F) == 0x0027:
                    cw = CW_ENABLE_OPERATION
                elif (status & 0x0008):
                    cw = CW_FAULT_RESET
                    if cycle_counter % 50 == 0:
                        print(f"[경고] 모터 {i} Fault 상태! status=0x{status:04X}")
                else:
                    cw = CW_ENABLE_OPERATION

                target_position = state['target_pulse']

                if state['trajectory'] is not None:
                    traj = state['trajectory']
                    elapsed = time.monotonic() - traj['start_time']
                    current_actual_pos = _read_actual_position(slave)
                    position_error = abs(traj['end'] - current_actual_pos)

                    if position_error < 50000:
                        target_position = traj['end']
                        state['target_pulse'] = traj['end']
                        state['trajectory'] = None
                        print(f"[완료] 모터 {i} 궤적 완료! 위치오차: {position_error} pulse")
                    else:
                        import math
                        progress = min(elapsed / traj['duration'], 1.0)
                        smooth_progress = (1.0 - math.cos(math.pi * progress)) / 2.0
                        target_position = int(traj['start'] + (traj['end'] - traj['start']) * smooth_progress)
                        state['target_pulse'] = target_position

                _write_csp_outputs(slave, cw, target_position)

                # 5. 상태 공유 (sync_error 상태 포함)
                with lock:
                    base_idx = i * 5  # 5개로 확장 (sync_error 추가)
                    shared_states[base_idx] = status
                    shared_states[base_idx + 1] = 1 if state['trajectory'] is not None else 0
                    current_pos = _read_actual_position(slave)
                    shared_states[base_idx + 2] = current_pos
                    shared_states[base_idx + 3] = state['offset']
                    shared_states[base_idx + 4] = 1 if sync_error_detected else 0

            # 6. 사이클 타임 유지
            loop_duration = time.monotonic() - loop_start_time
            sleep_time = cycle_time_s - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        print(f"EtherCAT 프로세스 에러: {e}")
    finally:
        print("\n[종료 시퀀스 시작]")
        try:
            master.read_state()
            if len(master.slaves) > 0 and master.slaves[0].state == pysoem.OP_STATE:
                print("  [안전] 모든 모터를 현재 위치에 고정합니다...")
                for i in range(num_slaves):
                    try:
                        current_pos = _read_actual_position(master.slaves[i])
                        local_motor_states[i]['trajectory'] = None
                        local_motor_states[i]['target_pulse'] = current_pos
                        master.slaves[i].output = struct.pack("<H", CW_ENABLE_OPERATION) + struct.pack("<i", current_pos)
                    except:
                        pass

                for _ in range(5):
                    master.send_processdata()
                    master.receive_processdata()
                    time.sleep(0.02)

                print("  [완료] 모터 정지 완료")

                print("  1단계: Disable Operation")
                for i in range(num_slaves):
                    try:
                        current_pos = _read_actual_position(master.slaves[i])
                        master.slaves[i].output = struct.pack("<H", CW_SWITCH_ON) + struct.pack("<i", current_pos)
                    except:
                        pass
                master.send_processdata()
                time.sleep(0.1)

                print("  2단계: Shutdown")
                for i in range(num_slaves):
                    try:
                        current_pos = _read_actual_position(master.slaves[i])
                        master.slaves[i].output = struct.pack("<H", CW_SHUTDOWN) + struct.pack("<i", current_pos)
                    except:
                        pass
                master.send_processdata()
                time.sleep(0.1)

                print("  3단계: Disable Voltage")
                for i in range(num_slaves):
                    try:
                        current_pos = _read_actual_position(master.slaves[i])
                        master.slaves[i].output = struct.pack("<H", CW_DISABLE_VOLTAGE) + struct.pack("<i", current_pos)
                    except:
                        pass
                master.send_processdata()
                time.sleep(0.2)

                print("  4단계: EtherCAT INIT 상태로 전환")
                master.state = pysoem.INIT_STATE
                master.write_state()
                time.sleep(0.1)

                print("  [완료] 종료 시퀀스 완료")
        except Exception as e:
            print(f"  [경고] 종료 중 에러 (무시): {e}")
        finally:
            master.close()
            print("EtherCAT 프로세스 종료.\n")


# --- 헬퍼 함수들 ---
def _read_status_word(slave):
    return struct.unpack("<H", slave.input[:2])[0]

def _read_actual_position(slave):
    return struct.unpack("<i", slave.input[2:6])[0]

def _write_csp_outputs(slave, cw, target_position):
    slave.output = struct.pack("<H", cw) + struct.pack("<i", target_position)

def _rpm_to_pulse_per_sec(rpm):
    return int((rpm / 60) * PULSES_PER_REVOLUTION)

def _sdo_reset_fault(slave):
    if struct.unpack("<H", slave.sdo_read(0x6041, 0))[0] & 0x0008:
        slave.sdo_write(0x6040, 0, struct.pack("<H", CW_FAULT_RESET))
        time.sleep(0.2)

def _setup_csp_mode(slave):
    slave.sdo_write(0x211F, 0, struct.pack("<H", (1 << 12)))
    time.sleep(0.01)
    slave.sdo_write(0x6060, 0, struct.pack("<b", 8))
    time.sleep(0.01)
    print(f"  [설정] CSP 모드(8) 설정 완료")

def _configure_csp_pdos(slave):
    slave.sdo_write(0x1C12, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1600, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1600, 1, struct.pack('<I', 0x60400010)); time.sleep(0.01)
    slave.sdo_write(0x1600, 2, struct.pack('<I', 0x607A0020)); time.sleep(0.01)
    slave.sdo_write(0x1600, 0, b'\x02'); time.sleep(0.01)
    slave.sdo_write(0x1C12, 1, struct.pack('<H', 0x1600)); time.sleep(0.01)
    slave.sdo_write(0x1C12, 0, b'\x01'); time.sleep(0.01)

    slave.sdo_write(0x1C13, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1A00, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1A00, 1, struct.pack('<I', 0x60410010)); time.sleep(0.01)
    slave.sdo_write(0x1A00, 2, struct.pack('<I', 0x60640020)); time.sleep(0.01)
    slave.sdo_write(0x1A00, 0, b'\x02'); time.sleep(0.01)
    slave.sdo_write(0x1C13, 1, struct.pack('<H', 0x1A00)); time.sleep(0.01)
    slave.sdo_write(0x1C13, 0, b'\x01'); time.sleep(0.01)
    print(f"  [설정] CSP PDO 매핑 완료")


# --- 메인 클래스들 ---
class EtherCATBusSafe:
    """
    [SAFE 버전] EtherCAT 버스 관리 클래스

    추가 기능:
    - max_sync_error_mm: 허용 동기화 오차 (mm 단위, 기본 0.5mm)
    - 위치 차이 모니터링
    - 긴급 정지 기능
    """

    def __init__(self, adapter_name: str, num_slaves: int, cycle_time_ms: int = 50,
                 max_sync_error_mm: float = DEFAULT_MAX_SYNC_ERROR_MM):
        self._command_queue = mp.Queue()
        self._lock = mp.Lock()
        # 각 슬레이브당 5개의 상태 저장 (status, moving, pos, offset, sync_error)
        self._shared_states = mp.Array('d', num_slaves * 5, lock=False)

        self._adapter_name = adapter_name
        self._num_slaves = num_slaves
        self._max_sync_error_mm = max_sync_error_mm

        # mm를 pulse로 변환 (z축 기준)
        self._max_sync_error_pulse = int(
            (max_sync_error_mm / z_axis_mm_per_rev) * PULSES_PER_REVOLUTION * 2
        )

        self._process = mp.Process(
            target=_ethercat_process_loop_safe,
            args=(
                adapter_name, num_slaves, cycle_time_ms / 1000.0,
                self._command_queue, self._shared_states, self._lock,
                self._max_sync_error_pulse
            )
        )
        self.motors: List[MotorSafe] = [
            MotorSafe(i, self._command_queue, self._shared_states, self._lock)
            for i in range(num_slaves)
        ]

    def start(self):
        self._process.start()
        print(f"[SAFE] EtherCAT 버스 시작 (동기화 오차 허용: {self._max_sync_error_mm}mm)")

    def stop(self):
        print("EtherCAT 버스 종료 중...")
        if self._process.is_alive():
            self._command_queue.put((-1, 'STOP_ALL', None))
            self._process.join(timeout=2)
            if self._process.is_alive():
                print("EtherCAT 프로세스가 정상적으로 종료되지 않아 강제 종료합니다.")
                self._process.terminate()
            print("EtherCAT 버스 프로세스 종료 완료.")
        else:
            print("EtherCAT 프로세스가 시작되지 않았거나 이미 종료되었습니다.")

    def reset_sync_error(self):
        """동기화 오류 상태를 리셋합니다."""
        self._command_queue.put((-1, 'RESET_SYNC_ERROR', None))
        print("[SAFE] 동기화 오류 리셋 명령 전송")

    @property
    def has_sync_error(self) -> bool:
        """동기화 오류가 발생했는지 확인합니다."""
        # 첫 번째 모터의 sync_error 플래그 확인
        return self._shared_states[4] != 0


class MotorSafe:
    """
    [SAFE 버전] 개별 모터 제어 클래스

    추가 기능:
    - sync_error 상태 확인
    """

    def __init__(self, slave_index: int, command_queue: mp.Queue,
                 shared_states: mp.Array, lock: mp.Lock):
        self._index = slave_index
        self._command_queue = command_queue
        self._shared_states = shared_states
        self._lock = lock
        self._axis = 'x'

    def set_axis(self, axis: str):
        if axis not in ['x', 'z']:
            raise ValueError("axis는 'x' 또는 'z'여야 합니다.")
        self._axis = axis
        self._command_queue.put((self._index, 'SET_AXIS', axis))
        print(f"[설정] 모터 {self._index}: 축을 '{axis}'로 설정")

    def set_origin(self):
        self._command_queue.put((self._index, 'SET_ORIGIN', None))
        print(f"[원점] 모터 {self._index}: 원점 설정 명령 전송")

    def set_profile_velocity(self, rpm):
        self._command_queue.put((self._index, 'SET_VELOCITY', rpm))
        print(f"[속도] 모터 {self._index}: {rpm} RPM")

    def set_profile_accel_decel(self, accel_rpm_per_sec, decel_rpm_per_sec=None):
        accel_val = int((accel_rpm_per_sec / 60) * PULSES_PER_REVOLUTION)
        decel_val = accel_val if decel_rpm_per_sec is None else int((decel_rpm_per_sec / 60) * PULSES_PER_REVOLUTION)
        self._command_queue.put((self._index, 'SET_ACCEL', (accel_val, decel_val)))
        print(f"[가감속] 모터 {self._index}: {accel_rpm_per_sec} RPM/s")

    def move_to_position_mm(self, target_mm):
        self._command_queue.put((self._index, 'MOVE_TO_MM', target_mm))
        print(f"[이동] 모터 {self._index}: {target_mm:.2f}mm")

    @property
    def status_word(self) -> int:
        return int(self._shared_states[self._index * 5])

    def is_moving(self) -> bool:
        return self._shared_states[self._index * 5 + 1] != 0

    @property
    def current_position_mm(self) -> float:
        base_idx = self._index * 5
        current_pos = self._shared_states[base_idx + 2]
        offset = self._shared_states[base_idx + 3]
        relative_pos = current_pos - offset
        revolutions = relative_pos / (PULSES_PER_REVOLUTION * 2)
        mm_per_rev = x_axis_mm_per_rev if self._axis == 'x' else z_axis_mm_per_rev
        return revolutions * mm_per_rev

    @property
    def current_position_pulse(self) -> int:
        return int(self._shared_states[self._index * 5 + 2])

    @property
    def offset_pulse(self) -> int:
        return int(self._shared_states[self._index * 5 + 3])

    @property
    def has_sync_error(self) -> bool:
        """동기화 오류가 발생했는지 확인합니다."""
        return self._shared_states[self._index * 5 + 4] != 0
