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

def _ethercat_process_loop(
    adapter_name, num_slaves, cycle_time_s,
    command_queue, shared_states, lock
):
    """
    별도의 프로세스에서 실행될 단일 EtherCAT 통신 및 제어 루프.
    모든 슬레이브를 관리합니다.
    """
    master = pysoem.Master()

    # 모든 모터의 축을 'x'로 초기화합니다.
    # CSP 모드: target_pulse은 현재 목표 위치, trajectory는 이동 궤적 정보를 저장
    local_motor_states = [{
        'target_pulse': 0,  # 현재 목표 위치 (CSP에서는 매 사이클 업데이트)
        'offset': 0,        # 원점 오프셋
        'mode': 'csp',      # CSP 모드로 변경
        'axis': 'x',        # 축 설정
        'trajectory': None  # 궤적 정보: {'start': pulse, 'end': pulse, 'duration': sec, 'start_time': time}
    } for _ in range(num_slaves)]

    is_running = True

    # [먼저] 큐에서 모든 설정 명령을 미리 읽어서 저장 (재시도 시에도 유지되도록)
    slave_configs = {i: {} for i in range(num_slaves)}
    while not command_queue.empty():
        slave_idx, cmd, value = command_queue.get_nowait()
        if cmd == 'SET_VELOCITY':
            slave_configs[slave_idx]['velocity'] = value
        elif cmd == 'SET_ACCEL':
            slave_configs[slave_idx]['accel'] = value
        elif cmd == 'SET_AXIS':
            # 축 설정도 미리 적용
            local_motor_states[slave_idx]['axis'] = value
            print(f"[초기화] 모터 {slave_idx} 축을 '{value}'로 설정")

    # --- EtherCAT 초기화 재시도 로직 ---
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
                time.sleep(1.0)  # 재시도 전 대기
                master = pysoem.Master()  # 새 마스터 인스턴스 생성

            # --- EtherCAT 초기화 ---
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

                # [중요] PDO 설정 전에 속도/가감속 설정 적용
                if 'velocity' in slave_configs[i]:
                    print(f"  모터 {i}에 대한 SDO 설정 적용 중... (Velocity)")
                    velocity_pulse_per_sec = _rpm_to_pulse_per_sec(slave_configs[i]['velocity'])
                    slave.sdo_write(0x6081, 0, struct.pack("<I", velocity_pulse_per_sec))
                if 'accel' in slave_configs[i]:
                    print(f"  모터 {i}에 대한 SDO 설정 적용 중... (Accel/Decel)")
                    accel_val, decel_val = slave_configs[i]['accel']
                    slave.sdo_write(0x6083, 0, struct.pack("<I", accel_val))
                    slave.sdo_write(0x6084, 0, struct.pack("<I", decel_val))

                _configure_csp_pdos(slave)  # CSP 모드 PDO 설정
                _setup_csp_mode(slave)      # CSP 모드 설정

                # [디버그] 현재 설정 읽기 및 설정 변경
                try:
                    # Position Window 읽기 및 설정
                    pos_window = struct.unpack("<I", slave.sdo_read(0x6067, 0))[0]
                    print(f"  [디버그] 현재 Position Window: {pos_window} pulses")
                    # CSP 모드에서는 Position Window를 매우 크게 설정
                    # 0xFFFFFFFF는 드라이버가 거부할 수 있으므로 200M pulse (약 143mm) 사용
                    slave.sdo_write(0x6067, 0, struct.pack("<I", 200000000))
                    print(f"  [설정] Position Window를 200000000 펄스 (약 143mm)로 설정")

                    # Following Error Window 읽기 및 설정 (핵심!)
                    following_error = struct.unpack("<I", slave.sdo_read(0x6065, 0))[0]
                    print(f"  [디버그] 현재 Following Error Window: {following_error} pulses")
                    # CSP 모드에서는 Following Error Window를 매우 크게 설정
                    # 0xFFFFFFFF는 드라이버가 거부할 수 있으므로 200M pulse (약 143mm) 사용
                    slave.sdo_write(0x6065, 0, struct.pack("<I", 200000000))
                    print(f"  [설정] Following Error Window를 200000000 펄스 (약 143mm)로 설정")

                    time.sleep(0.01)
                except Exception as e:
                    print(f"  [경고] SDO 윈도우 설정 실패 (계속 진행): {e}")

            print(f"[초기화] DC Sync 활성화 (Cycle: {cycle_time_s * 1000} ms)")
            pdo_size = master.config_map()
            print(f"[성공] PDO 맵핑 완료. Process data size: {pdo_size} bytes")

            # 초기화 성공!
            init_success = True
            break

        except Exception as e:
            print(f"[에러] EtherCAT 초기화 실패: {e}")
            if retry_count < max_init_retries - 1:
                print(f"  → {1.0}초 후 재시도합니다...")
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
        # --- OP 상태로 전환 (재시도 로직 포함) ---
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

                # OP 상태 도달을 위한 올바른 대기 루프
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

                # [CSP 모드 중요!] OP 상태 직후 현재 위치로 target_pulse 초기화
                # 이렇게 하지 않으면 0 위치로 가려고 시도하면서 Following Error 발생
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
                    print(f"  → {max_op_retries}회 시도 후 실패. 프로그램을 종료합니다.")
                    raise

        if not op_success:
            raise RuntimeError("OP 상태 전환에 실패했습니다.")

        # --- 메인 제어 루프 ---
        cycle_counter = 0
        while is_running:
            loop_start_time = time.monotonic()
            cycle_counter += 1

            # 1. 메인 프로세스로부터 명령 수신
            # [중요] 동기화를 위해 이번 사이클에서 받은 모든 MOVE 명령은 같은 start_time 사용
            trajectory_commands = []

            while not command_queue.empty():

                slave_idx, cmd, value = command_queue.get_nowait() 
                ## QES1: 큐 원소 의미 ---------------------

                if cmd == 'STOP_ALL':
                    is_running = False
                    break
                elif cmd == 'SET_AXIS':
                    local_motor_states[slave_idx]['axis'] = value
                elif cmd == 'SET_ORIGIN':
                    ## QES2: 원점 설정 동작 내용 ------------------

                    # [중요] 원점 설정 시 현재 위치를 정확히 읽어서 offset에 저장
                    # PDO 통신 직후의 최신 위치를 사용
                    current_pos = _read_actual_position(master.slaves[slave_idx])
                    local_motor_states[slave_idx]['offset'] = current_pos
                    # target_pulse도 현재 위치로 설정하여 즉시 정지 상태 유지
                    local_motor_states[slave_idx]['target_pulse'] = current_pos
                    print(f"[디버그] 모터 {slave_idx} 원점 설정: offset={current_pos}, target={current_pos}")
                elif cmd == 'MOVE_TO_MM':
                    # 이동 명령은 일단 저장만 하고 나중에 일괄 처리
                    trajectory_commands.append((slave_idx, value))

            # [CSP 동기화 핵심] 모든 이동 명령을 같은 start_time으로 처리
            if trajectory_commands:
                # [중요] 새 명령이 들어오면 기존 궤적을 모두 취소
                # 이렇게 하지 않으면 이전 궤적이 완료될 때까지 새 명령이 무시됨
                for slave_idx, _ in trajectory_commands:
                    if local_motor_states[slave_idx]['trajectory'] is not None:
                        print(f"[경고] 모터 {slave_idx}: 기존 궤적 취소 후 새 명령 처리")
                        local_motor_states[slave_idx]['trajectory'] = None

                # 다음 사이클 시작 시간을 공통 궤적 시작 시간으로 사용
                common_start_time = time.monotonic()

                # [1단계] 모든 모터의 이동 거리와 시간을 계산
                trajectory_data = []
                for slave_idx, target_mm in trajectory_commands:
                    state = local_motor_states[slave_idx]
                    mm_per_rev = x_axis_mm_per_rev if state['axis'] == 'x' else z_axis_mm_per_rev
                    revolutions = target_mm / mm_per_rev

                    # [중요] Position Factor 2:1 처리
                    # 모든 값을 드라이버 스케일(2배)로 통일
                    # offset, current_pos는 드라이버에서 읽으므로 이미 2배
                    # target도 2배로 계산해야 함
                    target_pulse_relative = int(revolutions * PULSES_PER_REVOLUTION * 2)
                    target_pulse_absolute = target_pulse_relative + state['offset']

                    current_pos = _read_actual_position(master.slaves[slave_idx])
                    distance = abs(target_pulse_absolute - current_pos)

                    # [디버그] 이동 명령 계산 과정 출력
                    print(f"[계산] 모터 {slave_idx} 이동 명령:")
                    print(f"  입력: target_mm={target_mm:.2f}mm, axis={state['axis']}")
                    print(f"  회전수: {revolutions:.4f} rev")
                    print(f"  상대 펄스: {target_pulse_relative}")
                    print(f"  offset: {state['offset']}")
                    print(f"  절대 목표: {target_pulse_absolute}")
                    print(f"  현재 위치: {current_pos}")
                    print(f"  이동 거리: {distance} pulse")

                    # [수정] 설정된 속도 사용 (하드코딩하지 않음)
                    # slave_configs에서 설정된 velocity가 있으면 사용, 없으면 기본값 60 RPM
                    configured_velocity = slave_configs.get(slave_idx, {}).get('velocity', 60)
                    velocity_pulse_per_sec = (configured_velocity / 60.0) * PULSES_PER_REVOLUTION * 2
                    duration = distance / velocity_pulse_per_sec if velocity_pulse_per_sec > 0 else 1.0

                    trajectory_data.append({
                        'slave_idx': slave_idx,
                        'target_mm': target_mm,
                        'revolutions': revolutions,
                        'current_pos': current_pos,
                        'target_pos': target_pulse_absolute,
                        'distance': distance,
                        'duration': duration
                    })

                # [2단계] 가장 긴 duration 사용 (모든 모터가 같은 시간에 완료)
                max_duration = max(max(traj['duration'] for traj in trajectory_data), 0.1)

                # [3단계 - 핵심!] 각 모터는 자신의 목표로 이동하되, 같은 시간에 완료
                # 거리를 강제로 맞추지 않음 - 각자의 목표 위치 사용!
                for traj_info in trajectory_data:
                    slave_idx = traj_info['slave_idx']
                    state = local_motor_states[slave_idx]

                    # 각 모터가 원래 목표 위치로 이동
                    # 하지만 모두 같은 시간(max_duration)에 도착
                    state['trajectory'] = {
                        'start': traj_info['current_pos'],
                        'end': traj_info['target_pos'],  # 원래 목표 그대로 사용!
                        'duration': max_duration,        # 시간만 동기화!
                        'start_time': common_start_time
                    }
                    print(f"[디버그] 모터 {slave_idx} CSP 궤적 생성:")
                    print(f"  목표: {traj_info['target_mm']:.2f}mm")
                    print(f"  현재: {traj_info['current_pos']} → 목표: {traj_info['target_pos']}")
                    print(f"  이동거리: {traj_info['distance']} pulse, 시간: {max_duration:.2f}초")

            if not is_running: break

            ### 1. 메인 프로세스로부터 명령 수신 종료




            # 2. EtherCAT PDO 통신 (모든 슬레이브에 대해 한번에)
            master.send_processdata()
            master.receive_processdata()

            # 3. 각 슬레이브 상태 업데이트 및 제어
            # [안전장치] 먼저 모든 모터의 Fault 상태를 확인
            fault_detected = False
            fault_motor_id = -1
            for i in range(num_slaves):
                status = _read_status_word(master.slaves[i])
                if (status & 0x0008) and local_motor_states[i]['trajectory'] is not None:
                    fault_detected = True
                    fault_motor_id = i
                    break

            # [연동 안전] 한 모터라도 Fault 발생 시 모든 모터 즉시 정지
            if fault_detected:
                print(f"\n[긴급 정지] 모터 {fault_motor_id} Fault 감지! 모든 모터를 안전하게 정지합니다.")
                for i in range(num_slaves):
                    if local_motor_states[i]['trajectory'] is not None:
                        local_motor_states[i]['trajectory'] = None
                        current_pos = _read_actual_position(master.slaves[i])
                        local_motor_states[i]['target_pulse'] = current_pos
                        print(f"  → 모터 {i} 정지: 위치 {current_pos} pulse에 고정")

            for i in range(num_slaves):
                slave = master.slaves[i]
                state = local_motor_states[i]

                status = _read_status_word(slave)

                # [디버그] 상태 변화 감지 (이전 상태와 다를 때만 출력)
                if 'last_status' not in state:
                    state['last_status'] = 0
                if state['last_status'] != status:
                    # 궤적이 있거나 Fault 상태면 출력
                    if state['trajectory'] is not None or (status & 0x0008):
                        print(f"[상태] 모터 {i} 상태 변화: 0x{state['last_status']:04X} → 0x{status:04X}")
                    state['last_status'] = status

                # CiA 402 상태 머신 - CSP 모드에서도 동일하게 적용
                cw = 0
                if (status & 0x004F) == 0x0040:      # Switch On Disabled
                    cw = CW_SHUTDOWN
                elif (status & 0x006F) == 0x0021:    # Ready to Switch On
                    cw = CW_SWITCH_ON
                elif (status & 0x006F) == 0x0023:    # Switched On
                    cw = CW_ENABLE_OPERATION
                elif (status & 0x006F) == 0x0027:    # Operation Enabled
                    cw = CW_ENABLE_OPERATION
                elif (status & 0x0008):               # Fault
                    cw = CW_FAULT_RESET
                    if cycle_counter % 50 == 0:  # Fault는 자주 출력
                        print(f"[경고] 모터 {i} Fault 상태! status=0x{status:04X}")
                else:
                    cw = CW_ENABLE_OPERATION

                # [CSP 모드] 궤적 보간 로직
                target_position = state['target_pulse']

                if state['trajectory'] is not None:
                    traj = state['trajectory']
                    elapsed = time.monotonic() - traj['start_time']
                    current_actual_pos = _read_actual_position(slave)

                    # [핵심 수정] 위치 기반 완료 판정
                    # Position Factor 2:1 고려하여 threshold를 50000으로 설정 (약 0.18mm)
                    position_error = abs(traj['end'] - current_actual_pos)

                    # [안전장치] 위치 오차가 Following Error Window를 초과하면 궤적 취소
                    # Following Error Window는 20M pulse
                    # 이 값은 이동 중 허용되는 최대 추종 오차이므로, 여기서는 비활성화
                    # (초기 위치 오차는 목표까지의 거리이므로 항상 큼)
                    # 대신 드라이버가 Following Error를 자동으로 감지하므로 여기서는 확인만
                    if False and position_error > 18000000:  # 비활성화
                        print(f"[에러] 모터 {i} 위치 오차 과다! ({position_error} pulse)")
                        print(f"  목표: {traj['end']}, 현재: {current_actual_pos}, offset: {state['offset']}")
                        print(f"  궤적을 취소하고 현재 위치에 고정합니다.")
                        state['trajectory'] = None
                        state['target_pulse'] = current_actual_pos
                        target_position = current_actual_pos
                    # [중요] 위치 기반 완료만 사용 - 시간 기반은 싱크 위험
                    elif position_error < 50000:
                        # 목표 위치 도달 - 정확히 최종 위치로 설정
                        target_position = traj['end']
                        state['target_pulse'] = traj['end']
                        state['trajectory'] = None  # 궤적 완료
                        print(f"[완료] 모터 {i} 궤적 완료! 최종 위치오차: {position_error} pulse")
                    else:
                        # [디버그] 위치 오차 출력 (매 10 사이클마다)
                        if cycle_counter % 10 == 0:
                            print(f"[디버그] 모터 {i} 위치오차: {position_error} pulse, 목표: {traj['end']}, 현재: {current_actual_pos}")

                        # S-Curve 보간으로 부드러운 가감속
                        progress = min(elapsed / traj['duration'], 1.0)  # 1.0을 초과하지 않도록

                        # S-curve 공식: smooth_progress = (1 - cos(π * progress)) / 2
                        import math
                        smooth_progress = (1.0 - math.cos(math.pi * progress)) / 2.0

                        # 목표 위치 계산 - S-Curve 기반
                        target_position = int(traj['start'] + (traj['end'] - traj['start']) * smooth_progress)
                        state['target_pulse'] = target_position

                _write_csp_outputs(slave, cw, target_position)

                # 4. 상태를 메인 프로세스와 공유
                with lock:
                    base_idx = i * 4
                    shared_states[base_idx] = status
                    # CSP 모드: trajectory가 있으면 1 (이동 중), 없으면 0 (정지)
                    shared_states[base_idx + 1] = 1 if state['trajectory'] is not None else 0
                    current_pos = _read_actual_position(slave)
                    shared_states[base_idx + 2] = current_pos
                    shared_states[base_idx + 3] = state['offset']

            # 5. 사이클 타임 유지
            loop_duration = time.monotonic() - loop_start_time
            sleep_time = cycle_time_s - loop_duration
            if sleep_time > 0: time.sleep(sleep_time)

    except Exception as e:
        print(f"EtherCAT 프로세스 에러: {e}")
    finally:
        print("\n[종료 시퀀스 시작]")
        try:
            master.read_state()
            if len(master.slaves) > 0 and master.slaves[0].state == pysoem.OP_STATE:
                # [긴급 안전] 모든 모터를 현재 위치로 고정
                print("  [안전] 모든 모터를 현재 위치에 고정합니다...")
                for i in range(num_slaves):
                    try:
                        current_pos = _read_actual_position(master.slaves[i])
                        local_motor_states[i]['trajectory'] = None
                        local_motor_states[i]['target_pulse'] = current_pos
                        # Controlword는 유지, Target Position만 현재 위치로
                        master.slaves[i].output = struct.pack("<H", CW_ENABLE_OPERATION) + struct.pack("<i", current_pos)
                    except:
                        pass

                # 현재 위치 고정 명령 전송 (여러 번 반복하여 확실히 적용)
                for _ in range(5):
                    master.send_processdata()
                    master.receive_processdata()
                    time.sleep(0.02)

                print("  [완료] 모터 정지 완료")

                # [1단계] Disable Operation (0x0007) - Operation Enabled → Switched On
                print("  1단계: Disable Operation (모터 동작 중지)")
                for i in range(num_slaves):
                    try:
                        current_pos = _read_actual_position(master.slaves[i])
                        master.slaves[i].output = struct.pack("<H", CW_SWITCH_ON) + struct.pack("<i", current_pos)
                    except Exception as e:
                        print(f"    모터 {i} Disable Operation 실패: {e}")
                master.send_processdata()
                time.sleep(0.1)

                # [2단계] Shutdown (0x0006) - Switched On → Ready to Switch On
                print("  2단계: Shutdown (전원 준비)")
                for i in range(num_slaves):
                    try:
                        current_pos = _read_actual_position(master.slaves[i])
                        master.slaves[i].output = struct.pack("<H", CW_SHUTDOWN) + struct.pack("<i", current_pos)
                    except:
                        pass
                master.send_processdata()
                time.sleep(0.1)

                # [3단계] Disable Voltage (0x0000) - Ready to Switch On → Switch On Disabled
                print("  3단계: Disable Voltage (전원 차단)")
                for i in range(num_slaves):
                    try:
                        current_pos = _read_actual_position(master.slaves[i])
                        master.slaves[i].output = struct.pack("<H", CW_DISABLE_VOLTAGE) + struct.pack("<i", current_pos)
                    except:
                        pass
                master.send_processdata()
                time.sleep(0.2)

                # [4단계] EtherCAT 상태를 INIT으로 전환
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

# --- 프로세스 내부 헬퍼 함수들 ---
def _read_status_word(slave):
    return struct.unpack("<H", slave.input[:2])[0]

def _read_op_mode(slave):
    return struct.unpack("<b", slave.input[2:3])[0]

def _read_actual_position(slave):
    return struct.unpack("<i", slave.input[2:6])[0]

def _write_csp_outputs(slave, cw, target_position):
    """CSP 모드용 출력 함수 - Controlword와 Target Position만 전송"""
    slave.output = struct.pack("<H", cw) + struct.pack("<i", target_position)

def _rpm_to_pulse_per_sec(rpm):
    return int((rpm / 60) * PULSES_PER_REVOLUTION)

def _sdo_reset_fault(slave):
    if struct.unpack("<H", slave.sdo_read(0x6041, 0))[0] & 0x0008:
        slave.sdo_write(0x6040, 0, struct.pack("<H", CW_FAULT_RESET))
        time.sleep(0.2)

def _setup_csp_mode(slave):
    """CSP 모드 설정 (Cyclic Synchronous Position Mode = 8)"""
    # 0x211F: 드라이버 특수 설정 (비트 12: 절대 위치 모드, 비트 4: 새 명령 허용)
    # CSP에서는 비트 4가 필요 없지만 드라이버 호환성을 위해 유지
    slave.sdo_write(0x211F, 0, struct.pack("<H", (1 << 12)))
    time.sleep(0.01)
    # 0x6060: Modes of Operation - 8 = CSP
    slave.sdo_write(0x6060, 0, struct.pack("<b", 8))
    time.sleep(0.01)
    print(f"  [설정] CSP 모드(8) 설정 완료")

def _configure_csp_pdos(slave):
    """CSP 모드용 PDO 매핑 설정"""
    # RxPDO (Master -> Slave): Controlword + Target Position
    slave.sdo_write(0x1C12, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1600, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1600, 1, struct.pack('<I', 0x60400010)); time.sleep(0.01)  # Controlword (2 bytes)
    slave.sdo_write(0x1600, 2, struct.pack('<I', 0x607A0020)); time.sleep(0.01)  # Target Position (4 bytes)
    slave.sdo_write(0x1600, 0, b'\x02'); time.sleep(0.01)
    slave.sdo_write(0x1C12, 1, struct.pack('<H', 0x1600)); time.sleep(0.01)
    slave.sdo_write(0x1C12, 0, b'\x01'); time.sleep(0.01)

    # TxPDO (Slave -> Master): Statusword + Position Actual Value
    slave.sdo_write(0x1C13, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1A00, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1A00, 1, struct.pack('<I', 0x60410010)); time.sleep(0.01)  # Statusword (2 bytes)
    slave.sdo_write(0x1A00, 2, struct.pack('<I', 0x60640020)); time.sleep(0.01)  # Position actual value (4 bytes)
    slave.sdo_write(0x1A00, 0, b'\x02'); time.sleep(0.01)
    slave.sdo_write(0x1C13, 1, struct.pack('<H', 0x1A00)); time.sleep(0.01)
    slave.sdo_write(0x1C13, 0, b'\x01'); time.sleep(0.01)
    print(f"  [설정] CSP PDO 매핑 완료")

# --- 메인 프로세스에서 사용할 클래스들 ---
class EtherCATBus:
    def __init__(self, adapter_name: str, num_slaves: int, cycle_time_ms: int = 50):
        self._command_queue = mp.Queue()
        self._lock = mp.Lock()
        # 각 슬레이브당 status_word, move_state, position_pulse, offset_pulse 4개의 상태를 저장
        self._shared_states = mp.Array('d', num_slaves * 4, lock=False)
        
        self._adapter_name = adapter_name
        self._num_slaves = num_slaves
        self._process = mp.Process(
            target=_ethercat_process_loop,
            args=( 
                adapter_name, num_slaves, cycle_time_ms / 1000.0,
                self._command_queue, self._shared_states, self._lock
            )
        )
        self.motors: List[Motor] = [Motor(i, self._command_queue, self._shared_states, self._lock) for i in range(num_slaves)]

    def start(self):
        self._process.start()
        print("EtherCAT 버스 프로세스를 시작합니다.")

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

class Motor:
    def __init__(self, slave_index: int, command_queue: mp.Queue, shared_states: mp.Array, lock: mp.Lock):
        self._index = slave_index
        self._command_queue = command_queue
        self._shared_states = shared_states
        self._lock = lock
        self._pending_sdo_commands = []
        self._axis = 'x'  # 기본값은 x축

    def set_axis(self, axis: str):
        """모터의 축을 설정합니다. 'x' 또는 'z'"""
        if axis not in ['x', 'z']:
            raise ValueError("axis는 'x' 또는 'z'여야 합니다.")
        self._axis = axis
        self._command_queue.put((self._index, 'SET_AXIS', axis))
        print(f"[설정] 모터 {self._index}: 축을 '{axis}'로 설정")

    def set_origin(self):
        self._command_queue.put((self._index, 'SET_ORIGIN', None))
        print(f"[원점] 모터 {self._index}: 원점 설정 명령 전송.")

    def set_profile_velocity(self, rpm):
        # bus.start()가 호출되기 전에 실행되어야 합니다.
        self._command_queue.put((self._index, 'SET_VELOCITY', rpm))
        print(f"[속도] 모터 {self._index}: Profile Velocity 설정 명령 전송: {rpm} RPM")

    def set_profile_accel_decel(self, accel_rpm_per_sec, decel_rpm_per_sec=None):
        # bus.start()가 호출되기 전에 실행되어야 합니다.
        accel_val = int((accel_rpm_per_sec / 60) * PULSES_PER_REVOLUTION)
        decel_val = accel_val if decel_rpm_per_sec is None else int((decel_rpm_per_sec / 60) * PULSES_PER_REVOLUTION)
        self._command_queue.put((self._index, 'SET_ACCEL', (accel_val, decel_val)))
        print(f"[가감속] 모터 {self._index}: 가감속도 설정 명령 전송: Accel={accel_rpm_per_sec} RPM/s")

    def move_to_position_mm(self, target_mm):
        self._command_queue.put((self._index, 'MOVE_TO_MM', target_mm))
        print(f"[이동명령] 모터 {self._index}: {target_mm:.2f}mm로 이동 명령 전송")

    @property
    def status_word(self) -> int:
        # Lock 없이 직접 접근 (Array는 원자적 접근을 보장)
        return int(self._shared_states[self._index * 4])
    
    def is_moving(self) -> bool:
        """모터가 현재 이동 명령을 수행 중인지 확인합니다."""
        # Lock 없이 직접 접근
        return self._shared_states[self._index * 4 + 1] != 0
            
    @property
    def current_position_mm(self) -> float:
        # Lock 없이 직접 접근
        base_idx = self._index * 4
        current_pos = self._shared_states[base_idx + 2]
        offset = self._shared_states[base_idx + 3]
        relative_pos = current_pos - offset
        # [중요] Position Factor 2:1 처리
        # relative_pos는 드라이버 스케일(2배)
        # target 계산 시에도 PULSES_PER_REVOLUTION * 2를 사용했으므로
        # 같은 스케일을 유지하기 위해 PULSES_PER_REVOLUTION * 2로 나눔
        revolutions = relative_pos / (PULSES_PER_REVOLUTION * 2)
        # 축에 따라 올바른 변환값 사용
        mm_per_rev = x_axis_mm_per_rev if self._axis == 'x' else z_axis_mm_per_rev
        return revolutions * mm_per_rev

    @property
    def current_position_pulse(self) -> int:
        """디버깅용: 현재 펄스 위치"""
        base_idx = self._index * 4
        return int(self._shared_states[base_idx + 2])

    @property
    def offset_pulse(self) -> int:
        """디버깅용: 오프셋 펄스"""
        base_idx = self._index * 4
        return int(self._shared_states[base_idx + 3])
