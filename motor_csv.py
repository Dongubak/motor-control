"""
motor_csv.py - CSV (Cyclic Synchronous Velocity) 모드 모터 제어 라이브러리

CiA 402 Mode 9: Cyclic Synchronous Velocity (CSV)

CSP 모드(motor_coupling.py) 대비 변경 사항:
1. 모드 9 (CSV): 매 사이클 속도 명령(0x60FF) 전송
2. 소프트웨어 사다리꼴 속도 프로파일 생성 (TrapezoidalProfile)
3. Cross Coupling: 위치 오차 → 속도 보정 (단위: 1/sec)
4. 궤적 완료 후 P 제어 위치 유지 (드라이브 내부 위치 루프 없음)
5. TxPDO에 실제 속도(0x606C) 추가

PDO 구성:
  RxPDO (마스터→드라이브): Controlword(0x6040) + Target Velocity(0x60FF)
  TxPDO (드라이브→마스터): Statusword(0x6041) + Actual Pos(0x6064) + Actual Vel(0x606C)
"""

import pysoem
import struct
import time
import math
import multiprocessing as mp
from collections import deque
from typing import List


# ─────────────────────────────────────────────────────────────
# CiA 402 Controlword 상수
# ─────────────────────────────────────────────────────────────
CW_SHUTDOWN         = 0x0006
CW_SWITCH_ON        = 0x0007
CW_ENABLE_OPERATION = 0x000F
CW_DISABLE_VOLTAGE  = 0x0000
CW_FAULT_RESET      = 0x0080

# ─────────────────────────────────────────────────────────────
# 기계 상수
# ─────────────────────────────────────────────────────────────
PULSES_PER_REVOLUTION = 8_388_608          # 모터 엔코더 분해능 (0x2002 = 2^23)
EFFECTIVE_PPR         = PULSES_PER_REVOLUTION      # 8,388,608 pulse/rev (전기적 기어비 1:1)

x_axis_mm_per_rev = 11.9993131404          # x축: 1 revolution당 이동 거리 (mm)
z_axis_mm_per_rev = 5.99965657019          # z축: 1 revolution당 이동 거리 (mm)

# ─────────────────────────────────────────────────────────────
# CSV 모드 기본값
# ─────────────────────────────────────────────────────────────
DEFAULT_MAX_SYNC_ERROR_MM  = 0.5   # 허용 동기화 오차 (mm)
DEFAULT_COUPLING_GAIN      = 0.5   # Cross Coupling 게인 [1/sec] — 1mm 오차 시 0.5mm/sec 보정
DEFAULT_MA_WINDOW          = 5     # 이동 평균 윈도우 (사이클 수)

POSITION_TOLERANCE_PULSE   = 50_000  # 궤적 완료 판정 오차 (pulse) ≈ 0.018mm (z축)
POSITION_HOLD_GAIN         = 2.0     # 위치 유지 P 게인 [1/sec] — 1mm 오차 시 2mm/sec 보정
POSITION_HOLD_MAX_VEL_MM_S = 5.0     # 위치 유지 최대 속도 (mm/sec)


# ─────────────────────────────────────────────────────────────
# 사다리꼴 속도 프로파일 생성기
# ─────────────────────────────────────────────────────────────
class TrapezoidalProfile:
    """
    CSV 모드용 소프트웨어 사다리꼴 속도 프로파일.

    3단계 구성: 가속 → 등속(코스팅) → 감속
    이동 거리가 짧으면 자동으로 삼각형 프로파일 적용 (등속 구간 없음).

    속도 단위: pulse/sec  (부호 포함, 방향 자동 결정)
    시간 단위: sec
    """

    def __init__(self, start_pos: int, end_pos: int,
                 max_vel_pps: float, accel_pps2: float):
        """
        Args:
            start_pos   : 시작 위치 (pulse)
            end_pos     : 목표 위치 (pulse)
            max_vel_pps : 최대 속도 (pulse/sec, 양수)
            accel_pps2  : 가감속도 (pulse/sec², 양수)
        """
        self.start_pos = start_pos
        self.end_pos   = end_pos
        distance       = end_pos - start_pos
        self._dir      = 1 if distance >= 0 else -1
        D              = abs(float(distance))
        self._A        = max(float(accel_pps2), 1.0)
        V              = max(float(max_vel_pps), 1.0)

        # 가속 구간에서 최대 속도 도달까지 필요한 거리
        d_accel = V * V / (2.0 * self._A)

        if D <= 2.0 * d_accel:
            # 삼각형 프로파일 — 최대 속도 미달
            self._V_peak  = math.sqrt(self._A * D) if D > 0 else 0.0
            self._t_accel = self._V_peak / self._A
            self._t_coast = 0.0
        else:
            # 사다리꼴 프로파일
            self._V_peak  = V
            self._t_accel = V / self._A
            d_coast       = D - 2.0 * d_accel
            self._t_coast = d_coast / V

        self.t_total = 2.0 * self._t_accel + self._t_coast

    def get_velocity(self, elapsed: float) -> int:
        """
        경과 시간(sec)에 대한 순간 속도를 반환한다 (pulse/sec, 부호 포함).

        프로파일 완료 후(elapsed > t_total)에는 0을 반환.
        """
        if elapsed <= 0.0 or self._V_peak == 0.0:
            return 0

        if elapsed <= self._t_accel:
            v = self._A * elapsed
        elif elapsed <= self._t_accel + self._t_coast:
            v = self._V_peak
        elif elapsed <= self.t_total:
            t_dec = elapsed - self._t_accel - self._t_coast
            v = max(0.0, self._V_peak - self._A * t_dec)
        else:
            v = 0.0

        return int(self._dir * v)

    def is_time_complete(self, elapsed: float) -> bool:
        """속도 프로파일 시간 완료 여부."""
        return elapsed >= self.t_total


# ─────────────────────────────────────────────────────────────
# 단위 변환 헬퍼
# ─────────────────────────────────────────────────────────────
def _mm_to_pulse(mm: float, axis: str = 'z') -> int:
    mm_per_rev = x_axis_mm_per_rev if axis == 'x' else z_axis_mm_per_rev
    return int((mm / mm_per_rev) * EFFECTIVE_PPR)

def _pulse_to_mm(pulse: float, axis: str = 'z') -> float:
    mm_per_rev = x_axis_mm_per_rev if axis == 'x' else z_axis_mm_per_rev
    return (pulse / EFFECTIVE_PPR) * mm_per_rev

def _rpm_to_pps(rpm: float) -> float:
    """RPM → pulse/sec"""
    return (rpm / 60.0) * EFFECTIVE_PPR

def _rpm_per_sec_to_pps2(rpm_per_sec: float) -> float:
    """RPM/sec → pulse/sec²"""
    return (rpm_per_sec / 60.0) * EFFECTIVE_PPR


# ─────────────────────────────────────────────────────────────
# PDO 입출력 헬퍼
# ─────────────────────────────────────────────────────────────
def _read_status_word(slave) -> int:
    return struct.unpack("<H", slave.input[0:2])[0]

def _read_actual_position(slave) -> int:
    return struct.unpack("<i", slave.input[2:6])[0]

def _read_actual_velocity(slave) -> int:
    return struct.unpack("<i", slave.input[6:10])[0]

def _write_csv_outputs(slave, cw: int, target_velocity: int):
    """CSV 모드 RxPDO 출력: Controlword(2B) + Target Velocity(4B)"""
    slave.output = struct.pack("<H", cw) + struct.pack("<i", target_velocity)


# ─────────────────────────────────────────────────────────────
# 초기화 헬퍼
# ─────────────────────────────────────────────────────────────
def _sdo_reset_fault(slave):
    if struct.unpack("<H", slave.sdo_read(0x6041, 0))[0] & 0x0008:
        slave.sdo_write(0x6040, 0, struct.pack("<H", CW_FAULT_RESET))
        time.sleep(0.2)

def _setup_csv_mode(slave):
    """드라이브를 CSV 모드(모드 9)로 설정"""
    slave.sdo_write(0x211F, 0, struct.pack("<H", (1 << 12)))
    time.sleep(0.01)
    slave.sdo_write(0x6060, 0, struct.pack("<b", 9))   # 9 = CSV
    time.sleep(0.01)
    print("  [설정] CSV 모드(9) 설정 완료")

def _configure_csv_pdos(slave):
    """CSV 모드용 PDO 매핑 구성"""
    # ── RxPDO 0x1600: Controlword(16bit) + Target Velocity(32bit) ──
    slave.sdo_write(0x1C12, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1600, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1600, 1, struct.pack('<I', 0x60400010)); time.sleep(0.01)
    slave.sdo_write(0x1600, 2, struct.pack('<I', 0x60FF0020)); time.sleep(0.01)
    slave.sdo_write(0x1600, 0, b'\x02'); time.sleep(0.01)
    slave.sdo_write(0x1C12, 1, struct.pack('<H', 0x1600)); time.sleep(0.01)
    slave.sdo_write(0x1C12, 0, b'\x01'); time.sleep(0.01)

    # ── TxPDO 0x1A00: Statusword(16bit) + Actual Pos(32bit) + Actual Vel(32bit) ──
    slave.sdo_write(0x1C13, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1A00, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1A00, 1, struct.pack('<I', 0x60410010)); time.sleep(0.01)
    slave.sdo_write(0x1A00, 2, struct.pack('<I', 0x60640020)); time.sleep(0.01)
    slave.sdo_write(0x1A00, 3, struct.pack('<I', 0x606C0020)); time.sleep(0.01)
    slave.sdo_write(0x1A00, 0, b'\x03'); time.sleep(0.01)
    slave.sdo_write(0x1C13, 1, struct.pack('<H', 0x1A00)); time.sleep(0.01)
    slave.sdo_write(0x1C13, 0, b'\x01'); time.sleep(0.01)
    print("  [설정] CSV PDO 매핑 완료")


# ─────────────────────────────────────────────────────────────
# EtherCAT 제어 루프 (별도 프로세스)
# ─────────────────────────────────────────────────────────────
def _ethercat_process_loop_csv(
    adapter_name, num_slaves, cycle_time_s,
    command_queue, shared_states, lock,
    max_sync_error_pulse,
    coupling_params,   # mp.Array: [0]=gain(1/sec), [1]=enabled(0|1)
    ma_window
):
    """
    [CSV 모드] EtherCAT 통신 및 속도 기반 제어 루프.

    shared_states 레이아웃 (슬레이브당 6슬롯):
      base+0 : statusword
      base+1 : is_moving (0|1)
      base+2 : actual_position (pulse)
      base+3 : offset (pulse)
      base+4 : sync_error (0|1)
      base+5 : actual_velocity (pulse/sec)
    """
    master = pysoem.Master()

    # 슬레이브별 로컬 상태
    local_states = [{
        'target_pulse'  : 0,
        'offset'        : 0,
        'axis'          : 'z',
        'trajectory'    : None,      # TrapezoidalProfile or None
        'profile_start' : 0.0,       # 프로파일 시작 시각 (monotonic)
        'traj_end'      : 0,         # 목표 위치 (pulse)
        'config_vel_pps': 0.0,       # 설정 최대 속도 (pulse/sec)
        'config_acc_pps2': 0.0,      # 설정 가속도 (pulse/sec²)
        'last_status'   : 0,
    } for _ in range(num_slaves)]

    is_running          = True
    sync_error_detected = False
    diff_histories = [deque(maxlen=ma_window) for _ in range(max(0, num_slaves - 1))]

    # 위치 유지 속도 한계 (pulse/sec)
    hold_max_vel = _mm_to_pulse(POSITION_HOLD_MAX_VEL_MM_S, 'z')

    # ── 초기 SDO 설정 수집 ──
    slave_configs = {i: {} for i in range(num_slaves)}
    while not command_queue.empty():
        try:
            idx, cmd, val = command_queue.get_nowait()
        except Exception:
            break
        if cmd == 'SET_VELOCITY':
            slave_configs[idx]['rpm'] = val
        elif cmd == 'SET_ACCEL':
            slave_configs[idx]['accel'] = val          # (accel_pps2, decel_pps2)
        elif cmd == 'SET_AXIS':
            local_states[idx]['axis'] = val
            print(f"[초기화] 모터 {idx} 축: '{val}'")

    # ── EtherCAT 초기화 (재시도 포함) ──
    init_success = False
    for retry in range(3):
        try:
            if retry > 0:
                print(f"\n[재시도 {retry}] EtherCAT 초기화 재시도...")
                try: master.close()
                except: pass
                time.sleep(1.0)
                master = pysoem.Master()

            print("[초기화] EtherCAT 어댑터 열기...")
            master.open(adapter_name)

            found = master.config_init()
            if found < num_slaves:
                raise RuntimeError(f"슬레이브 부족: 필요={num_slaves}, 발견={found}")
            print(f"[성공] {found}개 슬레이브 발견")

            for i in range(num_slaves):
                slave = master.slaves[i]
                print(f"[연결] Slave {i}: {slave.name}")
                slave.dc_sync(True, int(cycle_time_s * 1_000_000_000))
                _sdo_reset_fault(slave)

                cfg = slave_configs.get(i, {})
                if 'rpm' in cfg:
                    pps = _rpm_to_pps(cfg['rpm'])
                    local_states[i]['config_vel_pps'] = pps
                    slave.sdo_write(0x6081, 0, struct.pack("<I", int(pps)))
                if 'accel' in cfg:
                    acc_pps2, dec_pps2 = cfg['accel']
                    local_states[i]['config_acc_pps2'] = acc_pps2
                    slave.sdo_write(0x6083, 0, struct.pack("<I", int(acc_pps2)))
                    slave.sdo_write(0x6084, 0, struct.pack("<I", int(dec_pps2)))

                _configure_csv_pdos(slave)
                _setup_csv_mode(slave)

            pdo_size = master.config_map()
            print(f"[성공] PDO 맵핑 완료. Size: {pdo_size} bytes")
            print(f"[안전] 동기화 오차 임계값: "
                  f"{_pulse_to_mm(max_sync_error_pulse, 'z'):.3f}mm")
            print(f"[CSV] Cross Coupling 게인: {coupling_params[0]:.4f} [1/sec]")

            init_success = True
            break

        except Exception as e:
            print(f"[에러] 초기화 실패 (retry {retry}): {e}")
            if retry == 2:
                try: master.close()
                except: pass
                return

    if not init_success:
        return

    try:
        # ── OP 상태 전환 (재시도 포함) ──
        for op_retry in range(3):
            try:
                if op_retry > 0:
                    print(f"\n[재시도 {op_retry}] OP 전환 재시도...")
                    time.sleep(0.5)

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
                    raise RuntimeError("OP 상태 도달 실패")

                print("[성공] OP 상태 도달. 주기 통신 시작.")

                for i in range(num_slaves):
                    pos = _read_actual_position(master.slaves[i])
                    local_states[i]['target_pulse'] = pos
                    print(f"  [CSV 초기화] 모터 {i}: 현재 위치={pos} pulse")
                break

            except Exception as e:
                print(f"[에러] OP 전환 실패: {e}")
                if op_retry == 2:
                    raise

        # ═══════════════════════════════════════════════════════
        # 메인 제어 루프
        # ═══════════════════════════════════════════════════════
        cycle_counter = 0

        while is_running:
            loop_start    = time.monotonic()
            cycle_counter += 1

            # Cross Coupling 파라미터 (런타임 변경 가능)
            coupling_enabled   = coupling_params[1] != 0
            coupling_gain      = coupling_params[0]   # 1/sec

            # ── 1. 명령 수신 ──
            move_commands = []
            while not command_queue.empty():
                try:
                    idx, cmd, val = command_queue.get_nowait()
                except Exception:
                    break

                if cmd == 'STOP_ALL':
                    is_running = False
                    break
                elif cmd == 'SET_AXIS':
                    local_states[idx]['axis'] = val
                elif cmd == 'SET_ORIGIN':
                    pos = _read_actual_position(master.slaves[idx])
                    local_states[idx]['offset']       = pos
                    local_states[idx]['target_pulse'] = pos
                    local_states[idx]['trajectory']   = None
                    sync_error_detected = False
                    for h in diff_histories:
                        h.clear()
                    print(f"[원점] 모터 {idx}: offset={pos}")
                elif cmd == 'MOVE_TO_MM':
                    if sync_error_detected:
                        print(f"[경고] 동기화 오류! 모터 {idx} 이동 무시")
                    else:
                        move_commands.append((idx, val))
                elif cmd == 'RESET_SYNC_ERROR':
                    sync_error_detected = False
                    for h in diff_histories:
                        h.clear()
                    print("[안전] 동기화 오류 리셋")

            if not is_running:
                break

            # ── 2. PDO 통신 (이전 사이클의 출력 전송, 이번 사이클 입력 수신) ──
            master.send_processdata()
            master.receive_processdata()

            now = time.monotonic()

            # 실제 위치/속도 읽기
            positions  = [_read_actual_position(master.slaves[i]) for i in range(num_slaves)]
            velocities = [_read_actual_velocity(master.slaves[i])  for i in range(num_slaves)]
            relative_positions = [
                positions[i] - local_states[i]['offset']
                for i in range(num_slaves)
            ]

            # ── 3. 이동 명령 → 궤적 생성 ──
            if move_commands:
                common_start = now
                for idx, target_mm in move_commands:
                    state  = local_states[idx]
                    axis   = state['axis']
                    cur    = positions[idx]
                    target = _mm_to_pulse(target_mm, axis) + state['offset']

                    v_max  = state['config_vel_pps']
                    a_max  = state['config_acc_pps2']
                    if v_max <= 0:
                        v_max = _rpm_to_pps(30)
                    if a_max <= 0:
                        a_max = _rpm_per_sec_to_pps2(30)

                    profile = TrapezoidalProfile(cur, target, v_max, a_max)
                    state['trajectory']     = profile
                    state['profile_start']  = common_start
                    state['traj_end']       = target
                    state['target_pulse']   = cur

                    dist_mm = abs(_pulse_to_mm(target - cur, axis))
                    print(f"[궤적] 모터 {idx}: {target_mm:.2f}mm "
                          f"(거리={dist_mm:.2f}mm, 시간={profile.t_total:.2f}s)")

            # ── 4. 이동 중인 모터 목록 ──
            moving_motors = [
                i for i in range(num_slaves)
                if local_states[i]['trajectory'] is not None
            ]

            # ── 5. 동기화 모니터링 (이동 평균 필터) ──
            if num_slaves >= 2 and not sync_error_detected:
                for i in range(num_slaves - 1):
                    if (local_states[i]['trajectory'] is not None and
                            local_states[i + 1]['trajectory'] is not None):
                        diff = abs(relative_positions[i] - relative_positions[i + 1])
                        diff_histories[i].append(diff)
                        avg_diff = sum(diff_histories[i]) / len(diff_histories[i])

                        if avg_diff > max_sync_error_pulse:
                            sync_error_detected = True
                            raw_mm = _pulse_to_mm(diff,     'z')
                            avg_mm = _pulse_to_mm(avg_diff, 'z')
                            lim_mm = _pulse_to_mm(max_sync_error_pulse, 'z')
                            print(f"\n{'='*60}")
                            print(f"[긴급 정지] 동기화 오차 초과! M{i} vs M{i+1}")
                            print(f"  현재={raw_mm:.3f}mm, 이동평균={avg_mm:.3f}mm, "
                                  f"임계={lim_mm:.3f}mm")
                            print(f"{'='*60}")
                            for j in range(num_slaves):
                                if local_states[j]['trajectory'] is not None:
                                    local_states[j]['trajectory']   = None
                                    local_states[j]['target_pulse'] = positions[j]
                                    print(f"  → 모터 {j} 긴급 정지")
                            break

            # ── 6. Cross Coupling 속도 보정값 계산 ──
            coupling_correction = [0] * num_slaves   # pulse/sec

            if coupling_enabled and len(moving_motors) >= 2 and not sync_error_detected:
                avg_rel = (
                    sum(relative_positions[i] for i in moving_motors)
                    / len(moving_motors)
                )
                for i in moving_motors:
                    error_pulse = relative_positions[i] - avg_rel
                    # gain [1/sec] × error [pulse] = correction [pulse/sec]
                    coupling_correction[i] = int(coupling_gain * error_pulse)

                if cycle_counter % 50 == 0:
                    print(f"[Cross Coupling] 이동 모터: {moving_motors}")
                    for i in moving_motors:
                        err_mm  = _pulse_to_mm(relative_positions[i] - avg_rel, 'z')
                        corr_ms = _pulse_to_mm(coupling_correction[i], 'z')
                        print(f"  M{i}: 오차={err_mm:+.4f}mm, 보정={corr_ms:+.4f}mm/s")

            # ── 7. Fault 감지 ──
            for i in range(num_slaves):
                sw = _read_status_word(master.slaves[i])
                if (sw & 0x0008) and local_states[i]['trajectory'] is not None:
                    print(f"\n[긴급 정지] 모터 {i} Fault! status=0x{sw:04X}")
                    sync_error_detected = True  # 모터 Fault를 메인 프로세스로 전파
                    for j in range(num_slaves):
                        if local_states[j]['trajectory'] is not None:
                            local_states[j]['trajectory']   = None
                            local_states[j]['target_pulse'] = positions[j]
                            print(f"  → 모터 {j} 속도=0으로 정지")
                    break

            # ── 8. 각 슬레이브 속도 명령 결정 ──
            for i in range(num_slaves):
                slave = master.slaves[i]
                state = local_states[i]
                sw    = _read_status_word(slave)

                # Statusword 변화 로그
                if state['last_status'] != sw:
                    if state['trajectory'] is not None or (sw & 0x0008):
                        print(f"[상태] 모터 {i}: 0x{state['last_status']:04X} → 0x{sw:04X}")
                    state['last_status'] = sw

                # Controlword 결정
                if   (sw & 0x004F) == 0x0040: cw = CW_SHUTDOWN
                elif (sw & 0x006F) == 0x0021: cw = CW_SWITCH_ON
                elif (sw & 0x006F) == 0x0023: cw = CW_ENABLE_OPERATION
                elif (sw & 0x006F) == 0x0027: cw = CW_ENABLE_OPERATION
                elif (sw & 0x0008):
                    cw = CW_FAULT_RESET
                    if cycle_counter % 50 == 0:
                        print(f"[경고] 모터 {i} Fault 상태 (0x{sw:04X})")
                else:
                    cw = CW_ENABLE_OPERATION

                # ── 목표 속도 결정 ──
                target_velocity = 0

                if state['trajectory'] is not None:
                    traj    = state['trajectory']
                    elapsed = now - state['profile_start']
                    actual  = positions[i]

                    if traj.is_time_complete(elapsed):
                        # 프로파일 완료 → P 제어 위치 유지
                        pos_err = state['traj_end'] - actual
                        if abs(pos_err) < POSITION_TOLERANCE_PULSE:
                            # 완료 판정
                            state['trajectory']   = None
                            state['target_pulse'] = state['traj_end']
                            target_velocity = 0
                            err_mm = _pulse_to_mm(pos_err, 'z')
                            print(f"[완료] 모터 {i}: 궤적 완료 (잔류오차={err_mm:.4f}mm)")
                        else:
                            # P 제어로 잔류 오차 보정
                            v = int(POSITION_HOLD_GAIN * pos_err)
                            target_velocity = max(-hold_max_vel, min(hold_max_vel, v))
                    else:
                        # 사다리꼴 프로파일 속도 적용 + Cross Coupling 보정
                        profile_vel     = traj.get_velocity(elapsed)
                        target_velocity = profile_vel - coupling_correction[i]
                        state['target_pulse'] = actual

                else:
                    # 정지 상태: 위치 유지 P 제어 (CSV는 드라이브 내부 위치 루프 없음)
                    pos_err = state['target_pulse'] - positions[i]
                    if abs(pos_err) > POSITION_TOLERANCE_PULSE:
                        v = int(POSITION_HOLD_GAIN * pos_err)
                        target_velocity = max(-hold_max_vel, min(hold_max_vel, v))
                    else:
                        target_velocity = 0

                _write_csv_outputs(slave, cw, target_velocity)

                # 상태 공유 (메인 프로세스로)
                with lock:
                    base = i * 6
                    shared_states[base]     = sw
                    shared_states[base + 1] = 1 if state['trajectory'] is not None else 0
                    shared_states[base + 2] = positions[i]
                    shared_states[base + 3] = state['offset']
                    shared_states[base + 4] = 1 if sync_error_detected else 0
                    shared_states[base + 5] = velocities[i]

            # 디버그: 동기화 상태 (50사이클마다)
            if cycle_counter % 50 == 0 and num_slaves >= 2:
                for i in range(num_slaves - 1):
                    if (local_states[i]['trajectory'] is not None and
                            local_states[i + 1]['trajectory'] is not None):
                        diff    = abs(relative_positions[i] - relative_positions[i + 1])
                        diff_mm = _pulse_to_mm(diff, 'z')
                        if diff_histories[i]:
                            avg_mm = _pulse_to_mm(
                                sum(diff_histories[i]) / len(diff_histories[i]), 'z')
                            print(f"[동기화] M{i}-M{i+1}: {diff_mm:.3f}mm "
                                  f"(MA: {avg_mm:.3f}mm, {len(diff_histories[i])}/{ma_window})")

            # ── 9. 사이클 타임 유지 ──
            elapsed_loop = time.monotonic() - loop_start
            sleep_time   = cycle_time_s - elapsed_loop
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        print(f"EtherCAT 프로세스 에러: {e}")
    finally:
        print("\n[종료 시퀀스 시작]")
        try:
            master.read_state()
            if len(master.slaves) > 0 and master.slaves[0].state == pysoem.OP_STATE:
                print("  모든 모터 속도=0으로 정지...")
                for i in range(num_slaves):
                    try: _write_csv_outputs(master.slaves[i], CW_ENABLE_OPERATION, 0)
                    except: pass
                for _ in range(5):
                    master.send_processdata()
                    master.receive_processdata()
                    time.sleep(0.02)

                for i in range(num_slaves):
                    try: _write_csv_outputs(master.slaves[i], CW_SWITCH_ON, 0)
                    except: pass
                master.send_processdata(); time.sleep(0.1)

                for i in range(num_slaves):
                    try: _write_csv_outputs(master.slaves[i], CW_SHUTDOWN, 0)
                    except: pass
                master.send_processdata(); time.sleep(0.1)

                for i in range(num_slaves):
                    try: _write_csv_outputs(master.slaves[i], CW_DISABLE_VOLTAGE, 0)
                    except: pass
                master.send_processdata(); time.sleep(0.2)

                master.state = pysoem.INIT_STATE
                master.write_state()
                time.sleep(0.1)
                print("  [완료] 종료 시퀀스 완료")
        except Exception as e:
            print(f"  [경고] 종료 중 에러 (무시): {e}")
        finally:
            master.close()
            print("EtherCAT 프로세스 종료.\n")


# ─────────────────────────────────────────────────────────────
# 버스 관리 클래스
# ─────────────────────────────────────────────────────────────
class EtherCATBusCSV:
    """
    [CSV 버전] EtherCAT 버스 관리 클래스.

    Parameters
    ----------
    adapter_name      : EtherCAT 어댑터 경로 (\\Device\\NPF_{...})
    num_slaves        : 슬레이브(모터 드라이브) 수
    cycle_time_ms     : 제어 사이클 시간 (ms)
    max_sync_error_mm : 긴급 정지 동기화 오차 임계값 (mm)
    coupling_gain     : Cross Coupling 게인 [1/sec]
                        1.0 = 1mm 위치오차 시 1mm/sec 속도 보정
    enable_coupling   : Cross Coupling 활성화 여부
    ma_window         : 동기화 오차 이동 평균 윈도우 (사이클 수)
    """

    def __init__(self, adapter_name: str, num_slaves: int,
                 cycle_time_ms: int = 10,
                 max_sync_error_mm: float = DEFAULT_MAX_SYNC_ERROR_MM,
                 coupling_gain: float = DEFAULT_COUPLING_GAIN,
                 enable_coupling: bool = True,
                 ma_window: int = DEFAULT_MA_WINDOW):

        self._command_queue  = mp.Queue()
        self._lock           = mp.Lock()
        self._shared_states  = mp.Array('d', num_slaves * 6, lock=False)
        self._coupling_params = mp.Array('d', 2, lock=True)
        self._coupling_params[0] = coupling_gain
        self._coupling_params[1] = 1.0 if enable_coupling else 0.0

        self._adapter_name       = adapter_name
        self._num_slaves         = num_slaves
        self._max_sync_error_mm  = max_sync_error_mm
        self._ma_window          = ma_window
        self._max_sync_error_pulse = _mm_to_pulse(max_sync_error_mm, 'z')

        self._process = mp.Process(
            target=_ethercat_process_loop_csv,
            args=(
                adapter_name, num_slaves, cycle_time_ms / 1000.0,
                self._command_queue, self._shared_states, self._lock,
                self._max_sync_error_pulse,
                self._coupling_params,
                ma_window
            )
        )
        self.motors: List[MotorCSV] = [
            MotorCSV(i, self._command_queue, self._shared_states, self._lock)
            for i in range(num_slaves)
        ]

    def start(self):
        self._process.start()
        print(f"[CSV 버스 시작]")
        print(f"  동기화 오차 임계값: {self._max_sync_error_mm}mm")
        print(f"  이동 평균 윈도우: {self._ma_window} 사이클")
        print(f"  Cross Coupling 게인: {self._coupling_params[0]:.4f} [1/sec]")
        print(f"  Cross Coupling 활성화: {self._coupling_params[1] != 0}")

    def stop(self):
        print("EtherCAT 버스 종료 중...")
        if self._process.is_alive():
            self._command_queue.put((-1, 'STOP_ALL', None))
            self._process.join(timeout=3)
            if self._process.is_alive():
                print("강제 종료")
                self._process.terminate()
        print("EtherCAT 버스 종료 완료.")

    def reset_sync_error(self):
        self._command_queue.put((-1, 'RESET_SYNC_ERROR', None))

    @property
    def has_sync_error(self) -> bool:
        return self._shared_states[4] != 0

    # ── Cross Coupling 런타임 제어 ──

    @property
    def coupling_gain(self) -> float:
        return self._coupling_params[0]

    @coupling_gain.setter
    def coupling_gain(self, value: float):
        if value < 0.0:
            raise ValueError("coupling_gain은 0 이상이어야 합니다.")
        self._coupling_params[0] = value
        print(f"[Cross Coupling] 게인 변경: {value:.4f} [1/sec]")

    @property
    def coupling_enabled(self) -> bool:
        return self._coupling_params[1] != 0

    @coupling_enabled.setter
    def coupling_enabled(self, value: bool):
        self._coupling_params[1] = 1.0 if value else 0.0
        print(f"[Cross Coupling] {'활성화' if value else '비활성화'}")


# ─────────────────────────────────────────────────────────────
# 개별 모터 제어 클래스
# ─────────────────────────────────────────────────────────────
class MotorCSV:
    """
    [CSV 버전] 개별 모터 제어 클래스.

    shared_states 레이아웃 (슬레이브당 6슬롯):
      base+0 : statusword
      base+1 : is_moving
      base+2 : actual_position (pulse)
      base+3 : offset (pulse)
      base+4 : sync_error
      base+5 : actual_velocity (pulse/sec)
    """

    def __init__(self, slave_index: int, command_queue: mp.Queue,
                 shared_states: mp.Array, lock: mp.Lock):
        self._index  = slave_index
        self._q      = command_queue
        self._states = shared_states
        self._lock   = lock
        self._axis   = 'z'

    def set_axis(self, axis: str):
        if axis not in ('x', 'z'):
            raise ValueError("axis는 'x' 또는 'z'여야 합니다.")
        self._axis = axis
        self._q.put((self._index, 'SET_AXIS', axis))
        print(f"[설정] 모터 {self._index}: 축='{axis}'")

    def set_origin(self):
        self._q.put((self._index, 'SET_ORIGIN', None))
        print(f"[원점] 모터 {self._index}: 원점 설정")

    def set_profile_velocity(self, rpm: float):
        """이동 최대 속도 설정 (RPM)."""
        self._q.put((self._index, 'SET_VELOCITY', rpm))
        print(f"[속도] 모터 {self._index}: {rpm} RPM")

    def set_profile_accel_decel(self, accel_rpm_per_sec: float,
                                 decel_rpm_per_sec: float = None):
        """가감속 설정 (RPM/sec)."""
        acc = _rpm_per_sec_to_pps2(accel_rpm_per_sec)
        dec = acc if decel_rpm_per_sec is None else _rpm_per_sec_to_pps2(decel_rpm_per_sec)
        self._q.put((self._index, 'SET_ACCEL', (acc, dec)))
        print(f"[가감속] 모터 {self._index}: {accel_rpm_per_sec} RPM/s")

    def move_to_position_mm(self, target_mm: float):
        """목표 위치로 이동 (mm, 원점 기준)."""
        self._q.put((self._index, 'MOVE_TO_MM', target_mm))
        print(f"[이동] 모터 {self._index}: {target_mm:.2f}mm")

    # ── 상태 읽기 ──

    @property
    def status_word(self) -> int:
        return int(self._states[self._index * 6])

    def is_moving(self) -> bool:
        return self._states[self._index * 6 + 1] != 0

    @property
    def current_position_mm(self) -> float:
        base  = self._index * 6
        pos   = self._states[base + 2]
        offset= self._states[base + 3]
        return _pulse_to_mm(pos - offset, self._axis)

    @property
    def current_position_pulse(self) -> int:
        return int(self._states[self._index * 6 + 2])

    @property
    def offset_pulse(self) -> int:
        return int(self._states[self._index * 6 + 3])

    @property
    def current_velocity_mm_s(self) -> float:
        """실제 속도 (mm/sec, 드라이브 피드백)."""
        vel_pulse = self._states[self._index * 6 + 5]
        return _pulse_to_mm(vel_pulse, self._axis)

    @property
    def has_sync_error(self) -> bool:
        return self._states[self._index * 6 + 4] != 0
