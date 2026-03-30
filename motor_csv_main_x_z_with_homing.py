"""
motor_csv_main_x_z_with_homing.py — X축 Homing 후 X/Z 순차 이동 및 원점 복귀

[슬레이브 구성]
  Slave 0: X축 — 하드웨어 Homing(Mode 6) + CSV 이동
  Slave 1: Z축 1 — CSV 이동만 (Homing 없음, Cross Coupling)
  Slave 2: Z축 2 — CSV 이동만 (Homing 없음, Cross Coupling)
  총 슬레이브: 3개

[동작 순서]
  1단계: X축 하드웨어 Homing (Method 2)
         정방향(+) → POT 리밋 스위치 → 방향 전환 → Index(Z) 펄스 원점 확정
         Z축(Slave 1,2)는 CSV 모드로 현재 위치 유지 (이동 없음)
  2단계: X축 → TARGET_MM_X 이동 (CSV 모드)
  3단계: Z축 → TARGET_MM_Z 이동 (CSV 모드, Cross Coupling)
  4단계: X축 원점 복귀 (0mm)
  5단계: Z축 원점 복귀 (0mm)

[핵심 설계]
  - 1단계: 직접 pysoem + 연속 PDO 루프 (SM Watchdog 방지)
    · Slave 0: Homing PDO (CW 2B / SW+Pos 6B), Mode 6
    · Slave 1,2: CSV PDO (CW+TargetVel 6B / SW+Pos+Vel 10B), Mode 9, 속도=0 대기
  - 1단계 완료 후 EtherCAT 종료 → 어댑터 해제 (1초 대기)
  - 2~5단계: EtherCATBusCSV 재연결 (3슬레이브, Mode 9)
  - bit12_cleared 추적으로 잔류 Homing 플래그 오탐 방지

참조:
  homing_test.py              — 혼합 PDO 루프 (HW Homing + CSV 대기) 패턴
  motor_csv_main_homing_and_move.py — Homing 후 CSV 이동 패턴
  motor_csv_main2.py          — Z축 Cross Coupling 이동 패턴
"""

import pysoem
import struct
import time

from motor_csv import EtherCATBusCSV


# ─────────────────────────────────────────────────────────────
# 공통 설정
# ─────────────────────────────────────────────────────────────
ADAPTER    = r'\Device\NPF_{CD2150F2-B355-4A6F-95BA-EB897A3726BF}'  # 현장
# ADAPTER = r'\Device\NPF_{D1B66F5F-FF8A-4D4D-8C7C-2FF2547CE945}'  # 사무실

NUM_SLAVES = 3   # X(0) + Z1(1) + Z2(2)
X_IDX      = 0   # X축 슬레이브
Z1_IDX     = 1   # Z축 슬레이브 1
Z2_IDX     = 2   # Z축 슬레이브 2

CYCLE_TIME_S = 0.010   # 10 ms

# ─────────────────────────────────────────────────────────────
# 1단계: X축 Homing 파라미터
# ─────────────────────────────────────────────────────────────
# EFFECTIVE_PPR = 8,388,608 pulse/rev (= 2^23, 전기적 기어비 1:1)
EFFECTIVE_PPR    = 8_388_608

SEARCH_SPEED_RPM = 30           # 스위치 탐색 속도 (RPM)
ZERO_SPEED_RPM   = 1            # Index 탐색 속도 (RPM)
HOMING_ACCEL_UU  = 5_000_000   # 가감속 (UU/s²)

SEARCH_SPEED = int((SEARCH_SPEED_RPM / 60.0) * EFFECTIVE_PPR)
ZERO_SPEED   = int((ZERO_SPEED_RPM   / 60.0) * EFFECTIVE_PPR)

HOMING_METHOD    = 2   # Method 2: 정방향(+) + POT + Index 펄스
HOME_OFFSET      = 0
HOMING_DONE_BEH  = 0   # 완료 후 이동 없음 (0x201E)

HOMING_TIMEOUT_S = 600.0

# ─────────────────────────────────────────────────────────────
# 2~5단계: CSV 이동 파라미터
# ─────────────────────────────────────────────────────────────

###### 현장 용 ######
RPM_X             = 500     # X축 이동 속도 (RPM)
ACCEL_RPM_PER_S_X = 500     # X축 가감속 (RPM/sec)
TARGET_MM_X       = -1600    # X축 이동 목표 (mm)

RPM_Z             = 500     # Z축 이동 속도 (RPM)
ACCEL_RPM_PER_S_Z = 500     # Z축 가감속 (RPM/sec)
TARGET_MM_Z       = -1200     # Z축 이동 목표 (mm)

###### 사무실 내 시연 용 ######
# RPM_X             = 10
# ACCEL_RPM_PER_S_X = 10
# TARGET_MM_X       = -100
# RPM_Z             = 10
# ACCEL_RPM_PER_S_Z = 10
# TARGET_MM_Z       = -10

# Z축 Cross Coupling 파라미터
COUPLING_GAIN   = 0.01   # [1/sec]
MAX_SYNC_ERR_MM = 10.0   # 긴급 정지 임계값 (mm)
MA_WINDOW       = 5      # 이동 평균 윈도우

# ─────────────────────────────────────────────────────────────
# CiA 402 Controlword 상수
# ─────────────────────────────────────────────────────────────
CW_DISABLE_VOLTAGE  = 0x0000
CW_SHUTDOWN         = 0x0006
CW_SWITCH_ON        = 0x0007
CW_ENABLE_OPERATION = 0x000F
CW_HOMING_START     = 0x001F
CW_FAULT_RESET      = 0x0080

# ─────────────────────────────────────────────────────────────
# Homing 상태 머신 상수
# ─────────────────────────────────────────────────────────────
_ST_WAIT_OP   = 'WAIT_OP'
_ST_ENABLE    = 'ENABLE'
_ST_HOMING_B4 = 'HOMING_B4'
_ST_HOMING    = 'HOMING'
_ST_DONE      = 'DONE'
_ST_ERROR     = 'ERROR'


# ─────────────────────────────────────────────────────────────
# PDO 헬퍼
#   Slave 0 (Homing): RxPDO CW(2B)            / TxPDO SW(2B)+Pos(4B)
#   Slave 1,2 (CSV) : RxPDO CW(2B)+Vel(4B)   / TxPDO SW(2B)+Pos(4B)+Vel(4B)
# ─────────────────────────────────────────────────────────────
def _read_sw(slave) -> int:
    return struct.unpack('<H', slave.input[0:2])[0]

def _read_pos(slave) -> int:
    return struct.unpack('<i', slave.input[2:6])[0]

def _write_cw(slave, cw: int):
    """RxPDO 크기에 따라 CW만(2B) 또는 CW+vel(6B) 전송."""
    if len(slave.output) == 2:
        slave.output = struct.pack('<H', cw)
    else:
        slave.output = struct.pack('<H', cw) + struct.pack('<i', 0)


# ─────────────────────────────────────────────────────────────
# SDO 초기화 헬퍼 (OP 진입 전 전용)
# ─────────────────────────────────────────────────────────────
def _reset_fault(slave):
    try:
        sw = struct.unpack('<H', slave.sdo_read(0x6041, 0))[0]
        if sw & 0x0008:
            print(f"    [Fault] 감지 (0x{sw:04X}) → 리셋 중...")
            slave.sdo_write(0x6040, 0, struct.pack('<H', CW_FAULT_RESET))
            time.sleep(0.3)
    except Exception as e:
        print(f"    [경고] Fault 리셋 실패: {e}")


def _configure_homing_pdos(slave):
    """Homing 전용 PDO: RxPDO CW 2B / TxPDO SW+Pos 6B
    0x60FF(Target Velocity) 제외 — iX7NH가 Homing 속도를 덮어쓰는 문제 방지.
    """
    slave.sdo_write(0x1C12, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1600, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1600, 1, struct.pack('<I', 0x60400010)); time.sleep(0.01)
    slave.sdo_write(0x1600, 0, b'\x01'); time.sleep(0.01)
    slave.sdo_write(0x1C12, 1, struct.pack('<H', 0x1600)); time.sleep(0.01)
    slave.sdo_write(0x1C12, 0, b'\x01'); time.sleep(0.01)

    slave.sdo_write(0x1C13, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1A00, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1A00, 1, struct.pack('<I', 0x60410010)); time.sleep(0.01)
    slave.sdo_write(0x1A00, 2, struct.pack('<I', 0x60640020)); time.sleep(0.01)
    slave.sdo_write(0x1A00, 0, b'\x02'); time.sleep(0.01)
    slave.sdo_write(0x1C13, 1, struct.pack('<H', 0x1A00)); time.sleep(0.01)
    slave.sdo_write(0x1C13, 0, b'\x01'); time.sleep(0.01)
    print(f"    [PDO] Homing 전용 매핑 완료 (RxPDO: CW 2B, TxPDO: SW+Pos 6B)")


def _configure_csv_pdos(slave):
    """CSV PDO: RxPDO CW+TargetVel 6B / TxPDO SW+Pos+Vel 10B"""
    slave.sdo_write(0x1C12, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1600, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1600, 1, struct.pack('<I', 0x60400010)); time.sleep(0.01)
    slave.sdo_write(0x1600, 2, struct.pack('<I', 0x60FF0020)); time.sleep(0.01)
    slave.sdo_write(0x1600, 0, b'\x02'); time.sleep(0.01)
    slave.sdo_write(0x1C12, 1, struct.pack('<H', 0x1600)); time.sleep(0.01)
    slave.sdo_write(0x1C12, 0, b'\x01'); time.sleep(0.01)

    slave.sdo_write(0x1C13, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1A00, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1A00, 1, struct.pack('<I', 0x60410010)); time.sleep(0.01)
    slave.sdo_write(0x1A00, 2, struct.pack('<I', 0x60640020)); time.sleep(0.01)
    slave.sdo_write(0x1A00, 3, struct.pack('<I', 0x606C0020)); time.sleep(0.01)
    slave.sdo_write(0x1A00, 0, b'\x03'); time.sleep(0.01)
    slave.sdo_write(0x1C13, 1, struct.pack('<H', 0x1A00)); time.sleep(0.01)
    slave.sdo_write(0x1C13, 0, b'\x01'); time.sleep(0.01)
    print(f"    [PDO] CSV 매핑 완료 (RxPDO: CW+Vel 6B, TxPDO: SW+Pos+Vel 10B)")


def _setup_homing_params(slave):
    slave.sdo_write(0x6098, 0, struct.pack('<b', HOMING_METHOD))
    slave.sdo_write(0x6099, 1, struct.pack('<I', SEARCH_SPEED))
    slave.sdo_write(0x6099, 2, struct.pack('<I', ZERO_SPEED))
    slave.sdo_write(0x609A, 0, struct.pack('<I', HOMING_ACCEL_UU))
    slave.sdo_write(0x607C, 0, struct.pack('<i', HOME_OFFSET))
    slave.sdo_write(0x201E, 0, struct.pack('<H', HOMING_DONE_BEH))
    print(f"    [Homing 파라미터] Method={HOMING_METHOD}, "
          f"탐색={SEARCH_SPEED_RPM} RPM, Zero={ZERO_SPEED_RPM} RPM")


# ─────────────────────────────────────────────────────────────
# Homing 단계 종료 시퀀스
# ─────────────────────────────────────────────────────────────
def _graceful_shutdown(master, num_slaves: int):
    """CiA 402 단계적 종료."""
    print("  [1] 속도=0 유지...")
    for slave in master.slaves[:num_slaves]:
        try: _write_cw(slave, CW_ENABLE_OPERATION)
        except: pass
    for _ in range(5):
        try:
            master.send_processdata()
            master.receive_processdata()
        except: pass
        time.sleep(0.02)

    print("  [2] Switch On...")
    for slave in master.slaves[:num_slaves]:
        try: _write_cw(slave, CW_SWITCH_ON)
        except: pass
    for _ in range(3):
        try:
            master.send_processdata()
            master.receive_processdata()
        except: pass
        time.sleep(0.03)

    print("  [3] Shutdown...")
    for slave in master.slaves[:num_slaves]:
        try: _write_cw(slave, CW_SHUTDOWN)
        except: pass
    for _ in range(3):
        try:
            master.send_processdata()
            master.receive_processdata()
        except: pass
        time.sleep(0.03)

    print("  [4] Disable Voltage...")
    for slave in master.slaves[:num_slaves]:
        try: _write_cw(slave, CW_DISABLE_VOLTAGE)
        except: pass
    for _ in range(3):
        try:
            master.send_processdata()
            master.receive_processdata()
        except: pass
        time.sleep(0.03)

    master.state = pysoem.INIT_STATE
    master.write_state()
    time.sleep(0.1)


# ─────────────────────────────────────────────────────────────
# CSV 이동 대기 헬퍼
# ─────────────────────────────────────────────────────────────
def wait_motor(motor, label: str, timeout: float = 120.0) -> bool:
    """단일 모터 이동 완료 대기. 오류 시 True 반환."""
    t0 = time.monotonic()
    while motor.is_moving():
        p = motor.current_position_mm
        v = motor.current_velocity_mm_s
        print(f"\r  [{label}]  pos={p:8.3f}mm  ({v:+6.3f}mm/s)", end='')
        if motor.has_sync_error:
            print(f"\n[긴급 정지] 모터 이상 감지 (Fault)!")
            return True
        if time.monotonic() - t0 > timeout:
            print("\n[타임아웃]")
            return False
        time.sleep(0.05)
    print()
    if motor.has_sync_error:
        return True
    return False


def wait_motors(motor1, motor2, label: str, timeout: float = 120.0):
    """두 모터 이동 완료 대기 (위치 차이 표시). (max_diff_mm, 오류여부) 반환."""
    max_diff = 0.0
    t0 = time.monotonic()
    while motor1.is_moving() or motor2.is_moving():
        p1   = motor1.current_position_mm
        p2   = motor2.current_position_mm
        v1   = motor1.current_velocity_mm_s
        v2   = motor2.current_velocity_mm_s
        diff = abs(p1 - p2)
        max_diff = max(max_diff, diff)
        print(f"\r  [{label}]  "
              f"Z1={p1:7.2f}mm ({v1:+5.2f}mm/s)  "
              f"Z2={p2:7.2f}mm ({v2:+5.2f}mm/s)  "
              f"차이={diff:.3f}mm", end='')
        if motor1.has_sync_error or motor2.has_sync_error:
            print(f"\n[긴급 정지] 모터 이상 감지 (Fault 또는 동기화 오류)!")
            return max_diff, True
        if time.monotonic() - t0 > timeout:
            print("\n[타임아웃]")
            return max_diff, False
        time.sleep(0.05)
    print()
    if motor1.has_sync_error or motor2.has_sync_error:
        return max_diff, True
    return max_diff, False


# ─────────────────────────────────────────────────────────────
# Fault 코드 조회 (버스 종료 후 SDO)
# ─────────────────────────────────────────────────────────────
def read_drive_fault_codes(adapter: str, num_slaves: int):
    master = pysoem.Master()
    try:
        master.open(adapter)
        found = master.config_init()
        if found == 0:
            print("[Fault 조회] 슬레이브 없음")
            return
        print("\n" + "=" * 60)
        print("  [드라이브 Fault 코드 조회]")
        print("=" * 60)
        for i, slave in enumerate(master.slaves[:num_slaves]):
            print(f"\n  Slave {i} ({slave.name}):")
            try:
                code = struct.unpack("<H", slave.sdo_read(0x603F, 0))[0]
                print(f"    현재 Error Code (0x603F) : 0x{code:04X}"
                      f"  {'(에러 없음)' if code == 0 else ''}")
            except Exception as e:
                print(f"    0x603F 읽기 실패: {e}")
        print("=" * 60)
    except Exception as e:
        print(f"[Fault 조회 실패] {e}")
    finally:
        master.close()


# ─────────────────────────────────────────────────────────────
# 1단계: X축 하드웨어 Homing
# ─────────────────────────────────────────────────────────────
def run_homing() -> bool:
    """
    X축(Slave 0) 하드웨어 Homing 수행.
    Z축(Slave 1,2)은 CSV 모드로 현재 위치 유지.
    성공하면 True, 실패하면 False 반환.
    """
    master = pysoem.Master()
    homing_ok = False

    try:
        print(f"\n[1단계] EtherCAT 초기화 (Homing)")
        master.open(ADAPTER)
        found = master.config_init()
        if found < NUM_SLAVES:
            raise RuntimeError(f"슬레이브 부족: 필요={NUM_SLAVES}, 발견={found}")
        print(f"  {found}개 슬레이브 발견")
        all_slaves = master.slaves[:NUM_SLAVES]

        # ── SDO 설정 (OP 진입 전) ──
        print(f"\n[2단계] 슬레이브 SDO 설정")
        for i, slave in enumerate(all_slaves):
            print(f"\n  Slave {i}: {slave.name}")
            slave.dc_sync(True, int(CYCLE_TIME_S * 1_000_000_000))
            _reset_fault(slave)
            if i == X_IDX:
                _configure_homing_pdos(slave)
                _setup_homing_params(slave)
                slave.sdo_write(0x6060, 0, struct.pack('<b', 6))
                time.sleep(0.02)
                mode = struct.unpack('<b', slave.sdo_read(0x6061, 0))[0]
                print(f"    [모드] Homing(6) 설정 → 실제={mode}")
            else:
                _configure_csv_pdos(slave)
                slave.sdo_write(0x6060, 0, struct.pack('<b', 9))
                time.sleep(0.02)
                mode = struct.unpack('<b', slave.sdo_read(0x6061, 0))[0]
                print(f"    [모드] CSV(9) 설정 → 실제={mode}  (Homing 중 정지 대기)")

        # ── PDO 맵핑 + 모드 재설정 ──
        print(f"\n[3단계] PDO 맵핑")
        pdo_size = master.config_map()
        print(f"  PDO 맵핑: {pdo_size} bytes")
        all_slaves[X_IDX].sdo_write(0x6060, 0, struct.pack('<b', 6))
        time.sleep(0.02)
        print(f"  Slave {X_IDX} Homing 모드(6) 재설정 완료")

        print(f"\n[안전 확인]")
        print(f"  - X축(Slave {X_IDX}) 이동 경로 장애물 없음 확인")
        print(f"  - 정방향(+) 리밋 스위치(POT, DI1) 배선 확인")
        print(f"    Pin 6 → GND(0V),  NO → 24V(+),  COM → Drive Pin 11")
        print(f"  - Z축(Slave {Z1_IDX},{Z2_IDX}) Homing 중 이동하지 않음")

        # ── OP 전환 + 연속 PDO 루프 ──
        print(f"\n[4단계] OP 전환 + 연속 PDO 루프 (X축 Homing)")
        master.state = pysoem.OP_STATE
        master.write_state()

        phase         = _ST_WAIT_OP
        t_phase       = time.monotonic()
        t_homing      = 0.0
        b4_cycles     = 0
        bit12_cleared = False
        cycle_count   = 0

        while phase not in (_ST_DONE, _ST_ERROR):
            loop_start = time.monotonic()
            cycle_count += 1

            master.send_processdata()
            master.receive_processdata()
            now = time.monotonic()

            if phase == _ST_WAIT_OP:
                master.read_state()
                if all(s.state == pysoem.OP_STATE for s in all_slaves):
                    print(f"\n  OP 상태 확인 ({cycle_count}사이클). CiA 402 활성화 시작...")
                    phase   = _ST_ENABLE
                    t_phase = now
                elif now - t_phase > 5.0:
                    raise RuntimeError("OP 상태 도달 실패 (5초 타임아웃)")
                for slave in all_slaves:
                    _write_cw(slave, CW_DISABLE_VOLTAGE)

            elif phase == _ST_ENABLE:
                all_ok = True
                for slave in all_slaves:
                    sw = _read_sw(slave)
                    if sw & 0x0008:
                        _write_cw(slave, CW_FAULT_RESET);          all_ok = False
                    elif (sw & 0x004F) == 0x0040:
                        _write_cw(slave, CW_SHUTDOWN);              all_ok = False
                    elif (sw & 0x006F) == 0x0021:
                        _write_cw(slave, CW_SWITCH_ON);             all_ok = False
                    elif (sw & 0x006F) == 0x0023:
                        _write_cw(slave, CW_ENABLE_OPERATION);      all_ok = False
                    elif (sw & 0x006F) == 0x0027:
                        _write_cw(slave, CW_ENABLE_OPERATION)
                    else:
                        all_ok = False

                if all_ok:
                    pos_x  = _read_pos(all_slaves[X_IDX])
                    pos_z1 = _read_pos(all_slaves[Z1_IDX])
                    pos_z2 = _read_pos(all_slaves[Z2_IDX])
                    print(f"  모든 드라이브 Operation Enabled")
                    print(f"  현재 위치: X={pos_x:+d}, Z1={pos_z1:+d}, Z2={pos_z2:+d} pulse")
                    print(f"  Homing bit4 rising edge 준비 (3사이클)...")
                    phase     = _ST_HOMING_B4
                    b4_cycles = 0
                elif now - t_phase > 10.0:
                    raise RuntimeError("CiA 402 활성화 타임아웃 (10초)")

            elif phase == _ST_HOMING_B4:
                for slave in all_slaves:
                    _write_cw(slave, CW_ENABLE_OPERATION)
                b4_cycles += 1
                if b4_cycles >= 3:
                    print(f"  Homing Start (bit4=1)")
                    t_homing      = now
                    bit12_cleared = False
                    phase         = _ST_HOMING

            elif phase == _ST_HOMING:
                sw_x    = _read_sw(all_slaves[X_IDX])
                pos_x   = _read_pos(all_slaves[X_IDX])
                pos_z1  = _read_pos(all_slaves[Z1_IDX])
                pos_z2  = _read_pos(all_slaves[Z2_IDX])
                bit12   = bool(sw_x & (1 << 12))
                bit13   = bool(sw_x & (1 << 13))
                elapsed = now - t_homing

                if not bit12 and not bit13:
                    bit12_cleared = True

                # X축 Controlword
                if bit13:
                    _write_cw(all_slaves[X_IDX], CW_ENABLE_OPERATION)
                elif bit12 and bit12_cleared:
                    _write_cw(all_slaves[X_IDX], CW_ENABLE_OPERATION)
                else:
                    _write_cw(all_slaves[X_IDX], CW_HOMING_START)

                # Z축: 현재 위치 유지
                _write_cw(all_slaves[Z1_IDX], CW_ENABLE_OPERATION)
                _write_cw(all_slaves[Z2_IDX], CW_ENABLE_OPERATION)

                if bit12 and not bit12_cleared:
                    status = "잔류bit12"
                elif bit12:
                    status = "완료"
                elif bit13:
                    status = "에러"
                else:
                    status = "탐색중"
                print(f"\r  [Homing {elapsed:5.1f}s]  "
                      f"X:{status}(SW=0x{sw_x:04X}, pos={pos_x:+d})  "
                      f"Z1={pos_z1:+d}  Z2={pos_z2:+d}",
                      end='', flush=True)

                if bit13:
                    print(f"\n[오류] Homing Error (SW=0x{sw_x:04X})")
                    phase = _ST_ERROR
                elif bit12 and bit12_cleared:
                    print(f"\n  [Homing 완료] 경과={elapsed:.1f}s, X pos={pos_x:+d} pulse")
                    homing_ok = True
                    phase     = _ST_DONE
                elif elapsed > HOMING_TIMEOUT_S:
                    print(f"\n[타임아웃] {HOMING_TIMEOUT_S}s 초과")
                    phase = _ST_ERROR

            elapsed_loop = time.monotonic() - loop_start
            sleep_time   = CYCLE_TIME_S - elapsed_loop
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[중단] 사용자 인터럽트 (Homing)")
        raise
    except Exception as e:
        print(f"\n[오류] {type(e).__name__}: {e}")
    finally:
        print(f"\n[Homing 종료] 단계적 정지...")
        try:
            _graceful_shutdown(master, NUM_SLAVES)
        except Exception:
            pass
        master.close()
        print("[Homing 종료] EtherCAT 닫힘")

    return homing_ok


# ─────────────────────────────────────────────────────────────
# 2~5단계: CSV 모드 순차 이동
# ─────────────────────────────────────────────────────────────
def run_csv_sequence():
    """X/Z 순차 이동 및 원점 복귀 (CSV 모드)."""
    bus = EtherCATBusCSV(
        adapter_name=ADAPTER,
        num_slaves=NUM_SLAVES,
        cycle_time_ms=10,
        max_sync_error_mm=MAX_SYNC_ERR_MM,
        coupling_gain=COUPLING_GAIN,
        enable_coupling=True,   # Z축 이동 시 Cross Coupling 적용
        ma_window=MA_WINDOW,
    )

    motor_x  = bus.motors[X_IDX]
    motor_z1 = bus.motors[Z1_IDX]
    motor_z2 = bus.motors[Z2_IDX]

    try:
        # ── 축 설정 ──
        motor_x.set_axis('x')
        motor_z1.set_axis('z')
        motor_z2.set_axis('z')

        # ── 속도/가감속 설정 ──
        motor_x.set_profile_velocity(rpm=RPM_X)
        motor_x.set_profile_accel_decel(accel_rpm_per_sec=ACCEL_RPM_PER_S_X)
        motor_z1.set_profile_velocity(rpm=RPM_Z)
        motor_z1.set_profile_accel_decel(accel_rpm_per_sec=ACCEL_RPM_PER_S_Z)
        motor_z2.set_profile_velocity(rpm=RPM_Z)
        motor_z2.set_profile_accel_decel(accel_rpm_per_sec=ACCEL_RPM_PER_S_Z)

        # ── 버스 시작 ──
        bus.start()

        # ── 모든 모터 준비 대기 ──
        for motor, name in ((motor_x, 'X'), (motor_z1, 'Z1'), (motor_z2, 'Z2')):
            print(f"모터 {motor._index} ({name}축) 준비 대기 중...")
            t0 = time.monotonic()
            while (motor.status_word & 0x006F) != 0x0027:
                time.sleep(0.05)
                if time.monotonic() - t0 > 5:
                    raise RuntimeError(f"모터 {motor._index} 준비 타임아웃!")
            print(f"[완료] 모터 {motor._index} ({name}축) 준비 완료")

        # ── 원점 설정 ──
        motor_x.set_origin()
        motor_z1.set_origin()
        motor_z2.set_origin()
        time.sleep(0.5)
        print(f"\n원점 설정 후:")
        print(f"  X={motor_x.current_position_mm:.3f}mm, "
              f"Z1={motor_z1.current_position_mm:.3f}mm, "
              f"Z2={motor_z2.current_position_mm:.3f}mm")

        # ════════════════════════════════════════════════════
        # 2단계: X축 이동
        # ════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print(f"  [2단계] X축 이동")
        print(f"  목표: {TARGET_MM_X}mm  /  {RPM_X} RPM  /  가감속 {ACCEL_RPM_PER_S_X} RPM/s")
        print("=" * 60)

        motor_x.move_to_position_mm(TARGET_MM_X)
        time.sleep(0.2)

        err = wait_motor(motor_x, "X 이동", timeout=120.0)
        print(f"\n[2단계 결과] X={motor_x.current_position_mm:.3f}mm")
        if err:
            print("  [경고] X축 이상 발생으로 중단")
            return

        time.sleep(1.0)

        # ════════════════════════════════════════════════════
        # 3단계: Z축 이동 (Cross Coupling)
        # ════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print(f"  [3단계] Z축 이동 (Cross Coupling ON, Kc={COUPLING_GAIN})")
        print(f"  목표: {TARGET_MM_Z}mm  /  {RPM_Z} RPM  /  가감속 {ACCEL_RPM_PER_S_Z} RPM/s")
        print("=" * 60)

        bus.coupling_enabled = True
        motor_z1.move_to_position_mm(TARGET_MM_Z)
        motor_z2.move_to_position_mm(TARGET_MM_Z)
        time.sleep(0.2)

        max_diff_z, err_z = wait_motors(motor_z1, motor_z2, "Z 이동", timeout=120.0)
        print(f"\n[3단계 결과]")
        print(f"  최대 위치 차이: {max_diff_z:.3f}mm")
        print(f"  Z1={motor_z1.current_position_mm:.3f}mm, "
              f"Z2={motor_z2.current_position_mm:.3f}mm")
        if err_z:
            print("  [경고] Z축 이상 발생으로 중단")
            return

        time.sleep(1.0)

        # ════════════════════════════════════════════════════
        # 4단계: X축 원점 복귀
        # ════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print(f"  [4단계] X축 원점 복귀 (0mm)")
        print("=" * 60)

        motor_x.move_to_position_mm(0)
        time.sleep(0.2)

        err = wait_motor(motor_x, "X 복귀", timeout=120.0)
        print(f"\n[4단계 결과] X={motor_x.current_position_mm:.3f}mm")
        if err:
            print("  [경고] X축 복귀 중 이상 발생")

        time.sleep(1.0)

        # ════════════════════════════════════════════════════
        # 5단계: Z축 원점 복귀 (Cross Coupling)
        # ════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print(f"  [5단계] Z축 원점 복귀 (0mm, Cross Coupling ON)")
        print("=" * 60)

        bus.coupling_enabled = True
        motor_z1.move_to_position_mm(0)
        motor_z2.move_to_position_mm(0)
        time.sleep(0.2)

        max_diff_ret, _ = wait_motors(motor_z1, motor_z2, "Z 복귀", timeout=120.0)
        print(f"\n[5단계 결과]")
        print(f"  최대 위치 차이: {max_diff_ret:.3f}mm")
        print(f"  Z1={motor_z1.current_position_mm:.3f}mm, "
              f"Z2={motor_z2.current_position_mm:.3f}mm")

        time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n\n사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"\n\n에러 발생: {e}")
    finally:
        bus.stop()
        time.sleep(0.5)
        read_drive_fault_codes(ADAPTER, NUM_SLAVES)


# ─────────────────────────────────────────────────────────────
# 메인
# ─────────────────────────────────────────────────────────────
def main():
    print("=" * 62)
    print("  X축 Homing 후 X/Z 순차 이동 및 원점 복귀")
    print(f"  Slave 0: X축 (Homing + 이동)")
    print(f"  Slave 1,2: Z축 (이동만, Cross Coupling)")
    print(f"  [Homing] Method {HOMING_METHOD}, 탐색={SEARCH_SPEED_RPM} RPM")
    print(f"  [X 이동] 목표={TARGET_MM_X}mm, {RPM_X} RPM")
    print(f"  [Z 이동] 목표={TARGET_MM_Z}mm, {RPM_Z} RPM")
    print("=" * 62)

    try:
        ok = run_homing()
    except KeyboardInterrupt:
        print("\n종료합니다.")
        return

    if not ok:
        print("\nHoming 실패. 이동을 중단합니다.")
        return

    print(f"\nHoming 완료. 1초 대기 후 CSV 이동 시작...")
    time.sleep(1.0)

    run_csv_sequence()


if __name__ == '__main__':
    main()
