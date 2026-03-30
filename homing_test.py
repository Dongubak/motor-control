"""
homing_test.py — Z축 Homing 테스트

[현재 테스트 환경 구성]
  슬레이브 0 (Z축): 하단 리미트 스위치(POT, DI1) 장착 → 하드웨어 Homing Mode(6)
  슬레이브 1 (Z축): 리미트 스위치 없음 → CSV Mode(9) 정지 대기 → 소프트웨어 원점 설정

[핵심 설계 — motor_csv.py 연속 PDO 루프 패턴 적용]
  - CYCLE_TIME_S = 10ms (motor_csv_main2.py 와 동일)
  - OP 진입 후 PDO 루프를 절대 중단하지 않음 → SM Watchdog(100ms) 방지
  - CiA 402 활성화·Homing 시작·완료 모두 PDO 루프 내 상태 머신으로 처리
  - SDO 작업은 OP 진입 전(Safe-OP 단계)에만 수행
  - 안전 확인 메시지도 OP 진입 전에 출력 → PDO 갭 없음

현장 테스트(3모터)는 homing_test_field.py 사용
참조: MotorControl_Doc/202603160900-homing-limit-switch.md
"""

import pysoem
import struct
import time

# ─────────────────────────────────────────────────────────────
# 설정값
# ─────────────────────────────────────────────────────────────
# ADAPTER = r'\Device\NPF_{CD2150F2-B355-4A6F-95BA-EB897A3726BF}'  # 현장
ADAPTER = r'\Device\NPF_{D1B66F5F-FF8A-4D4D-8C7C-2FF2547CE945}'    # ASIX USB Gigabit

NUM_SLAVES    = 2
HW_HOMING_IDX = 0   # 리미트 스위치 있음 → 하드웨어 Homing
SW_ORIGIN_IDX = 1   # 리미트 스위치 없음 → 소프트웨어 원점만

CYCLE_TIME_S  = 0.010   # 10 ms (motor_csv_main2.py 와 동일)

# ── Homing 속도 / 가속도 (슬레이브 0 전용) ────────────────────
# EFFECTIVE_PPR = 8,388,608 pulse/rev (= 2^23, 전기적 기어비 1:1)
EFFECTIVE_PPR    = 8_388_608

SEARCH_SPEED_RPM = 30           # 스위치 탐색 속도 (RPM) — 권장 10~30
ZERO_SPEED_RPM   = 1            # Index 탐색 속도 (RPM) — 권장 1~5
HOMING_ACCEL_UU  = 5_000_000   # 가감속 (UU/s²)

SEARCH_SPEED = int((SEARCH_SPEED_RPM / 60.0) * EFFECTIVE_PPR)
ZERO_SPEED   = int((ZERO_SPEED_RPM   / 60.0) * EFFECTIVE_PPR)

HOMING_METHOD   = 2    # Method 2: 정방향(CCW↓) + POT + Index 펄스
HOME_OFFSET     = 0    # 원점 오프셋 = 0
HOMING_DONE_BEH = 0    # 완료 후 동작: 이동 없음

HOMING_TIMEOUT_S = 120.0   # 하드웨어 Homing 최대 허용 시간 (초)

# ─────────────────────────────────────────────────────────────
# CiA 402 Controlword 상수
# ─────────────────────────────────────────────────────────────
CW_DISABLE_VOLTAGE  = 0x0000
CW_SHUTDOWN         = 0x0006
CW_SWITCH_ON        = 0x0007
CW_ENABLE_OPERATION = 0x000F
CW_HOMING_START     = 0x001F   # Enable Operation + bit4 ON → Homing 시작
CW_FAULT_RESET      = 0x0080

# ─────────────────────────────────────────────────────────────
# PDO 루프 상태 머신 상태
# ─────────────────────────────────────────────────────────────
_ST_WAIT_OP   = 'WAIT_OP'    # EtherCAT OP 상태 확인 대기
_ST_ENABLE    = 'ENABLE'     # CiA 402 활성화 (PDO 기반)
_ST_HOMING_B4 = 'HOMING_B4'  # bit4=0 유지 (rising edge 확보)
_ST_HOMING    = 'HOMING'     # bit4=1 Homing 진행
_ST_DONE      = 'DONE'
_ST_ERROR     = 'ERROR'


# ─────────────────────────────────────────────────────────────
# PDO 헬퍼
# Slave 0 (Homing PDO): RxPDO CW(2B) / TxPDO SW(2B)+Pos(4B)
# Slave 1 (CSV PDO)   : RxPDO CW(2B)+TargetVel(4B) / TxPDO SW(2B)+Pos(4B)+Vel(4B)
# ─────────────────────────────────────────────────────────────
def _read_sw(slave) -> int:
    return struct.unpack('<H', slave.input[0:2])[0]

def _read_pos(slave) -> int:
    return struct.unpack('<i', slave.input[2:6])[0]

def _write_cw(slave, cw: int):
    """RxPDO 크기에 따라 CW 만(2B) 또는 CW+vel(6B) 전송."""
    if len(slave.output) == 2:
        slave.output = struct.pack('<H', cw)
    else:
        slave.output = struct.pack('<H', cw) + struct.pack('<i', 0)


# ─────────────────────────────────────────────────────────────
# 초기화 헬퍼 — SDO 전용, OP 진입 전에만 사용
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
    """Homing 전용 PDO — 0x60FF(Target Velocity) 제외.

    일부 드라이브(iX7NH)는 Homing 모드에서도 RxPDO의 0x60FF를 속도 명령으로
    읽어 내부 Homing 속도(0x6099)를 0으로 덮어쓰므로 제외한다.

    RxPDO: CW 2B  /  TxPDO: SW+Pos 6B
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
    """CSV 모드 PDO (motor_csv._configure_csv_pdos 와 동일).
    RxPDO: CW+TargetVel 6B  /  TxPDO: SW+Pos+Vel 10B
    """
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
    print(f"    [PDO] CSV 구조 매핑 완료")


def _setup_homing_params(slave):
    slave.sdo_write(0x6098, 0, struct.pack('<b',  HOMING_METHOD))
    slave.sdo_write(0x6099, 1, struct.pack('<I',  SEARCH_SPEED))
    slave.sdo_write(0x6099, 2, struct.pack('<I',  ZERO_SPEED))
    slave.sdo_write(0x609A, 0, struct.pack('<I',  HOMING_ACCEL_UU))
    slave.sdo_write(0x607C, 0, struct.pack('<i',  HOME_OFFSET))
    slave.sdo_write(0x201E, 0, struct.pack('<H',  HOMING_DONE_BEH))
    print(f"    [Homing 파라미터] Method={HOMING_METHOD}, "
          f"탐색={SEARCH_SPEED_RPM} RPM, Zero={ZERO_SPEED_RPM} RPM")


# ─────────────────────────────────────────────────────────────
# 종료 / 진단
# ─────────────────────────────────────────────────────────────
def _graceful_shutdown(master, num_slaves: int):
    """CiA 402 단계적 종료: OP Enabled → Switch On → Ready → Disabled."""
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

    print("  [3] Shutdown (Ready to Switch On)...")
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


def _print_fault_codes(master, num_slaves: int):
    print("\n[진단] 드라이브 Fault 코드:")
    for i, slave in enumerate(master.slaves[:num_slaves]):
        try:
            code = struct.unpack('<H', slave.sdo_read(0x603F, 0))[0]
            print(f"  Slave {i}: 0x{code:04X}  {'(에러 없음)' if code == 0 else ''}")
        except Exception as e:
            print(f"  Slave {i}: 조회 실패 ({e})")


# ─────────────────────────────────────────────────────────────
# 메인
# ─────────────────────────────────────────────────────────────
def main():
    master = pysoem.Master()

    try:
        print("=" * 62)
        print("  Z축 Homing 테스트 [현재 테스트 환경 — 모터 2대]")
        print(f"  슬레이브 {HW_HOMING_IDX}: Z축, 리미트 스위치 O → 하드웨어 Homing")
        print(f"  슬레이브 {SW_ORIGIN_IDX}: Z축, 리미트 스위치 X → 소프트웨어 원점")
        print(f"  Method {HOMING_METHOD} / 탐색: {SEARCH_SPEED_RPM} RPM"
              f" / Zero: {ZERO_SPEED_RPM} RPM")
        print(f"  사이클: {int(CYCLE_TIME_S * 1000)}ms (연속 PDO 루프)")
        print("=" * 62)

        # ── 1단계: EtherCAT 초기화 ──
        print(f"\n[1단계] EtherCAT 초기화")
        master.open(ADAPTER)
        found = master.config_init()
        if found < NUM_SLAVES:
            raise RuntimeError(f"슬레이브 부족: 필요={NUM_SLAVES}, 발견={found}")
        print(f"  {found}개 슬레이브 발견")
        all_slaves = master.slaves[:NUM_SLAVES]

        # ── 2단계: SDO 설정 (OP 진입 전 — PDO 갭 없음) ──
        print(f"\n[2단계] 슬레이브 SDO 설정")
        for i, slave in enumerate(all_slaves):
            print(f"\n  Slave {i}: {slave.name}")
            slave.dc_sync(True, int(CYCLE_TIME_S * 1_000_000_000))
            _reset_fault(slave)
            if i == HW_HOMING_IDX:
                _configure_homing_pdos(slave)
                _setup_homing_params(slave)
                slave.sdo_write(0x6060, 0, struct.pack('<b', 6))
                time.sleep(0.02)
                mode_pre = struct.unpack('<b', slave.sdo_read(0x6061, 0))[0]
                print(f"    [모드] Homing(6) 설정 → 실제={mode_pre}")
            else:
                _configure_csv_pdos(slave)
                slave.sdo_write(0x6060, 0, struct.pack('<b', 9))
                time.sleep(0.02)
                mode_pre = struct.unpack('<b', slave.sdo_read(0x6061, 0))[0]
                print(f"    [모드] CSV(9) 설정 → 실제={mode_pre}  (정지 대기)")

        # ── 3단계: PDO 맵핑 + 모드 재확인 (Safe-OP, OP 진입 전) ──
        print(f"\n[3단계] PDO 맵핑")
        pdo_size = master.config_map()
        print(f"  PDO 맵핑: {pdo_size} bytes")

        # config_map() 후 모드 재설정 — 아직 Safe-OP이므로 PDO 갭 없음
        all_slaves[HW_HOMING_IDX].sdo_write(0x6060, 0, struct.pack('<b', 6))
        time.sleep(0.02)
        print(f"  Slave {HW_HOMING_IDX} Homing 모드(6) 재설정 완료")

        # ── 안전 확인 메시지 (OP 진입 전에 출력 — PDO 갭 없음) ──
        print(f"\n[안전 확인]")
        print(f"  [슬레이브 {HW_HOMING_IDX}] 하드웨어 Homing 수행")
        print(f"    - Z축 이동 경로 전 구간 장애물 없음 확인")
        print(f"    - 하단 리미트 스위치(POT, DI1, Pin 11) 배선 확인")
        print(f"      Pin 6 → GND(0V),  NO → 24V(+),  COM → Drive Pin 11")
        print(f"    - 상단 리미트 스위치(NOT, DI2, Pin 12) 배선 확인")
        print(f"  [슬레이브 {SW_ORIGIN_IDX}] 소프트웨어 원점 설정")
        print(f"    - Homing 시작 전 슬레이브 {HW_HOMING_IDX}과 동일한 물리적 높이에")
        print(f"      수동으로 위치시킬 것")
        print(f"    - Homing 중에는 이동하지 않음")

        # ── 4단계: OP 전환 + 연속 PDO 루프 시작 ──────────────────
        # OP 진입 후 PDO 루프를 절대 중단하지 않는다.
        # SM Watchdog(Lost Command Time=100ms)이 작동하지 않도록
        # 모든 상태 전환(CiA 402 활성화, Homing 시작/완료)을 루프 내
        # 상태 머신으로 처리한다.
        print(f"\n[4단계] OP 전환 + 연속 PDO 루프 (10ms 사이클)")
        master.state = pysoem.OP_STATE
        master.write_state()

        phase         = _ST_WAIT_OP
        t_phase       = time.monotonic()
        t_homing      = 0.0
        b4_cycles     = 0
        bit12_cleared = False   # 이번 세션에서 bit12가 한 번이라도 0이 됐는지 추적
        homing_ok     = False
        cycle_count   = 0

        while phase not in (_ST_DONE, _ST_ERROR):
            loop_start = time.monotonic()
            cycle_count += 1

            master.send_processdata()
            master.receive_processdata()
            now = time.monotonic()

            # ────────────────────────────────────────────────────
            # 상태 머신
            # ────────────────────────────────────────────────────
            if phase == _ST_WAIT_OP:
                # OP 상태 확인 — 주기적으로 read_state() 호출
                master.read_state()
                if all(s.state == pysoem.OP_STATE for s in all_slaves):
                    print(f"\n  OP 상태 확인 ({cycle_count}사이클). CiA 402 활성화 시작...")
                    phase   = _ST_ENABLE
                    t_phase = now
                elif now - t_phase > 5.0:
                    raise RuntimeError("OP 상태 도달 실패 (5초 타임아웃)")
                # OP 대기 중: 모든 슬레이브에 CW=0x0000 유지
                for slave in all_slaves:
                    _write_cw(slave, CW_DISABLE_VOLTAGE)

            elif phase == _ST_ENABLE:
                # CiA 402 상태 머신 — Statusword 기반 전환 (motor_csv.py 패턴)
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
                        _write_cw(slave, CW_ENABLE_OPERATION)       # OP Enabled 유지
                    else:
                        all_ok = False

                if all_ok:
                    pos0 = _read_pos(all_slaves[HW_HOMING_IDX])
                    pos1 = _read_pos(all_slaves[SW_ORIGIN_IDX])
                    print(f"  모든 드라이브 Operation Enabled")
                    print(f"  현재 위치: M{HW_HOMING_IDX}={pos0:+d} pulse, "
                          f"M{SW_ORIGIN_IDX}={pos1:+d} pulse")
                    print(f"  Homing bit4 rising edge 준비 (CW=0x000F, 3사이클)...")
                    phase     = _ST_HOMING_B4
                    b4_cycles = 0
                elif now - t_phase > 10.0:
                    raise RuntimeError("CiA 402 활성화 타임아웃 (10초)")

            elif phase == _ST_HOMING_B4:
                # Operation Enabled 상태에서 bit4=0 을 최소 3사이클 유지.
                # 다음 상태(_ST_HOMING)에서 bit4=1 로 전환 → rising edge 확보.
                for slave in all_slaves:
                    _write_cw(slave, CW_ENABLE_OPERATION)
                b4_cycles += 1
                if b4_cycles >= 3:
                    print(f"  Homing Start (CW=0x001F, bit4=1)")
                    t_homing      = now
                    bit12_cleared = False   # 새 호밍 세션 시작 전 리셋
                    phase         = _ST_HOMING

            elif phase == _ST_HOMING:
                sw     = _read_sw(all_slaves[HW_HOMING_IDX])
                pos    = _read_pos(all_slaves[HW_HOMING_IDX])
                pos_sw = _read_pos(all_slaves[SW_ORIGIN_IDX])
                bit12  = bool(sw & (1 << 12))   # Homing Attained
                bit13  = bool(sw & (1 << 13))   # Homing Error
                elapsed = now - t_homing

                # bit12 클리어 추적:
                # 이전 세션에서 bit12=1이 잔류한 경우 드라이브가 CW_HOMING_START를
                # 수신하면 bit12를 0으로 클리어하고 새 호밍을 시작한다.
                # bit12_cleared=True 가 되어야만 완료로 인정한다.
                if not bit12 and not bit13:
                    bit12_cleared = True

                # HW 슬레이브 Controlword
                if bit13:
                    _write_cw(all_slaves[HW_HOMING_IDX], CW_ENABLE_OPERATION)
                elif bit12 and bit12_cleared:
                    # 이번 세션의 실제 완료 — Homing Start 비트 해제
                    _write_cw(all_slaves[HW_HOMING_IDX], CW_ENABLE_OPERATION)
                else:
                    # 탐색 중이거나, bit12=1이지만 아직 잔류 플래그 (bit12_cleared=False)
                    # 두 경우 모두 CW_HOMING_START 유지
                    _write_cw(all_slaves[HW_HOMING_IDX], CW_HOMING_START)

                # SW 슬레이브: 현재 위치 유지
                for i, slave in enumerate(all_slaves):
                    if i != HW_HOMING_IDX:
                        _write_cw(slave, CW_ENABLE_OPERATION)

                # 상태 표시 (매 사이클)
                if bit12 and not bit12_cleared:
                    status = "잔류bit12"   # 이전 세션 잔류 — 실제 호밍 대기 중
                elif bit12:
                    status = "완료"
                elif bit13:
                    status = "에러"
                else:
                    status = "탐색중"
                print(f"\r  [{elapsed:5.1f}s]  "
                      f"M{HW_HOMING_IDX}:{status}(SW=0x{sw:04X}, pos={pos:+d})  "
                      f"M{SW_ORIGIN_IDX}:정지(pos={pos_sw:+d})",
                      end='', flush=True)

                if bit13:
                    print(f"\n[오류] Homing Error (Statusword=0x{sw:04X})")
                    homing_ok = False
                    phase     = _ST_ERROR
                elif bit12 and bit12_cleared:
                    print(f"\n[완료] Homing 완료! 경과={elapsed:.1f}s")
                    homing_ok = True
                    phase     = _ST_DONE
                elif elapsed > HOMING_TIMEOUT_S:
                    print(f"\n[타임아웃] {HOMING_TIMEOUT_S}s 초과")
                    homing_ok = False
                    phase     = _ST_ERROR

            # ── 사이클 타임 유지 (motor_csv.py 패턴) ──
            elapsed_loop = time.monotonic() - loop_start
            sleep_time   = CYCLE_TIME_S - elapsed_loop
            if sleep_time > 0:
                time.sleep(sleep_time)

        # ── 5단계: 결과 처리 ──
        print(f"\n[5단계] 결과 처리")
        if homing_ok:
            pos_hw = _read_pos(all_slaves[HW_HOMING_IDX])
            pos_sw = _read_pos(all_slaves[SW_ORIGIN_IDX])
            print(f"\n  Homing 완료 위치:")
            print(f"    M{HW_HOMING_IDX}(HW): {pos_hw:+d} pulse  (드라이브 원점 확정)")
            print(f"    M{SW_ORIGIN_IDX}(SW): {pos_sw:+d} pulse  (현재 위치 그대로)")
            print(f"\n  Homing 테스트 완료!")
            print(f"\n  [다음 단계 안내]")
            print(f"    EtherCATBusCSV.start() 후 각 모터에 set_origin() 호출:")
            print(f"      motor{HW_HOMING_IDX}.set_origin()  → Homing 완료 위치가 0mm")
            print(f"      motor{SW_ORIGIN_IDX}.set_origin()  → 현재 위치가 0mm (수동 정렬 전제)")
        else:
            print(f"\n  Homing 실패")
            print(f"\n  점검사항:")
            print(f"    1. DI1(POT) 배선 재확인 (Driver CM으로 DI1 ON/OFF 확인)")
            print(f"    2. Index 펄스는 내부 엔코더에서 자동 감지 — 배선 재확인 불필요")
            print(f"    3. SEARCH_SPEED_RPM 값을 낮춰 재시도")
            _print_fault_codes(master, NUM_SLAVES)

    except KeyboardInterrupt:
        print("\n\n[중단] 사용자 인터럽트")
    except Exception as e:
        print(f"\n[오류] {type(e).__name__}: {e}")
    finally:
        print(f"\n[종료] 단계적 정지 시퀀스...")
        try:
            _graceful_shutdown(master, NUM_SLAVES)
        except Exception:
            pass
        master.close()
        print("[종료] EtherCAT 닫힘")


if __name__ == '__main__':
    main()
