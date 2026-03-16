"""
homing_test_field.py — 현장 테스트 환경용 Z축 Homing 테스트

[현장 테스트 환경 구성]
  슬레이브 0 (X축): CSV Mode(9) 유지, Homing 없음
  슬레이브 1 (Z축): 하단 리미트 스위치(POT, DI1) 장착 → 하드웨어 Homing Mode(6)
  슬레이브 2 (Z축): 하단 리미트 스위치(POT, DI1) 장착 → 하드웨어 Homing Mode(6)

동작 방식:
  - 슬레이브 1, 2 (Z축) 두 모터를 동시에 하드웨어 Homing 수행
  - 슬레이브 0 (X축) 은 Homing 중 현재 위치 유지
  - 두 Z축 모터 모두 bit12(Homing Attained)가 SET 되면 완료

현재 테스트 환경(2모터)은 homing_test.py 사용
참조: MotorControl_Doc/202603160900-homing-limit-switch.md
"""

import pysoem
import struct
import time

# ─────────────────────────────────────────────────────────────
# 설정값
# ─────────────────────────────────────────────────────────────
ADAPTER     = r'\Device\NPF_{CD2150F2-B355-4A6F-95BA-EB897A3726BF}'
NUM_SLAVES  = 3          # 전체 슬레이브 수 (현장: 3개)

# 슬레이브 역할
X_IDX        = 0         # X축 — Homing 없음, CSV Mode 유지
Z_HW_IDX     = [1, 2]   # Z축 — 리미트 스위치 있음, 하드웨어 Homing

CYCLE_TIME_S = 0.004     # EtherCAT 사이클 (4 ms)

# ── Homing 속도 / 가속도 (Z축 슬레이브 공통) ─────────────────
# EFFECTIVE_PPR = 16,777,216 pulse/rev  (= PULSES_PER_REVOLUTION × 2)
# 1 RPM = 16,777,216 / 60 ≈ 279,620 UU/s
EFFECTIVE_PPR    = 16_777_216

SEARCH_SPEED_RPM = 10      # 스위치 탐색 속도 (RPM) — 권장 10~30
ZERO_SPEED_RPM   = 1       # Zero(Index 펄스) 탐색 속도 (RPM) — 권장 1~5
HOMING_ACCEL_UU  = 5_000_000   # 가감속 (UU/s²)

SEARCH_SPEED = int((SEARCH_SPEED_RPM / 60.0) * EFFECTIVE_PPR)  # ≈ 2,796,202
ZERO_SPEED   = int((ZERO_SPEED_RPM   / 60.0) * EFFECTIVE_PPR)  # ≈   279,620

HOMING_METHOD   = 2    # Method 2: 정방향(CCW↓) + POT + Index 펄스
HOME_OFFSET     = 0    # 원점 오프셋 = 0
HOMING_DONE_BEH = 0    # 완료 후 동작: 0 = 이동 없음 (0x201E)

HOMING_TIMEOUT_S = 120.0   # Homing 최대 허용 시간 (초)

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
# PDO 헬퍼 (CSV PDO 구조)
# TxPDO: Statusword(2B) + Actual Pos(4B) + Actual Vel(4B)
# RxPDO: Controlword(2B) + Target Velocity(4B, Homing 모드에서 무시)
# ─────────────────────────────────────────────────────────────
def _read_sw(slave) -> int:
    return struct.unpack('<H', slave.input[0:2])[0]

def _read_pos(slave) -> int:
    return struct.unpack('<i', slave.input[2:6])[0]

def _write_cw(slave, cw: int):
    slave.output = struct.pack('<H', cw) + struct.pack('<i', 0)


# ─────────────────────────────────────────────────────────────
# 초기화 헬퍼
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


def _configure_pdos(slave):
    """CSV 구조 PDO 매핑 (motor_csv.py의 _configure_csv_pdos 와 동일)."""
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


def _set_mode(slave, mode: int):
    slave.sdo_write(0x211F, 0, struct.pack('<H', 1 << 12))
    time.sleep(0.01)
    slave.sdo_write(0x6060, 0, struct.pack('<b', mode))
    time.sleep(0.02)


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
# CiA 402 상태 전환
# ─────────────────────────────────────────────────────────────
def _cia402_enable_all(master, all_slaves, cycle_time_s: float, timeout_s: float = 8.0) -> bool:
    print("[CiA 402] 드라이브 활성화 중...")
    t0 = time.monotonic()

    while time.monotonic() - t0 < timeout_s:
        master.send_processdata()
        master.receive_processdata()

        all_enabled = True
        for slave in all_slaves:
            sw = _read_sw(slave)
            if sw & 0x0008:
                _write_cw(slave, CW_FAULT_RESET);      all_enabled = False
            elif (sw & 0x004F) == 0x0040:
                _write_cw(slave, CW_SHUTDOWN);          all_enabled = False
            elif (sw & 0x006F) == 0x0021:
                _write_cw(slave, CW_SWITCH_ON);         all_enabled = False
            elif (sw & 0x006F) == 0x0023:
                _write_cw(slave, CW_ENABLE_OPERATION);  all_enabled = False
            elif (sw & 0x006F) == 0x0027:
                _write_cw(slave, CW_ENABLE_OPERATION)
            else:
                all_enabled = False

        if all_enabled:
            print("  → 모든 드라이브 Operation Enabled")
            return True
        time.sleep(cycle_time_s)

    print("  [오류] Operation Enabled 타임아웃")
    return False


# ─────────────────────────────────────────────────────────────
# Homing 실행 (Z축 두 모터 동시)
# ─────────────────────────────────────────────────────────────
def _run_homing_dual(master, all_slaves, z_indices: list,
                     cycle_time_s: float, timeout_s: float) -> bool:
    """
    z_indices 에 해당하는 슬레이브들을 동시에 하드웨어 Homing 수행.
    나머지 슬레이브는 Operation Enabled 상태로 현재 위치 유지.

    완료 조건 : 모든 대상 슬레이브의 Statusword bit12 = 1 (Homing Attained)
    실패 조건 : 임의 슬레이브 Statusword bit13 = 1 (Homing Error) 또는 타임아웃
    """
    z_set      = set(z_indices)
    z_slaves   = [all_slaves[i] for i in z_indices]
    attained   = [False] * len(z_indices)

    print(f"\n[Homing] 슬레이브 {z_indices} 동시 시작 (bit4=1)...")

    # 첫 프레임: Z축 → Homing Start, X축 → Enable Operation
    for i, slave in enumerate(all_slaves):
        _write_cw(slave, CW_HOMING_START if i in z_set else CW_ENABLE_OPERATION)
    master.send_processdata()
    master.receive_processdata()

    t0 = time.monotonic()

    while True:
        master.send_processdata()
        master.receive_processdata()
        elapsed = time.monotonic() - t0

        status_parts = []
        any_error = False

        for k, slave in enumerate(z_slaves):
            sw    = _read_sw(slave)
            pos   = _read_pos(slave)
            bit12 = bool(sw & (1 << 12))   # Homing Attained
            bit13 = bool(sw & (1 << 13))   # Homing Error

            if bit13:
                print(f"\n[오류] 슬레이브 {z_indices[k]}: Homing Error!"
                      f" Statusword=0x{sw:04X}")
                _write_cw(slave, CW_ENABLE_OPERATION)
                any_error = True
            elif bit12:
                attained[k] = True
                _write_cw(slave, CW_ENABLE_OPERATION)
                status_parts.append(f"M{z_indices[k]}:완료(pos={pos:+d})")
            else:
                _write_cw(slave, CW_HOMING_START)
                status_parts.append(f"M{z_indices[k]}:탐색중(pos={pos:+d})")

        # X축 슬레이브 유지
        for i, slave in enumerate(all_slaves):
            if i not in z_set:
                _write_cw(slave, CW_ENABLE_OPERATION)

        print(f"\r  [{elapsed:5.1f}s]  " + "  ".join(status_parts),
              end='', flush=True)

        if any_error:
            print()
            return False

        if all(attained):
            print(f"\n[Homing] 완료! 경과: {elapsed:.1f}s")
            return True

        if elapsed > timeout_s:
            print(f"\n[타임아웃] {timeout_s}s 초과")
            return False

        time.sleep(cycle_time_s)


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
        print("  Z축 Homing 테스트 [현장 테스트 환경 — 모터 3대]")
        print(f"  슬레이브 {X_IDX}: X축, Homing 없음 (CSV 유지)")
        print(f"  슬레이브 {Z_HW_IDX}: Z축, 리미트 스위치 O → 하드웨어 Homing 동시 수행")
        print(f"  Method {HOMING_METHOD} / 탐색: {SEARCH_SPEED_RPM} RPM"
              f" / Zero: {ZERO_SPEED_RPM} RPM")
        print("=" * 62)

        # ── 1단계: EtherCAT 초기화 ──
        print(f"\n[1단계] EtherCAT 초기화")
        master.open(ADAPTER)
        found = master.config_init()
        if found < NUM_SLAVES:
            raise RuntimeError(f"슬레이브 부족: 필요={NUM_SLAVES}, 발견={found}")
        print(f"  {found}개 슬레이브 발견")

        all_slaves = master.slaves[:NUM_SLAVES]

        # ── 2단계: 슬레이브 SDO 설정 ──
        print(f"\n[2단계] 슬레이브 SDO 설정")
        for i, slave in enumerate(all_slaves):
            print(f"\n  Slave {i}: {slave.name}")
            slave.dc_sync(True, int(CYCLE_TIME_S * 1_000_000_000))
            _reset_fault(slave)
            _configure_pdos(slave)

            if i in Z_HW_IDX:
                _setup_homing_params(slave)
                _set_mode(slave, 6)   # Homing Mode
                mode_act = struct.unpack('<b', slave.sdo_read(0x6061, 0))[0]
                print(f"    [모드] Homing(6) 설정 → 실제={mode_act}")
            else:
                # X축: CSV 모드 유지
                _set_mode(slave, 9)   # CSV Mode
                mode_act = struct.unpack('<b', slave.sdo_read(0x6061, 0))[0]
                print(f"    [모드] CSV(9) 설정 → 실제={mode_act}  (X축, Homing 없음)")

        # ── 3단계: PDO 맵핑 및 OP 상태 전환 ──
        print(f"\n[3단계] PDO 맵핑 및 OP 상태 전환")
        pdo_size = master.config_map()
        print(f"  PDO 맵핑: {pdo_size} bytes")

        master.state = pysoem.OP_STATE
        master.write_state()

        is_op = False
        for _ in range(int(5.0 / CYCLE_TIME_S)):
            master.send_processdata()
            master.receive_processdata()
            master.read_state()
            if all(s.state == pysoem.OP_STATE for s in all_slaves):
                is_op = True
                break
            time.sleep(CYCLE_TIME_S)

        if not is_op:
            raise RuntimeError("OP 상태 도달 실패")
        print("  OP 상태 도달 완료")

        # ── 4단계: CiA 402 활성화 ──
        print(f"\n[4단계] CiA 402 활성화")
        if not _cia402_enable_all(master, all_slaves, CYCLE_TIME_S):
            raise RuntimeError("드라이브 활성화 실패")
        time.sleep(0.3)

        # 초기 위치 표시
        print(f"\n  현재 위치:")
        for i, slave in enumerate(all_slaves):
            axis = "Z" if i in Z_HW_IDX else "X"
            print(f"    M{i}({axis}): {_read_pos(slave):+d} pulse")

        # ── 5단계: 안전 확인 ──
        print(f"\n[5단계] Homing 시작 전 안전 확인")
        print(f"  [슬레이브 {Z_HW_IDX}] 두 Z축 동시 하드웨어 Homing")
        print(f"    - Z축 이동 경로 전 구간 장애물 없음 확인")
        print(f"    - 슬레이브 1 하단 리미트 스위치(POT, DI1, Pin 11) 배선 확인")
        print(f"    - 슬레이브 2 하단 리미트 스위치(POT, DI1, Pin 11) 배선 확인")
        print(f"      Pin 6 → GND(0V),  NO → 24V(+),  COM → Drive Pin 11")
        print(f"    - 상단 리미트 스위치(NOT, DI2, Pin 12) 양쪽 모두 확인")
        print(f"    - Homing 중 Cross Coupling 미사용 → 두 모터 독립 탐색")
        print(f"      완료 시점 차이 가능, 비정상적인 위치 차이 발생 시 즉시 비상정지")
        print(f"  [슬레이브 {X_IDX}] X축은 현재 위치 유지 (이동 없음)")

        try:
            input(f"\n  [Enter] Homing 시작 / Ctrl+C 취소: ")
        except EOFError:
            print("\n  [건너뜀] 비대화형 모드 — 자동 시작")

        # ── 6단계: Z축 두 모터 동시 Homing 실행 ──
        print(f"\n[6단계] 슬레이브 {Z_HW_IDX} 동시 하드웨어 Homing 실행")
        homing_ok = _run_homing_dual(
            master, all_slaves, Z_HW_IDX, CYCLE_TIME_S, HOMING_TIMEOUT_S
        )

        # ── 7단계: 결과 처리 ──
        print(f"\n[7단계] 결과 처리")
        if homing_ok:
            print(f"\n  Homing 완료 위치:")
            for i in Z_HW_IDX:
                pos = _read_pos(all_slaves[i])
                sw  = _read_sw(all_slaves[i])
                print(f"    M{i}(Z): {pos:+d} pulse  (Statusword=0x{sw:04X})")

            pos_diff = abs(_read_pos(all_slaves[Z_HW_IDX[0]])
                           - _read_pos(all_slaves[Z_HW_IDX[1]]))
            print(f"\n  두 Z축 완료 위치 차이: {pos_diff} pulse"
                  f"  ({'양호' if pos_diff < 10_000 else '주의: 차이 큼'})")

            # CSV 모드 복귀 (Z축)
            print(f"\n  CSV Mode(9)로 복귀 중...")
            for i in Z_HW_IDX:
                _set_mode(all_slaves[i], 9)
                mode_act = struct.unpack('<b', all_slaves[i].sdo_read(0x6061, 0))[0]
                print(f"    M{i}: 실제 모드={mode_act}")
            time.sleep(0.1)

            print(f"\n  Homing 테스트 완료!")
            print(f"\n  [다음 단계 안내]")
            print(f"    EtherCATBusCSV.start() 후 각 Z축 모터에 set_origin() 호출:")
            for i in Z_HW_IDX:
                print(f"      motor{i}.set_origin()  → Homing 완료 위치가 0mm")
        else:
            print(f"\n  Homing 실패")
            print(f"\n  점검사항:")
            print(f"    1. 각 Z축 드라이브의 DI1(POT) 배선 재확인")
            print(f"       (드라이브 모니터링 툴로 DI1 ON/OFF 수동 확인)")
            print(f"    2. Index 펄스는 내부 엔코더에서 자동 감지 — 배선 재확인 불필요")
            print(f"    3. SEARCH_SPEED_RPM 값을 낮춰 재시도")
            _print_fault_codes(master, NUM_SLAVES)

    except KeyboardInterrupt:
        print("\n\n[중단] 사용자 인터럽트")
    except Exception as e:
        print(f"\n[오류] {type(e).__name__}: {e}")
    finally:
        print(f"\n[종료] 정지 시퀀스...")
        try:
            for slave in master.slaves[:NUM_SLAVES]:
                try:
                    _write_cw(slave, CW_DISABLE_VOLTAGE)
                except Exception:
                    pass
            for _ in range(5):
                try:
                    master.send_processdata()
                    master.receive_processdata()
                except Exception:
                    pass
                time.sleep(0.02)
            master.state = pysoem.INIT_STATE
            master.write_state()
            time.sleep(0.1)
        except Exception:
            pass
        master.close()
        print("[종료] EtherCAT 닫힘")


if __name__ == '__main__':
    main()
