"""
motor_csv_main_homing_and_move.py - X축 Homing 후 타겟 이동

[동작 순서]
  1단계: 하드웨어 Homing (Mode 6, Method 2)
         정방향(+) 이동 → POT 리밋 스위치 감지 → 방향 전환 → Index(Z) 펄스 원점 확정
  2단계: CSV 모드(Mode 9)로 TARGET_MM 위치 이동
         Homing 완료 위치를 원점(0mm)으로 설정 후 이동

[핵심 설계]
  - 1단계: 직접 pysoem + 연속 PDO 루프 상태 머신 (SM Watchdog 방지)
  - 1단계 완료 후 EtherCAT 종료 → 어댑터 해제
  - 2단계: EtherCATBusCSV 재연결 (Mode 9, CSV PDO)
  - bit12_cleared 추적으로 이전 세션 잔류 플래그 오탐 방지

참조:
  motor_csv_main_x_homing.py  — 1단계 Homing 구조
  motor_csv_main_x.py         — 2단계 CSV 이동 구조
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

NUM_SLAVES = 1    # X축 슬레이브 0만
MOTOR_IDX  = 0    # 슬레이브 0 = X축

CYCLE_TIME_S = 0.010   # 10 ms

# ─────────────────────────────────────────────────────────────
# 1단계: Homing 파라미터
# ─────────────────────────────────────────────────────────────
# EFFECTIVE_PPR = 8,388,608 pulse/rev (= 2^23, 전기적 기어비 1:1)
EFFECTIVE_PPR    = 8_388_608

SEARCH_SPEED_RPM = 30           # 스위치 탐색 속도 (RPM) — 권장 10~30
ZERO_SPEED_RPM   = 1            # Index 탐색 속도 (RPM) — 권장 1~5
HOMING_ACCEL_UU  = 5_000_000   # 가감속 (UU/s²)

SEARCH_SPEED = int((SEARCH_SPEED_RPM / 60.0) * EFFECTIVE_PPR)
ZERO_SPEED   = int((ZERO_SPEED_RPM   / 60.0) * EFFECTIVE_PPR)

HOMING_METHOD    = 2   # Method 2: 정방향(+) + POT + Index 펄스
HOME_OFFSET      = 0   # 원점 오프셋 = 0
HOMING_DONE_BEH  = 0   # 완료 후 동작: 이동 없음 (0x201E)

HOMING_TIMEOUT_S = 600.0   # Homing 최대 허용 시간 (초)

# ─────────────────────────────────────────────────────────────
# 2단계: CSV 이동 파라미터
# ─────────────────────────────────────────────────────────────

###### 현장 용 ######
RPM             = 1000   # 최대 이동 속도 (RPM)
ACCEL_RPM_PER_S = 1000   # 가감속 (RPM/sec)
TARGET_MM       = -1600    # 이동 목표 (mm)  ※ X축 이동 한계 내로 설정

###### 사무실 내 시연 용 ######
# RPM             = 10
# ACCEL_RPM_PER_S = 10
# TARGET_MM       = -100

# ─────────────────────────────────────────────────────────────
# CiA 402 Controlword 상수 (Homing 단계용)
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
# Homing PDO 헬퍼 (RxPDO CW 2B / TxPDO SW+Pos 6B)
# ─────────────────────────────────────────────────────────────
def _read_sw(slave) -> int:
    return struct.unpack('<H', slave.input[0:2])[0]

def _read_pos(slave) -> int:
    return struct.unpack('<i', slave.input[2:6])[0]

def _write_cw(slave, cw: int):
    slave.output = struct.pack('<H', cw)


# ─────────────────────────────────────────────────────────────
# Homing 초기화 헬퍼 (SDO 전용, OP 진입 전)
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
    """Homing 전용 PDO: RxPDO CW 2B / TxPDO SW+Pos 6B"""
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
# Homing 종료 시퀀스
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
    """모터 이동 완료까지 실시간 상태 출력하며 대기. 오류 시 True 반환."""
    t0 = time.monotonic()
    while motor.is_moving():
        p = motor.current_position_mm
        v = motor.current_velocity_mm_s
        print(f"\r  [{label}]  X={p:8.3f}mm  ({v:+6.3f}mm/s)", end='')
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
# 1단계: 하드웨어 Homing
# ─────────────────────────────────────────────────────────────
def run_homing() -> bool:
    """X축 하드웨어 Homing을 수행한다. 성공하면 True, 실패하면 False 반환."""
    master = pysoem.Master()
    homing_ok = False

    try:
        print(f"\n[1단계] EtherCAT 초기화 (Homing)")
        master.open(ADAPTER)
        found = master.config_init()
        if found < NUM_SLAVES:
            raise RuntimeError(f"슬레이브 부족: 필요={NUM_SLAVES}, 발견={found}")
        print(f"  {found}개 슬레이브 발견")
        slave = master.slaves[MOTOR_IDX]

        print(f"\n[2단계] 슬레이브 SDO 설정")
        print(f"  Slave {MOTOR_IDX}: {slave.name}")
        slave.dc_sync(True, int(CYCLE_TIME_S * 1_000_000_000))
        _reset_fault(slave)
        _configure_homing_pdos(slave)
        _setup_homing_params(slave)
        slave.sdo_write(0x6060, 0, struct.pack('<b', 6))
        time.sleep(0.02)
        mode_pre = struct.unpack('<b', slave.sdo_read(0x6061, 0))[0]
        print(f"    [모드] Homing(6) 설정 → 실제={mode_pre}")

        print(f"\n[3단계] PDO 맵핑")
        pdo_size = master.config_map()
        print(f"  PDO 맵핑: {pdo_size} bytes")
        slave.sdo_write(0x6060, 0, struct.pack('<b', 6))
        time.sleep(0.02)
        print(f"  Homing 모드(6) 재설정 완료")

        print(f"\n[4단계] OP 전환 + 연속 PDO 루프 (Homing)")
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
                if slave.state == pysoem.OP_STATE:
                    print(f"\n  OP 상태 확인 ({cycle_count}사이클). CiA 402 활성화 시작...")
                    phase   = _ST_ENABLE
                    t_phase = now
                elif now - t_phase > 5.0:
                    raise RuntimeError("OP 상태 도달 실패 (5초 타임아웃)")
                _write_cw(slave, CW_DISABLE_VOLTAGE)

            elif phase == _ST_ENABLE:
                sw = _read_sw(slave)
                if sw & 0x0008:
                    _write_cw(slave, CW_FAULT_RESET)
                elif (sw & 0x004F) == 0x0040:
                    _write_cw(slave, CW_SHUTDOWN)
                elif (sw & 0x006F) == 0x0021:
                    _write_cw(slave, CW_SWITCH_ON)
                elif (sw & 0x006F) == 0x0023:
                    _write_cw(slave, CW_ENABLE_OPERATION)
                elif (sw & 0x006F) == 0x0027:
                    _write_cw(slave, CW_ENABLE_OPERATION)
                    pos = _read_pos(slave)
                    print(f"  Operation Enabled. 현재 위치: {pos:+d} pulse")
                    print(f"  Homing bit4 rising edge 준비 (3사이클)...")
                    phase     = _ST_HOMING_B4
                    b4_cycles = 0
                else:
                    _write_cw(slave, CW_DISABLE_VOLTAGE)
                if now - t_phase > 10.0:
                    raise RuntimeError("CiA 402 활성화 타임아웃 (10초)")

            elif phase == _ST_HOMING_B4:
                _write_cw(slave, CW_ENABLE_OPERATION)
                b4_cycles += 1
                if b4_cycles >= 3:
                    print(f"  Homing Start (bit4=1)")
                    t_homing      = now
                    bit12_cleared = False
                    phase         = _ST_HOMING

            elif phase == _ST_HOMING:
                sw      = _read_sw(slave)
                pos     = _read_pos(slave)
                bit12   = bool(sw & (1 << 12))
                bit13   = bool(sw & (1 << 13))
                elapsed = now - t_homing

                if not bit12 and not bit13:
                    bit12_cleared = True

                if bit13:
                    _write_cw(slave, CW_ENABLE_OPERATION)
                elif bit12 and bit12_cleared:
                    _write_cw(slave, CW_ENABLE_OPERATION)
                else:
                    _write_cw(slave, CW_HOMING_START)

                if bit12 and not bit12_cleared:
                    status = "잔류bit12"
                elif bit12:
                    status = "완료"
                elif bit13:
                    status = "에러"
                else:
                    status = "탐색중"
                print(f"\r  [Homing {elapsed:5.1f}s]  "
                      f"X:{status}(SW=0x{sw:04X}, pos={pos:+d})",
                      end='', flush=True)

                if bit13:
                    print(f"\n[오류] Homing Error (SW=0x{sw:04X})")
                    phase = _ST_ERROR
                elif bit12 and bit12_cleared:
                    print(f"\n  [Homing 완료] 경과={elapsed:.1f}s, pos={pos:+d} pulse")
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
# 2단계: CSV 모드 이동
# ─────────────────────────────────────────────────────────────
def run_csv_move():
    """X축을 CSV 모드로 TARGET_MM 위치까지 이동한다."""
    bus = EtherCATBusCSV(
        adapter_name=ADAPTER,
        num_slaves=NUM_SLAVES,
        cycle_time_ms=10,
        enable_coupling=False,
    )
    motor_x = bus.motors[0]

    try:
        motor_x.set_axis('x')
        motor_x.set_profile_velocity(rpm=RPM)
        motor_x.set_profile_accel_decel(accel_rpm_per_sec=ACCEL_RPM_PER_S)

        bus.start()

        print(f"\n모터 {motor_x._index} (X축) 준비 대기 중...")
        t0 = time.monotonic()
        while (motor_x.status_word & 0x006F) != 0x0027:
            time.sleep(0.05)
            if time.monotonic() - t0 > 5:
                raise RuntimeError(f"모터 {motor_x._index} 준비 타임아웃!")
        print(f"[완료] 모터 {motor_x._index} (X축) 준비 완료")

        motor_x.set_origin()
        time.sleep(0.5)
        print(f"\n원점 설정 후: X={motor_x.current_position_mm:.3f}mm")

        print("\n" + "=" * 60)
        print(f"  X축 이동")
        print(f"  목표: {TARGET_MM}mm  /  {RPM} RPM  /  가감속 {ACCEL_RPM_PER_S} RPM/s")
        print("=" * 60)

        motor_x.move_to_position_mm(TARGET_MM)
        time.sleep(0.2)

        err = wait_motor(motor_x, "X축 이동", timeout=120.0)

        print(f"\n[결과]")
        print(f"  최종 위치: X={motor_x.current_position_mm:.3f}mm")
        if err:
            print("  [경고] 이상 발생으로 중단됨 (Fault)")

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
    print("  X축 Homing 후 타겟 이동")
    print(f"  [Homing] Method 2, 탐색={SEARCH_SPEED_RPM} RPM, Zero={ZERO_SPEED_RPM} RPM")
    print(f"  [이동]   목표={TARGET_MM}mm, {RPM} RPM, 가감속={ACCEL_RPM_PER_S} RPM/s")
    print(f"  사이클: {int(CYCLE_TIME_S * 1000)}ms")
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

    run_csv_move()


if __name__ == '__main__':
    main()
