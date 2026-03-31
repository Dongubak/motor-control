"""
motor_csv_main_x.py - X축 단독 이동 (CSV 모드)

슬레이브 0 (X축) 서보모터를 목표 위치로 이동한다.
원점 복귀 없음. 단일 모터 제어.

motor_csv_main2.py (Z축 2대) 와 동일한 EtherCATBusCSV 패턴 사용.
"""

from motor_csv import EtherCATBusCSV
import time
import pysoem
import struct


# ─────────────────────────────────────────────────────────────
# Fault 코드 조회 (CiA 402 / iX7NH 제조사 코드)
# ─────────────────────────────────────────────────────────────
_FAULT_NAMES = {
    0x0000: "에러 없음",
    0x1000: "일반 오류",
    0x2310: "과전류",
    0x3120: "연속 과전류",
    0x3310: "과전압",
    0x3320: "저전압",
    0x4210: "모터 과열",
    0x5114: "EtherCAT Watchdog 타임아웃",
    0x6010: "소프트웨어 오버플로우",
    0x7300: "추종 오차 초과 (Following Error)",
    0x7500: "디바이스 특정 오류",
    0x8110: "EtherCAT 통신 오류",
    0x8130: "통신 타임아웃 (Heartbeat)",
    0x8611: "위치 추종 오차",
}


def _fault_name(code: int) -> str:
    if code in _FAULT_NAMES:
        return _FAULT_NAMES[code]
    masked = code & 0xFF00
    if masked != 0:
        return _FAULT_NAMES.get(masked, f"제조사 특정 오류 (0x{code:04X})")
    return f"제조사 특정 오류 (0x{code:04X})"


def read_drive_fault_codes(adapter: str, num_slaves: int):
    """버스 종료 후 SDO로 각 드라이브의 Fault 코드 조회."""
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
                print(f"    현재 Error Code (0x603F) : 0x{code:04X}  →  {_fault_name(code)}")
            except Exception as e:
                print(f"    0x603F 읽기 실패: {e}")

            try:
                num_errors = struct.unpack("<B", slave.sdo_read(0x1003, 0))[0]
                if num_errors == 0:
                    print(f"    에러 이력 (0x1003)     : 없음")
                else:
                    print(f"    에러 이력 (0x1003)     : {num_errors}개")
                    for j in range(1, min(num_errors + 1, 4)):
                        hist     = struct.unpack("<I", slave.sdo_read(0x1003, j))[0]
                        std_code = hist & 0xFFFF
                        mfr_code = (hist >> 16) & 0xFFFF
                        print(f"      [{j}] std=0x{std_code:04X} ({_fault_name(std_code)})"
                              f",  mfr=0x{mfr_code:04X}")
            except Exception as e:
                print(f"    0x1003 읽기 실패: {e}")

        print("=" * 60)
    except Exception as e:
        print(f"[Fault 조회 실패] {e}")
    finally:
        master.close()


def wait_motor(motor, label: str, timeout: float = 60.0):
    """모터 이동 완료까지 실시간 상태 출력하며 대기."""
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
# 메인
# ─────────────────────────────────────────────────────────────
def main():
    # ── 장치 설정 ──
    # ADAPTER    = r'\Device\NPF_{CD2150F2-B355-4A6F-95BA-EB897A3726BF}'  # 현장
    ADAPTER = r'\Device\NPF_{D1B66F5F-FF8A-4D4D-8C7C-2FF2547CE945}'  # 사무실
    # ADAPTER    = r'\Device\NPF_{60F43190-600E-4F16-972D-A254AA5F3E19}'  # 현장
    NUM_MOTORS = 1   # X축 슬레이브 0만 제어

    #### 현장에서 확인할 변수 목록 ####
    # RPM
    # ACCEL_RPM_PER_S
    # TARGET_MM

    ###### 현장 용 ######
    # RPM             = 1000   # 최대 이동 속도 (RPM)
    # ACCEL_RPM_PER_S = 1000   # 가감속 (RPM/sec)
    # TARGET_MM       = 500    # 이동 목표 (mm)  ※ X축 이동 한계 내로 설정

    ###### 사무실 내 시연 용 ######
    RPM             = 30
    ACCEL_RPM_PER_S = 30
    TARGET_MM       = -200
    # ── 버스 생성 (Cross Coupling 비활성 — 단일 모터) ──
    bus = EtherCATBusCSV(
        adapter_name=ADAPTER,
        num_slaves=NUM_MOTORS,
        cycle_time_ms=10,
        enable_coupling=False,
    )

    motor_x = bus.motors[0]   # 슬레이브 0 = X축

    try:
        # ── 축 설정 ──
        motor_x.set_axis('x')

        # ── 속도/가감속 설정 ──
        motor_x.set_profile_velocity(rpm=RPM)
        motor_x.set_profile_accel_decel(accel_rpm_per_sec=ACCEL_RPM_PER_S)

        # ── 버스 시작 ──
        bus.start()

        # ── 모터 준비 대기 ──
        print(f"모터 {motor_x._index} (X축) 준비 대기 중...")
        t0 = time.monotonic()
        while (motor_x.status_word & 0x006F) != 0x0027:
            time.sleep(0.05)
            if time.monotonic() - t0 > 5:
                raise RuntimeError(f"모터 {motor_x._index} 준비 타임아웃!")
        print(f"[완료] 모터 {motor_x._index} (X축) 준비 완료")

        # ── 원점 설정 ──
        motor_x.set_origin()
        time.sleep(0.5)
        print(f"\n원점 설정 후: X={motor_x.current_position_mm:.3f}mm")

        # ════════════════════════════════════════════════════
        # 타겟 위치로 이동
        # ════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print(f"  X축 이동")
        print(f"  목표: +{TARGET_MM}mm  /  {RPM} RPM  /  가감속 {ACCEL_RPM_PER_S} RPM/s")
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
        read_drive_fault_codes(ADAPTER, NUM_MOTORS)


if __name__ == "__main__":
    main()
