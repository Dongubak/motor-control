"""
motor_csv_main.py - CSV 모드 Cross Coupling 테스트 실행 파일

motor_csv.py의 EtherCATBusCSV 클래스를 사용하여
두 모터를 CSV 모드로 동기 이동한다.
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


# ─────────────────────────────────────────────────────────────
# 메인
# ─────────────────────────────────────────────────────────────
def main():
    # ── 장치 설정 ──
    ADAPTER    = r'\Device\NPF_{CD2150F2-B355-4A6F-95BA-EB897A3726BF}'
    NUM_MOTORS = 3

    # ── CSV 모드 파라미터 ──
    RPM              = 10        # 최대 이동 속도 (RPM)
    ACCEL_RPM_PER_S  = 10        # 가감속 (RPM/sec)
    TARGET_MM        = 10        # 이동 목표 (mm)

    # ── Cross Coupling 파라미터 ──
    COUPLING_GAIN   = 0.01        # [1/sec]: 1mm 오차 → 0.5mm/sec 속도 보정
    ENABLE_COUPLING = True
    MAX_SYNC_ERR_MM = 10.0       # 긴급 정지 임계값 (mm)
    MA_WINDOW       = 5          # 이동 평균 윈도우

    # ── 버스 생성 ──
    bus = EtherCATBusCSV(
        adapter_name=ADAPTER,
        num_slaves=NUM_MOTORS,
        cycle_time_ms=10,
        max_sync_error_mm=MAX_SYNC_ERR_MM,
        coupling_gain=COUPLING_GAIN,
        enable_coupling=ENABLE_COUPLING,
        ma_window=MA_WINDOW,
    )

    motor1 = bus.motors[1]
    motor2 = bus.motors[2]

    try:
        # ── 축 설정 ──
        motor1.set_axis('z')
        motor2.set_axis('z')

        # ── 속도/가감속 설정 ──
        motor1.set_profile_velocity(rpm=RPM)
        motor1.set_profile_accel_decel(accel_rpm_per_sec=ACCEL_RPM_PER_S)
        motor2.set_profile_velocity(rpm=RPM)
        motor2.set_profile_accel_decel(accel_rpm_per_sec=ACCEL_RPM_PER_S)

        # ── 버스 시작 ──
        bus.start()

        # ── 모터 준비 대기 ──
        for motor in (motor1, motor2):
            print(f"모터 {motor._index} 준비 대기 중...")
            t0 = time.monotonic()
            while (motor.status_word & 0x006F) != 0x0027:
                time.sleep(0.05)
                if time.monotonic() - t0 > 5:
                    raise RuntimeError(f"모터 {motor._index} 준비 타임아웃!")
            print(f"[완료] 모터 {motor._index} 준비 완료")

        # ── 원점 설정 ──
        motor1.set_origin()
        motor2.set_origin()
        time.sleep(0.5)

        print(f"\n원점 설정 후:")
        print(f"  M1={motor1.current_position_mm:.2f}mm, "
              f"M2={motor2.current_position_mm:.2f}mm")

        # ════════════════════════════════════════════════════
        # 테스트 1: Cross Coupling ON — 전진
        # ════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print(f"  [테스트 1] CSV Cross Coupling ON  (Kc={COUPLING_GAIN})")
        print("=" * 60)
        bus.coupling_enabled = True
        bus.coupling_gain    = COUPLING_GAIN

        motor1.move_to_position_mm(-TARGET_MM)
        motor2.move_to_position_mm(-TARGET_MM)
        time.sleep(0.2)

        max_diff_test1 = 0.0
        t0 = time.monotonic()

        while motor1.is_moving() or motor2.is_moving():
            p1   = motor1.current_position_mm
            p2   = motor2.current_position_mm
            v1   = motor1.current_velocity_mm_s
            v2   = motor2.current_velocity_mm_s
            diff = abs(p1 - p2)
            max_diff_test1 = max(max_diff_test1, diff)

            print(f"\r  M1={p1:7.2f}mm ({v1:+5.2f}mm/s)  "
                  f"M2={p2:7.2f}mm ({v2:+5.2f}mm/s)  "
                  f"차이={diff:.3f}mm", end='')

            if motor1.has_sync_error or motor2.has_sync_error:
                print(f"\n[긴급 정지] 동기화 오류 감지!")
                break
            if time.monotonic() - t0 > 60:
                print("\n[타임아웃]")
                break
            time.sleep(0.05)

        print(f"\n\n[테스트 1 결과]")
        print(f"  최대 위치 차이: {max_diff_test1:.3f}mm")
        print(f"  최종 위치: M1={motor1.current_position_mm:.2f}mm, "
              f"M2={motor2.current_position_mm:.2f}mm")

        time.sleep(1.0)

        # ════════════════════════════════════════════════════
        # 테스트 2: Cross Coupling OFF — 원점 복귀
        # ════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print(f"  [테스트 2] CSV Cross Coupling OFF — 원점 복귀")
        print("=" * 60)
        bus.coupling_enabled = False

        motor1.move_to_position_mm(0)
        motor2.move_to_position_mm(0)
        time.sleep(0.2)

        max_diff_test2 = 0.0
        t0 = time.monotonic()

        while motor1.is_moving() or motor2.is_moving():
            p1   = motor1.current_position_mm
            p2   = motor2.current_position_mm
            diff = abs(p1 - p2)
            max_diff_test2 = max(max_diff_test2, diff)

            print(f"\r  M1={p1:7.2f}mm  M2={p2:7.2f}mm  차이={diff:.3f}mm", end='')

            if motor1.has_sync_error:
                print("\n[긴급 정지]")
                break
            if time.monotonic() - t0 > 60:
                print("\n[타임아웃]")
                break
            time.sleep(0.05)

        print(f"\n\n[테스트 2 결과]")
        print(f"  최대 위치 차이 (Coupling OFF): {max_diff_test2:.3f}mm")
        print(f"  최종 위치: M1={motor1.current_position_mm:.2f}mm, "
              f"M2={motor2.current_position_mm:.2f}mm")

        time.sleep(1.0)

        # ════════════════════════════════════════════════════
        # 결과 요약
        # ════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print("  [결과 요약]")
        print("=" * 60)
        print(f"  테스트 1 (Coupling ON ):  최대 위치차 {max_diff_test1:.3f}mm")
        print(f"  테스트 2 (Coupling OFF):  최대 위치차 {max_diff_test2:.3f}mm")
        if max_diff_test2 > 0:
            improvement = (1 - max_diff_test1 / max_diff_test2) * 100
            print(f"  동기화 개선: {improvement:.1f}%")
        print("=" * 60)

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
