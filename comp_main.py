"""
comp_main.py - Cross Coupling 위치 보정이 적용된 이중 모터 제어

main_legacy.py와 동일한 구조이나, 이동 중 두 모터의 위치를 매 10ms 사이클마다
비교하여 앞선 모터는 감속, 뒤처진 모터는 가속하는 Cross Coupling 보정이 적용됨.

Cross Coupling 원리 (motor_coupling.py):
  - 매 사이클: avg = (M1_pos + M2_pos) / 2
  - 각 모터 보정량 = COUPLING_GAIN × (motor_pos - avg)
  - target -= 보정량  →  앞서면 감속 / 뒤처지면 가속

설정값:
  COUPLING_GAIN     : 보정 강도 (0.0 = 보정없음, 1.0 = 최대)
  MAX_SYNC_ERROR_MM : 이 값 초과 시 긴급 정지
"""

from motor_coupling import EtherCATBusCoupling
import time

# ─────────────────────────────────────────────
#  설정값 (여기서만 수정)
# ─────────────────────────────────────────────
ADAPTER           = r'\Device\NPF_{CD2150F2-B355-4A6F-95BA-EB897A3726BF}'
NUM_MOTORS        = 2
TARGET_MM         = 10       # 이동 목표 (mm)
VELOCITY_RPM      = 20        # 이동 속도 (RPM)
ACCEL_RPM         = 20        # 가감속도 (RPM/s)
COUPLING_GAIN     = 0.1       # Cross Coupling 게인 (0.0 ~ 1.0)
MAX_SYNC_ERROR_MM = 20.0       # 긴급 정지 임계 (mm)


def wait_move(motor1, motor2, label: str = "이동", timeout: float = 60) -> tuple[bool, float]:
    """
    두 모터가 모두 정지할 때까지 대기하며 실시간 위치 차이를 출력한다.

    Returns:
        (성공 여부, 이동 중 최대 위치 차이 mm)
    """
    time.sleep(0.2)  # 명령 반영 대기
    max_diff_mm = 0.0
    start = time.monotonic()

    while motor1.is_moving() or motor2.is_moving():
        m1 = motor1.current_position_mm
        m2 = motor2.current_position_mm
        diff = abs(m1 - m2)
        if diff > max_diff_mm:
            max_diff_mm = diff

        print(
            f"--> [{label}]  "
            f"M1: {m1:7.3f}mm  "
            f"M2: {m2:7.3f}mm  "
            f"차이: {diff:.3f}mm  "
            f"(최대: {max_diff_mm:.3f}mm)",
            end='\r'
        )

        if motor1.has_sync_error or motor2.has_sync_error:
            print(f"\n[긴급 정지] 동기화 오차({MAX_SYNC_ERROR_MM}mm) 초과! 모터를 정지합니다.")
            return False, max_diff_mm

        time.sleep(0.05)

        if time.monotonic() - start > timeout:
            print(f"\n[경고] {timeout}초 타임아웃: 이동이 완료되지 않았습니다.")
            return False, max_diff_mm

    return True, max_diff_mm


def main():
    bus = EtherCATBusCoupling(
        adapter_name=ADAPTER,
        num_slaves=NUM_MOTORS,
        cycle_time_ms=10,
        max_sync_error_mm=MAX_SYNC_ERROR_MM,
        coupling_gain=COUPLING_GAIN,
        enable_coupling=True,
    )

    motor1 = bus.motors[0]
    motor2 = bus.motors[1]

    try:
        # ── SDO 설정 (버스 시작 전) ──────────────────────────
        motor1.set_axis('z')
        motor2.set_axis('z')
        motor1.set_profile_velocity(rpm=VELOCITY_RPM)
        motor1.set_profile_accel_decel(accel_rpm_per_sec=ACCEL_RPM)
        motor2.set_profile_velocity(rpm=VELOCITY_RPM)
        motor2.set_profile_accel_decel(accel_rpm_per_sec=ACCEL_RPM)

        # ── 버스 시작 ─────────────────────────────────────────
        bus.start()

        # ── 모터 준비 대기 ────────────────────────────────────
        for motor in [motor1, motor2]:
            print(f"모터 {motor._index} 준비 대기 중...")
            t0 = time.monotonic()
            while (motor.status_word & 0x006F) != 0x0027:
                time.sleep(0.05)
                if time.monotonic() - t0 > 5:
                    status = motor.status_word
                    raise RuntimeError(
                        f"모터 {motor._index} 타임아웃! 상태: 0x{status:04X}"
                    )
            print(f"[완료] 모터 {motor._index} 준비 완료 ({time.monotonic() - t0:.2f}s)")

        # ── 원점 설정 ─────────────────────────────────────────
        motor1.set_origin()
        motor2.set_origin()
        time.sleep(0.5)
        print(f"\n원점: M1={motor1.current_position_mm:.3f}mm  M2={motor2.current_position_mm:.3f}mm")

        # ── Cross Coupling 이동 ───────────────────────────────
        print("\n" + "=" * 60)
        print(f"  [이동]  목표: {TARGET_MM}mm  게인: {COUPLING_GAIN}  속도: {VELOCITY_RPM}RPM")
        print("=" * 60)

        motor1.move_to_position_mm(TARGET_MM)
        motor2.move_to_position_mm(TARGET_MM)

        ok_go, max_diff_go = wait_move(motor1, motor2, label="이동")
        print(f"\n이동 완료: M1={motor1.current_position_mm:.3f}mm  "
              f"M2={motor2.current_position_mm:.3f}mm  "
              f"최종 차이: {abs(motor1.current_position_mm - motor2.current_position_mm):.3f}mm  "
              f"이동 중 최대 차이: {max_diff_go:.3f}mm")

        if not ok_go:
            bus.reset_sync_error()

        time.sleep(1)

        # ── 원점 복귀 ─────────────────────────────────────────
        print("\n" + "=" * 60)
        print("  [원점 복귀]")
        print("=" * 60)

        motor1.move_to_position_mm(0)
        motor2.move_to_position_mm(0)

        ok_ret, max_diff_ret = wait_move(motor1, motor2, label="복귀")
        print(f"\n복귀 완료: M1={motor1.current_position_mm:.3f}mm  "
              f"M2={motor2.current_position_mm:.3f}mm  "
              f"최종 차이: {abs(motor1.current_position_mm - motor2.current_position_mm):.3f}mm  "
              f"복귀 중 최대 차이: {max_diff_ret:.3f}mm")

        if not ok_ret:
            bus.reset_sync_error()

        # ── 종합 결과 ─────────────────────────────────────────
        print("\n" + "=" * 60)
        print("  [결과 요약]")
        print(f"  게인 (COUPLING_GAIN)    : {COUPLING_GAIN}")
        print(f"  이동 중 최대 위치차     : {max_diff_go:.3f} mm")
        print(f"  복귀 중 최대 위치차     : {max_diff_ret:.3f} mm")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\n사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"\n에러 발생: {e}")
    finally:
        bus.stop()


if __name__ == "__main__":
    main()
