"""
main_dual_coupling.py - Cross Coupling 적용 이동 및 원점 복귀

Cross Coupling을 활성화한 상태로 목표 위치까지 이동 후 원점으로 복귀합니다.

사용법:
    python main_dual_coupling.py
"""

from motor_coupling import EtherCATBusCoupling
import time


def wait_move(motor1, motor2, timeout=60):
    """두 모터가 이동 완료될 때까지 위치를 출력하며 대기합니다."""
    time.sleep(0.2)
    start = time.monotonic()
    while motor1.is_moving() or motor2.is_moving():
        pos_diff = abs(motor1.current_position_mm - motor2.current_position_mm)
        print(f"--> M1: {motor1.current_position_mm:7.2f}mm  "
              f"M2: {motor2.current_position_mm:7.2f}mm  "
              f"차이: {pos_diff:.3f}mm", end='\r')

        if motor1.has_sync_error or motor2.has_sync_error:
            print(f"\n[긴급 정지] 동기화 오류 감지!")
            return False

        time.sleep(0.05)
        if time.monotonic() - start > timeout:
            print(f"\n[경고] 이동 타임아웃!")
            return False
    return True


def main():
    # --- 장치 포트 및 슬레이브 수 설정 ---
    adapter = r'\Device\NPF_{CD2150F2-B355-4A6F-95BA-EB897A3726BF}'
    # NUM_MOTORS = 3
    NUM_MOTORS = 2

    # --- 이동 목표 위치 (mm) ---
    TARGET_MM = 10

    # --- Cross Coupling 설정 ---
    COUPLING_GAIN     = 0.05   # 게인 (0.0 ~ 1.0)
    MAX_SYNC_ERROR_MM = 1000     # 동기화 오차 허용 범위 (mm)

    # --- 버스 생성 ---
    bus = EtherCATBusCoupling(
        adapter_name=adapter,
        num_slaves=NUM_MOTORS,
        cycle_time_ms=10,
        max_sync_error_mm=MAX_SYNC_ERROR_MM,
        coupling_gain=COUPLING_GAIN,
        enable_coupling=True,
    )

    # motor1 = bus.motors[1]
    # motor2 = bus.motors[2]

    motor1 = bus.motors[0]
    motor2 = bus.motors[1]

    try:
        # --- 축 / SDO 설정 (버스 시작 전) ---
        motor1.set_axis('z')
        motor2.set_axis('z')
        motor1.set_profile_velocity(rpm=10)
        motor1.set_profile_accel_decel(accel_rpm_per_sec=5)
        motor2.set_profile_velocity(rpm=10)
        motor2.set_profile_accel_decel(accel_rpm_per_sec=5)

        # --- 버스 시작 ---
        bus.start()

        # --- 모터 준비 대기 ---
        for motor in [motor1, motor2]:
            print(f"모터 {motor._index} 준비 대기 중...")
            start_time = time.monotonic()
            while (motor.status_word & 0x006F) != 0x0027:
                time.sleep(0.05)
                if time.monotonic() - start_time > 5:
                    raise RuntimeError(f"모터 {motor._index} 타임아웃!")
            print(f"[완료] 모터 {motor._index} 준비 완료!")

        # --- 원점 설정 ---
        motor1.set_origin()
        motor2.set_origin()
        time.sleep(0.5)
        print(f"\n원점 위치: M1={motor1.current_position_mm:.2f}mm  M2={motor2.current_position_mm:.2f}mm")

        # --- Cross Coupling 이동 ---
        print("\n" + "="*60)
        print(f"  [Cross Coupling 이동]  목표: {TARGET_MM}mm  게인: {COUPLING_GAIN}")
        print("="*60)

        motor1.move_to_position_mm(TARGET_MM)
        motor2.move_to_position_mm(TARGET_MM)

        ok = wait_move(motor1, motor2)
        max_diff_go = abs(motor1.current_position_mm - motor2.current_position_mm)

        print(f"\n이동 완료: M1={motor1.current_position_mm:.2f}mm  "
              f"M2={motor2.current_position_mm:.2f}mm  "
              f"최종 차이: {max_diff_go:.3f}mm")

        if not ok:
            bus.reset_sync_error()

        time.sleep(1)

        # --- 원점 복귀 ---
        print("\n--- 원점 복귀 ---")
        motor1.move_to_position_mm(0)
        motor2.move_to_position_mm(0)

        ok = wait_move(motor1, motor2)
        max_diff_ret = abs(motor1.current_position_mm - motor2.current_position_mm)

        print(f"\n복귀 완료: M1={motor1.current_position_mm:.2f}mm  "
              f"M2={motor2.current_position_mm:.2f}mm  "
              f"최종 차이: {max_diff_ret:.3f}mm")

        print("\n" + "="*60)
        print("  [완료]")
        print(f"  이동 시 최종 위치차: {max_diff_go:.3f}mm")
        print(f"  복귀 시 최종 위치차: {max_diff_ret:.3f}mm")
        print("="*60)

    except KeyboardInterrupt:
        print("\n\n사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"\n\n에러 발생: {e}")
    finally:
        bus.stop()


if __name__ == "__main__":
    main()
