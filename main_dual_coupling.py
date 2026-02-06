"""
main_dual_coupling.py - Cross Coupling 기능 테스트 코드

특징:
1. motor_coupling.py의 EtherCATBusCoupling 클래스 사용
2. Cross Coupling 게인 런타임 조절
3. 위치 차이 모니터링 및 동기화 품질 평가

사용법:
    python main_dual_coupling.py
"""

from motor_coupling import EtherCATBusCoupling
import time


def main():
    # --- 장치 포트 및 슬레이브 수 설정 ---
    adapter = r'\Device\NPF_{A3C1307F-C4D5-4126-8FCF-A191BF2B1257}'
    NUM_MOTORS = 2

    # --- Cross Coupling 설정 ---
    COUPLING_GAIN = 0.05         # 초기 게인 (0.0 ~ 1.0)
    ENABLE_COUPLING = True      # Cross Coupling 활성화
    MAX_SYNC_ERROR_MM = 0.5     # 동기화 오차 허용 범위 (mm)

    # --- 버스 생성 ---
    bus = EtherCATBusCoupling(
        adapter_name=adapter,
        num_slaves=NUM_MOTORS,
        cycle_time_ms=10,
        max_sync_error_mm=MAX_SYNC_ERROR_MM,
        coupling_gain=COUPLING_GAIN,
        enable_coupling=ENABLE_COUPLING
    )

    motor1 = bus.motors[0]
    motor2 = bus.motors[1]

    try:
        # --- 축 설정 ---
        motor1.set_axis('z')
        motor2.set_axis('z')

        # --- SDO 설정 ---
        motor1.set_profile_velocity(rpm=50)
        motor1.set_profile_accel_decel(accel_rpm_per_sec=50)
        motor2.set_profile_velocity(rpm=50)
        motor2.set_profile_accel_decel(accel_rpm_per_sec=50)

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

        print(f"\n원점 설정 후:")
        print(f"  M1={motor1.current_position_mm:.2f}mm, M2={motor2.current_position_mm:.2f}mm")

        # --- Cross Coupling 테스트 ---
        print("\n" + "="*60)
        print("  [Cross Coupling 테스트]")
        print(f"  게인: {bus.coupling_gain:.2f}")
        print(f"  활성화: {bus.coupling_enabled}")
        print("="*60)

        # 테스트 1: Cross Coupling 활성화 상태로 이동
        print("\n--- 테스트 1: Cross Coupling ON (Kc=0.1) ---")
        bus.coupling_enabled = True
        bus.coupling_gain = 0.1

        motor1.move_to_position_mm(-50)
        motor2.move_to_position_mm(-50)

        time.sleep(0.2)
        max_diff_test1 = 0

        start_time = time.monotonic()
        while motor1.is_moving() or motor2.is_moving():
            pos_diff = abs(motor1.current_position_mm - motor2.current_position_mm)
            max_diff_test1 = max(max_diff_test1, pos_diff)
            print(f"--> M1: {motor1.current_position_mm:7.2f}mm, M2: {motor2.current_position_mm:7.2f}mm, 차이: {pos_diff:.3f}mm", end='\r')

            if motor1.has_sync_error or motor2.has_sync_error:
                print(f"\n[긴급 정지] 동기화 오류!")
                break

            time.sleep(0.05)
            if time.monotonic() - start_time > 60:
                break

        print(f"\n\n[테스트 1 결과] Cross Coupling ON")
        print(f"  최대 위치 차이: {max_diff_test1:.3f}mm")
        print(f"  최종 위치: M1={motor1.current_position_mm:.2f}mm, M2={motor2.current_position_mm:.2f}mm")

        time.sleep(1)

        # 원점 복귀
        print("\n--- 원점 복귀 ---")
        motor1.move_to_position_mm(0)
        motor2.move_to_position_mm(0)

        time.sleep(0.2)
        while motor1.is_moving() or motor2.is_moving():
            print(f"--> M1: {motor1.current_position_mm:7.2f}mm, M2: {motor2.current_position_mm:7.2f}mm", end='\r')
            if motor1.has_sync_error:
                break
            time.sleep(0.05)

        print(f"\n원점 복귀 완료: M1={motor1.current_position_mm:.2f}mm, M2={motor2.current_position_mm:.2f}mm")
        time.sleep(1)

        # 테스트 2: Cross Coupling 비활성화 상태로 이동
        print("\n--- 테스트 2: Cross Coupling OFF ---")
        bus.coupling_enabled = False

        motor1.move_to_position_mm(-50)
        motor2.move_to_position_mm(-50)

        time.sleep(0.2)
        max_diff_test2 = 0

        start_time = time.monotonic()
        while motor1.is_moving() or motor2.is_moving():
            pos_diff = abs(motor1.current_position_mm - motor2.current_position_mm)
            max_diff_test2 = max(max_diff_test2, pos_diff)
            print(f"--> M1: {motor1.current_position_mm:7.2f}mm, M2: {motor2.current_position_mm:7.2f}mm, 차이: {pos_diff:.3f}mm", end='\r')

            if motor1.has_sync_error or motor2.has_sync_error:
                print(f"\n[긴급 정지] 동기화 오류!")
                bus.reset_sync_error()
                time.sleep(0.5)
                break

            time.sleep(0.05)
            if time.monotonic() - start_time > 60:
                break

        print(f"\n\n[테스트 2 결과] Cross Coupling OFF")
        print(f"  최대 위치 차이: {max_diff_test2:.3f}mm")
        print(f"  최종 위치: M1={motor1.current_position_mm:.2f}mm, M2={motor2.current_position_mm:.2f}mm")

        # 최종 원점 복귀
        print("\n--- 최종 원점 복귀 ---")
        bus.coupling_enabled = True
        motor1.move_to_position_mm(0)
        motor2.move_to_position_mm(0)

        time.sleep(0.2)
        while motor1.is_moving() or motor2.is_moving():
            print(f"--> M1: {motor1.current_position_mm:7.2f}mm, M2: {motor2.current_position_mm:7.2f}mm", end='\r')
            time.sleep(0.05)

        print(f"\n")

        # --- 결과 요약 ---
        print("\n" + "="*60)
        print("  [테스트 결과 요약]")
        print("="*60)
        print(f"  테스트 1 (Coupling ON):  최대 위치차 {max_diff_test1:.3f}mm")
        print(f"  테스트 2 (Coupling OFF): 최대 위치차 {max_diff_test2:.3f}mm")

        if max_diff_test1 < max_diff_test2:
            improvement = ((max_diff_test2 - max_diff_test1) / max_diff_test2) * 100
            print(f"\n  → Cross Coupling으로 동기화 {improvement:.1f}% 개선!")
        else:
            print(f"\n  → 추가 튜닝이 필요합니다.")

        print("="*60)
        time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"\n\n에러 발생: {e}")
    finally:
        bus.stop()


if __name__ == "__main__":
    main()
