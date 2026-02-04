"""
main_dual_safe.py - 안전 동기화 기능이 포함된 2개 모터 테스트 코드

특징:
1. motor_safe.py의 EtherCATBusSafe 클래스 사용
2. 위치 차이 모니터링 활성화
3. 동기화 오차 초과 시 긴급 정지
4. 동기화 오류 상태 확인 및 복구

사용법:
    python main_dual_safe.py
"""

from motor_safe import EtherCATBusSafe
import time


def main():
    # --- 장치 포트 및 슬레이브 수 설정 ---
    adapter = r'\Device\NPF_{A3C1307F-C4D5-4126-8FCF-A191BF2B1257}'
    NUM_MOTORS = 2  # 2개 모터 사용

    # --- [안전 설정] 동기화 오차 허용 범위 ---
    # 중량물 리프팅의 경우 0.5mm 권장
    # 정밀 조립의 경우 0.1mm 권장
    MAX_SYNC_ERROR_MM = 0.5  # mm 단위

    # --- 안전 버스 관리자 생성 ---
    bus = EtherCATBusSafe(
        adapter_name=adapter,
        num_slaves=NUM_MOTORS,
        cycle_time_ms=10,
        max_sync_error_mm=MAX_SYNC_ERROR_MM  # 안전 파라미터
    )

    # --- 각 모터에 대한 핸들 가져오기 ---
    motor1 = bus.motors[0]  # 슬레이브 인덱스 0 사용
    motor2 = bus.motors[1]  # 슬레이브 인덱스 1 사용

    try:
        # --- 축 설정 (버스 시작 전) ---
        motor1.set_axis('z')
        motor2.set_axis('z')

        # --- SDO 설정 (버스 시작 전) ---
        motor1.set_profile_velocity(rpm=50)
        motor1.set_profile_accel_decel(accel_rpm_per_sec=50)

        motor2.set_profile_velocity(rpm=50)
        motor2.set_profile_accel_decel(accel_rpm_per_sec=50)

        # --- 버스 시작 (안전 모드 활성화) ---
        bus.start()

        # --- 모터 준비 대기 ---
        for motor in [motor1, motor2]:
            print(f"모터 {motor._index} 준비 대기 중...")
            start_time = time.monotonic()
            while (motor.status_word & 0x006F) != 0x0027:
                time.sleep(0.05)
                if time.monotonic() - start_time > 5:
                    status = motor.status_word
                    print(f"[에러] 모터 {motor._index} 타임아웃! 현재 상태: 0x{status:04X}")
                    if status & 0x0008:
                        print(f"  → Fault 상태 감지됨!")
                    raise RuntimeError(f"모터 {motor._index}가 Operation Enabled 상태에 도달하지 못했습니다.")
            print(f"[완료] 모터 {motor._index} 준비 완료! ({time.monotonic() - start_time:.2f}초 소요)")

        # --- 원점 설정 ---
        motor1.set_origin()
        motor2.set_origin()
        time.sleep(0.5)

        print(f"\n원점 설정 후 위치:")
        print(f"  M1={motor1.current_position_mm:.2f}mm, M2={motor2.current_position_mm:.2f}mm")
        print(f"  위치 차이: {abs(motor1.current_position_mm - motor2.current_position_mm):.3f}mm")

        # --- 이동 전 상태 검증 ---
        print("\n--- 이동 전 모터 상태 검증 ---")
        for motor in [motor1, motor2]:
            status = motor.status_word
            print(f"모터 {motor._index}:")
            print(f"  Status Word: 0x{status:04X}")
            print(f"  Operation Enabled: {(status & 0x006F) == 0x0027}")
            print(f"  Fault: {bool(status & 0x0008)}")
            print(f"  Sync Error: {motor.has_sync_error}")  # 동기화 오류 상태 확인

            # Fault 상태 복구
            if status & 0x0008:
                print(f"[경고] 모터 {motor._index}가 Fault 상태입니다. 리셋을 시도합니다...")
                for attempt in range(3):
                    time.sleep(0.5)
                    status = motor.status_word
                    if not (status & 0x0008):
                        print(f"[성공] 모터 {motor._index} Fault 리셋 완료!")
                        break
                    print(f"  리셋 시도 {attempt + 1}/3...")
                else:
                    raise RuntimeError(f"모터 {motor._index} Fault 상태를 해제할 수 없습니다!")

        print("[검증 완료] 모든 모터가 정상 작동 가능 상태입니다.")

        # --- 동시 이동 시작 (안전 모드) ---
        print("\n" + "="*60)
        print("  [안전 동기화 모드] 동시 이동 시작")
        print(f"  허용 오차: {MAX_SYNC_ERROR_MM}mm")
        print("="*60)

        motor1.move_to_position_mm(-50)
        motor2.move_to_position_mm(-50)

        print("모터 이동 중...")
        time.sleep(0.2)

        start_time = time.monotonic()
        while motor1.is_moving() or motor2.is_moving():
            # 위치 및 동기화 상태 출력
            pos_diff = abs(motor1.current_position_mm - motor2.current_position_mm)
            print(f"--> M1: {motor1.current_position_mm:7.2f}mm, M2: {motor2.current_position_mm:7.2f}mm, 차이: {pos_diff:.3f}mm", end='\r')

            # [안전] 동기화 오류 감지
            if motor1.has_sync_error or motor2.has_sync_error:
                print(f"\n\n{'!'*60}")
                print(f"  [긴급 정지] 동기화 오류 감지!")
                print(f"  M1: {motor1.current_position_mm:.2f}mm")
                print(f"  M2: {motor2.current_position_mm:.2f}mm")
                print(f"  위치 차이: {pos_diff:.3f}mm (허용: {MAX_SYNC_ERROR_MM}mm)")
                print(f"{'!'*60}\n")
                break

            time.sleep(0.05)

            if time.monotonic() - start_time > 60:
                print("\n[경고] 타임아웃!")
                break

        # 이동 결과 확인
        print(f"\n\n--> 이동 결과:")
        print(f"    M1: {motor1.current_position_mm:.2f}mm, M2: {motor2.current_position_mm:.2f}mm")
        print(f"    위치 차이: {abs(motor1.current_position_mm - motor2.current_position_mm):.3f}mm")
        print(f"    동기화 오류: {motor1.has_sync_error or motor2.has_sync_error}")

        # 동기화 오류가 발생한 경우 복구 여부 확인
        if motor1.has_sync_error or motor2.has_sync_error:
            print("\n[경고] 동기화 오류로 인해 이동이 중단되었습니다.")
            print("원점 복귀를 위해 동기화 오류를 리셋합니다...")

            # 사용자 확인 (실제 운용 시에는 안전 확인 후 진행)
            time.sleep(2)

            # 동기화 오류 리셋
            bus.reset_sync_error()
            time.sleep(0.5)

            print(f"동기화 오류 상태: {motor1.has_sync_error or motor2.has_sync_error}")

        print("\n[완료] 이동 완료!")
        time.sleep(1)

        # --- 동시 원점 복귀 ---
        print("\n--- 동시 원점 복귀 시작 ---")
        motor1.move_to_position_mm(0)
        motor2.move_to_position_mm(0)

        print("모터 복귀 중...")
        time.sleep(0.2)

        start_time = time.monotonic()
        while motor1.is_moving() or motor2.is_moving():
            pos_diff = abs(motor1.current_position_mm - motor2.current_position_mm)
            print(f"--> M1: {motor1.current_position_mm:7.2f}mm, M2: {motor2.current_position_mm:7.2f}mm, 차이: {pos_diff:.3f}mm", end='\r')

            # [안전] 동기화 오류 감지
            if motor1.has_sync_error or motor2.has_sync_error:
                print(f"\n\n[긴급 정지] 복귀 중 동기화 오류 감지!")
                break

            time.sleep(0.05)

            if time.monotonic() - start_time > 60:
                print("\n[경고] 타임아웃!")
                break

        print(f"\n\n--> 복귀 결과:")
        print(f"    M1: {motor1.current_position_mm:.2f}mm, M2: {motor2.current_position_mm:.2f}mm")
        print(f"    위치 차이: {abs(motor1.current_position_mm - motor2.current_position_mm):.3f}mm")
        print(f"[디버그] 최종 펄스 - M1: {motor1.current_position_pulse}, M2: {motor2.current_position_pulse}")

        print("\n[완료] 원점 복귀 완료!")
        print("\n" + "="*60)
        print("  모든 작업 완료 (안전 동기화 모드)")
        print("="*60)
        time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n사용자에 의해 프로그램이 중단되었습니다.")
    except Exception as e:
        print(f"\n\n메인 루프에서 에러 발생: {e}")
    finally:
        # --- 버스 종료 ---
        bus.stop()


if __name__ == "__main__":
    main()
