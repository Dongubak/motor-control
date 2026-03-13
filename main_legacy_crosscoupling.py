## main_legacy.py 기반 + Cross Coupling 추가
## Safe 기능 없음, Fault 복구 로직 포함

from motor_coupling import EtherCATBusCoupling
import time


def main():
    # --- 장치 포트 및 슬레이브 수 설정 ---
    adapter = r'\Device\NPF_{CD2150F2-B355-4A6F-95BA-EB897A3726BF}'

    NUM_MOTORS = 3

    # --- 이동 목표 위치 (mm) ---
    TARGET_MM = 1

    # --- Cross Coupling 설정 ---
    COUPLING_GAIN     = 0.02    # 게인 (0.0 ~ 1.0)
    MAX_SYNC_ERROR_MM = 1000   # 동기화 오차 허용 범위 (mm) — safe 기능 사실상 비활성

    # --- Cross Coupling 버스 생성 ---
    bus = EtherCATBusCoupling(
        adapter_name=adapter,
        num_slaves=NUM_MOTORS,
        cycle_time_ms=10,
        max_sync_error_mm=MAX_SYNC_ERROR_MM,
        coupling_gain=COUPLING_GAIN,
        enable_coupling=True,
    )

    motor1 = bus.motors[1]
    motor2 = bus.motors[2]

    try:
        # --- 축 설정 (버스 시작 전) ---
        motor1.set_axis('z')
        motor2.set_axis('z')

        # --- SDO 설정 (버스 시작 전) ---
        motor1.set_profile_velocity(rpm=20)
        motor1.set_profile_accel_decel(accel_rpm_per_sec=20)

        motor2.set_profile_velocity(rpm=20)
        motor2.set_profile_accel_decel(accel_rpm_per_sec=20)

        # --- 버스 시작 ---
        bus.start()

        # --- 사용할 모터만 준비 대기 ---
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

        print(f"원점 설정 후 위치: M1={motor1.current_position_mm:.2f}mm, M2={motor2.current_position_mm:.2f}mm")
        print(f"[디버그] 펄스 정보 - M1: {motor1.current_position_pulse}, M2: {motor2.current_position_pulse}")
        print(f"[디버그] 오프셋 정보 - M1: {motor1.offset_pulse}, M2: {motor2.offset_pulse}")
        print(f"[디버그] 펄스-오프셋 차이 - M1: {motor1.current_position_pulse - motor1.offset_pulse}, M2: {motor2.current_position_pulse - motor2.offset_pulse}")

        # --- 이동 전 상태 검증 및 복구 ---
        print("\n--- 이동 전 모터 상태 검증 ---")
        for motor in [motor1, motor2]:
            status = motor.status_word
            print(f"모터 {motor._index} 상태:")
            print(f"  Status Word: 0x{status:04X}")
            print(f"  Operation Enabled: {(status & 0x006F) == 0x0027}")
            print(f"  Fault: {bool(status & 0x0008)}")
            print(f"  현재 위치: {motor.current_position_pulse} pulse")

            if status & 0x0008:
                print(f"[경고] 모터 {motor._index}가 Fault 상태입니다. 리셋을 시도합니다...")
                for attempt in range(3):
                    time.sleep(0.5)
                    status = motor.status_word
                    if not (status & 0x0008):
                        print(f"[성공] 모터 {motor._index} Fault 리셋 완료!")
                        break
                    print(f"  리셋 시도 {attempt + 1}/3...")
                    time.sleep(0.5)
                else:
                    print(f"[에러] 모터 {motor._index} Fault 리셋 실패! 안전하게 종료합니다.")
                    raise RuntimeError(f"모터 {motor._index} Fault 상태를 해제할 수 없습니다! (status=0x{status:04X})")

                print(f"모터 {motor._index} Operation Enabled 상태 복구 대기 중...")
                start_time = time.monotonic()
                while (motor.status_word & 0x006F) != 0x0027:
                    time.sleep(0.05)
                    if time.monotonic() - start_time > 3:
                        raise RuntimeError(f"모터 {motor._index} 상태 복구 실패! (status=0x{motor.status_word:04X})")
                print(f"[완료] 모터 {motor._index} 정상 상태로 복구되었습니다.")

            if (status & 0x006F) != 0x0027:
                raise RuntimeError(f"모터 {motor._index} 상태 이상! (status=0x{status:04X})")

        print("[검증 완료] 모든 모터가 정상 작동 가능 상태입니다.")
        print(f"[Cross Coupling] 게인: {bus.coupling_gain:.2f}  활성화: {bus.coupling_enabled}")

        # --- 동시 이동 시작 (Cross Coupling + CSP 모드) ---
        print(f"\n--- 동시 이동 시작 (Cross Coupling ON, 목표: {TARGET_MM}mm) ---")
        motor1.move_to_position_mm(TARGET_MM)
        motor2.move_to_position_mm(TARGET_MM)

        print("모터 이동 중...")
        time.sleep(0.2)

        start_time = time.monotonic()
        while motor1.is_moving() or motor2.is_moving():
            pos_diff = abs(motor1.current_position_mm - motor2.current_position_mm)
            print(f"--> 이동 중... M1: {motor1.current_position_mm:7.2f}mm, M2: {motor2.current_position_mm:7.2f}mm, 차이: {pos_diff:.3f}mm", end='\r')
            time.sleep(0.05)
            if time.monotonic() - start_time > 60:
                print("\n[경고] 타임아웃: 60초 내에 이동이 완료되지 않았습니다.")
                break

        final_diff_go = abs(motor1.current_position_mm - motor2.current_position_mm)
        print(f"\n--> 이동 완료! M1: {motor1.current_position_mm:.2f}mm, M2: {motor2.current_position_mm:.2f}mm")
        print(f"    위치 차이: {final_diff_go:.3f}mm")

        print("\n[완료] 동시 이동 완료!")
        time.sleep(1)

        # --- 동시 원점 복귀 (Cross Coupling + CSP 모드) ---
        print("\n--- 동시 원점 복귀 시작 (Cross Coupling ON) ---")
        motor1.move_to_position_mm(0)
        motor2.move_to_position_mm(0)

        print("모터 복귀 중...")
        time.sleep(0.2)

        start_time = time.monotonic()
        while motor1.is_moving() or motor2.is_moving():
            pos_diff = abs(motor1.current_position_mm - motor2.current_position_mm)
            print(f"--> 복귀 중... M1: {motor1.current_position_mm:7.2f}mm, M2: {motor2.current_position_mm:7.2f}mm, 차이: {pos_diff:.3f}mm", end='\r')
            time.sleep(0.05)
            if time.monotonic() - start_time > 60:
                print("\n[경고] 타임아웃: 60초 내에 복귀가 완료되지 않았습니다.")
                break

        final_diff_ret = abs(motor1.current_position_mm - motor2.current_position_mm)
        print(f"\n--> 복귀 완료! M1: {motor1.current_position_mm:.2f}mm, M2: {motor2.current_position_mm:.2f}mm")
        print(f"    위치 차이: {final_diff_ret:.3f}mm")
        print(f"[디버그] 최종 펄스 - M1: {motor1.current_position_pulse}, M2: {motor2.current_position_pulse}")
        print(f"[디버그] 오프셋 펄스 - M1: {motor1.offset_pulse}, M2: {motor2.offset_pulse}")
        print(f"[디버그] 상대 펄스 - M1: {motor1.current_position_pulse - motor1.offset_pulse}, M2: {motor2.current_position_pulse - motor2.offset_pulse}")

        print("\n[완료] 원점 복귀 완료!")
        print(f"\n  이동 시 최종 위치차: {final_diff_go:.3f}mm")
        print(f"  복귀 시 최종 위치차: {final_diff_ret:.3f}mm")
        print("\n--- 모든 작업 완료 ---")
        time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n사용자에 의해 프로그램이 중단되었습니다.")
    except Exception as e:
        print(f"\n메인 루프에서 에러 발생: {e}")
    finally:
        bus.stop()


if __name__ == "__main__":
    main()
