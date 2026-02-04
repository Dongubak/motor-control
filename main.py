from motor import EtherCATBus
import time

def main():
    # --- 장치 포트 및 슬레이브 수 설정 ---

    # adapter = r'\Device\NPF_{F8EF3044-171D-407C-A166-9EEA524F051C}' < 이전 변수 값임
    
    adapter = r'\Device\NPF_{A3C1307F-C4D5-4126-8FCF-A191BF2B1257}'
    NUM_MOTORS = 1

    # --- 단일 버스 관리자 생성 ---
    # 사이클 타임을 10ms로 단축하여 더 부드러운 움직임 구현
    bus = EtherCATBus(adapter_name=adapter, num_slaves=NUM_MOTORS, cycle_time_ms=10)
    
    # --- 각 모터에 대한 핸들 가져오기 ---
    motor1 = bus.motors[0]  # 슬레이브 인덱스 0 사용

    # [2개 모터로 확장 시]
    # NUM_MOTORS = 2로 변경하고 아래 주석 해제
    # motor2 = bus.motors[1]

    try:
        # --- 축 설정 (버스 시작 전) ---
        motor1.set_axis('z')
        # motor2.set_axis('z')  # 2개 모터 사용 시 주석 해제

        # --- SDO 설정 (버스 시작 전) ---
        # 버스가 OP 상태에 들어가기 전에 속도, 가감속도 등 파라미터를 설정합니다.
        motor1.set_profile_velocity(rpm=50)  # 속도는 50 RPM
        motor1.set_profile_accel_decel(accel_rpm_per_sec=50)  # 가감속도를 50으로 낮춤 (부드러운 움직임)

        # motor2.set_profile_velocity(rpm=50)  # 2개 모터 사용 시 주석 해제
        # motor2.set_profile_accel_decel(accel_rpm_per_sec=50)

        # --- 버스 시작 (모든 모터에 대한 통신 시작) ---
        bus.start()
        
        # --- 사용할 모터만 준비 대기 ---
        for motor in [motor1]:  # 2개 모터 사용 시: [motor1, motor2]
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
        # motor2.set_origin()  # 2개 모터 사용 시 주석 해제
        time.sleep(0.5)  # 원점 설정 명령이 처리될 충분한 시간 제공

        print(f"원점 설정 후 위치: M1={motor1.current_position_mm:.2f}mm")
        print(f"[디버그] 펄스 정보 - M1: {motor1.current_position_pulse}")
        print(f"[디버그] 오프셋 정보 - M1: {motor1.offset_pulse}")
        print(f"[디버그] 펄스-오프셋 차이 - M1: {motor1.current_position_pulse - motor1.offset_pulse}")

        # --- 이동 전 상태 검증 및 복구 ---
        print("\n--- 이동 전 모터 상태 검증 ---")
        for motor in [motor1]:  # 2개 모터 사용 시: [motor1, motor2]
            status = motor.status_word
            print(f"모터 {motor._index} 상태:")
            print(f"  Status Word: 0x{status:04X}")
            print(f"  Operation Enabled: {(status & 0x006F) == 0x0027}")
            print(f"  Fault: {bool(status & 0x0008)}")
            print(f"  현재 위치: {motor.current_position_pulse} pulse")

            # Fault 상태 감지 및 복구 시도
            if status & 0x0008:
                print(f"[경고] 모터 {motor._index}가 Fault 상태입니다. 리셋을 시도합니다...")

                # Fault 리셋 시도 (최대 3회)
                for attempt in range(3):
                    time.sleep(0.5)  # Fault 리셋 대기
                    status = motor.status_word

                    if not (status & 0x0008):
                        print(f"[성공] 모터 {motor._index} Fault 리셋 완료!")
                        break

                    print(f"  리셋 시도 {attempt + 1}/3...")
                    time.sleep(0.5)
                else:
                    # 3회 시도 후에도 Fault가 해제되지 않으면 종료
                    print(f"[에러] 모터 {motor._index} Fault 리셋 실패! 안전하게 종료합니다.")
                    raise RuntimeError(f"모터 {motor._index} Fault 상태를 해제할 수 없습니다! (status=0x{status:04X})")

                # Fault 리셋 후 Operation Enabled 상태 대기
                print(f"모터 {motor._index} Operation Enabled 상태 복구 대기 중...")
                start_time = time.monotonic()
                while (motor.status_word & 0x006F) != 0x0027:
                    time.sleep(0.05)
                    if time.monotonic() - start_time > 3:
                        print(f"[에러] 모터 {motor._index}가 Operation Enabled 상태로 복구되지 않았습니다!")
                        raise RuntimeError(f"모터 {motor._index} 상태 복구 실패! (status=0x{motor.status_word:04X})")
                print(f"[완료] 모터 {motor._index} 정상 상태로 복구되었습니다.")

            # Operation Enabled 상태 확인
            if (status & 0x006F) != 0x0027:
                print(f"[에러] 모터 {motor._index}가 Operation Enabled 상태가 아닙니다!")
                raise RuntimeError(f"모터 {motor._index} 상태 이상! (status=0x{status:04X})")

        print("[검증 완료] 모든 모터가 정상 작동 가능 상태입니다.")

        # --- 이동 시작 (CSP 모드) ---
        print("\n--- 이동 시작 (CSP 모드) ---")
        # CSP 모드에서는 명령을 보내면 궤적이 자동으로 생성됩니다.
        motor1.move_to_position_mm(-50)
        # motor2.move_to_position_mm(-50)  # 2개 모터 사용 시 주석 해제

        # 이동이 완료될 때까지 대기 (is_moving()이 False가 될 때까지)
        print("모터 이동 중...")
        time.sleep(0.2)  # 명령 처리 대기

        start_time = time.monotonic()
        while motor1.is_moving():  # 2개 모터 사용 시: motor1.is_moving() or motor2.is_moving()
            print(f"--> 이동 중... M1: {motor1.current_position_mm:.2f} mm", end='\r')
            time.sleep(0.05)

            if time.monotonic() - start_time > 60:
                print("\n[경고] 타임아웃: 60초 내에 이동이 완료되지 않았습니다.")
                break

        print(f"\n--> 이동 완료! M1: {motor1.current_position_mm:.2f} mm")

        print("\n[완료] 이동 완료!")
        time.sleep(1)

        # --- 원점 복귀 (CSP 모드) ---
        print("\n--- 원점 복귀 시작 (CSP 모드) ---")
        # 원점 복귀 명령을 보냅니다.
        motor1.move_to_position_mm(0)
        # motor2.move_to_position_mm(0)  # 2개 모터 사용 시 주석 해제

        # 이동이 완료될 때까지 대기
        print("모터 복귀 중...")
        time.sleep(0.2)  # 명령 처리 대기

        start_time = time.monotonic()
        while motor1.is_moving():  # 2개 모터 사용 시: motor1.is_moving() or motor2.is_moving()
            print(f"--> 복귀 중... M1: {motor1.current_position_mm:.2f} mm", end='\r')
            time.sleep(0.05)

            if time.monotonic() - start_time > 60:
                print("\n[경고] 타임아웃: 60초 내에 복귀가 완료되지 않았습니다.")
                break

        print(f"\n--> 복귀 완료! M1: {motor1.current_position_mm:.2f} mm")
        print(f"[디버그] 최종 펄스 - M1: {motor1.current_position_pulse}")
        print(f"[디버그] 오프셋 펄스 - M1: {motor1.offset_pulse}")
        print(f"[디버그] 상대 펄스 - M1: {motor1.current_position_pulse - motor1.offset_pulse}")

        print("\n[완료] 원점 복귀 완료!")
        print("\n--- 모든 작업 완료 ---")
        time.sleep(0.5)  # Fault 발생 방지를 위해 2초에서 0.5초로 단축

    except KeyboardInterrupt:
        print("\n사용자에 의해 프로그램이 중단되었습니다.")
    except Exception as e:
        print(f"\n메인 루프에서 에러 발생: {e}")
    finally:
        # --- 버스 종료 (모든 모터 통신 종료) ---
        bus.stop()

if __name__ == "__main__":
    main()
