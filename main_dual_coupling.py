"""
main_dual_coupling.py - Cross Coupling 적용 이동 및 원점 복귀

Cross Coupling을 활성화한 상태로 목표 위치까지 이동 후 원점으로 복귀합니다.

사용법:
    python main_dual_coupling.py
"""

from motor_coupling import EtherCATBusCoupling
import time
import pysoem
import struct

# CiA 402 표준 에러 코드 (0x603F / 0x1003)
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
    """에러 코드를 사람이 읽을 수 있는 이름으로 변환."""
    if code in _FAULT_NAMES:
        return _FAULT_NAMES[code]
    # 상위 바이트 마스크로 분류 검색 (0x0000 폴백 방지: 비표준 코드를 "에러 없음"으로 오표시)
    masked = code & 0xFF00
    if masked != 0:
        return _FAULT_NAMES.get(masked, f"제조사 특정 오류 (0x{code:04X})")
    return f"제조사 특정 오류 (0x{code:04X})"


def read_drive_fault_codes(adapter: str, num_slaves: int):
    """
    버스 종료 후 SDO로 각 드라이브의 Fault 코드를 읽어 출력한다.
    - 0x603F : 현재(마지막) 에러 코드
    - 0x1003 : 에러 이력 (최대 3개)
    """
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
                # 현재 에러 코드
                code = struct.unpack("<H", slave.sdo_read(0x603F, 0))[0]
                print(f"    현재 Error Code (0x603F) : 0x{code:04X}  →  {_fault_name(code)}")
            except Exception as e:
                print(f"    0x603F 읽기 실패: {e}")

            try:
                # 에러 이력 개수
                num_errors = struct.unpack("<B", slave.sdo_read(0x1003, 0))[0]
                if num_errors == 0:
                    print(f"    에러 이력 (0x1003)     : 없음")
                else:
                    print(f"    에러 이력 (0x1003)     : {num_errors}개")
                    for j in range(1, min(num_errors + 1, 4)):
                        hist = struct.unpack("<I", slave.sdo_read(0x1003, j))[0]
                        std_code = hist & 0xFFFF          # 하위 16비트: 표준 에러 코드
                        mfr_code = (hist >> 16) & 0xFFFF  # 상위 16비트: 제조사 코드
                        print(f"      [{j}] std=0x{std_code:04X} ({_fault_name(std_code)})"
                              f",  mfr=0x{mfr_code:04X}")
            except Exception as e:
                print(f"    0x1003 읽기 실패: {e}")

        print("=" * 60)
    except Exception as e:
        print(f"[Fault 조회 실패] {e}")
    finally:
        master.close()


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
    adapter = r'\Device\NPF_{CD2150F2-B355-4A6F-95BA-EB897A3726BF}'  # Realtek USB 2.5GbE
    NUM_MOTORS = 3

    # --- 이동 목표 위치 (mm) ---
    TARGET_MM = 10

    # --- Cross Coupling 설정 ---
    COUPLING_GAIN = 0.001     # 초기 게인 (0.0 ~ 1.0)
    ENABLE_COUPLING = True      # Cross Coupling 활성화
    MAX_SYNC_ERROR_MM = 10     # 동기화 오차 허용 범위 (mm)
    MA_WINDOW = 5               # 이동 평균 윈도우 (10ms × 5 = 50ms 평균)
    TARGET_MM = 6

    # --- 버스 생성 ---
    bus = EtherCATBusCoupling(
        adapter_name=adapter,
        num_slaves=NUM_MOTORS,
        cycle_time_ms=10,
        max_sync_error_mm=MAX_SYNC_ERROR_MM,
        coupling_gain=COUPLING_GAIN,
        enable_coupling=ENABLE_COUPLING,
        ma_window=MA_WINDOW
    )

    motor1 = bus.motors[1]
    motor2 = bus.motors[2]

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

        # 테스트 1: Cross Coupling 활성화 상태로 이동
        print(f"\n--- 테스트 1: Cross Coupling ON (Kc={COUPLING_GAIN}) ---")
        bus.coupling_enabled = True
        bus.coupling_gain = COUPLING_GAIN

        motor1.move_to_position_mm(TARGET_MM)
        motor2.move_to_position_mm(TARGET_MM)

        time.sleep(0.2)
        max_diff_test1 = 0

        start_time = time.monotonic()
        while motor1.is_moving() or motor2.is_moving():
            pos_diff = abs(motor1.current_position_mm - motor2.current_position_mm)
            max_diff_test1 = max(max_diff_test1, pos_diff)
            print(f"--> M1: {motor1.current_position_mm:7.2f}mm, M2: {motor2.current_position_mm:7.2f}mm, 차이: {pos_diff:.3f}mm", end='\r')

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
        motor1.move_to_position_mm(TARGET_MM)
        motor2.move_to_position_mm(TARGET_MM)

        time.sleep(0.2)
        while motor1.is_moving() or motor2.is_moving():
            print(f"--> M1: {motor1.current_position_mm:7.2f}mm, M2: {motor2.current_position_mm:7.2f}mm", end='\r')
            if motor1.has_sync_error:
                break
            time.sleep(0.05)

        print(f"\n원점 복귀 완료: M1={motor1.current_position_mm:.2f}mm, M2={motor2.current_position_mm:.2f}mm")
        time.sleep(1)

        print(f"\n")

        print("\n" + "="*60)
        print("  [완료]")
        print(f"  이동 시 최종 위치차: {max_diff_go:.3f}mm")
        print(f"  복귀 시 최종 위치차: {max_diff_ret:.3f}mm")
        print("="*60)
        print(f"  테스트 1 (Coupling ON):  최대 위치차 {max_diff_test1:.3f}mm")

        print("="*60)
        time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"\n\n에러 발생: {e}")
    finally:
        bus.stop()
        time.sleep(0.5)  # 서브프로세스가 어댑터를 해제할 때까지 대기
        read_drive_fault_codes(adapter, NUM_MOTORS)


if __name__ == "__main__":
    main()
