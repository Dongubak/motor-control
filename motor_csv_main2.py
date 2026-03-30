"""
motor_csv_main2.py - CSV 모드 Cross Coupling 실운용 파일

타겟 위치 이동과 원점 복귀 모두 Cross Coupling을 적용한다.
(motor_csv_main.py의 테스트 비교 구조를 제거하고 실운용에 맞게 정리)
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


def wait_motors(motor1, motor2, label: str, timeout: float = 60.0):
    """두 모터가 모두 이동 완료될 때까지 실시간 상태를 출력하며 대기."""
    max_diff = 0.0
    t0 = time.monotonic()

    while motor1.is_moving() or motor2.is_moving():
        p1   = motor1.current_position_mm
        p2   = motor2.current_position_mm
        v1   = motor1.current_velocity_mm_s
        v2   = motor2.current_velocity_mm_s
        diff = abs(p1 - p2)
        max_diff = max(max_diff, diff)

        print(f"\r  [{label}]  "
              f"M1={p1:7.2f}mm ({v1:+5.2f}mm/s)  "
              f"M2={p2:7.2f}mm ({v2:+5.2f}mm/s)  "
              f"차이={diff:.3f}mm", end='')

        if motor1.has_sync_error or motor2.has_sync_error:
            print(f"\n[긴급 정지] 모터 이상 감지 (Fault 또는 동기화 오류)!")
            return max_diff, True

        if time.monotonic() - t0 > timeout:
            print("\n[타임아웃]")
            return max_diff, False

        time.sleep(0.05)

    print()
    # 루프 탈출 직전에 Fault가 발생했을 경우 재확인
    if motor1.has_sync_error or motor2.has_sync_error:
        return max_diff, True
    return max_diff, False


# ─────────────────────────────────────────────────────────────
# 메인
# ─────────────────────────────────────────────────────────────
def main():
    # ── 장치 설정 ──
    ADAPTER    = r'\Device\NPF_{CD2150F2-B355-4A6F-95BA-EB897A3726BF}' # 현장
    # ADAPTER = r'\Device\NPF_{D1B66F5F-FF8A-4D4D-8C7C-2FF2547CE945}'

    #### 현장에서 확인할 변수 목록 ####
    # NUM_MOTORS
    # RPM
    # ACCEL_RPM_PER_S
    # TARGET_MM

    # NUM_MOTORS = 3 # 현장
    NUM_MOTORS = 3

    # ── CSV 모드 파라미터 ──


    ###### 현장 용 ######
    RPM             = 500    # 최대 이동 속도 (RPM)
    ACCEL_RPM_PER_S = 500    # 가감속 (RPM/sec)
    TARGET_MM       = 1000    # 이동 목표 (mm)  ※ z축 이동 한계 ~12.89mm 이내로 설정


    ###### 사무실 내 시연 용 ######

    # RPM             = 10    # 최대 이동 속도 (RPM)
    # ACCEL_RPM_PER_S = 10    # 가감속 (RPM/sec)
    # TARGET_MM       = 10    # 이동 목표 (mm)  ※ z축 이동 한계 ~12.89mm 이내로 설정

    # ── Cross Coupling 파라미터 ──
    COUPLING_GAIN   = 0.01  # [1/sec]: 1mm 오차 → 0.01mm/sec 속도 보정
    ENABLE_COUPLING = True
    MAX_SYNC_ERR_MM = 10.0  # 긴급 정지 임계값 (mm)
    MA_WINDOW       = 5     # 이동 평균 윈도우

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

    ###### 현장 시연용 #####
    motor1 = bus.motors[1]
    motor2 = bus.motors[2]

    #### 사무실 시연 용 #####
    # motor1 = bus.motors[0]
    # motor2 = bus.motors[1]

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
        # 1단계: Cross Coupling ON — 타겟 위치로 이동
        # ════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print(f"  [1단계] 타겟 이동  (Cross Coupling ON, Kc={COUPLING_GAIN})")
        print(f"  목표: -{TARGET_MM}mm")
        print("=" * 60)

        bus.coupling_enabled = True
        bus.coupling_gain    = COUPLING_GAIN

        motor1.move_to_position_mm(TARGET_MM)
        motor2.move_to_position_mm(TARGET_MM)
        time.sleep(0.2)

        max_diff_fwd, err_fwd = wait_motors(motor1, motor2, "전진", timeout=60.0)

        print(f"\n[1단계 결과]")
        print(f"  최대 위치 차이: {max_diff_fwd:.3f}mm")
        print(f"  최종 위치: M1={motor1.current_position_mm:.2f}mm, "
              f"M2={motor2.current_position_mm:.2f}mm")

        if err_fwd:
            print("[경고] 이상 발생으로 중단됨 (Fault 또는 동기화 오류). 원점 복귀를 건너뜁니다.")
        else:
            time.sleep(1.0)

            # ════════════════════════════════════════════════════
            # 2단계: Cross Coupling ON — 원점 복귀
            # ════════════════════════════════════════════════════
            print("\n" + "=" * 60)
            print(f"  [2단계] 원점 복귀  (Cross Coupling ON, Kc={COUPLING_GAIN})")
            print("=" * 60)

            bus.coupling_enabled = True

            motor1.move_to_position_mm(TARGET_MM)
            motor2.move_to_position_mm(TARGET_MM)
            time.sleep(0.2)

            max_diff_ret, _ = wait_motors(motor1, motor2, "복귀", timeout=60.0)

            print(f"\n[2단계 결과]")
            print(f"  최대 위치 차이: {max_diff_ret:.3f}mm")
            print(f"  최종 위치: M1={motor1.current_position_mm:.2f}mm, "
                  f"M2={motor2.current_position_mm:.2f}mm")

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
