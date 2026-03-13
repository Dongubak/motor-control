"""
motor_initial_setting.py - Z축 모터 수동 이동 스크립트

[대상 모터]
  모터 1 (slave index 1) : Z축
  모터 2 (slave index 2) : Z축
  모터 0 (X축)           : 이동 불가

[명령어]
  m 1 <mm>  : 모터 1을 지정 거리(mm)만큼 상대 이동   예) m 1 1.0
  m 2 <mm>  : 모터 2를 지정 거리(mm)만큼 상대 이동   예) m 2 -1.0
  s         : 현재 위치 출력
  q         : 종료
"""

from motor import EtherCATBus
import time

# ─── 설정 ──────────────────────────────────────────────────────────────────
ADAPTER      = r'\Device\NPF_{CD2150F2-B355-4A6F-95BA-EB897A3726BF}'
NUM_MOTORS   = 3
CYCLE_MS     = 10

PROFILE_VELOCITY_RPM    = 30   # rpm
PROFILE_ACCEL_RPM_PER_S = 30   # rpm/s

# 이동 가능한 모터 인덱스 (main_dual_coupling.py와 동일)
ALLOWED_MOTOR_INDICES = {1, 2}

AXIS_SETTINGS = {
    0: 'x',  # 통신만 유지, 이동 불가
    1: 'z',
    2: 'z',
}
# ──────────────────────────────────────────────────────────────────────────


def wait_for_ready(motor, timeout=10.0):
    start = time.monotonic()
    while (motor.status_word & 0x006F) != 0x0027:
        time.sleep(0.05)
        if time.monotonic() - start > timeout:
            raise RuntimeError(
                f"모터 {motor._index} 준비 타임아웃! "
                f"Status=0x{motor.status_word:04X}"
            )


def move_motor_by(motor, delta_mm):
    """delta_mm 만큼 상대 이동하고 완료까지 대기."""
    current = motor.current_position_mm
    target  = current + delta_mm
    print(f"  [이동] 모터 {motor._index}: {current:+.3f}mm → {target:+.3f}mm  (Δ{delta_mm:+.3f}mm)")
    motor.move_to_position_mm(target)

    time.sleep(0.15)
    timeout = time.monotonic() + 60.0
    while motor.is_moving():
        print(f"  --> M{motor._index}: {motor.current_position_mm:+8.3f}mm", end='\r')
        time.sleep(0.05)
        if time.monotonic() > timeout:
            print("\n  [경고] 이동 타임아웃!")
            return
    print(f"  --> M{motor._index}: {motor.current_position_mm:+8.3f}mm  [완료]")


def parse_command(line, motors):
    """반환값: True(계속), False(종료)"""
    parts = line.strip().split()
    if not parts:
        return True

    cmd = parts[0].lower()

    if cmd == 'q':
        return False

    if cmd == 's':
        for idx in sorted(ALLOWED_MOTOR_INDICES):
            print(f"  모터 {idx}: {motors[idx].current_position_mm:+8.3f} mm")
        print()
        return True

    if cmd == 'm':
        if len(parts) < 3:
            print("  사용법: m <모터번호> <거리mm>   예) m 1 1.0  /  m 2 -1.0")
            return True
        try:
            idx      = int(parts[1])
            delta_mm = float(parts[2])
        except ValueError:
            print("  [오류] 숫자 형식이 올바르지 않습니다.")
            return True

        if idx not in ALLOWED_MOTOR_INDICES:
            print(f"  [오류] 이동 가능한 모터는 {sorted(ALLOWED_MOTOR_INDICES)} 입니다. (모터 0은 이동 불가)")
            return True

        move_motor_by(motors[idx], delta_mm)
        return True

    print(f"  [알 수 없는 명령] '{cmd}'  →  m / s / q 중 하나를 입력하세요.")
    return True


HELP_TEXT = f"""
─────────────────────────────────────────────────────
  m 1 <mm>   모터 1 상대 이동   예) m 1 1.0  /  m 1 -1.0
  m 2 <mm>   모터 2 상대 이동   예) m 2 1.0  /  m 2 -1.0
  s          현재 위치 출력
  q          종료
─────────────────────────────────────────────────────
"""


def main():
    print("=" * 50)
    print("  Z축 모터 수동 이동")
    print(f"  대상: 모터 {sorted(ALLOWED_MOTOR_INDICES)}   속도: {PROFILE_VELOCITY_RPM} RPM")
    print("=" * 50)

    bus = EtherCATBus(
        adapter_name=ADAPTER,
        num_slaves=NUM_MOTORS,
        cycle_time_ms=CYCLE_MS,
    )
    motors = bus.motors

    try:
        for idx, axis in AXIS_SETTINGS.items():
            motors[idx].set_axis(axis)
            motors[idx].set_profile_velocity(rpm=PROFILE_VELOCITY_RPM)
            motors[idx].set_profile_accel_decel(accel_rpm_per_sec=PROFILE_ACCEL_RPM_PER_S)

        bus.start()

        print("\n모터 준비 대기 중...")
        for idx in sorted(ALLOWED_MOTOR_INDICES):
            wait_for_ready(motors[idx])
            print(f"  [완료] 모터 {idx} 준비 완료  (Status=0x{motors[idx].status_word:04X})")

        print(HELP_TEXT)

        while True:
            try:
                line = input("명령> ").strip()
            except EOFError:
                break
            if not parse_command(line, motors):
                break

    except KeyboardInterrupt:
        print("\n\n사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"\n에러 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        bus.stop()
        print("종료 완료.")


if __name__ == "__main__":
    main()
