"""
motor_initial_setting.py - 모터 1/2 원점 정렬 스크립트

[목적]
  모터 1의 현재 위치를 기준(원점)으로 설정하고,
  모터 2를 모터 1과 같은 위치가 되도록 수동 이동합니다.

[절차]
  1. 스크립트 실행 → 모터 준비 완료 대기
  2. 'ref' 입력 → 모터 1의 현재 위치를 기준으로 설정
  3. 'm 2 <거리mm>' 으로 모터 2 이동 → 이동 중/후 차이 실시간 표시
  4. 차이가 0에 수렴하면 'done' 입력 → 두 모터 원점 설정 후 종료

[명령어]
  ref             : 모터 1 현재 위치를 기준 원점으로 설정
  m <번호> <mm>  : 해당 모터를 지정 거리(mm)만큼 상대 이동
  s               : 현재 위치 및 차이 출력
  done            : 모터 2 원점 설정 (정렬 완료 후 종료)
  q               : 종료
"""

from motor import EtherCATBus, PULSES_PER_REVOLUTION, z_axis_mm_per_rev
import time

# ─── 설정 ──────────────────────────────────────────────────────────────────
ADAPTER      = r'\Device\NPF_{CD2150F2-B355-4A6F-95BA-EB897A3726BF}'
NUM_MOTORS   = 3
CYCLE_MS     = 10

AXIS_SETTINGS = {
    0: 'x',
    1: 'z',
    2: 'z',
}

PROFILE_VELOCITY_RPM    = 10   # rpm  (낮을수록 안전)
PROFILE_ACCEL_RPM_PER_S = 10   # rpm/s

# 정렬 대상 모터 인덱스
MOTOR_REF_IDX    = 1   # 기준 모터 (원점이 될 모터)
MOTOR_TARGET_IDX = 2   # 맞춰야 할 모터
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


def calc_diff_mm(motor_target, ref_pulse):
    """
    모터2(target)가 기준 위치(ref_pulse)에서 얼마나 떨어져 있는지 반환합니다.
    양수: 모터2가 기준보다 앞쪽 (음수 방향으로 이동 필요)
    음수: 모터2가 기준보다 뒤쪽 (양수 방향으로 이동 필요)
    """
    diff_pulse = motor_target.current_position_pulse - ref_pulse
    return diff_pulse / (PULSES_PER_REVOLUTION * 2) * z_axis_mm_per_rev


def print_status(motor_ref, motor_target, ref_pulse):
    """현재 위치와 정렬 차이를 출력합니다."""
    print(f"\n  모터 {motor_ref._index} (기준): {motor_ref.current_position_mm:+8.3f} mm")
    print(f"  모터 {motor_target._index} (정렬): {motor_target.current_position_mm:+8.3f} mm")
    if ref_pulse is not None:
        diff = calc_diff_mm(motor_target, ref_pulse)
        print(f"  차이 (M{motor_target._index} - M{motor_ref._index} 기준): {diff:+8.3f} mm", end="")
        if abs(diff) < 0.05:
            print("  ← 정렬 완료 수준")
        elif abs(diff) < 0.5:
            print("  ← 거의 정렬됨")
        else:
            direction = "음수(-)" if diff > 0 else "양수(+)"
            print(f"  ← 모터 {motor_target._index}를 {direction} 방향으로 이동")
    else:
        print("  [차이] 기준 미설정 ('ref' 명령 실행 필요)")
    print()


def move_motor_by(motor, delta_mm, ref_pulse=None, motor_ref_idx=None):
    """delta_mm 만큼 상대 이동. ref_pulse가 있으면 이동 중 차이를 실시간 표시."""
    current = motor.current_position_mm
    target  = current + delta_mm
    print(f"  [이동] 모터 {motor._index}: {current:+.3f}mm → {target:+.3f}mm  (Δ{delta_mm:+.3f}mm)")
    motor.move_to_position_mm(target)

    time.sleep(0.15)
    timeout = time.monotonic() + 60.0
    while motor.is_moving():
        if ref_pulse is not None and motor._index != motor_ref_idx:
            diff = calc_diff_mm(motor, ref_pulse)
            print(f"  --> M{motor._index}: {motor.current_position_mm:+8.3f}mm  |  "
                  f"기준과 차이: {diff:+8.3f}mm", end='\r')
        else:
            print(f"  --> M{motor._index}: {motor.current_position_mm:+8.3f}mm", end='\r')
        time.sleep(0.05)
        if time.monotonic() > timeout:
            print("\n  [경고] 이동 타임아웃!")
            break

    if ref_pulse is not None and motor._index != motor_ref_idx:
        diff = calc_diff_mm(motor, ref_pulse)
        print(f"  --> M{motor._index}: {motor.current_position_mm:+8.3f}mm  |  "
              f"기준과 차이: {diff:+8.3f}mm  [완료]")
    else:
        print(f"  --> M{motor._index}: {motor.current_position_mm:+8.3f}mm  [완료]")


def parse_command(line, motors, ref_state):
    """
    ref_state: {'ref_pulse': None 또는 int}  (가변 dict로 상태 공유)
    반환값: True(계속), False(종료)
    """
    parts = line.strip().split()
    if not parts:
        return True

    cmd = parts[0].lower()
    motor_ref    = motors[MOTOR_REF_IDX]
    motor_target = motors[MOTOR_TARGET_IDX]
    ref_pulse    = ref_state['ref_pulse']

    # ── 종료 ──────────────────────────────────────────────────
    if cmd == 'q':
        return False

    # ── 기준 설정 ─────────────────────────────────────────────
    if cmd == 'ref':
        motor_ref.set_origin()
        time.sleep(0.4)  # 큐 처리 대기
        ref_state['ref_pulse'] = motor_ref.offset_pulse
        print(f"\n  [기준 설정] 모터 {motor_ref._index} 현재 위치를 원점으로 설정")
        print(f"  기준 펄스: {ref_state['ref_pulse']}")
        diff = calc_diff_mm(motor_target, ref_state['ref_pulse'])
        print(f"  현재 차이: {diff:+.3f}mm  "
              f"(모터 {motor_target._index}를 {-diff:+.3f}mm 이동하면 정렬됩니다)\n")
        return True

    # ── 현재 위치 및 차이 출력 ────────────────────────────────
    if cmd == 's':
        print_status(motor_ref, motor_target, ref_pulse)
        return True

    # ── 이동: m <인덱스> <거리mm> ─────────────────────────────
    if cmd == 'm':
        if len(parts) < 3:
            print("  사용법: m <모터번호> <거리mm>   예) m 2 -5.0")
            return True
        try:
            idx      = int(parts[1])
            delta_mm = float(parts[2])
        except ValueError:
            print("  [오류] 숫자 형식이 올바르지 않습니다.")
            return True

        if idx < 0 or idx >= len(motors):
            print(f"  [오류] 모터 번호는 0 ~ {len(motors)-1} 범위여야 합니다.")
            return True

        move_motor_by(motors[idx], delta_mm, ref_pulse, MOTOR_REF_IDX)

        # 이동 후 차이 요약
        if ref_pulse is not None and idx == MOTOR_TARGET_IDX:
            diff = calc_diff_mm(motor_target, ref_pulse)
            print(f"\n  [정렬 상태] 차이: {diff:+.3f}mm", end="")
            if abs(diff) < 0.05:
                print("  → 'done' 명령으로 원점 설정 가능")
            else:
                print(f"  → 모터 {MOTOR_TARGET_IDX}를 {-diff:+.3f}mm 더 이동 권장")
            print()
        return True

    # ── 정렬 완료: done ────────────────────────────────────────
    if cmd == 'done':
        if ref_pulse is None:
            print("  [오류] 먼저 'ref' 명령으로 기준을 설정하세요.")
            return True

        diff = calc_diff_mm(motor_target, ref_pulse)
        print(f"\n  현재 차이: {diff:+.3f}mm")
        if abs(diff) > 1.0:
            print(f"  [경고] 차이가 {diff:+.3f}mm로 큽니다. 계속하려면 'done' 재입력.")
            ref_state['done_confirm'] = True
            return True

        if ref_state.get('done_confirm') or abs(diff) <= 1.0:
            motor_target.set_origin()
            time.sleep(0.3)
            print(f"  [완료] 모터 {motor_target._index} 원점 설정 완료")
            print(f"  최종 차이: {calc_diff_mm(motor_target, ref_pulse):+.3f}mm")
            print("\n  두 모터 모두 원점 설정이 완료되었습니다.")
            return False  # 종료

        return True

    print(f"  [알 수 없는 명령] '{cmd}'  →  ref / m / s / done / q 중 하나를 입력하세요.")
    return True


HELP_TEXT = """
─────────────────────────────────────────────────────
  [원점 정렬 절차]
  1. ref          → 모터 1(기준) 현재 위치를 원점으로 설정
  2. m 2 <mm>     → 모터 2 이동 (이동 중 차이 실시간 표시)
                     예) m 2 -5.0  (모터2를 -5mm 이동)
  3. s            → 현재 위치 및 차이 확인
  4. done         → 차이가 0에 수렴하면 모터2 원점 설정 후 종료

  [기타]
  m 1 <mm>        → 모터 1도 이동 가능 (ref 재설정 권장)
  q               → 종료 (원점 설정 없이)
─────────────────────────────────────────────────────
"""


def main():
    print("=" * 55)
    print("  모터 원점 정렬 (Motor Origin Alignment)")
    print("=" * 55)
    print(f"  기준 모터  : 모터 {MOTOR_REF_IDX}")
    print(f"  정렬 모터  : 모터 {MOTOR_TARGET_IDX}")
    print(f"  속도       : {PROFILE_VELOCITY_RPM} RPM")
    print("=" * 55)

    bus = EtherCATBus(
        adapter_name=ADAPTER,
        num_slaves=NUM_MOTORS,
        cycle_time_ms=CYCLE_MS,
    )
    motors = bus.motors

    try:
        for idx, axis in AXIS_SETTINGS.items():
            if idx < NUM_MOTORS:
                motors[idx].set_axis(axis)
                motors[idx].set_profile_velocity(rpm=PROFILE_VELOCITY_RPM)
                motors[idx].set_profile_accel_decel(
                    accel_rpm_per_sec=PROFILE_ACCEL_RPM_PER_S
                )

        bus.start()

        print("\n모터 준비 대기 중...")
        for m in [motors[MOTOR_REF_IDX], motors[MOTOR_TARGET_IDX]]:
            wait_for_ready(m)
            print(f"  [완료] 모터 {m._index} 준비 완료  (Status=0x{m.status_word:04X})")

        print(HELP_TEXT)

        ref_state = {'ref_pulse': None, 'done_confirm': False}

        while True:
            # 프롬프트에 현재 차이 표시
            if ref_state['ref_pulse'] is not None:
                diff = calc_diff_mm(motors[MOTOR_TARGET_IDX], ref_state['ref_pulse'])
                prompt = f"[차이: {diff:+.3f}mm] 명령> "
            else:
                prompt = "[ref 미설정] 명령> "

            try:
                line = input(prompt).strip()
            except EOFError:
                break

            if not parse_command(line, motors, ref_state):
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
