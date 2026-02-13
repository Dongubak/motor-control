# 궤적 보간 (S-Curve)

#trajectory #interpolation #motion-planning

> ← [[MOC-motor-control|진입점]] / [[202602031200-ethercat-motor-control-system|시스템 개요]]

## 개요

[[202602031204-csp-mode|CSP 모드]]에서 마스터가 수행하는 궤적 보간. S-Curve를 사용하여 부드러운 가감속을 구현한다.

## S-Curve 보간 공식

```python
# motor.py:399-403
progress = min(elapsed / duration, 1.0)
smooth_progress = (1.0 - math.cos(math.pi * progress)) / 2.0
target_position = int(start + (end - start) * smooth_progress)
```

### 수식

$$
\text{smooth\_progress} = \frac{1 - \cos(\pi \times \text{progress})}{2}
$$

여기서 $\text{progress} = \frac{t}{T}$ (정규화된 시간)

## 특성

### 속도 프로파일

$$
v(t) = \frac{\pi}{2T} \sin\left(\frac{\pi t}{T}\right)
$$

- **시작**: 속도 0 (부드러운 출발)
- **중간**: 최대 속도
- **종료**: 속도 0 (부드러운 정지)

### 가속도 프로파일

$$
a(t) = \frac{\pi^2}{2T^2} \cos\left(\frac{\pi t}{T}\right)
$$

- **연속적**: 급격한 변화 없음 (저크 최소화)
- **대칭적**: 가속과 감속이 대칭

## 궤적 데이터 구조

```python
# motor.py:292-297
state['trajectory'] = {
    'start': current_pos,       # 시작 위치 (pulse)
    'end': target_pos,          # 목표 위치 (pulse)
    'duration': max_duration,   # 이동 시간 (sec)
    'start_time': common_start_time  # 시작 시각
}
```

## Duration 계산

```python
# motor.py:265-269
configured_velocity = slave_configs.get(slave_idx, {}).get('velocity', 60)
velocity_pulse_per_sec = (configured_velocity / 60.0) * PULSES_PER_REVOLUTION * 2
duration = distance / velocity_pulse_per_sec
```

**공식**:

$$
T = \frac{\text{distance}}{\text{velocity}}
$$

**예제** (z축, 50mm, 50 RPM):
```
distance = 83,905,336 pulse
velocity = 50/60 rev/s × 8,388,608 pulse/rev × 2 = 13,981,013 pulse/s
duration = 83,905,336 / 13,981,013 = 6.0초
```

## 완료 판정

### 위치 기반 판정

```python
# motor.py:372, 387-392
position_error = abs(traj['end'] - current_actual_pos)
if position_error < 50000:  # 약 0.18mm
    state['trajectory'] = None  # 궤적 완료
```

**Threshold**: 50,000 pulse

**이유**: 시간 기반 판정은 싱크 위험이 있음

### 시간 기반 판정 (미사용)

```python
# 사용하지 않음 - 주석 처리됨
# if elapsed >= traj['duration']:
#     state['trajectory'] = None
```

**문제점**: 통신 지연으로 동기화 어긋날 수 있음

## Linear vs S-Curve 비교

| 특성 | Linear | S-Curve |
|------|--------|---------|
| 속도 변화 | 급격함 | 부드러움 |
| 가속도 | 불연속 | 연속 |
| 저크 | 무한대 | 유한 |
| 진동 | 발생 | 최소화 |
| 계산 부하 | 낮음 | 중간 |

**선택 이유**: 기계적 부하 및 진동 최소화

## 궤적 취소

새 명령 수신 시 기존 궤적 즉시 취소:

```python
# motor.py:228-233
for slave_idx, _ in trajectory_commands:
    if local_motor_states[slave_idx]['trajectory'] is not None:
        print(f"[경고] 모터 {slave_idx}: 기존 궤적 취소 후 새 명령 처리")
        local_motor_states[slave_idx]['trajectory'] = None
```

**효과**: 즉각적인 응답성 확보

## 디버깅 정보

```python
# motor.py:298-301
print(f"[디버그] 모터 {slave_idx} CSP 궤적 생성:")
print(f"  목표: {target_mm:.2f}mm")
print(f"  현재: {current_pos} → 목표: {target_pos}")
print(f"  이동거리: {distance} pulse, 시간: {max_duration:.2f}초")
```

## 개선 방향

### 현재 구현
- S-Curve 보간
- 속도 설정 기반 duration

### 가능한 개선
- **7차 다항식**: 더 부드러운 가감속
- **가감속도 제한**: 독립적인 가속도 설정
- **Multi-segment**: 복잡한 경로 생성

## 관련 노트

- [[202602031204-csp-mode|CSP 모드]]
- [[202602031203-ethercat-process-loop|EtherCAT 프로세스 루프]]
- [[202602031210-synchronization|다축 동기화]]
- [[202602031208-position-control|위치 제어]]

---

**생성일**: 2026-02-03
**태그**: #motion-planning #algorithm #smoothing
