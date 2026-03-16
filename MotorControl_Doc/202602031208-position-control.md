# 위치 제어

#position-control #motion-control #mm-unit

> ← [[MOC-motor-control|진입점]] / [[202602031200-ethercat-motor-control-system|시스템 개요]]

## 개요

mm 단위의 절대 위치 제어 시스템. 원점 기준으로 상대 위치를 제어하며, [[202602031207-trajectory-interpolation|S-Curve 보간]]을 통해 부드러운 이동을 구현한다.

## 제어 흐름

```
사용자 입력 (mm)
    ↓
회전수 변환 (mm → revolutions)
    ↓
펄스 변환 (revolutions → pulse)
    ↓
Position Factor 처리 (×2)
    ↓
원점 오프셋 적용 (+ offset)
    ↓
절대 위치 계산 (absolute pulse)
    ↓
궤적 생성 (S-Curve)
    ↓
CSP 모드 전송
```

## 단위 변환

### mm → pulse 변환

```python
# motor.py:242-250
mm_per_rev = x_axis_mm_per_rev if state['axis'] == 'x' else z_axis_mm_per_rev
revolutions = target_mm / mm_per_rev
target_pulse_relative = int(revolutions * PULSES_PER_REVOLUTION * 2)
target_pulse_absolute = target_pulse_relative + state['offset']
```

**변환 계수**:
- **x축**: 11.9993 mm/rev
- **z축**: 5.9997 mm/rev

**펄스 분해능**: 8,388,608 pulse/rev

**Position Factor**: 2:1 (드라이버 설정)

### 예제 계산 (z축, -50mm)

```
Step 1: mm → revolutions
  -50mm ÷ 5.9997mm/rev = -8.3335 rev

Step 2: revolutions → pulse (user scale)
  -8.3335 rev × 8,388,608 pulse/rev = -69,905,168 pulse

Step 3: Position Factor 적용 (×2)
  -69,905,168 × 2 = -139,810,336 pulse (driver scale)

Step 4: 원점 오프셋 적용
  -139,810,336 + offset = absolute_pulse
```

## 원점 설정

### 원점 설정 메서드

```python
# Motor 클래스
motor.set_origin()
```

### 내부 구현

```python
# motor.py:218-221
current_pos = _read_actual_position(master.slaves[slave_idx])
local_motor_states[slave_idx]['offset'] = current_pos
local_motor_states[slave_idx]['target_pulse'] = current_pos
```

**효과**:
- 현재 위치가 0mm가 됨
- 이후 모든 명령은 이 원점 기준으로 상대 이동

### 원점 설정 시점

```python
# main.py:54-56
motor1.set_origin()
time.sleep(0.5)  # 명령 처리 대기
```

**타이밍**: 모터 준비 완료 직후

## 위치 읽기

### mm 단위 읽기

```python
# motor.py:636-649
@property
def current_position_mm(self) -> float:
    current_pos = self._shared_states[base_idx + 2]
    offset = self._shared_states[base_idx + 3]
    relative_pos = current_pos - offset
    revolutions = relative_pos / (PULSES_PER_REVOLUTION * 2)
    mm_per_rev = x_axis_mm_per_rev if self._axis == 'x' else z_axis_mm_per_rev
    return revolutions * mm_per_rev
```

**역변환 과정**:
```
absolute_pulse → (- offset) → relative_pulse
→ (÷ (8388608 × 2)) → revolutions
→ (× mm_per_rev) → mm
```

### 펄스 단위 읽기 (디버깅)

```python
motor.current_position_pulse  # 절대 펄스 위치
motor.offset_pulse            # 원점 오프셋 펄스
```

## 위치 명령

### API

```python
motor.move_to_position_mm(-50)  # -50mm로 이동
```

### 내부 처리

```python
# motor.py:621-623
def move_to_position_mm(self, target_mm):
    self._command_queue.put((self._index, 'MOVE_TO_MM', target_mm))
```

**비동기**: 명령 큐에 추가 후 즉시 반환

**실행**: [[202602031203-ethercat-process-loop|프로세스 루프]]에서 처리

## 이동 완료 확인

```python
# main.py:115-117
while motor1.is_moving():
    print(f"현재 위치: {motor1.current_position_mm:.2f} mm")
    time.sleep(0.05)
```

### is_moving() 구현

```python
# motor.py:630-633
def is_moving(self) -> bool:
    return self._shared_states[self._index * 4 + 1] != 0
```

**판정**: `trajectory`가 `None`이 아니면 이동 중

## 위치 정밀도

### 펄스 분해능

**z축 기준**:
```
5.9997mm/rev ÷ (8,388,608 pulse/rev × 2) = 0.000357 mm/pulse
```

**약 0.36 μm/pulse**

### 완료 판정 Threshold

```python
# motor.py:387
if position_error < 50000:  # 약 0.18mm
```

**z축**: 50,000 pulse × 0.000357 mm/pulse ≈ **0.18mm**

## Position Factor 2:1 처리

드라이버 설정으로 인해 모든 펄스 값이 2배:

```python
# 사용자 스케일: 8,388,608 pulse/rev
# 드라이버 스케일: 16,777,216 pulse/rev (×2)
```

**중요**: 모든 계산에서 일관되게 ×2 적용

**참고**: `motor.py:14`, `motor.py:249`, `motor.py:646`

## 축 설정의 영향

```python
motor.set_axis('z')  # z축 사용
```

**변환 계수 변경**:
- x축: 11.9993 mm/rev
- z축: 5.9997 mm/rev (정확히 x축의 1/2)

**실제 기구**: 리드스크류 피치가 다름

## 관련 노트

- [[202602031202-motor-class|Motor 클래스]]
- [[202602031207-trajectory-interpolation|궤적 보간]]
- [[202602031204-csp-mode|CSP 모드]]

---

**생성일**: 2026-02-03
**태그**: #control #unit-conversion #precision
