# Cross Coupling 알고리즘

#cross-coupling #synchronization #control #algorithm

> ← [[MOC-motor-control|진입점]] / [[202602031210-synchronization|다축 동기화]] / [[202602041300-safety-synchronization|안전 동기화]]

## 개요

Cross Coupling은 두 모터의 **위치 오차를 실시간으로 보정**하여 동기화 정밀도를 높이는 알고리즘이다.

```
기존 방식 (시간 동기화만):
  Motor 0 ─────────────────────→ 목표
  Motor 1 ─────────→ 지연 발생!
                     ↑
            감지만 하고 정지

Cross Coupling (위치 오차 보정):
  Motor 0 ───────────────────→ 목표
              ↓ 감속
  Motor 1 ───────────────────→ 목표
              ↑ 가속
        실시간으로 맞춤
```

---

## 알고리즘 원리

### 1. 상대 위치 계산

```python
# 오프셋 보정된 상대 위치
relative_positions = [
    positions[i] - local_motor_states[i]['offset']
    for i in range(num_slaves)
]
```

### 2. 평균 위치 계산

```python
avg_relative_position = sum(relative_positions) / num_slaves
```

### 3. 오차 및 보정량 계산

```python
for i in range(num_slaves):
    # 평균 위치로부터의 오차
    error_from_avg = relative_positions[i] - avg_relative_position

    # 게인 적용
    coupling_correction[i] = int(coupling_gain * error_from_avg)
```

### 4. 목표 위치 보정

```python
# S-Curve 보간 후 보정 적용
target_position = int(traj['start'] + (traj['end'] - traj['start']) * smooth_progress)
target_position -= coupling_correction[i]  # 보정 적용
```

---

## 보정 원리

| 상황 | error_from_avg | correction | 효과 |
|------|----------------|------------|------|
| 모터가 앞서감 | 양수 (+) | 양수 (+) | 목표에서 차감 → 감속 |
| 모터가 뒤처짐 | 음수 (-) | 음수 (-) | 목표에서 차감 → 가속 |

```
예시: Motor 0 = 1000 pulse, Motor 1 = 900 pulse
      평균 = 950 pulse

Motor 0: error = 1000 - 950 = +50 → 감속
Motor 1: error = 900 - 950 = -50 → 가속
```

---

## 구현 파일

### motor_coupling.py

```python
class EtherCATBusCoupling:
    def __init__(self, ..., coupling_gain=0.1, enable_coupling=True):
        # Cross Coupling 파라미터 (프로세스 간 공유)
        self._coupling_params = mp.Array('d', 2, lock=True)
        self._coupling_params[0] = coupling_gain
        self._coupling_params[1] = 1.0 if enable_coupling else 0.0

    @property
    def coupling_gain(self) -> float:
        return self._coupling_params[0]

    @coupling_gain.setter
    def coupling_gain(self, value: float):
        self._coupling_params[0] = value

    @property
    def coupling_enabled(self) -> bool:
        return self._coupling_params[1] != 0

    @coupling_enabled.setter
    def coupling_enabled(self, value: bool):
        self._coupling_params[1] = 1.0 if value else 0.0
```

### 핵심 코드 위치

| 기능 | 파일 | 라인 |
|------|------|------|
| 보정량 계산 | motor_coupling.py | 300-320 |
| 목표 위치 보정 | motor_coupling.py | 380-385 |
| 런타임 파라미터 | motor_coupling.py | 530-560 |

---

## 파라미터 튜닝

### Coupling Gain (Kc)

| 값 | 특성 | 권장 상황 |
|----|------|----------|
| 0.05 | 약한 보정, 안정적 | 초기 테스트 |
| 0.10 | 중간 보정 (기본값) | 일반 동기화 |
| 0.20 | 강한 보정 | 정밀 동기화 필요 |
| 0.50+ | 매우 강함 | ⚠️ 발진 위험 |

### 튜닝 절차

```
1단계: Kc = 0.05로 시작
2단계: 저속(10 RPM)에서 테스트
3단계: 위치 오차 로그 확인
4단계: Kc를 0.05씩 증가
5단계: 고속(50 RPM)에서 검증
```

---

## 런타임 제어

```python
# Cross Coupling 활성화/비활성화
bus.coupling_enabled = True   # 활성화
bus.coupling_enabled = False  # 비활성화

# 게인 조절
bus.coupling_gain = 0.15  # 0.0 ~ 1.0
```

---

## 테스트 코드

### main_dual_coupling.py

```python
# 테스트 1: Cross Coupling ON
bus.coupling_enabled = True
bus.coupling_gain = 0.1
motor1.move_to_position_mm(-50)
motor2.move_to_position_mm(-50)

# 테스트 2: Cross Coupling OFF
bus.coupling_enabled = False
motor1.move_to_position_mm(-50)
motor2.move_to_position_mm(-50)

# 결과 비교
print(f"Coupling ON:  최대 위치차 {max_diff_test1:.3f}mm")
print(f"Coupling OFF: 최대 위치차 {max_diff_test2:.3f}mm")
```

---

## 적용 조건

Cross Coupling이 적용되는 조건:

```python
if coupling_enabled and num_slaves >= 2 and all_moving and not sync_error_detected:
    # Cross Coupling 적용
```

| 조건 | 설명 |
|------|------|
| `coupling_enabled` | 런타임에서 활성화 상태 |
| `num_slaves >= 2` | 2개 이상의 모터 필요 |
| `all_moving` | **모든** 모터가 이동 중 |
| `not sync_error_detected` | 동기화 오류 미발생 |

---

## 안전 기능과의 관계

```
┌─────────────────────────────────────────────────────────┐
│                    제어 루프                              │
├─────────────────────────────────────────────────────────┤
│ 1. PDO 통신                                              │
│ 2. 위치 읽기                                              │
│ 3. [안전] 위치 차이 모니터링 → 임계값 초과 시 긴급 정지      │
│ 4. [Cross Coupling] 보정량 계산                           │
│ 5. 궤적 보간 (S-Curve)                                    │
│ 6. [Cross Coupling] 목표 위치 보정 적용                    │
│ 7. CSP 출력                                              │
└─────────────────────────────────────────────────────────┘
```

- 안전 기능이 **먼저** 실행되어 임계값 초과 시 정지
- Cross Coupling은 정상 동작 범위 내에서만 보정

---

## 브랜치 정보

| 항목 | 값 |
|------|-----|
| 브랜치 | `crossCoupling` |
| 기반 브랜치 | `dev` |
| 커밋 해시 | `7d71d30` |
| 원격 저장소 | https://github.com/Dongubak/motor-control |

---

## 관련 노트

- [[202602031210-synchronization|다축 동기화]] - 기본 시간 동기화
- [[202602041300-safety-synchronization|안전 동기화]] - 위치 모니터링 및 긴급 정지
- [[202602031207-trajectory-interpolation|궤적 보간]] - S-Curve 알고리즘
- [[202602031203-ethercat-process-loop|프로세스 루프]] - 제어 루프 구조

---

## 향후 개선 방향

1. **적응형 게인**: 속도/부하에 따라 게인 자동 조절
2. **PID 기반 보정**: 비례-적분-미분 제어 적용
3. **예측 보정**: 다음 사이클 위치 예측하여 선제적 보정

---

**생성일**: 2026-02-05
**태그**: #cross-coupling #synchronization #algorithm #control
