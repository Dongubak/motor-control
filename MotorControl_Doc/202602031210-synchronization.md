# 다축 동기화

#synchronization #multi-axis #coordination

> ← [[MOC-motor-control|진입점]] / [[202602031200-ethercat-motor-control-system|시스템 개요]]

## 개요

여러 모터를 동시에 제어하여 같은 시간에 목표 위치에 도달하도록 하는 기능. [[202602031204-csp-mode|CSP 모드]]의 장점을 활용한다.

## 동기화 전략

### 핵심 원칙: 시간 동기화

```python
# motor.py:281-282
# [2단계] 가장 긴 duration 사용 (모든 모터가 같은 시간에 완료)
max_duration = max(max(traj['duration'] for traj in trajectory_data), 0.1)
```

**정책**: 이동 거리가 달라도, **이동 시간은 동일**

## 동기화 알고리즘

### 1단계: 공통 시작 시간 설정

```python
# motor.py:236
common_start_time = time.monotonic()
```

**중요**: 모든 모터가 같은 `start_time`을 공유

### 2단계: 각 모터의 이동 시간 계산

```python
# motor.py:238-279
trajectory_data = []
for slave_idx, target_mm in trajectory_commands:
    # 거리 계산
    distance = abs(target_pulse_absolute - current_pos)

    # 개별 속도 기반 시간 계산
    velocity_pulse_per_sec = (configured_velocity / 60.0) * PULSES_PER_REVOLUTION * 2
    duration = distance / velocity_pulse_per_sec

    trajectory_data.append({
        'slave_idx': slave_idx,
        'target_pos': target_pulse_absolute,
        'distance': distance,
        'duration': duration
    })
```

**각 모터마다**: 이동 거리와 속도에 따라 개별 duration 계산

### 3단계: 최대 Duration 선택

```python
# motor.py:281-282
max_duration = max(max(traj['duration'] for traj in trajectory_data), 0.1)
```

**예제**:
- Motor 1: 50mm → 6.0초 필요
- Motor 2: 30mm → 3.6초 필요
- **선택**: 6.0초 (최대값)

### 4단계: 모든 모터에 동일 Duration 적용

```python
# motor.py:284-297
for traj_info in trajectory_data:
    state['trajectory'] = {
        'start': traj_info['current_pos'],
        'end': traj_info['target_pos'],      # 각자의 목표!
        'duration': max_duration,            # 같은 시간!
        'start_time': common_start_time      # 같은 시작!
    }
```

**결과**:
- Motor 1: 50mm를 6.0초에 이동
- Motor 2: 30mm를 6.0초에 이동 (속도 느려짐)

## 속도 조정 메커니즘

### 자동 속도 조정

각 모터는 설정된 `max_duration`에 맞춰 자동으로 속도를 조정한다.

**Motor 1** (긴 거리):
```
실제 속도 = 설정 속도 (50 RPM)
```

**Motor 2** (짧은 거리):
```
실제 속도 = (거리 / max_duration)
         = (30mm / 6.0초)
         = 설정 속도보다 느림
```

### [[202602031207-trajectory-interpolation|S-Curve 보간]]

모든 모터가 같은 S-Curve 프로파일을 사용:

```python
# motor.py:399-406
progress = min(elapsed / traj['duration'], 1.0)
smooth_progress = (1.0 - math.cos(math.pi * progress)) / 2.0
target_position = int(traj['start'] + (traj['end'] - traj['start']) * smooth_progress)
```

**효과**: 모든 모터가 같은 가감속 패턴

## 동시 도착 보장

### 이론적 동시성

```
t = 0초      : 모든 모터 출발
t = 6.0초    : 모든 모터 도착
```

### 실제 완료 판정

```python
# motor.py:387-392
position_error = abs(traj['end'] - current_actual_pos)
if position_error < 50000:  # 약 0.18mm
    state['trajectory'] = None  # 완료
```

**독립 판정**: 각 모터가 개별적으로 완료 판정

**허용 오차**: ±0.18mm (z축 기준)

## 예제: 2개 모터 동기화

### 시나리오

```python
motor1.move_to_position_mm(-50)  # 50mm 이동
motor2.move_to_position_mm(-30)  # 30mm 이동
```

### 실행 과정

**설정**:
- 속도: 50 RPM (양쪽 동일)
- z축: 5.9997 mm/rev

**계산**:

Motor 1:
```
거리: 50mm
회전수: 50 / 5.9997 = 8.334 rev
시간: 8.334 rev ÷ (50/60 rev/s) = 10.0초
```

Motor 2:
```
거리: 30mm
회전수: 30 / 5.9997 = 5.000 rev
시간: 5.000 rev ÷ (50/60 rev/s) = 6.0초
```

**동기화**:
```
max_duration = max(10.0, 6.0) = 10.0초

Motor 1: 50mm를 10.0초에 이동 (50 RPM)
Motor 2: 30mm를 10.0초에 이동 (30 RPM 상당)
```

**결과**: 두 모터 모두 10초 후 동시 도착

## 명령 일괄 처리

### 같은 사이클에서 처리

```python
# motor.py:206-224
trajectory_commands = []  # 임시 저장

while not command_queue.empty():
    slave_idx, cmd, value = command_queue.get_nowait()
    if cmd == 'MOVE_TO_MM':
        trajectory_commands.append((slave_idx, value))

# 모든 명령을 한번에 처리
if trajectory_commands:
    # 동기화 로직 실행
```

**중요**: 큐에 쌓인 모든 이동 명령을 한 번에 처리

### 사용자 코드

```python
# main_dual.py:107-108
motor1.move_to_position_mm(-50)
motor2.move_to_position_mm(-50)
```

**비동기**: 명령 큐에 추가만 하고 즉시 반환

**동기화**: [[202602031203-ethercat-process-loop|프로세스 루프]]의 다음 사이클에서 일괄 처리

## 위치 차이 확인

```python
# main_dual.py:124
print(f"위치 차이: {abs(motor1.current_position_mm - motor2.current_position_mm):.2f} mm")
```

**이상적**: 0.00mm
**실제**: ±0.2mm 이내 (Following Error 및 분해능)

## 동기화 품질 향상 방법

### 현재 구현
- ✅ 공통 시작 시간
- ✅ 최대 Duration 사용
- ✅ S-Curve 보간
- ✅ 위치 기반 완료 판정

### 추가 개선 가능
- **전자 기어**: 모든 모터를 가상 마스터 축에 동기화
- **Cross Coupling**: 위치 오차를 상호 보정
- **Jerk 제한**: 더 부드러운 동작

## 비동기 vs 동기 제어

| 특성 | 비동기 | 동기 (현재) |
|------|--------|-------------|
| 도착 시간 | 각자 다름 | 동일 |
| 속도 | 설정값 유지 | 자동 조정 |
| 위치 차이 | 클 수 있음 | 최소화 |
| 용도 | 독립 동작 | 연동 동작 |

**선택**: 연동 동작을 위해 동기 제어 사용

## 관련 노트

- [[202602031204-csp-mode|CSP 모드]]
- [[202602031207-trajectory-interpolation|궤적 보간]]
- [[202602031203-ethercat-process-loop|EtherCAT 프로세스 루프]]

---

**생성일**: 2026-02-03
**태그**: #synchronization #coordination #multi-axis
