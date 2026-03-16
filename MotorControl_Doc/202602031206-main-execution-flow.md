# 메인 실행 흐름

#execution-flow #workflow #main-program

> ← [[MOC-motor-control|진입점]] / [[202602031200-ethercat-motor-control-system|시스템 개요]]

## 개요

`main.py`의 실행 흐름을 단계별로 분석. 모터 초기화부터 제어, 종료까지 전체 과정을 다룬다.

## 파일 위치

`main.py`

## 실행 단계

### 0. 설정 단계

```python
# main.py:6-10
adapter = r'\Device\NPF_{A3C1307F-C4D5-4126-8FCF-A191BF2B1257}'
NUM_MOTORS = 1

bus = EtherCATBus(adapter_name=adapter, num_slaves=NUM_MOTORS, cycle_time_ms=10)
motor1 = bus.motors[0]
```

**주요 설정**:
- 네트워크 어댑터 경로
- 슬레이브 수: 1개
- 사이클 타임: 10ms

**관련**: [[202602031201-ethercat-bus-class|EtherCATBus 클래스]]

### 1. 축 및 파라미터 설정 (버스 시작 전)

```python
# main.py:25-31
motor1.set_axis('z')
motor1.set_profile_velocity(rpm=50)
motor1.set_profile_accel_decel(accel_rpm_per_sec=50)
```

**중요**: 반드시 `bus.start()` **이전**에 호출

**이유**: SDO 설정은 PREOP 상태에서만 가능

**관련**: [[202602031202-motor-class|Motor 클래스]]

### 2. 버스 시작

```python
# main.py:37
bus.start()
```

**내부 동작**:
1. [[202602031203-ethercat-process-loop|통신 프로세스]] 시작
2. EtherCAT 초기화 (재시도 로직)
3. [[202602031204-csp-mode|CSP 모드]] 설정
4. OP 상태 전환

**비동기**: 별도 프로세스에서 실행

### 3. 모터 준비 대기

```python
# main.py:40-48
while (motor.status_word & 0x006F) != 0x0027:
    time.sleep(0.05)
    if time.monotonic() - start_time > 5:
        raise RuntimeError("모터가 Operation Enabled 상태에 도달하지 못했습니다.")
```

**목표 상태**: `0x0027` (Operation Enabled)

**타임아웃**: 5초

**관련**: [[202602031205-cia402-state-machine|CiA 402 상태 머신]]

### 4. 원점 설정

```python
# main.py:54-61
motor1.set_origin()
time.sleep(0.5)

print(f"원점 설정 후 위치: M1={motor1.current_position_mm:.2f}mm")
print(f"[디버그] 펄스 정보 - M1: {motor1.current_position_pulse}")
print(f"[디버그] 오프셋 정보 - M1: {motor1.offset_pulse}")
```

**효과**: 현재 위치를 0mm로 설정

**디버깅**: 펄스 및 오프셋 정보 출력

**관련**: [[202602031208-position-control|위치 제어]]

### 5. 이동 전 상태 검증

```python
# main.py:64-102
for motor in [motor1]:
    status = motor.status_word
    print(f"모터 {motor._index} 상태:")
    print(f"  Status Word: 0x{status:04X}")
    print(f"  Operation Enabled: {(status & 0x006F) == 0x0027}")
    print(f"  Fault: {bool(status & 0x0008)}")
```

**검증 항목**:
- Statusword 확인
- Operation Enabled 상태 확인
- Fault 상태 확인 및 복구

**관련**: [[202602031209-fault-handling|Fault 처리]]

#### Fault 복구 로직

```python
# main.py:73-85
if status & 0x0008:  # Fault 감지
    for attempt in range(3):  # 최대 3회 재시도
        time.sleep(0.5)
        status = motor.status_word
        if not (status & 0x0008):
            print(f"[성공] Fault 리셋 완료!")
            break
    else:
        raise RuntimeError(f"Fault 상태를 해제할 수 없습니다!")
```

**재시도**: 3회
**대기 시간**: 0.5초

### 6. 위치 이동 명령

```python
# main.py:107-123
motor1.move_to_position_mm(-50)
time.sleep(0.2)  # 명령 처리 대기

while motor1.is_moving():
    print(f"--> 이동 중... M1: {motor1.current_position_mm:.2f} mm", end='\r')
    time.sleep(0.05)
    if time.monotonic() - start_time > 60:
        print("\n[경고] 타임아웃!")
        break

print(f"\n--> 이동 완료! M1: {motor1.current_position_mm:.2f} mm")
```

**목표**: -50mm

**모니터링**: 50ms 주기로 현재 위치 출력

**타임아웃**: 60초

**관련**: [[202602031207-trajectory-interpolation|궤적 보간]]

### 7. 원점 복귀

```python
# main.py:131-150
motor1.move_to_position_mm(0)
time.sleep(0.2)

while motor1.is_moving():
    print(f"--> 복귀 중... M1: {motor1.current_position_mm:.2f} mm", end='\r')
    time.sleep(0.05)

print(f"\n--> 복귀 완료! M1: {motor1.current_position_mm:.2f} mm")
print(f"[디버그] 최종 펄스 - M1: {motor1.current_position_pulse}")
```

**목표**: 0mm (원점)

**최종 확인**: 펄스 및 오프셋 정보 출력

### 8. 종료 처리

```python
# main.py:156-162
except KeyboardInterrupt:
    print("\n사용자에 의해 프로그램이 중단되었습니다.")
except Exception as e:
    print(f"\n메인 루프에서 에러 발생: {e}")
finally:
    bus.stop()  # 반드시 실행
```

**안전 종료**:
1. 모든 모터를 현재 위치에 고정
2. [[202602031205-cia402-state-machine|CiA 402 종료 시퀀스]] 실행
3. EtherCAT 통신 종료

**관련**: [[202602031203-ethercat-process-loop|프로세스 루프]] 종료 단계

## 실행 시간 예상

| 단계 | 예상 시간 |
|------|-----------|
| 초기화 | 1~3초 |
| 준비 대기 | 1~2초 |
| 원점 설정 | 0.5초 |
| 상태 검증 | 0.5초 |
| -50mm 이동 | 6~10초 |
| 원점 복귀 | 6~10초 |
| 종료 | 1초 |
| **합계** | **16~27초** |

## 예상 출력

```
[초기화] EtherCAT 어댑터 열기...
[성공] 1개의 슬레이브를 찾았습니다.
모터 0 준비 대기 중...
[완료] 모터 0 준비 완료! (1.23초 소요)
원점 설정 후 위치: M1=0.00mm
[디버그] 펄스 정보 - M1: 12345678
[디버그] 오프셋 정보 - M1: 12345678
[디버그] 펄스-오프셋 차이 - M1: 0

--- 이동 전 모터 상태 검증 ---
모터 0 상태:
  Status Word: 0x0027
  Operation Enabled: True
  Fault: False
  현재 위치: 12345678 pulse
[검증 완료] 모든 모터가 정상 작동 가능 상태입니다.

--- 이동 시작 (CSP 모드) ---
모터 이동 중...
--> 이동 중... M1: -25.32 mm
--> 이동 완료! M1: -50.00 mm

[완료] 이동 완료!

--- 원점 복귀 시작 (CSP 모드) ---
모터 복귀 중...
--> 복귀 중... M1: -25.11 mm
--> 복귀 완료! M1: 0.00 mm
[디버그] 최종 펄스 - M1: 12345678
[디버그] 오프셋 펄스 - M1: 12345678
[디버그] 상대 펄스 - M1: 0

[완료] 원점 복귀 완료!

--- 모든 작업 완료 ---
EtherCAT 버스 종료 중...
[종료 시퀀스 시작]
  [안전] 모든 모터를 현재 위치에 고정합니다...
  [완료] 모터 정지 완료
  1단계: Disable Operation (모터 동작 중지)
  2단계: Shutdown (전원 준비)
  3단계: Disable Voltage (전원 차단)
  4단계: EtherCAT INIT 상태로 전환
  [완료] 종료 시퀀스 완료
EtherCAT 프로세스 종료.
EtherCAT 버스 프로세스 종료 완료.
```

## 2개 모터 확장 (main_dual.py)

### 주요 차이점

```python
# main_dual.py:7
NUM_MOTORS = 2

# main_dual.py:14-15
motor1 = bus.motors[0]
motor2 = bus.motors[1]

# main_dual.py:107-108
motor1.move_to_position_mm(-50)
motor2.move_to_position_mm(-50)

# main_dual.py:115
while motor1.is_moving() or motor2.is_moving():
```

**동기화**: [[202602031210-synchronization|다축 동기화]] 참조

## 에러 처리 전략

### 1. 초기화 실패
- EtherCAT 재시도 (최대 3회)
- 슬레이브 검색 실패 시 종료

### 2. 준비 대기 타임아웃
- 5초 초과 시 RuntimeError 발생
- Fault 상태 여부 출력

### 3. 이동 중 에러
- Following Error 발생 시 자동 복구
- 60초 타임아웃

### 4. 키보드 인터럽트
- Ctrl+C 감지 시 안전 종료

## 디버깅 팁

### 상태 확인
```python
print(f"Status: 0x{motor.status_word:04X}")
print(f"Position: {motor.current_position_mm:.2f}mm")
print(f"Moving: {motor.is_moving()}")
```

### 펄스 정보 확인
```python
print(f"Pulse: {motor.current_position_pulse}")
print(f"Offset: {motor.offset_pulse}")
print(f"Relative: {motor.current_position_pulse - motor.offset_pulse}")
```

## 관련 노트

- [[202602031200-ethercat-motor-control-system|시스템 개요]]
- [[202602031201-ethercat-bus-class|EtherCATBus 클래스]]
- [[202602031202-motor-class|Motor 클래스]]
- [[202602031208-position-control|위치 제어]]
- [[202602031209-fault-handling|Fault 처리]]

---

**생성일**: 2026-02-03
**태그**: #workflow #main-program #execution
