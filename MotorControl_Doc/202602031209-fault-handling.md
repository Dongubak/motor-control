# Fault 처리

#fault #error-handling #safety

> ← [[MOC-motor-control|진입점]] / [[202602031200-ethercat-motor-control-system|시스템 개요]]

## 개요

모터 에러(Fault) 상태를 자동으로 감지하고 복구하는 안전 시스템. [[202602031205-cia402-state-machine|CiA 402 상태 머신]]의 Fault 상태를 처리한다.

## Fault 감지

### Statusword 비트 확인

```python
# motor.py:314-318
status = _read_status_word(master.slaves[i])
if (status & 0x0008):  # 비트 3: Fault
    fault_detected = True
    fault_motor_id = i
```

**Fault 비트**: Statusword의 비트 3 (0x0008)

## 자동 복구 (프로세스 루프)

### 1차 복구: 자동 Fault Reset

```python
# motor.py:356-358
elif (status & 0x0008):  # Fault
    cw = CW_FAULT_RESET  # 0x0080
```

**동작**: 매 사이클마다 자동으로 `CW_FAULT_RESET` 전송

**전환**: Fault → Ready to Switch On

### 연동 안전: 모든 모터 즉시 정지

```python
# motor.py:320-328
if fault_detected:
    print(f"\n[긴급 정지] 모터 {fault_motor_id} Fault 감지! 모든 모터를 안전하게 정지합니다.")
    for i in range(num_slaves):
        if local_motor_states[i]['trajectory'] is not None:
            local_motor_states[i]['trajectory'] = None
            current_pos = _read_actual_position(master.slaves[i])
            local_motor_states[i]['target_pulse'] = current_pos
```

**정책**: 한 모터라도 Fault 발생 → **모든 모터 궤적 취소**

**이유**: 연동 동작 중 한 축만 멈추면 위험

## 수동 복구 (메인 프로그램)

### 2차 복구: 재시도 로직

```python
# main.py:70-85
if status & 0x0008:  # Fault 감지
    for attempt in range(3):  # 최대 3회 시도
        time.sleep(0.5)
        status = motor.status_word
        if not (status & 0x0008):
            print(f"[성공] 모터 {motor._index} Fault 리셋 완료!")
            break
    else:
        # 3회 시도 후 실패 시 종료
        raise RuntimeError(f"모터 Fault 상태를 해제할 수 없습니다!")
```

**재시도 횟수**: 최대 3회
**대기 시간**: 0.5초

### 3차 복구: 상태 복구 대기

```python
# main.py:88-95
while (motor.status_word & 0x006F) != 0x0027:  # Operation Enabled 대기
    time.sleep(0.05)
    if time.monotonic() - start_time > 3:
        raise RuntimeError(f"모터 상태 복구 실패!")
```

**목표**: Operation Enabled (0x0027) 상태로 복귀
**타임아웃**: 3초

## Following Error 방지

### Following Error Window 확대

```python
# motor.py:114
slave.sdo_write(0x6065, 0, struct.pack("<I", 200000000))  # 200M pulse
```

**기본값**: 수천~수만 pulse
**변경값**: 200,000,000 pulse ≈ **143mm** (z축 기준)

**이유**: [[202602031204-csp-mode|CSP 모드]]에서는 초기에 목표-현재 위치 차이가 큼

### 위치 오차 모니터링 (비활성화)

```python
# motor.py:379-385 (비활성화됨)
if False and position_error > 18000000:
    print(f"[에러] 모터 {i} 위치 오차 과다!")
    state['trajectory'] = None
```

**현재**: 드라이버가 자동으로 Following Error 감지하므로 비활성화

## Fault 발생 원인

### 일반적인 원인
1. **Following Error**: 목표와 현재 위치 차이 과다
2. **Over Current**: 과전류 감지
3. **Over Voltage**: 과전압 감지
4. **Encoder Error**: 엔코더 신호 이상
5. **Communication Timeout**: EtherCAT 통신 끊김

### 디버깅 정보 출력

```python
# motor.py:357-358
if cycle_counter % 50 == 0:  # 주기적 출력
    print(f"[경고] 모터 {i} Fault 상태! status=0x{status:04X}")
```

**주기**: 50 사이클마다 (500ms)

## 초기화 시 Fault Reset

```python
# motor.py:516-519
def _sdo_reset_fault(slave):
    if struct.unpack("<H", slave.sdo_read(0x6041, 0))[0] & 0x0008:
        slave.sdo_write(0x6040, 0, struct.pack("<H", CW_FAULT_RESET))
        time.sleep(0.2)
```

**타이밍**: 슬레이브 검색 직후 (OP 상태 전환 전)

## 타임아웃 처리

### 준비 대기 타임아웃

```python
# main.py:42-47
if time.monotonic() - start_time > 5:
    status = motor.status_word
    print(f"[에러] 모터 {motor._index} 타임아웃! 현재 상태: 0x{status:04X}")
    if status & 0x0008:
        print(f"  → Fault 상태 감지됨!")
    raise RuntimeError(f"모터가 Operation Enabled 상태에 도달하지 못했습니다.")
```

**타임아웃**: 5초

### 이동 타임아웃

```python
# main.py:119-121
if time.monotonic() - start_time > 60:
    print("\n[경고] 타임아웃: 60초 내에 이동이 완료되지 않았습니다.")
    break
```

**타임아웃**: 60초

## 안전 종료

프로그램 종료 시 안전하게 모터 정지:

```python
# motor.py:434-449
# 1. 현재 위치로 고정
for i in range(num_slaves):
    current_pos = _read_actual_position(master.slaves[i])
    local_motor_states[i]['target_pulse'] = current_pos

# 2. 위치 고정 명령 전송 (5회 반복)
for _ in range(5):
    master.send_processdata()
    time.sleep(0.02)
```

**반복**: 5회 (확실한 적용)

## 실무 팁

### Fault 발생 시 체크리스트
1. ✅ Statusword 확인 (0x6041)
2. ✅ 드라이버 LED 상태 확인
3. ✅ Following Error Window 설정 확인 (0x6065)
4. ✅ EtherCAT 케이블 연결 확인
5. ✅ 모터 전원 확인

### 예방 조치
- Following Error Window를 충분히 크게 설정
- 속도/가감속도를 적절히 조정
- 사이클 타임 안정성 확보 (10ms ±1ms 이내)

## 관련 노트

- [[202602031205-cia402-state-machine|CiA 402 상태 머신]]
- [[202602031204-csp-mode|CSP 모드]]
- [[202602031203-ethercat-process-loop|EtherCAT 프로세스 루프]]

---

**생성일**: 2026-02-03
**태그**: #safety #error-handling #recovery
