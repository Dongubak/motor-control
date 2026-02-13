# EtherCAT 프로세스 루프

#ethercat #realtime #multiprocessing

> ← [[MOC-motor-control|진입점]] / [[202602031200-ethercat-motor-control-system|시스템 개요]]

## 개요

별도 프로세스에서 실행되는 실시간 EtherCAT 통신 및 제어 루프. 모든 슬레이브를 관리하며 주기적으로 PDO 통신을 수행한다.

## 함수 시그니처

```python
def _ethercat_process_loop(
    adapter_name, num_slaves, cycle_time_s,
    command_queue, shared_states, lock
)
```

**위치**: `motor.py:18-498`

## 실행 단계

### 1. 초기화 단계

#### 1.1 명령 큐 선처리
```python
# motor.py:40-51
while not command_queue.empty():
    slave_idx, cmd, value = command_queue.get_nowait()
    if cmd == 'SET_VELOCITY': ...
    elif cmd == 'SET_AXIS': ...
```

**목적**: 재시도 시에도 설정이 유지되도록 미리 저장

#### 1.2 EtherCAT 초기화 (재시도 로직)
```python
# motor.py:54-143
for retry_count in range(max_init_retries):
    master.open(adapter_name)
    found_slaves = master.config_init()
    # PDO 설정, SDO 설정
```

**재시도 횟수**: 최대 3회
**대기 시간**: 1초

**관련**: [[202602031204-csp-mode|CSP 모드 PDO 설정]]

#### 1.3 OP 상태 전환
```python
# motor.py:146-195
master.state = pysoem.OP_STATE
master.write_state()
```

**중요**: OP 상태 직후 현재 위치로 `target_pulse` 초기화하여 Following Error 방지

### 2. 메인 제어 루프

#### 루프 구조
```python
# motor.py:197-425
while is_running:
    # 1. 명령 수신
    # 2. PDO 통신
    # 3. 상태 업데이트 및 제어
    # 4. 사이클 타임 유지
```

**사이클 타임**: 기본 10ms (조정 가능)

#### 2.1 명령 처리
```python
# motor.py:203-225
while not command_queue.empty():
    slave_idx, cmd, value = command_queue.get_nowait()
    if cmd == 'MOVE_TO_MM':
        trajectory_commands.append((slave_idx, value))
```

**명령 타입**:
- `STOP_ALL`: 루프 종료
- `SET_AXIS`: 축 변경
- `SET_ORIGIN`: 원점 설정
- `MOVE_TO_MM`: 위치 이동

#### 2.2 궤적 생성 (동기화 로직)

```python
# motor.py:227-302
if trajectory_commands:
    common_start_time = time.monotonic()  # 공통 시작 시간

    # 1단계: 각 모터의 이동 거리 계산
    # 2단계: 가장 긴 duration 선택
    max_duration = max(traj['duration'] for traj in trajectory_data)

    # 3단계: 모든 모터가 같은 시간에 완료되도록 설정
    state['trajectory'] = {
        'start': current_pos,
        'end': target_pos,
        'duration': max_duration,  # 동기화!
        'start_time': common_start_time
    }
```

**핵심**: [[202602031210-synchronization|다축 동기화]] - 모든 모터가 동시에 도착

#### 2.3 PDO 통신
```python
# motor.py:305-307
master.send_processdata()   # TX: Controlword + Target Position
master.receive_processdata() # RX: Statusword + Actual Position
```

**주기**: 매 사이클마다 (10ms)

#### 2.4 안전 장치 - Fault 연동
```python
# motor.py:310-328
if fault_detected:
    for i in range(num_slaves):
        local_motor_states[i]['trajectory'] = None
        local_motor_states[i]['target_pulse'] = current_pos
```

**정책**: 한 모터라도 Fault 발생 시 **모든 모터 즉시 정지**

**관련**: [[202602031209-fault-handling|Fault 처리]]

#### 2.5 상태 머신 및 제어
```python
# motor.py:330-420
for i in range(num_slaves):
    status = _read_status_word(slave)

    # CiA 402 상태 머신
    if (status & 0x004F) == 0x0040:  # Switch On Disabled
        cw = CW_SHUTDOWN
    elif (status & 0x006F) == 0x0021:  # Ready to Switch On
        cw = CW_SWITCH_ON
    # ...

    # 궤적 보간
    if state['trajectory'] is not None:
        # S-Curve 보간
        smooth_progress = (1.0 - math.cos(math.pi * progress)) / 2.0
        target_position = int(start + (end - start) * smooth_progress)
```

**관련**:
- [[202602031205-cia402-state-machine|CiA 402 상태 머신]]
- [[202602031207-trajectory-interpolation|궤적 보간]]

### 3. 종료 단계

```python
# motor.py:429-497
finally:
    # 1. 현재 위치 고정
    for i in range(num_slaves):
        current_pos = _read_actual_position(master.slaves[i])
        master.slaves[i].output = struct.pack("<H", CW_ENABLE_OPERATION) + struct.pack("<i", current_pos)

    # 2. Disable Operation (0x0007)
    # 3. Shutdown (0x0006)
    # 4. Disable Voltage (0x0000)
    # 5. INIT 상태로 전환
```

**안전 종료 시퀀스**: [[202602031205-cia402-state-machine|CiA 402 표준]]을 따름

## 성능 최적화

### 사이클 타임 보장
```python
# motor.py:421-424
loop_duration = time.monotonic() - loop_start_time
sleep_time = cycle_time_s - loop_duration
if sleep_time > 0: time.sleep(sleep_time)
```

### Lock-free 공유 메모리
```python
# motor.py:412-419
with lock:
    shared_states[base_idx] = status
    shared_states[base_idx + 1] = is_moving
    shared_states[base_idx + 2] = current_pos
    shared_states[base_idx + 3] = offset
```

**최소화 원칙**: Lock 사용을 최소화하여 실시간성 보장

## 디버깅 정보

주기적으로 상태 변화를 출력:
```python
# motor.py:336-343
if state['last_status'] != status:
    if state['trajectory'] is not None or (status & 0x0008):
        print(f"[상태] 모터 {i} 상태 변화: 0x{old:04X} → 0x{new:04X}")
```

## 관련 노트

- [[202602031201-ethercat-bus-class|EtherCATBus 클래스]]
- [[202602031204-csp-mode|CSP 모드]]
- [[202602031205-cia402-state-machine|CiA 402 상태 머신]]
- [[202602031207-trajectory-interpolation|궤적 보간]]
- [[202602031210-synchronization|다축 동기화]]

---

**생성일**: 2026-02-03
**태그**: #core-logic #realtime #control-loop
