# PDO 통신 (Process Data Object)

#ethercat #pdo #communication #multi-slave

> ← [[MOC-motor-control|진입점]] / [[202602031200-ethercat-motor-control-system|시스템 개요]]

## 개요

PDO(Process Data Object)는 EtherCAT에서 실시간 데이터를 교환하는 메커니즘이다. 매 사이클마다 마스터와 슬레이브 간에 제어 데이터(RxPDO)와 상태 데이터(TxPDO)를 교환한다.

## PDO 개념

### SDO vs PDO

| 구분 | SDO (Service Data Object) | PDO (Process Data Object) |
|------|---------------------------|---------------------------|
| 용도 | 설정, 파라미터 | 실시간 제어 |
| 타이밍 | 비주기적 (요청-응답) | 주기적 (매 사이클) |
| 데이터 크기 | 가변 (최대 수 KB) | 고정 (수~수십 바이트) |
| 예시 | 속도 설정, 모드 변경 | 위치 명령, 상태 읽기 |

### RxPDO와 TxPDO

```
마스터 (PC)                    슬레이브 (드라이브)
    │                              │
    │──── RxPDO (명령) ──────────→│  Controlword, Target Position
    │                              │
    │←─── TxPDO (상태) ────────────│  Statusword, Actual Position
    │                              │
```

**RxPDO**: Receive PDO (슬레이브 입장에서 받는 데이터)
**TxPDO**: Transmit PDO (슬레이브 입장에서 보내는 데이터)

---

## 단일 슬레이브 PDO 구조

### [[202602031204-csp-mode|CSP 모드]] PDO 매핑

#### RxPDO (마스터 → 슬레이브)

```python
# motor.py:535-541
slave.sdo_write(0x1600, 1, struct.pack('<I', 0x60400010))  # Controlword (2 bytes)
slave.sdo_write(0x1600, 2, struct.pack('<I', 0x607A0020))  # Target Position (4 bytes)
```

| 오프셋 | 객체 | 이름 | 크기 |
|--------|------|------|------|
| 0 | 0x6040 | Controlword | 2 bytes |
| 2 | 0x607A | Target Position | 4 bytes |
| **합계** | | | **6 bytes** |

#### TxPDO (슬레이브 → 마스터)

```python
# motor.py:543-550
slave.sdo_write(0x1A00, 1, struct.pack('<I', 0x60410010))  # Statusword (2 bytes)
slave.sdo_write(0x1A00, 2, struct.pack('<I', 0x60640020))  # Position Actual (4 bytes)
```

| 오프셋 | 객체 | 이름 | 크기 |
|--------|------|------|------|
| 0 | 0x6041 | Statusword | 2 bytes |
| 2 | 0x6064 | Position Actual Value | 4 bytes |
| **합계** | | | **6 bytes** |

---

## 2개 슬레이브 PDO 구조

### EtherCAT 프레임 구조

2개 모터를 사용할 때, 하나의 EtherCAT 프레임에 모든 데이터가 포함된다:

```
┌─────────────────────────────────────────────────────────────────┐
│                    EtherCAT 프레임 (1개)                         │
├─────────────────────────────────────────────────────────────────┤
│ Ethernet Header │ EtherCAT Header │ PDO 데이터 │ FCS           │
└─────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────┐
│                      PDO 데이터 영역                             │
├────────────────────────┬────────────────────────────────────────┤
│    슬레이브 0 (6+6)    │         슬레이브 1 (6+6)               │
├────────────────────────┼────────────────────────────────────────┤
│ Output(6) │ Input(6)   │    Output(6)    │     Input(6)        │
│ (RxPDO)   │ (TxPDO)    │    (RxPDO)      │     (TxPDO)         │
└────────────────────────┴────────────────────────────────────────┘
```

**총 PDO 크기**: (6 + 6) × 2 = **24 bytes**

### 메모리 레이아웃 상세

#### Output (마스터 → 슬레이브)

```
바이트 오프셋:  0   1   2   3   4   5   6   7   8   9  10  11
              ├───────┼───────────────┼───────┼───────────────┤
슬레이브 0:   │ CW(0) │ Target Pos(0) │       │               │
슬레이브 1:   │       │               │ CW(1) │ Target Pos(1) │
              └───────┴───────────────┴───────┴───────────────┘

CW = Controlword (2 bytes)
Target Pos = Target Position (4 bytes, signed int32)
```

#### Input (슬레이브 → 마스터)

```
바이트 오프셋:  0   1   2   3   4   5   6   7   8   9  10  11
              ├───────┼───────────────┼───────┼───────────────┤
슬레이브 0:   │ SW(0) │ Actual Pos(0) │       │               │
슬레이브 1:   │       │               │ SW(1) │ Actual Pos(1) │
              └───────┴───────────────┴───────┴───────────────┘

SW = Statusword (2 bytes)
Actual Pos = Position Actual Value (4 bytes, signed int32)
```

---

## 코드에서의 PDO 접근

### PySOEM 데이터 구조

```python
# motor.py:78-79
slave = master.slaves[i]  # 슬레이브 객체

# Output (마스터 → 슬레이브)
slave.output = bytes  # RxPDO 데이터

# Input (슬레이브 → 마스터)
slave.input = bytes   # TxPDO 데이터
```

**중요**: 각 슬레이브는 자신의 `output`과 `input` 버퍼를 갖는다.

### Output 쓰기 (명령 전송)

```python
# motor.py:509-511
def _write_csp_outputs(slave, cw, target_position):
    """CSP 모드용 출력 함수"""
    slave.output = struct.pack("<H", cw) + struct.pack("<i", target_position)
```

**구조**:
- `<H`: Little-endian unsigned short (2 bytes) - Controlword
- `<i`: Little-endian signed int (4 bytes) - Target Position

### Input 읽기 (상태 수신)

```python
# motor.py:500-507
def _read_status_word(slave):
    return struct.unpack("<H", slave.input[:2])[0]

def _read_actual_position(slave):
    return struct.unpack("<i", slave.input[2:6])[0]
```

**바이트 슬라이싱**:
- `[:2]`: 처음 2바이트 (Statusword)
- `[2:6]`: 3~6번째 바이트 (Position Actual Value)

---

## 2개 모터 PDO 통신 흐름

### 통신 사이클 (10ms 주기)

```python
# motor.py:305-307 (프로세스 루프 내부)
master.send_processdata()    # 1. 모든 슬레이브에 Output 전송
master.receive_processdata() # 2. 모든 슬레이브에서 Input 수신
```

#### 1. send_processdata()

```
┌─────────────────────────────────────────────────────────────────┐
│                     송신 데이터 (Output)                         │
├─────────────────────────────────────────────────────────────────┤
│  master.slaves[0].output  │  master.slaves[1].output            │
│  (CW0 + TargetPos0)       │  (CW1 + TargetPos1)                 │
└─────────────────────────────────────────────────────────────────┘
                              ↓
                    EtherCAT 프레임 생성
                              ↓
              네트워크로 전송 (브로드캐스트)
                              ↓
┌─────────────┐         ┌─────────────┐
│ 슬레이브 0  │ ──────→ │ 슬레이브 1  │
│ (Motor 0)   │         │ (Motor 1)   │
│ 자신의 데이터│         │ 자신의 데이터│
│ 추출 & 적용 │         │ 추출 & 적용 │
└─────────────┘         └─────────────┘
```

#### 2. receive_processdata()

```
┌─────────────┐         ┌─────────────┐
│ 슬레이브 0  │ ←────── │ 슬레이브 1  │
│ Input 삽입  │         │ Input 삽입  │
└─────────────┘         └─────────────┘
                              ↓
              네트워크에서 프레임 수신
                              ↓
                    EtherCAT 프레임 파싱
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                     수신 데이터 (Input)                          │
├─────────────────────────────────────────────────────────────────┤
│  master.slaves[0].input   │  master.slaves[1].input             │
│  (SW0 + ActualPos0)       │  (SW1 + ActualPos1)                 │
└─────────────────────────────────────────────────────────────────┘
```

### 데이터 처리 순서

```python
# motor.py:330-420 (간략화)
for i in range(num_slaves):  # 모든 슬레이브에 대해
    slave = master.slaves[i]

    # 1. 상태 읽기 (Input)
    status = _read_status_word(slave)
    current_pos = _read_actual_position(slave)

    # 2. 제어 로직 (궤적 계산)
    target_position = calculate_trajectory(...)

    # 3. 명령 쓰기 (Output)
    _write_csp_outputs(slave, controlword, target_position)
```

---

## 동기화 메커니즘

### Distributed Clock (DC)

```python
# motor.py:82
slave.dc_sync(True, int(cycle_time_s * 1_000_000_000))  # 나노초 단위
```

**DC Sync 활성화**:
- 모든 슬레이브가 동일한 시간 기준 사용
- 10ms 주기로 동기화된 인터럽트 발생
- 나노초 단위 정밀도

### 동기화된 데이터 적용

```
시간 ─────────────────────────────────────────────────────→

t=0ms      t=10ms     t=20ms     t=30ms
  │          │          │          │
  ▼          ▼          ▼          ▼
┌────┐     ┌────┐     ┌────┐     ┌────┐
│Send│     │Send│     │Send│     │Send│  마스터
└────┘     └────┘     └────┘     └────┘
  │          │          │          │
  │          │          │          │
  ▼          ▼          ▼          ▼
┌────┐     ┌────┐     ┌────┐     ┌────┐
│Sync│     │Sync│     │Sync│     │Sync│  슬레이브 0
└────┘     └────┘     └────┘     └────┘
┌────┐     ┌────┐     ┌────┐     ┌────┐
│Sync│     │Sync│     │Sync│     │Sync│  슬레이브 1
└────┘     └────┘     └────┘     └────┘

모든 슬레이브가 같은 순간에 새 명령을 적용!
```

---

## 2개 모터 예제 코드

### main_dual.py에서의 사용

```python
# 버스 생성 (2개 슬레이브)
bus = EtherCATBus(adapter_name=adapter, num_slaves=2, cycle_time_ms=10)

motor1 = bus.motors[0]  # 슬레이브 인덱스 0
motor2 = bus.motors[1]  # 슬레이브 인덱스 1

# 동시 이동 명령 (큐에 추가)
motor1.move_to_position_mm(-50)
motor2.move_to_position_mm(-50)
```

### 내부 처리 과정

```
사용자 명령                    명령 큐                    프로세스 루프
────────────────────────────────────────────────────────────────────────
motor1.move_to_position_mm()  →  (0, 'MOVE_TO_MM', -50)
                                         ↓
motor2.move_to_position_mm()  →  (1, 'MOVE_TO_MM', -50)
                                         ↓
                                    다음 사이클에서
                                    일괄 처리
                                         ↓
                              ┌──────────────────────┐
                              │ trajectory_commands  │
                              │ = [(0, -50), (1, -50)]│
                              └──────────────────────┘
                                         ↓
                              common_start_time 설정
                                         ↓
                              각 모터에 궤적 생성
                                         ↓
                              PDO로 동시 전송
```

---

## PDO 관련 SDO 설정

### PDO 할당 레지스터

**RxPDO 할당 (0x1C12)**:
```python
# motor.py:535, 540-541
slave.sdo_write(0x1C12, 0, b'\x00')           # 할당 초기화
slave.sdo_write(0x1C12, 1, struct.pack('<H', 0x1600))  # RxPDO1 할당
slave.sdo_write(0x1C12, 0, b'\x01')           # 할당 완료 (1개)
```

**TxPDO 할당 (0x1C13)**:
```python
# motor.py:544, 549-550
slave.sdo_write(0x1C13, 0, b'\x00')           # 할당 초기화
slave.sdo_write(0x1C13, 1, struct.pack('<H', 0x1A00))  # TxPDO1 할당
slave.sdo_write(0x1C13, 0, b'\x01')           # 할당 완료 (1개)
```

### PDO 매핑 엔트리 형식

```
0x60400010
  ││││││└─ 비트 크기 (0x10 = 16비트 = 2바이트)
  ││└──── 서브인덱스 (0x00)
  └────── 인덱스 (0x6040 = Controlword)
```

---

## 성능 고려사항

### 지터 (Jitter)

| 원인 | 영향 | 해결책 |
|------|------|--------|
| OS 스케줄링 | 불규칙한 사이클 | 실시간 OS 사용 |
| 네트워크 지연 | 동기화 오차 | DC Sync 활성화 |
| 프로세스 부하 | 타이밍 변동 | 별도 프로세스 분리 |

### 현재 구현의 최적화

```python
# motor.py:421-424
loop_duration = time.monotonic() - loop_start_time
sleep_time = cycle_time_s - loop_duration
if sleep_time > 0:
    time.sleep(sleep_time)  # 남은 시간만 대기
```

**효과**: 처리 시간을 제외한 정확한 10ms 주기 유지

---

## 디버깅 팁

### PDO 데이터 확인

```python
# 디버깅용 코드 예시
print(f"슬레이브 0 Output: {slave0.output.hex()}")
print(f"슬레이브 0 Input: {slave0.input.hex()}")
print(f"슬레이브 1 Output: {slave1.output.hex()}")
print(f"슬레이브 1 Input: {slave1.input.hex()}")
```

### 예상 출력

```
슬레이브 0 Output: 0f00 12345678  (CW=0x000F, Pos=0x78563412)
슬레이브 0 Input:  2700 87654321  (SW=0x0027, Pos=0x21436587)
슬레이브 1 Output: 0f00 12345678
슬레이브 1 Input:  2700 87654321
```

**Little-endian 주의**: 바이트 순서가 뒤집힘

---

## 관련 노트

- [[202602031203-ethercat-process-loop|EtherCAT 프로세스 루프]] - PDO 통신 루프
- [[202602031204-csp-mode|CSP 모드]] - PDO 매핑 설정
- [[202602031210-synchronization|다축 동기화]] - 동기화 알고리즘
- [[202602031201-ethercat-bus-class|EtherCATBus 클래스]] - 버스 관리

---

**생성일**: 2026-02-04
**태그**: #communication #protocol #multi-slave #pdo
