# EtherCATBus 클래스

#ethercat #multiprocessing #bus-management

> ← [[MOC-motor-control|진입점]] / [[202602031200-ethercat-motor-control-system|시스템 개요]]

## 개요

EtherCAT 통신 버스를 관리하는 최상위 클래스. 멀티프로세싱을 통해 실시간 통신 루프를 별도 프로세스에서 실행한다.

## 클래스 구조

```python
class EtherCATBus:
    def __init__(self, adapter_name: str, num_slaves: int, cycle_time_ms: int = 50)
    def start(self)
    def stop(self)
```

**위치**: `motor.py:554-587`

## 주요 책임

### 1. 프로세스 간 통신 관리

```python
self._command_queue = mp.Queue()           # 명령 큐
self._shared_states = mp.Array('d', ...)   # 공유 메모리
self._lock = mp.Lock()                     # 동기화 락
```

**관련**: [[202602031203-ethercat-process-loop|EtherCAT 프로세스 루프]]

### 2. 모터 인스턴스 제공

```python
self.motors: List[Motor] = [Motor(i, ...) for i in range(num_slaves)]
```

각 슬레이브에 대한 [[202602031202-motor-class|Motor]] 인스턴스를 생성하여 제공한다.

### 3. 라이프사이클 관리

- `start()`: [[202602031203-ethercat-process-loop|통신 프로세스]] 시작
- `stop()`: 안전한 종료 시퀀스 실행

## 공유 상태 구조

각 모터당 4개의 상태값을 공유 메모리에 저장:

| 인덱스 | 내용 | 타입 |
|--------|------|------|
| i*4 + 0 | status_word | int |
| i*4 + 1 | move_state (0=정지, 1=이동) | int |
| i*4 + 2 | current_position_pulse | int |
| i*4 + 3 | offset_pulse | int |

**참조**: `motor.py:559`

## 멀티프로세싱 패턴

메인 프로세스와 통신 프로세스를 분리하여:
- **메인 프로세스**: 사용자 로직 실행
- **통신 프로세스**: 실시간 EtherCAT 통신

**장점**:
- 통신 사이클 타임 보장 (10ms)
- GIL(Global Interpreter Lock) 영향 최소화
- 안정적인 실시간 성능

## 예제

```python
bus = EtherCATBus(
    adapter_name=r'\Device\NPF_{...}',
    num_slaves=1,
    cycle_time_ms=10
)
bus.start()
motor = bus.motors[0]
# ... 모터 제어 ...
bus.stop()
```

## 관련 노트

- [[202602031200-ethercat-motor-control-system|시스템 개요]]
- [[202602031202-motor-class|Motor 클래스]]
- [[202602031203-ethercat-process-loop|EtherCAT 프로세스 루프]]

---

**생성일**: 2026-02-03
**태그**: #class #architecture
