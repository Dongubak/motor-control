# EtherCAT 모터 제어 시스템

#ethercat #motor-control #industrial-automation #python

> ← [[MOC-motor-control|진입점으로 돌아가기]]

## 개요

EtherCAT 프로토콜을 사용한 산업용 모터 위치 제어 시스템. PySOEM 라이브러리를 통해 실시간 통신을 구현하고, CiA 402 표준을 준수한다.

## 핵심 구성요소

### 아키텍처
- **통신 계층**: [[202602031203-ethercat-process-loop|EtherCAT 프로세스 루프]]
- **제어 계층**: [[202602031201-ethercat-bus-class|EtherCATBus 클래스]], [[202602031202-motor-class|Motor 클래스]]
- **프로토콜**: [[202602031204-csp-mode|CSP 모드]], [[202602031205-cia402-state-machine|CiA 402 상태 머신]]

### 주요 기능
- [[202602031208-position-control|위치 제어]] (mm 단위)
- [[202602031207-trajectory-interpolation|S-Curve 궤적 보간]]
- [[202602031210-synchronization|다축 동기화]]
- [[202602031209-fault-handling|Fault 자동 복구]]

## 실행 흐름

[[202602031206-main-execution-flow|메인 실행 흐름]]을 참조하라.

1. 어댑터 연결 및 초기화
2. 모터 상태 머신 전환
3. 원점 설정
4. 위치 제어 명령 실행
5. 안전 종료

## 기술 사양

| 항목 | 값 |
|------|-----|
| 통신 프로토콜 | EtherCAT |
| 제어 표준 | CiA 402 |
| 동작 모드 | CSP (Cyclic Synchronous Position) |
| 사이클 타임 | 10ms |
| Position Factor | 2:1 |
| 펄스/회전 | 8,388,608 |

## 파일 구조

```
motor_control/
├── motor.py          # [[202602031201-ethercat-bus-class|버스]] 및 [[202602031202-motor-class|모터]] 클래스
├── main.py           # [[202602031206-main-execution-flow|메인 실행 파일]]
└── check_adapter.py  # 네트워크 어댑터 확인
```

## 참고 문헌

- CiA 402 Device Profile for Drives and Motion Control
- EtherCAT Technology Group Specifications
- PySOEM Documentation

---

**생성일**: 2026-02-03
**태그**: #index #system-overview
