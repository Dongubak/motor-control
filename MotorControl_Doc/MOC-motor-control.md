# 🎯 EtherCAT 모터 제어 - 진입점

#MOC #index #entry-point

> **이 노트는 모터 제어 시스템의 진입점입니다.**
> 아래 구조를 따라 원하는 주제로 이동하세요.

---

## 📚 전체 개요

[[202602031200-ethercat-motor-control-system|시스템 개요]]를 먼저 읽으면 전체 구조를 파악할 수 있습니다.

---

## 🗺️ 학습 경로

### 🚀 빠른 시작 (Quick Start)

1. [[202602031206-main-execution-flow|메인 실행 흐름]] - main.py가 어떻게 동작하는지
2. [[202602031202-motor-class|Motor 클래스]] - 모터를 어떻게 제어하는지
3. [[202602031208-position-control|위치 제어]] - mm 단위로 이동하는 방법

### 📖 심화 학습 (Deep Dive)

#### 아키텍처
```
EtherCATBus (버스 관리)
    └── Motor (사용자 인터페이스)
            └── _ethercat_process_loop (실시간 통신)
```

- [[202602031201-ethercat-bus-class|EtherCATBus 클래스]] - 전체 버스 관리
- [[202602031202-motor-class|Motor 클래스]] - 개별 모터 제어
- [[202602031203-ethercat-process-loop|EtherCAT 프로세스 루프]] ⭐ **핵심 로직**

#### 프로토콜 및 통신
```
EtherCAT (통신) → PDO (데이터 교환) → CiA 402 (상태 머신) → CSP (위치 제어)
```

- [[202602031211-pdo-communication|PDO 통신]] ⭐ **다중 슬레이브 통신** (NEW!)
- [[202602031204-csp-mode|CSP 모드]] - Cyclic Synchronous Position
- [[202602031205-cia402-state-machine|CiA 402 상태 머신]] - 모터 상태 전환

#### 제어 알고리즘
```
mm 입력 → 펄스 변환 → S-Curve 보간 → CSP 전송
```

- [[202602031208-position-control|위치 제어]] - 단위 변환 및 원점 설정
- [[202602031207-trajectory-interpolation|궤적 보간]] ⭐ **S-Curve 알고리즘**

#### 안전 및 고급 기능

- [[202602031209-fault-handling|Fault 처리]] - 에러 자동 복구
- [[202602031210-synchronization|다축 동기화]] - 여러 모터 동시 제어
- [[202602041300-safety-synchronization|안전 동기화 분석]] ⭐ **중량물 리프팅**
- [[202602051000-cross-coupling|Cross Coupling]] ⭐ **위치 오차 보정** (NEW!)

#### 인터페이스

- [[202602061000-pyqt-gui|PyQt5 GUI]] ⭐ **실시간 모니터링** (NEW!)

#### 개발 환경

- [[202602041400-git-github-setup|Git/GitHub 연동]]

---

## 📁 파일 매핑

| 소스 코드 | 관련 노트 |
|-----------|-----------|
| `motor.py:18-498` | [[202602031203-ethercat-process-loop\|프로세스 루프]] |
| `motor.py:554-587` | [[202602031201-ethercat-bus-class\|EtherCATBus]] |
| `motor.py:588-662` | [[202602031202-motor-class\|Motor]] |
| `main.py` | [[202602031206-main-execution-flow\|실행 흐름]] |
| `motor_coupling.py` | [[202602051000-cross-coupling\|Cross Coupling]] |
| `gui_motor_control.py` | [[202602061000-pyqt-gui\|PyQt5 GUI]] |

---

## 🔗 주요 백링크 허브

### 가장 많이 연결된 노트 (Hub Notes)

1. **[[202602031203-ethercat-process-loop|EtherCAT 프로세스 루프]]** - 7개 연결
   - 모든 제어 로직의 중심

2. **[[202602031204-csp-mode|CSP 모드]]** - 5개 연결
   - 위치 제어의 핵심 모드

3. **[[202602031205-cia402-state-machine|CiA 402 상태 머신]]** - 4개 연결
   - 모터 상태 전환의 표준

---

## 🏷️ 태그 인덱스

### 아키텍처
- `#class` - [[202602031201-ethercat-bus-class|EtherCATBus]], [[202602031202-motor-class|Motor]]
- `#architecture` - [[202602031201-ethercat-bus-class|EtherCATBus]]

### 프로토콜 및 통신
- `#ethercat` - [[202602031203-ethercat-process-loop|프로세스 루프]], [[202602031204-csp-mode|CSP]], [[202602031211-pdo-communication|PDO 통신]]
- `#cia402` - [[202602031205-cia402-state-machine|상태 머신]]
- `#protocol` - [[202602031204-csp-mode|CSP]], [[202602031205-cia402-state-machine|CiA 402]]
- `#pdo` - [[202602031211-pdo-communication|PDO 통신]]
- `#multi-slave` - [[202602031211-pdo-communication|PDO 통신]], [[202602031210-synchronization|다축 동기화]]

### 제어
- `#control` - [[202602031208-position-control|위치 제어]]
- `#motion-planning` - [[202602031207-trajectory-interpolation|궤적 보간]]
- `#synchronization` - [[202602031210-synchronization|다축 동기화]]
- `#cross-coupling` - [[202602051000-cross-coupling|Cross Coupling]]

### 인터페이스
- `#gui` - [[202602061000-pyqt-gui|PyQt5 GUI]]
- `#pyqt5` - [[202602061000-pyqt-gui|PyQt5 GUI]]
- `#monitoring` - [[202602061000-pyqt-gui|PyQt5 GUI]]

### 안전
- `#safety` - [[202602031209-fault-handling|Fault 처리]], [[202602041300-safety-synchronization|안전 동기화]]
- `#error-handling` - [[202602031209-fault-handling|Fault 처리]]
- `#critical` - [[202602041300-safety-synchronization|안전 동기화 분석]]

### 개발 환경
- `#git` - [[202602041400-git-github-setup|Git/GitHub 연동]]
- `#github` - [[202602041400-git-github-setup|Git/GitHub 연동]]
- `#version-control` - [[202602041400-git-github-setup|Git/GitHub 연동]]

---

## ❓ FAQ

**Q: 모터가 동작하지 않아요**
→ [[202602031205-cia402-state-machine|CiA 402 상태 머신]]에서 현재 상태 확인

**Q: Fault가 발생했어요**
→ [[202602031209-fault-handling|Fault 처리]]에서 복구 방법 확인

**Q: 두 모터를 동시에 움직이고 싶어요**
→ [[202602031210-synchronization|다축 동기화]] 참조

**Q: 이동 속도를 조절하고 싶어요**
→ [[202602031207-trajectory-interpolation|궤적 보간]]에서 duration 계산 확인

**Q: 2개 모터의 PDO 통신은 어떻게 동작하나요?**
→ [[202602031211-pdo-communication|PDO 통신]]에서 다중 슬레이브 구조 확인

**Q: apm-ec08am3k과 apm-ec08am3k2는 분해능이 다른가요**
→ 브레이크 여부만 다름 (분해능 확인.png 참고)


**Q: 중량물 리프팅에서 두 모터 동기화가 안전한가요?**
→ [[202602041300-safety-synchronization|안전 동기화 분석]]에서 현재 한계 및 개선 방안 확인

**Q: 두 모터의 위치 차이를 모니터링하나요?**
→ motor_safe.py, motor_coupling.py에서 구현 완료. [[202602061000-pyqt-gui|PyQt5 GUI]]에서 실시간 그래프 확인

**Q: Cross Coupling이 뭔가요?**
→ [[202602051000-cross-coupling|Cross Coupling]]에서 알고리즘 및 게인 튜닝 방법 확인

**Q: GUI로 모터를 제어할 수 있나요?**
→ [[202602061000-pyqt-gui|PyQt5 GUI]] - `python gui_motor_control.py`로 실행

---

## 📝 변경 이력

| 날짜 | 변경 내용 |
|------|-----------|
| 2026-02-06 | PyQt5 GUI 문서 추가 (16개 노트) |
| 2026-02-05 | Cross Coupling 문서 추가 (15개 노트) |
| 2026-02-04 | Git/GitHub 연동 문서 추가 (14개 노트) |
| 2026-02-04 | 안전 동기화 분석 노트 추가 (13개 노트) |
| 2026-02-04 | PDO 통신 노트 추가 (12개 노트) |
| 2026-02-03 | 초기 문서 작성 (11개 노트) |

---

**생성일**: 2026-02-03
**유형**: Map of Content (MOC)
