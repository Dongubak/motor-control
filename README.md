# EtherCAT 모터 제어 시스템

EtherCAT 프로토콜을 사용한 산업용 모터 제어 시스템입니다.

## 파일 구조

```
motor_control/
├── motor.py              # 핵심 라이브러리 (EtherCATBus, Motor 클래스)
├── main.py               # 1개 모터 제어 (현재 버전)
├── main_dual.py          # 2개 모터 동시 제어 (확장 버전)
├── main_legacy.py        # 원본 백업
├── check_adapter.py      # 네트워크 어댑터 확인 스크립트
└── requirements.txt      # 의존성 패키지
```

## 사용 방법

### 1단계: 어댑터 확인

네트워크 어댑터를 확인합니다:

```bash
python check_adapter.py
```

출력된 어댑터 경로를 복사하여 main.py의 `adapter` 변수에 붙여넣으세요.

### 2단계: 모터 제어 실행

**1개 모터 제어:**
```bash
python main.py
```

**2개 모터 동시 제어:**
```bash
python main_dual.py
```

## 주요 설정값

### main.py (1개 모터)
- **모터 수**: 1개
- **슬레이브 인덱스**: 0
- **축**: z축 (5.9996 mm/rev)
- **속도**: 50 RPM
- **가감속도**: 50 RPM/s
- **이동 거리**: -50mm → 0mm (원점 복귀)

### main_dual.py (2개 모터)
- **모터 수**: 2개
- **슬레이브 인덱스**: 0, 1
- **축**: z축 (동기화)
- **속도**: 50 RPM
- **동시 이동**: 두 모터가 같은 시간에 목표에 도달

## 기술 사양

- **통신 프로토콜**: EtherCAT (PySOEM)
- **제어 표준**: CiA 402
- **동작 모드**: CSP (Cyclic Synchronous Position)
- **사이클 타임**: 10ms
- **보간 방식**: S-Curve (부드러운 가감속)

## 안전 기능

- Fault 상태 자동 감지 및 복구
- Following Error 모니터링 (200M pulse = 약 143mm)
- 한 모터 Fault 시 모든 모터 즉시 정지
- 안전한 종료 시퀀스

## 확장 가이드

### 1개 → 2개 모터로 확장

1. [main.py:7](main.py#L7)에서 `NUM_MOTORS = 2`로 변경
2. 주석 처리된 motor2 관련 코드 활성화:
   - 모터 핸들 가져오기
   - 축 설정
   - 속도/가감속도 설정
   - 원점 설정
   - 이동 명령
   - 상태 검증 루프

또는 `main_dual.py`를 복사하여 사용하세요.

## 문제 해결

### 어댑터를 찾을 수 없음
- Npcap 또는 WinPcap 설치 확인
- 관리자 권한으로 실행

### 모터가 Fault 상태
- 모터 전원 확인
- EtherCAT 케이블 연결 확인
- 드라이버 설정 확인 (0x6065, 0x6067)

### 타임아웃 발생
- 사이클 타임 확인 (기본 10ms)
- 네트워크 부하 확인
- 속도/가감속도 값 조정

## 개발 정보

- **PySOEM**: EtherCAT 마스터 라이브러리
- **멀티프로세싱**: 실시간 통신 루프 분리
- **Position Factor**: 2:1 (드라이버 스케일 고려)

## 라이선스

프로젝트 라이선스를 명시하세요.
