# CSV 모드 Cross Coupling 제어 설명서

## 1. CSV 모드란?

**CSV (Cyclic Synchronous Velocity, 주기적 동기 속도 제어)**는 CiA 402 표준 모드 9번이다.

| 모드 | 번호 | 마스터가 보내는 명령 | 드라이브 내부 루프 |
|------|------|---------------------|-------------------|
| CSP  | 8    | 위치 (0x607A)        | 위치 + 속도 + 전류 |
| **CSV** | **9** | **속도 (0x60FF)**  | **속도 + 전류** |
| CST  | 10   | 토크 (0x6071)        | 전류 |

### CSP와 CSV의 핵심 차이

```
CSP 모드:
  마스터 → 위치 명령 → 드라이브 (위치 루프 내장) → 모터

CSV 모드:
  마스터 → 속도 명령 → 드라이브 (속도 루프 내장) → 모터
  마스터가 직접 위치 루프(궤적 생성 + 위치 유지 P 제어) 담당
```

---

## 2. PDO 구성

### RxPDO (마스터 → 드라이브, 6 bytes)

| 오프셋 | 오브젝트 | 크기 | 설명 |
|--------|----------|------|------|
| 0      | 0x6040   | 16bit | Controlword |
| 2      | 0x60FF   | 32bit | Target Velocity (pulse/sec) |

### TxPDO (드라이브 → 마스터, 10 bytes)

| 오프셋 | 오브젝트 | 크기 | 설명 |
|--------|----------|------|------|
| 0      | 0x6041   | 16bit | Statusword |
| 2      | 0x6064   | 32bit | Actual Position (pulse) |
| 6      | 0x606C   | 32bit | Actual Velocity (pulse/sec) |

> **CSP와 비교**: CSP는 RxPDO에 0x607A (Target Position)을 사용.
> CSV는 0x60FF (Target Velocity)로 교체된다.

---

## 3. 사다리꼴 속도 프로파일 (TrapezoidalProfile)

CSV 모드에서는 드라이브 내부에 위치 루프가 없으므로,
마스터(이 코드)가 직접 속도 프로파일을 생성하여 매 사이클 전송한다.

### 3단계 구조

```
속도 (pulse/sec)
  │
V │        ┌──────────────┐
  │       /                \
  │      / ← 가속 →  → 감속 →\
  │─────/                    \──────
  └─────────────────────────────── 시간 (sec)
       t_a       t_coast    t_a
```

| 구간 | 시간 | 속도 |
|------|------|------|
| 가속 | 0 ~ t_a | v = A × t |
| 등속 | t_a ~ t_a + t_coast | v = V_peak |
| 감속 | t_a + t_coast ~ t_total | v = V_peak - A × (t - t_a - t_coast) |

- `t_a = V_peak / A` (가속에 필요한 시간)
- `t_coast = (거리 - 2 × d_accel) / V_peak`

### 삼각형 프로파일 (이동 거리 짧을 때)

이동 거리 D < 2 × d_accel 이면 최대 속도에 도달하지 못하므로
삼각형 프로파일로 자동 전환된다:

```
V_peak = sqrt(A × D)     (최대 속도 자동 계산)
t_total = 2 × V_peak / A
```

---

## 4. Cross Coupling 알고리즘 (속도 기반)

### 기본 원리

CSP 모드: 위치 보정 → `target_pos -= Kc × pos_error`
CSV 모드: **속도 보정** → `target_vel -= Kc × pos_error`

```
이동 중인 모터들의 평균 위치:
  avg_pos = (pos[M1] + pos[M2]) / 2

각 모터의 위치 오차:
  error[i] = pos[i] - avg_pos

속도 보정값 (pulse/sec):
  correction[i] = Kc × error[i]

보정 후 속도 명령:
  v_cmd[i] = v_profile[i] - correction[i]
```

### 게인 단위 설명

```
Kc의 단위: 1/sec  (= (mm/sec 보정) / (mm 오차))

Kc = 1.0 의 의미:
  1mm 위치 오차 → 1mm/sec 속도 보정

예시 (이동 속도 5mm/sec, 1mm 오차):
  Kc = 0.5 → 0.5mm/sec 보정 (오차의 10%)
  Kc = 1.0 → 1.0mm/sec 보정 (오차의 20%)
  Kc = 5.0 → 5.0mm/sec 보정 (오차의 100%) ← 발산 위험
```

### 게인 튜닝 가이드

| Kc 값 | 특성 | 권장 상황 |
|-------|------|-----------|
| 0.1 ~ 0.3 | 약한 결합, 안정적 | 초기 테스트 |
| 0.5 ~ 1.0 | 중간 결합 | 일반 동기 이동 |
| 1.0 ~ 3.0 | 강한 결합 | 정밀 동기 필요 시 |
| > 5.0 | 발산 가능 | 사용 금지 |

**튜닝 절차:**
1. `ENABLE_COUPLING = False`로 기준 동기 오차 측정
2. `Kc = 0.5`부터 시작
3. 최대 위치 차이가 줄어드는지 확인
4. 진동/발산 없이 오차가 최소화되는 Kc 선택
5. 발산 증상: 위치 차이가 시간이 지나도 수렴하지 않고 증가

---

## 5. 위치 유지 P 제어

CSV 모드는 드라이브 내부에 위치 루프가 없으므로,
궤적 완료 후에도 마스터가 위치를 유지해야 한다.

### 동작 원리

```
궤적 시간 완료 후:
  pos_error = target_pos - actual_pos
  v_hold    = Kp × pos_error

  단, |v_hold| ≤ POSITION_HOLD_MAX_VEL (기본 5mm/sec)

|pos_error| < POSITION_TOLERANCE (기본 0.018mm):
  → 완료 판정, v_cmd = 0
```

### 관련 상수 (motor_csv.py)

```python
POSITION_HOLD_GAIN         = 2.0   # Kp [1/sec]
POSITION_HOLD_MAX_VEL_MM_S = 5.0   # 위치 유지 최대 속도 (mm/sec)
POSITION_TOLERANCE_PULSE   = 50_000  # 완료 판정 오차 (~0.018mm)
```

---

## 6. 파라미터 설명 (motor_csv_main.py)

```python
RPM             = 30     # 최대 이동 속도
                         # 단위: RPM (출력축/모터축 여부는 기어비 설정에 따라 다름)

ACCEL_RPM_PER_S = 30     # 가감속
                         # 단위: RPM/sec

TARGET_MM       = 50     # 이동 목표 거리 (mm)

COUPLING_GAIN   = 0.5    # Cross Coupling 게인 [1/sec]
                         # 1mm 오차 → 0.5mm/sec 속도 보정

MAX_SYNC_ERR_MM = 10.0   # 긴급 정지 임계값 (mm)
                         # 이동 평균이 이 값 초과 시 모든 모터 즉시 정지

MA_WINDOW       = 5      # 이동 평균 윈도우 (사이클 수)
                         # 10ms × 5 = 50ms 평균 → 노이즈 스파이크 차단
```

---

## 7. 파일 구조

```
motor_csv.py          드라이버 라이브러리
│
├── TrapezoidalProfile        사다리꼴 속도 프로파일 생성기
├── _ethercat_process_loop_csv  EtherCAT 제어 루프 (별도 프로세스)
├── EtherCATBusCSV            버스 관리 클래스
└── MotorCSV                  개별 모터 제어 클래스

motor_csv_main.py     실행 파일
└── main()            테스트 시퀀스 (Coupling ON/OFF 비교)

motor_csv_manual.md   이 문서
```

---

## 8. 실행 방법

```bash
python motor_csv_main.py
```

### 실행 순서

1. EtherCAT 초기화 및 슬레이브 검색
2. CSV 모드 PDO 매핑 설정 (SDO)
3. OP 상태 전환
4. 원점 설정 (`set_origin`)
5. 테스트 1: Coupling ON, `-TARGET_MM` 이동
6. 테스트 2: Coupling OFF, 원점 복귀
7. 결과 비교 출력
8. 버스 종료 후 Fault 코드 SDO 조회

---

## 9. CSP vs CSV 비교

| 항목 | CSP (motor_coupling.py) | CSV (motor_csv.py) |
|------|------------------------|-------------------|
| 드라이브 명령 | 위치 (0x607A) | 속도 (0x60FF) |
| 궤적 생성 | 코사인 보간 | 사다리꼴 속도 프로파일 |
| Cross Coupling | 위치 보정 | 속도 보정 |
| 게인 단위 | 무차원 (비율) | 1/sec |
| 위치 유지 | 드라이브 내장 | 마스터 P 제어 |
| TxPDO | SW + Pos | SW + Pos + Vel |
| 가감속 제어 | 드라이브 0x6083/0x6084 | 소프트웨어 프로파일 |

### CSV 모드의 장점 (동기화 측면)

- 속도 보정이 직접적: 위치 오차 → 속도 즉시 조정
- 매 사이클 속도가 재계산되므로 누적 오차 없음
- 두 모터의 속도를 동시에 조정하여 자연스러운 동기화

### CSV 모드의 주의사항

- 드라이브 내부 위치 루프 없음 → 정지 중 외력에 의한 위치 드리프트 가능
- 위치 유지 P 게인 (`POSITION_HOLD_GAIN`) 너무 높으면 진동 발생
- 속도 명령이 0이 되어도 관성으로 약간의 오버슈트 발생 (감속 구간에서 처리)
