# Git 및 GitHub 연동 설정

#git #github #version-control #setup

> ← [[MOC-motor-control|진입점]] / [[202602031200-ethercat-motor-control-system|시스템 개요]]

## 개요

motor_control 프로젝트를 Git으로 버전 관리하고 GitHub 원격 저장소와 연동한다.

## 저장소 정보

| 항목 | 값 |
|------|-----|
| 로컬 경로 | `C:\Users\x0011\OneDrive\바탕 화면\motor_control` |
| 원격 저장소 | https://github.com/Dongubak/motor-control |
| 기본 브랜치 | `main` |
| 초기 커밋일 | 2026-02-04 |

---

## 설정 과정

### 1단계: Git 로컬 저장소 초기화

```bash
cd "c:\Users\x0011\OneDrive\바탕 화면\motor_control"
git init
```

**결과**: `.git` 디렉토리 생성

### 2단계: Git 사용자 설정

```bash
git config user.name "dongubak"
git config user.email "x001125@gmail.com"
```

### 3단계: .gitignore 파일 생성

```gitignore
# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
build/
venv/

# IDE
.vscode/
.idea/

# Claude Code
.claude/

# Backup directory
backup/

# OS generated files
.DS_Store
Thumbs.db
```

**포함된 파일**: PDF, XML, CSV 문서 포함

### 4단계: GitHub 원격 저장소 생성

1. https://github.com 접속
2. **New repository** 클릭
3. 설정:
   - Repository name: `motor-control`
   - Description: `EtherCAT 모터 제어 시스템 (PySOEM, CiA 402)`
   - ⚠️ README, .gitignore 추가하지 않음 (로컬에 이미 존재)
4. **Create repository** 클릭

### 5단계: 원격 저장소 연결

```bash
git remote add origin https://github.com/dongubak/motor-control.git
git remote -v
```

**확인**:
```
origin  https://github.com/dongubak/motor-control.git (fetch)
origin  https://github.com/dongubak/motor-control.git (push)
```

### 6단계: 초기 커밋 및 푸시

```bash
# 파일 스테이징
git add -A

# 커밋
git commit -m "Initial commit: EtherCAT motor control system"

# 브랜치 이름 변경 (master → main)
git branch -M main

# 푸시
git push -u origin main
```

---

## 프로젝트 파일 구조

```
motor_control/
├── .git/                  # Git 저장소
├── .gitignore             # Git 제외 파일 목록
├── README.md              # 프로젝트 설명
├── requirements.txt       # Python 의존성
│
├── motor.py               # 핵심 라이브러리
├── motor_safe.py          # 안전 기능 추가 버전
│
├── main.py                # 단일 모터 예제
├── main_dual.py           # 2개 모터 동기화 예제
├── main_dual_safe.py      # 안전 동기화 예제
├── main_legacy.py         # 원본 백업
│
├── check_adapter.py       # 어댑터 확인 유틸리티
├── mapping_test.py        # PDO 매핑 테스트
├── rplidar.py             # LiDAR 관련 (별도)
│
├── motor_params.csv       # 모터 파라미터 데이터
├── motor_params.txt       # 모터 파라미터 (텍스트)
│
└── [문서]
    ├── [LSMecapion] iX7NH_KOR_Ver1.6_251001.pdf
    └── LS ELECTRIC EtherCAT Drive_V0.98e_20250711.xml
```

---

## 자주 사용하는 Git 명령어

### 상태 확인

```bash
git status          # 현재 상태
git log --oneline   # 커밋 히스토리
git diff            # 변경 내용 확인
```

### 변경 사항 커밋

```bash
git add <file>      # 특정 파일 스테이징
git add -A          # 모든 변경 스테이징
git commit -m "메시지"
```

### 원격 저장소 동기화

```bash
git push            # 원격으로 푸시
git pull            # 원격에서 풀
git fetch           # 변경 사항만 가져오기
```

### 브랜치 관리

```bash
git branch              # 브랜치 목록
git checkout -b <name>  # 새 브랜치 생성 및 전환
git merge <branch>      # 브랜치 병합
```

---

## GitHub CLI (선택사항)

GitHub CLI를 설치하면 터미널에서 직접 저장소를 관리할 수 있다.

### 설치

```bash
winget install GitHub.cli
```

### 인증

```bash
gh auth login
```

### 주요 명령어

```bash
gh repo create          # 저장소 생성
gh pr create            # Pull Request 생성
gh issue list           # 이슈 목록
```

---

## 문제 해결

### 인증 오류

**증상**: `fatal: Authentication failed`

**해결**:
1. GitHub Personal Access Token 생성
   - GitHub → Settings → Developer settings → Personal access tokens
2. 토큰을 비밀번호로 사용

### 대용량 파일 경고

**증상**: `warning: File is large`

**해결**:
- Git LFS 사용 (100MB 이상 파일)
- 또는 .gitignore에 추가

### 원격 저장소 URL 변경

```bash
git remote set-url origin <new-url>
```

---

## 관련 노트

- [[202602031200-ethercat-motor-control-system|시스템 개요]]
- [[MOC-motor-control|진입점]]

---

**생성일**: 2026-02-04
**태그**: #git #github #devops #version-control
