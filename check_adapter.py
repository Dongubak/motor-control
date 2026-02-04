"""
EtherCAT 어댑터 확인 스크립트
사용 가능한 네트워크 어댑터 목록을 출력합니다.
"""
import pysoem

def check_adapters():
    print("=" * 60)
    print("사용 가능한 네트워크 어댑터 목록")
    print("=" * 60)

    adapters = pysoem.find_adapters()

    if not adapters:
        print("어댑터를 찾을 수 없습니다.")
        print("\n확인 사항:")
        print("1. Npcap 또는 WinPcap이 설치되어 있는지 확인")
        print("2. 관리자 권한으로 실행했는지 확인")
        return

    for i, adapter in enumerate(adapters):
        print(f"\n[{i}] {adapter.name}")
        print(f"    설명: {adapter.desc}")
        print(f"    사용 경로: r'{adapter.name}'")

    print("\n" + "=" * 60)
    print("main.py에서 사용할 어댑터 경로를 복사하세요.")
    print("=" * 60)

if __name__ == "__main__":
    check_adapters()
