"""
reset_drives.py - 비정상 종료 후 서보 드라이버 복구 스크립트

비정상 종료(예외, 강제 종료, 크래시) 이후 서보 드라이버가 Enable 상태로
남아 있을 때 SDO 통신으로 드라이버를 안전하게 해제한다.

동작 원리:
- PDO 매핑 없이 SDO 통신만 사용 → 어떤 상태에서도 동작 가능
- config_init()으로 슬레이브를 PRE-OP까지 이동
- SDO로 Controlword에 Disable Voltage(0x0000) 전송
- EtherCAT 버스를 INIT 상태로 복귀 후 종료

사용법:
    python reset_drives.py
    python reset_drives.py --slaves 2      (슬레이브 수 지정)
    python reset_drives.py --check-only    (상태 확인만, 변경 없음)
"""

import pysoem
import struct
import time
import sys
import argparse

# ============================================================
# 설정 (main_dual_coupling.py와 동일하게 유지)
# ============================================================
ADAPTER = r'\Device\NPF_{CD2150F2-B355-4A6F-95BA-EB897A3726BF}'  # Realtek USB 2.5GbE

# CiA 402 Controlword
CW_DISABLE_VOLTAGE  = 0x0000
CW_SHUTDOWN         = 0x0006
CW_SWITCH_ON        = 0x0007
CW_ENABLE_OPERATION = 0x000F
CW_FAULT_RESET      = 0x0080

# EtherCAT 상태 이름 매핑
STATE_NAMES = {
    0x01: "INIT",
    0x02: "PRE-OP",
    0x03: "BOOT",
    0x04: "SAFE-OP",
    0x08: "OP",
    0x11: "INIT+ERR",
    0x12: "PRE-OP+ERR",
    0x14: "SAFE-OP+ERR",
}

# CiA 402 Statusword 상태 이름
def decode_cia402_state(status: int) -> str:
    if status & 0x0008:
        return "Fault"
    masked = status & 0x006F
    return {
        0x0000: "Not Ready to Switch On",
        0x0040: "Switch On Disabled",
        0x0021: "Ready to Switch On",
        0x0023: "Switched On",
        0x0027: "Operation Enabled",
        0x0007: "Quick Stop Active",
        0x000F: "Fault Reaction Active",
    }.get(masked, f"Unknown(0x{status:04X})")


def read_drive_status(slave) -> tuple[int, str]:
    """SDO로 Statusword를 읽고 (값, 상태명) 반환"""
    try:
        data = slave.sdo_read(0x6041, 0)
        status = struct.unpack("<H", data)[0]
        return status, decode_cia402_state(status)
    except Exception as e:
        return -1, f"읽기 실패({e})"


def reset_slave(slave, index: int, check_only: bool) -> bool:
    """단일 슬레이브를 Disable Voltage 상태로 전환. 성공 여부 반환."""
    status, state_name = read_drive_status(slave)

    print(f"\n  [Slave {index}] {slave.name}")
    print(f"    CiA 402 상태: {state_name}  (Statusword=0x{status:04X})")

    if check_only:
        return True

    if status == -1:
        print(f"    → SDO 통신 불가. 건너뜀.")
        return False

    # ① Fault 상태이면 먼저 Fault Reset
    if status & 0x0008:
        try:
            slave.sdo_write(0x6040, 0, struct.pack("<H", CW_FAULT_RESET))
            time.sleep(0.3)
            status, state_name = read_drive_status(slave)
            print(f"    Fault Reset 후: {state_name}")
        except Exception as e:
            print(f"    Fault Reset 실패: {e}")

    # ② Disable Voltage — 토크 즉시 해제
    try:
        slave.sdo_write(0x6040, 0, struct.pack("<H", CW_DISABLE_VOLTAGE))
        time.sleep(0.1)
        _, final_state = read_drive_status(slave)
        print(f"    → Disable Voltage 전송 완료: {final_state}")
        return True
    except Exception as e:
        print(f"    → Disable Voltage 실패: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="서보 드라이버 복구 스크립트")
    parser.add_argument("--slaves", type=int, default=0,
                        help="예상 슬레이브 수 (0=자동감지)")
    parser.add_argument("--check-only", action="store_true",
                        help="상태 확인만 수행, 드라이버 변경 없음")
    args = parser.parse_args()

    print("=" * 55)
    if args.check_only:
        print("  서보 드라이버 상태 확인 (변경 없음)")
    else:
        print("  서보 드라이버 복구 스크립트")
    print("=" * 55)
    print(f"  어댑터: {ADAPTER}")

    master = pysoem.Master()

    try:
        # ── 1단계: EtherCAT 마스터 열기 ──────────────────────────
        print("\n[1단계] EtherCAT 마스터 열기...")
        master.open(ADAPTER)
        print("  → 성공")

        # ── 2단계: 슬레이브 검색 ──────────────────────────────────
        print("\n[2단계] 슬레이브 검색 (PRE-OP 전환)...")
        num_found = master.config_init()
        print(f"  → {num_found}개 슬레이브 발견")

        if num_found == 0:
            print("\n연결된 슬레이브가 없습니다. 종료합니다.")
            return

        if args.slaves > 0 and num_found != args.slaves:
            print(f"  [경고] 예상 {args.slaves}개, 발견 {num_found}개 — 계속 진행합니다.")

        # EtherCAT 상태 출력
        master.read_state()
        print("\n  슬레이브 EtherCAT 상태:")
        for i, slave in enumerate(master.slaves):
            ec_state = STATE_NAMES.get(slave.state, f"0x{slave.state:02X}")
            print(f"    Slave {i}: {slave.name:30s} | EtherCAT={ec_state}")

        # ── 3단계: 각 슬레이브 SDO 복구 ──────────────────────────
        print("\n[3단계] 드라이브 상태 확인 및 Disable Voltage 전송...")
        results = []
        for i, slave in enumerate(master.slaves):
            ok = reset_slave(slave, i, args.check_only)
            results.append(ok)

        # ── 4단계: EtherCAT INIT 상태로 복귀 ─────────────────────
        if not args.check_only:
            print("\n[4단계] EtherCAT INIT 상태로 전환...")
            master.state = pysoem.INIT_STATE
            master.write_state()
            time.sleep(0.3)

            master.read_state()
            all_init = all(s.state == pysoem.INIT_STATE for s in master.slaves)
            if all_init:
                print("  → 모든 슬레이브 INIT 상태 도달")
            else:
                for i, slave in enumerate(master.slaves):
                    ec_state = STATE_NAMES.get(slave.state, f"0x{slave.state:02X}")
                    print(f"  Slave {i}: {ec_state}")

        # ── 결과 요약 ─────────────────────────────────────────────
        print("\n" + "=" * 55)
        if args.check_only:
            print("  상태 확인 완료 (드라이버 변경 없음)")
        else:
            success = sum(results)
            print(f"  복구 완료: {success}/{num_found} 슬레이브 성공")
            if success < num_found:
                print("  일부 슬레이브 복구 실패 — 드라이버 전원을 껐다 켜세요.")
        print("=" * 55)

    except KeyboardInterrupt:
        print("\n\n사용자에 의해 중단.")
    except Exception as e:
        print(f"\n[에러] {e}")
        sys.exit(1)
    finally:
        master.close()
        print("\nEtherCAT 마스터 닫힘.")


if __name__ == "__main__":
    main()
