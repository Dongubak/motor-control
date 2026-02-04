from __future__ import annotations

import struct
import math
import time
import multiprocessing as mp
from dataclasses import dataclass
from typing import List, Tuple, Generator
from queue import Empty

import serial

SYNC = 0xA5
CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_FORCE_SCAN = 0x21
CMD_INFO = 0x50
CMD_HEALTH = 0x52
CMD_MOTOR_PWM = 0xF0  # payload: uint16 little-endian (0-1023)


class LidarProtocolError(Exception):
    pass


@dataclass
class DeviceInfo:
    model: int
    firmware: Tuple[int, int]
    hardware: int
    serial: str


@dataclass
class DeviceHealth:
    status: str  # Good | Warning | Error
    error_code: int

@dataclass
class Point:
    """라이다 측정 포인트 (3D 좌표 및 강도)"""
    x: float
    y: float
    z: float
    intensity: int


class S2M1Lidar:
    def __init__(self, port: str, baudrate: int = 1_000_000, timeout: float = 2.0):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(0.1)

    def _send(self, cmd: int, payload: bytes = b"") -> None:
        if payload:
            length = len(payload)
            checksum = length
            for b in payload:
                checksum ^= b
            packet = bytes([SYNC, cmd, length]) + payload + bytes([checksum])
        else:
            packet = bytes([SYNC, cmd])
        self.ser.write(packet)
        self.ser.flush()

    def _read_desc(self) -> Tuple[int, int, int]:
        desc = self.ser.read(7)
        if len(desc) != 7 or desc[0:2] != b"\xA5\x5A":
            raise LidarProtocolError("Descriptor missing or malformed.")
        raw_len = int.from_bytes(desc[2:6], "little")
        payload_size = raw_len & 0x3FFFFFFF
        mode_bits = (raw_len >> 30) & 0x3
        mode_byte = desc[6]
        return payload_size, mode_byte, mode_bits

    def reset(self) -> None:
        self._send(CMD_RESET)
        time.sleep(2.0)
        self.ser.reset_input_buffer()

    def stop(self) -> None:
        self._send(CMD_STOP)
        time.sleep(0.05)

    def set_motor_pwm(self, pwm: int) -> None:
        pwm = max(0, min(1023, int(pwm)))
        self._send(CMD_MOTOR_PWM, struct.pack("<H", pwm))
        time.sleep(0.02)

    def get_info(self) -> DeviceInfo:
        self._send(CMD_INFO)
        payload_size, _, _ = self._read_desc()
        if payload_size != 20:
            raise LidarProtocolError(f"Info payload length {payload_size}")
        raw = self.ser.read(20)
        if len(raw) != 20:
            raise LidarProtocolError("Info payload underflow.")
        return DeviceInfo(
            model=raw[0],
            firmware=(raw[2], raw[1]),
            hardware=raw[3],
            serial=raw[4:20].hex().upper(),
        )

    def get_health(self) -> DeviceHealth:
        self._send(CMD_HEALTH)
        payload_size, _, _ = self._read_desc()
        if payload_size != 3:
            raise LidarProtocolError(f"Health payload length {payload_size}")
        raw = self.ser.read(3)
        if len(raw) != 3:
            raise LidarProtocolError("Health payload underflow.")
        status_code = raw[0]
        status = {0: "Good", 1: "Warning", 2: "Error"}.get(status_code, "Unknown")
        error_code = raw[1] | (raw[2] << 8)
        return DeviceHealth(status=status, error_code=error_code)

    def start_scan(self, force: bool = False) -> None:
        self.ser.reset_input_buffer()
        self._send(CMD_STOP)
        time.sleep(0.05)
        self._send(CMD_FORCE_SCAN if force else CMD_SCAN)
        size, mode_byte, mode_bits = self._read_desc()
        if size != 5:
            raise LidarProtocolError(
                f"Scan payload length {size} (mode_bits={mode_bits}, mode_byte=0x{mode_byte:02X})"
            )
        if mode_byte not in (0x40, 0x00, 0x81):
            raise LidarProtocolError(f"Scan response mode 0x{mode_byte:02X}")
        time.sleep(0.1)

    def close(self) -> None:
        try:
            self.stop()
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def __enter__(self) -> "S2M1Lidar":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


# 간단 헬퍼 API -------------------------------------------------------------
def start_lidar(port: str, pwm: int = 660, force: bool = False) -> S2M1Lidar:
    lidar = S2M1Lidar(port)
    health = lidar.get_health()
    if health.status == "Error":
        raise LidarProtocolError(
            f"Health Error (code={health.error_code}), check power/wiring."
        )
    lidar.set_motor_pwm(pwm)
    time.sleep(0.2)
    lidar.start_scan(force=force)
    return lidar


def _stream_raw_scans(
    lidar: S2M1Lidar,
    max_underflows: int = 50,
) -> Generator[Tuple[bool, float, float, int], None, None]:
    """
    내부 헬퍼 함수: 라이다의 원시 측정 데이터를 파싱하여 start_bit와 함께 반환합니다.
    Yields: (start_bit, angle, distance, quality)
    """
    underflows = 0
    while True:
        try:
            chunk = lidar.ser.read(5)
            if len(chunk) != 5:
                underflows += 1
                if underflows >= max_underflows:
                    raise LidarProtocolError("Measurement underflow.")
                continue
            underflows = 0

            b0, b1, b2, b3, b4 = chunk
            start_bit = bool(b0 & 0x1)
            inv_start_bit = bool(b0 & 0x2)
            if start_bit == inv_start_bit:
                continue

            if (b1 & 0x1) != 1:
                continue

            angle_q6 = (b1 >> 1) | (b2 << 7)
            angle = angle_q6 / 64.0
            dist_q2 = b3 | (b4 << 8)
            distance = dist_q2 / 4.0
            quality = b0 >> 2

            yield start_bit, angle, distance, quality
        except serial.SerialException:
            print("Lidar 시리얼 포트 에러 발생. 재연결을 시도합니다...")
            time.sleep(2)
            # 재연결 로직을 추가할 수 있음
            break


def stream_frames(
    lidar: S2M1Lidar,
    drop_invalid: bool = True,
) -> Generator[List[Point], None, None]:
    """
    라이다 데이터를 1회전 단위의 프레임으로 묶어서 반환합니다.
    각 포인트는 (x, y, z, intensity) 형태의 Point 객체입니다.
    """
    current_frame: List[Point] = []
    for start_bit, angle, distance, quality in _stream_raw_scans(lidar):
        if start_bit:
            if current_frame:
                yield current_frame
            current_frame = []

        if drop_invalid and distance <= 0:
            continue

        # 각도(도)와 거리(mm)를 x, y 좌표로 변환
        angle_rad = math.radians(angle)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        
        current_frame.append(Point(x=x, y=y, z=0.0, intensity=quality))


def stop_lidar(lidar: S2M1Lidar) -> None:
    try:
        lidar.set_motor_pwm(0)
        lidar.stop()
    finally:
        if lidar and lidar.ser and lidar.ser.is_open:
            lidar.close()

# --- Multiprocessing을 위한 새로운 함수 및 클래스 ---

def _lidar_process_loop(port: str, pwm: int, command_queue: mp.Queue, data_queue: mp.Queue):
    """
    별도의 프로세스에서 실행될 라이다 데이터 수집 루프.
    """
    lidar = None
    is_running = True
    try:
        lidar = start_lidar(port, pwm)
        print("✅ Lidar 데이터 수집 프로세스 시작.")
        frame_generator = stream_frames(lidar, drop_invalid=True)
        
        while is_running:
            # 메인 프로세스로부터 명령 확인 (Non-blocking)
            try:
                if command_queue.get_nowait() == 'STOP':
                    is_running = False
                    break
            except Empty:
                pass # 명령이 없으면 계속 진행

            # 다음 프레임을 생성하고 데이터 큐에 넣기
            try:
                frame = next(frame_generator)
                # 큐가 너무 많이 쌓이는 것을 방지 (최신 데이터 유지)
                if data_queue.qsize() > 2:
                    data_queue.get_nowait() # 오래된 프레임 버리기
                data_queue.put(frame)
            except StopIteration:
                break
            except Exception as e:
                print(f"Lidar 프레임 생성 중 에러: {e}")
                time.sleep(1)

    except Exception as e:
        print(f"Lidar 프로세스 에러: {e}")
    finally:
        if lidar:
            stop_lidar(lidar)
        print("Lidar 프로세스 종료.")


class LidarController:
    """
    RPLIDAR를 제어하고 프레임 단위로 데이터를 제공하는 클래스.
    별도의 프로세스를 사용하여 데이터를 수집합니다.
    """
    def __init__(self, port: str, pwm: int = 750):
        self._command_queue = mp.Queue()
        self._data_queue = mp.Queue()
        self._process = mp.Process(
            target=_lidar_process_loop,
            args=(port, pwm, self._command_queue, self._data_queue)
        )
        self._latest_frame: List[Point] = []

    def start(self):
        """라이다를 시작하고 데이터 수집 프로세스를 시작합니다."""
        self._process.start()
        print("Lidar 제어 프로세스를 시작합니다.")

    def stop(self):
        """데이터 수집을 중지하고 라이다 프로세스를 종료합니다."""
        print("Lidar 종료 중...")
        self._command_queue.put('STOP')
        self._process.join(timeout=3)
        if self._process.is_alive():
            print("Lidar 프로세스가 정상적으로 종료되지 않아 강제 종료합니다.")
            self._process.terminate()
        print("✅ Lidar 종료 완료.")

    def get_latest_frame(self) -> List[Point] | None:
        """
        가장 최근에 수신된 완전한 프레임을 반환합니다.
        큐에 여러 프레임이 쌓여있으면 가장 최신 것만 남기고 나머지는 버립니다.
        """
        # 큐를 비우고 마지막 프레임만 가져오기
        while not self._data_queue.empty():
            try:
                self._latest_frame = self._data_queue.get_nowait()
            except Empty:
                break
        
        return self._latest_frame if self._latest_frame else None
