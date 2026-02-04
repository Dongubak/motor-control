from motor import EtherCATBus
from rplidar import LidarController, Point
import open3d as o3d
import numpy as np
import time
import math

def homogeneous_transform(px, py, pz, tx, ty, tz, rx, ry, rz):
    """
    점(p)에 대해 회전(R) 및 이동(T) 변환을 적용합니다.
    Transformed_Point = T * R * P
    회전 순서: Y축 -> X축
    """
    # 각도를 라디안으로 변환
    rad_x = math.radians(rx)
    rad_y = math.radians(ry)
    rad_z = math.radians(rz)

    # Y축 회전 행렬
    Ry = np.array([
        [math.cos(rad_y),  0, math.sin(rad_y)],
        [0,                1, 0],
        [-math.sin(rad_y), 0, math.cos(rad_y)]
    ])

    # X축 회전 행렬
    Rx = np.array([
        [1, 0,                 0],
        [0, math.cos(rad_x), -math.sin(rad_x)],
        [0, math.sin(rad_x),  math.cos(rad_x)]
    ])

    # 전체 회전 행렬. Y축 회전 후 X축 회전을 적용합니다.
    # 행렬 곱셈 순서는 연산 순서의 역순이므로 Rx @ Ry 입니다.
    R = Rx @ Ry

    # 원본 포인트 벡터
    P = np.array([px, py, pz])

    # 회전 적용
    P_rotated = R @ P

    # 이동 적용
    P_transformed = P_rotated + np.array([tx, ty, tz])

    return P_transformed[0], P_transformed[1], P_transformed[2]


### ============================================================
### MAIN
### ============================================================
def main():
    # --- 장치 포트 및 슬레이브 수 설정 ---
    adapter = r'\Device\NPF_{F8EF3044-171D-407C-A166-9EEA524F051C}'
    lidar_port = "COM4"
    NUM_MOTORS = 4

    # --- 단일 버스 관리자 생성 ---
    # 사이클 타임을 10ms로 설정하여 부드러운 움직임 구현
    bus = EtherCATBus(adapter_name=adapter, num_slaves=NUM_MOTORS, cycle_time_ms=10)

    # --- 맵핑에 사용할 모터 선택 (motor1 사용) ---
    motor = bus.motors[1]  # motor1 사용

    # --- 라이다 컨트롤러 생성 ---
    lidar = LidarController(port=lidar_port)

    # --- Open3D 시각화 설정 ---
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="3D Point Cloud Mapping", width=1280, height=720)
    pcd = o3d.geometry.PointCloud()
    is_geometry_added = False

    try:
        # --- 축 설정 (버스 시작 전) ---
        motor.set_axis('x')  # X축 방향으로 이동

        # --- SDO 설정 (버스 시작 전) ---
        motor.set_profile_velocity(rpm=300)
        motor.set_profile_accel_decel(accel_rpm_per_sec=100)

        # --- 버스 및 라이다 시작 ---
        bus.start()
        lidar.start()

        # --- 모터 준비 대기 ---
        print(f"모터 {motor._index} 준비 대기 중...")
        start_time = time.monotonic()
        while (motor.status_word & 0x006F) != 0x0027:
            time.sleep(0.05)
            if time.monotonic() - start_time > 5:
                status = motor.status_word
                print(f"[에러] 모터 {motor._index} 타임아웃! 현재 상태: 0x{status:04X}")
                if status & 0x0008:
                    print(f"  → Fault 상태 감지됨!")
                raise RuntimeError(f"모터 {motor._index}가 Operation Enabled 상태에 도달하지 못했습니다.")
        print(f"[완료] 모터 {motor._index} 준비 완료! ({time.monotonic() - start_time:.2f}초 소요)")

        # --- 원점 설정 ---
        motor.set_origin()
        time.sleep(0.5)  # 원점 설정 명령이 처리될 충분한 시간 제공

        print(f"원점 설정 후 위치: {motor.current_position_mm:.2f}mm")

        # --- 1800mm 위치로 이동하며 맵핑 시작 ---
        print("\n--- 1800mm 위치로 이동하며 맵핑 시작 ---")
        motor.move_to_position_mm(100)

        # 이동이 시작될 때까지 대기
        print("이동 시작 대기 중...")
        time.sleep(0.2)  # 명령 처리 대기

        # 이동이 완료될 때까지 라이다 데이터 수집
        print("맵핑 진행 중...")
        start_time = time.monotonic()

        while motor.is_moving():
            # 1. 최신 라이다 프레임과 모터 위치 가져오기
            lidar_frame = lidar.get_latest_frame()
            motor_pos_mm = motor.current_position_mm

            if lidar_frame:
                new_points = []
                for p in lidar_frame:
                    # 라이다 좌표 -> 월드 좌표 변환
                    wx, wy, wz = homogeneous_transform(
                        px=p.x, py=p.y, pz=p.z,
                        tx=motor_pos_mm, ty=0, tz=0,
                        rx=90, ry=-90, rz=0
                    )
                    new_points.append([wx, wy, wz])

                if new_points:
                    pcd.points.extend(o3d.utility.Vector3dVector(np.array(new_points)))
                    if not is_geometry_added:
                        vis.add_geometry(pcd)
                        is_geometry_added = True
                    else:
                        vis.update_geometry(pcd)

            # Open3D 뷰어 업데이트
            vis.poll_events()
            vis.update_renderer()

            print(f"--> 맵핑 중... 모터 위치: {motor_pos_mm:.2f} mm, 누적 포인트: {len(pcd.points)}", end='\r')

            # 타임아웃 체크 (60초)
            if time.monotonic() - start_time > 60:
                print("\n[경고] 타임아웃: 60초 내에 이동이 완료되지 않았습니다.")
                break

        print(f"\n[완료] 맵핑 완료! 최종 위치: {motor.current_position_mm:.2f} mm, 총 포인트: {len(pcd.points)}")

        # --- 포인트 클라우드 데이터 저장 ---
        if len(pcd.points) > 0:
            print("\n[저장] 포인트 클라우드 데이터를 pcd.txt에 저장 중...")
            output_file = "pcd.txt"

            # numpy 배열로 변환
            points_array = np.asarray(pcd.points)

            # intensity 값 생성 (모든 포인트에 대해 1.0으로 설정)
            intensity = np.ones((len(points_array), 1))

            # x, y, z, i 순서로 결합
            data_with_intensity = np.hstack([points_array, intensity])

            # 파일로 저장
            np.savetxt(output_file, data_with_intensity, fmt='%.6f %.6f %.6f %.6f',
                      header='x y z intensity', comments='')

            print(f"[완료] {len(points_array)}개의 포인트를 {output_file}에 저장했습니다.")
        else:
            print("\n[경고] 저장할 포인트가 없습니다.")

        # --- 모터 원점 복귀 ---
        print("\n--- 모터 원점 복귀 시작 ---")
        motor.move_to_position_mm(0)

        print("원점 복귀 중...")
        time.sleep(0.2)  # 명령 처리 대기

        start_time = time.monotonic()
        while motor.is_moving():
            print(f"--> 복귀 중... 현재 위치: {motor.current_position_mm:.2f} mm", end='\r')

            # Open3D 뷰어 업데이트
            vis.poll_events()
            vis.update_renderer()

            time.sleep(0.05)

            if time.monotonic() - start_time > 60:
                print("\n[경고] 타임아웃: 60초 내에 복귀가 완료되지 않았습니다.")
                break

        print(f"\n[완료] 원점 복귀 완료! 최종 위치: {motor.current_position_mm:.2f} mm")
        print("\n--- 모든 작업 완료. 뷰어 창을 닫거나 Ctrl+C로 종료하세요. ---")

        # 사용자가 뷰어 창을 닫을 때까지 대기
        while vis.poll_events():
            vis.update_renderer()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[긴급 정지] 사용자에 의해 프로그램이 중단되었습니다.")
        print("[안전] 모터를 현재 위치에 즉시 정지시킵니다...")
        # 현재 위치를 원점으로 설정하여 움직임 방지
        motor.set_origin()
        time.sleep(0.3)
    except Exception as e:
        print(f"\n메인 루프에서 에러 발생: {e}")
        print("[안전] 모터를 현재 위치에 정지시킵니다...")
        try:
            motor.set_origin()
            time.sleep(0.3)
        except:
            pass
    finally:
        # --- 장치 종료 ---
        print("[종료] 버스 및 라이다 종료 중...")
        bus.stop()
        lidar.stop()
        try:
            vis.destroy_window()
        except:
            pass
        print("[완료] 안전하게 종료되었습니다.")

if __name__ == "__main__":
    main()
