import cv2
import numpy as np
from cv2 import aruco
from collections import deque
import time

class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)
    
    def filter(self, new_value):
        self.values.append(new_value)
        return sum(self.values) / len(self.values)

# 카메라 보정 데이터 로드
cam_matrix = np.load('/home/messi/camera_ar_marker/camera_ar_marker/new_calib_data/camMatrix.npy')
dist_coeffs = np.load('/home/messi/camera_ar_marker/camera_ar_marker/new_calib_data/distCoeffs.npy')

# ArUco 사전 및 파라미터 정의
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
marker_size = 0.119  # 119mm

# 이동 평균 필터 인스턴스 생성
distance_filter = MovingAverageFilter(window_size=5)
angle_filter = MovingAverageFilter(window_size=5)

# 웹캠 초기화
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

# 출력 주기를 관리하기 위한 타이머
last_output_time = time.time()

while True:
    # 웹캠으로부터 이미지 캡처
    ret, frame = cap.read()
    if not ret:
        print("웹캠으로부터 이미지를 읽을 수 없습니다.")
        break

    # Grayscale로 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ArUco 마커 검출
    marker_corners, marker_ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if marker_corners:
        # 마커의 위치와 회전 추정
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(marker_corners, marker_size, cam_matrix, dist_coeffs)

        for i in range(len(marker_ids)):
            # 마커의 좌표 축 그리기
            cv2.drawFrameAxes(frame, cam_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)

            # 마커까지의 거리 계산
            distance = np.sqrt(tvecs[i][0][2]**2 + tvecs[i][0][0]**2 + tvecs[i][0][1]**2)
            filtered_distance = distance_filter.filter(distance)

            # 회전 각도 계산 및 0~360도 범위로 변환
            rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
            y_rotation_angle = np.degrees(np.arctan2(rotation_matrix[0, 2], rotation_matrix[2, 2]))
            angle_360 = (y_rotation_angle + 360) % 360  # 0~360도 범위로 변환
            filtered_angle = angle_filter.filter(angle_360)

            # 0.5초마다 출력
            current_time = time.time()
            if current_time - last_output_time >= 0.5:
                # 회전 방향 계산 및 출력
                if 0 <= filtered_angle <= 180:
                    rotation_direction = "시계 방향"
                    rotation_degrees = filtered_angle
                else:
                    rotation_direction = "반시계 방향"
                    rotation_degrees = 360 - filtered_angle

                # 최종 출력
                print(f"Marker ID: {marker_ids[i][0]}")
                print(f"Actual Angle: {y_rotation_angle:.2f} degrees | Filtered Angle: {rotation_direction} {rotation_degrees:.2f} degrees")
                print(f"Actual Distance: {distance:.2f} meters | Filtered Distance: {filtered_distance:.2f} meters")
                
                # 마지막 출력 시간 업데이트
                last_output_time = current_time

    # 이미지 출력
    cv2.imshow('Aruco Marker Detection', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()
