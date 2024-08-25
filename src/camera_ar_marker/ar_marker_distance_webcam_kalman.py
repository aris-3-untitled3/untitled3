import cv2
import numpy as np
from cv2 import aruco
from filterpy.kalman import KalmanFilter
from collections import deque

class KalmanFilter1D:
    def __init__(self, process_variance, measurement_variance):
        self.kf = KalmanFilter(dim_x=1, dim_z=1)
        self.kf.x = np.array([[0.]])  # 초기 상태
        self.kf.F = np.array([[1.]])  # 상태 전이 행렬
        self.kf.H = np.array([[1.]])  # 측정 행렬
        self.kf.P *= 1000.            # 초기 상태 불확실성
        self.kf.R = measurement_variance  # 측정 노이즈 공분산 (측정값 신뢰도)
        self.kf.Q = process_variance      # 프로세스 노이즈 공분산 (모델의 신뢰도)

    def predict(self):
        self.kf.predict()

    def update(self, measurement):
        self.kf.update(measurement)
        return self.kf.x[0, 0]

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

# 칼만 필터 인스턴스 생성
process_variance = 1e-1  # 예측에 대한 신뢰도 (작을수록 모델의 예측에 의존)
measurement_variance = 1e-9  # 측정값에 대한 신뢰도 (작을수록 측정값에 의존)
distance_kalman_filter = KalmanFilter1D(process_variance, measurement_variance)
angle_kalman_filter = KalmanFilter1D(process_variance, measurement_variance)

# 이동 평균 필터 인스턴스 생성
distance_moving_average_filter = MovingAverageFilter(window_size=5)
angle_moving_average_filter = MovingAverageFilter(window_size=5)

# 웹캠 초기화
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

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
            filtered_distance_kalman = distance_kalman_filter.update(distance)
            filtered_distance_moving_avg = distance_moving_average_filter.filter(distance)

            # 회전 각도 계산
            rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
            y_rotation_angle = np.degrees(np.arctan2(rotation_matrix[0, 2], rotation_matrix[2, 2]))
            filtered_angle_kalman = angle_kalman_filter.update(y_rotation_angle)
            filtered_angle_moving_avg = angle_moving_average_filter.filter(y_rotation_angle)

            # 실제값, 칼만 필터, 이동 평균 필터의 결과를 모두 출력
            print(f"Marker ID: {marker_ids[i][0]}")
            print(f"Actual Angle: {y_rotation_angle:.2f} degrees | Kalman Filtered Angle: {filtered_angle_kalman:.2f} degrees | Moving Avg Filtered Angle: {filtered_angle_moving_avg:.2f} degrees")
            print(f"Actual Distance: {distance:.2f} meters | Kalman Filtered Distance: {filtered_distance_kalman:.2f} meters | Moving Avg Filtered Distance: {filtered_distance_moving_avg:.2f} meters")

            # 마커 ID와 거리 표시
            cv2.putText(frame, f"ID: {marker_ids[i][0]} Dist: {filtered_distance_kalman:.2f}m",
                        (int(marker_corners[i][0][0][0]), int(marker_corners[i][0][0][1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 이미지 출력
    cv2.imshow('Aruco Marker Detection', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()
