import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np
from collections import deque
import time

class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)
    
    def filter(self, new_value):
        self.values.append(new_value)
        return sum(self.values) / len(self.values)

class ArucoMarkerSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_marker_subscriber')
        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',  # 압축된 이미지 토픽 구독
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Load calibration data
        self.cam_matrix = np.load('/home/messi/camera_ar_marker/camera_ar_marker/new_calib_data/camMatrix.npy')
        self.dist_coeffs = np.load('/home/messi/camera_ar_marker/camera_ar_marker/new_calib_data/distCoeffs.npy')

        # Define ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.marker_size = 0.119  # 119mm

        # 이동 평균 필터 인스턴스 생성
        self.distance_filter = MovingAverageFilter(window_size=5)
        self.angle_filter = MovingAverageFilter(window_size=5)

        # 출력 주기를 관리하기 위한 타이머
        self.last_output_time = time.time()

    def image_callback(self, msg):
        # 압축된 이미지를 OpenCV 이미지로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        marker_corners, marker_ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if marker_corners:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(marker_corners, self.marker_size, self.cam_matrix, self.dist_coeffs)

            for i in range(len(marker_ids)):
                cv2.drawFrameAxes(cv_image, self.cam_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)

                # 마커까지의 거리 계산
                distance = np.sqrt(tvecs[i][0][2]**2 + tvecs[i][0][0]**2 + tvecs[i][0][1]**2)
                filtered_distance = self.distance_filter.filter(distance)

                # 회전 각도 계산 및 0~360도 범위로 변환
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                y_rotation_angle = np.degrees(np.arctan2(rotation_matrix[0, 2], rotation_matrix[2, 2]))
                angle_360 = (y_rotation_angle + 360) % 360  # 0~360도 범위로 변환
                filtered_angle = self.angle_filter.filter(angle_360)

                # 0.5초마다 출력
                current_time = time.time()
                if current_time - self.last_output_time >= 0.5:
                    # 회전 방향 계산 및 출력
                    if 0 <= filtered_angle <= 180:
                        rotation_direction = "시계 방향"
                        rotation_degrees = filtered_angle
                    else:
                        rotation_direction = "반시계 방향"
                        rotation_degrees = 360 - filtered_angle

                    # 최종 출력
                    self.get_logger().info(f"Marker ID: {marker_ids[i][0]} Actual Angle: {y_rotation_angle:.2f} degrees | Filtered Angle: {rotation_direction} {rotation_degrees:.2f} degrees")
                    self.get_logger().info(f"Actual Distance: {distance:.2f} meters | Filtered Distance: {filtered_distance:.2f} meters")
                    
                    # 마지막 출력 시간 업데이트
                    self.last_output_time = current_time

        cv2.imshow("Aruco Marker Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_marker_subscriber = ArucoMarkerSubscriber()
    rclpy.spin(aruco_marker_subscriber)
    aruco_marker_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
