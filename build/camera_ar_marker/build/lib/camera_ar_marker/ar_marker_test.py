import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np
import time

class ArucoMarkerSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_marker_subscriber')
        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed', 
            self.image_callback,
            10)
        self.marker_publisher = self.create_publisher(Float32MultiArray, 'aruco_marker_info', 10)
        self.bridge = CvBridge()

        # 캘리브레이션 데이터 로드
        self.cam_matrix = np.load('/home/messi/camera_ar_marker/src/new_calib_data/camMatrix.npy')
        self.dist_coeffs = np.load('/home/messi/camera_ar_marker/src/new_calib_data/distCoeffs.npy')

        # ArUco 딕셔너리 및 파라미터 설정
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.marker_size = 0.119  # 88mm

        # 마커별 필터를 위한 변수 초기화 (ID별로 별도 리스트 관리)
        self.marker_data = {
            1: {'distance_list': [], 'angle_list': [], 'start_time': time.time()},
            2: {'distance_list': [], 'angle_list': [], 'start_time': time.time()},
            3: {'distance_list': [], 'angle_list': [], 'start_time': time.time()},
            4: {'distance_list': [], 'angle_list': [], 'start_time': time.time()}
        }

    def image_callback(self, msg):
        # 압축된 이미지를 OpenCV 이미지로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        marker_corners, marker_ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if marker_corners:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(marker_corners, self.marker_size, self.cam_matrix, self.dist_coeffs)

            for i in range(len(marker_ids)):
                marker_id = marker_ids[i][0]

                if marker_id in self.marker_data:  # 마커 ID가 1, 2, 3, 4 중 하나인지 확인
                    distance = np.sqrt(tvecs[i][0][2]**2 + tvecs[i][0][0]**2 + tvecs[i][0][1]**2)
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                    y_rotation_angle = np.degrees(np.arctan2(rotation_matrix[0, 2], rotation_matrix[2, 2]))

                    self.marker_data[marker_id]['distance_list'].append(distance)
                    self.marker_data[marker_id]['angle_list'].append(y_rotation_angle)

                    # 1.5초가 지났을 때 평균 필터 적용
                    if time.time() - self.marker_data[marker_id]['start_time'] >= 1.5:
                        avg_distance = np.mean(self.marker_data[marker_id]['distance_list'])
                        avg_angle = np.mean(self.marker_data[marker_id]['angle_list'])

                        # 퍼블리시할 메시지 생성 및 발행
                        msg = Float32MultiArray()
                        msg.data = [float(marker_id), avg_distance, avg_angle]  # 마커 ID를 float으로 변환
                        self.marker_publisher.publish(msg)

                        # 로깅
                        self.get_logger().info(f'ID {marker_id}: 평균 거리: {avg_distance:.2f}m, 평균 각도: {avg_angle:.2f} degrees')

                        # 필터 데이터 초기화
                        self.marker_data[marker_id]['distance_list'] = []
                        self.marker_data[marker_id]['angle_list'] = []
                        self.marker_data[marker_id]['start_time'] = time.time()

        cv2.imshow("Aruco Marker Detection", cv_image)
        cv2.waitKey(1)


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
