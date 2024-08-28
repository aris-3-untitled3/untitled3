# marker_id 추가로 발행한 코드
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np
from std_msgs.msg import String

class ParkingMarkerSubscriber(Node):
    def __init__(self):
        super().__init__('parking_marker_subscriber')
        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',  # 압축된 이미지 토픽 구독
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # 각도와 거리를 발행하는 퍼블리셔 생성
        self.angle_publisher = self.create_publisher(String, '/ar_marker_angle', 10)
        self.distance_publisher = self.create_publisher(String, '/ar_marker_distance', 10)  # 거리 발행
        self.aruco_id_publisher = self.create_publisher(String, '/ar_marker_id', 10)
        self.marker_center_publisher = self.create_publisher(String, '/ar_marker_center', 10)  # 마커의 중심 좌표 발행


        # Load calibration data
        self.cam_matrix = np.load('src/new_calib_data/camMatrix.npy')
        self.dist_coeffs = np.load('src/new_calib_data/distCoeffs.npy')

        # Define ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.marker_size = 0.119  # 88mm 203

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

                # 회전 각도 계산
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                y_rotation_angle = np.degrees(np.arctan2(rotation_matrix[0, 2], rotation_matrix[2, 2]))

                # 각도 변환 (0 ~ 360도 범위)
                if y_rotation_angle < 0:
                    angle = 360 + y_rotation_angle
                else:
                    angle = y_rotation_angle

                # 마커의 중심 좌표 계산
                marker_center_x = np.mean(marker_corners[i][0][:, 0])  # x 좌표
                marker_center_y = np.mean(marker_corners[i][0][:, 1])  # y 좌표
                image_center_x = cv_image.shape[1] / 2  # 이미지의 중심 x 좌표
                
                # 마커 중심과 이미지 중심의 차이 계산
                center_offset = marker_center_x - image_center_x

                print(marker_ids, type(marker_ids))


                self.angle_publisher.publish(String(data=f"{angle:.2f}"))
                self.distance_publisher.publish(String(data=f"{distance:.2f}"))
                # print(f"-----------{marker_ids}-----------")
                # marker id publisher
                marker_id = str(marker_ids[i][0])
                self.aruco_id_publisher.publish(String(data=str(marker_id)))
                self.marker_center_publisher.publish(String(data=f"{center_offset:.2f}"))  # 중심 좌표 발행


        cv2.imshow("Aruco Marker Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    parking_marker_subscriber = ParkingMarkerSubscriber()
    rclpy.spin(parking_marker_subscriber)
    parking_marker_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

