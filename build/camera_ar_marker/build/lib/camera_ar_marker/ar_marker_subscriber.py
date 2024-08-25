import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np

class ArucoMarkerSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_marker_subscriber')
        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',  # 압축된 이미지 토픽 구독
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # 각도와 거리를 발행하는 퍼블리셔 생성
        self.angle_publisher = self.create_publisher(String, '/ar_marker_angle', 10)
        self.distance_publisher = self.create_publisher(String, '/ar_marker_distance', 10)
        self.id_publisher = self.create_publisher(String, '/ar_marker_id', 10)  # 마커 ID 발행

        # Load calibration data
        self.cam_matrix = np.load('/home/messi/camera_ar_marker/src/new_calib_data/camMatrix.npy')
        self.dist_coeffs = np.load('/home/messi/camera_ar_marker/src/new_calib_data/distCoeffs.npy')

        # Define ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.marker_size = 0.203  # 88mm

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

                # 각도, 거리, ID 발행
                marker_id = marker_ids[i][0]
                self.id_publisher.publish(String(data=f"{marker_id}"))
                self.angle_publisher.publish(String(data=f"{angle:.2f}"))
                self.distance_publisher.publish(String(data=f"{distance:.2f}"))

                self.get_logger().info(f"Marker ID {marker_id}: Rotation Angle: {angle:.2f} degrees, Distance: {distance:.2f} meters")

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
