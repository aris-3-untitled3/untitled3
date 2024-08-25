# ar_marker_subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np
from sensor_msgs_py import point_cloud2

class ArucoMarkerSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_marker_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',  # 압축된 이미지 토픽을 구독합니다.
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        # Load calibration data
        self.cam_matrix = np.load('/home/messi/camera_ar_marker/camera_ar_marker/calib_data/camMatrix.npy')
        self.dist_coeffs = np.load('/home/messi/camera_ar_marker/camera_ar_marker/calib_data/distCoeffs.npy')

        # Define ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.marker_size = 0.088  # 88mm

    def listener_callback(self, msg):
        # 압축된 이미지를 OpenCV 이미지로 변환합니다.
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        marker_corners, marker_ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if marker_corners:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(marker_corners, self.marker_size, self.cam_matrix, self.dist_coeffs)

            for i in range(len(marker_ids)):
                cv2.drawFrameAxes(cv_image, self.cam_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
                distance = np.sqrt(tvecs[i][0][2]**2 + tvecs[i][0][0]**2 + tvecs[i][0][1]**2)
                self.get_logger().info(f"Marker ID: {marker_ids[i][0]} Distance: {distance:.2f} meters")

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
