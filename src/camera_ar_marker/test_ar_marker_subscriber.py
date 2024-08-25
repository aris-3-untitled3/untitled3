import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np

class TestArucoMarkerSubscriber(Node):
    def __init__(self):
        super().__init__('test_aruco_marker_subscriber')
        
        # Timer를 통해 웹캠에서 이미지를 캡처하고 처리합니다.
        self.timer = self.create_timer(0.1, self.process_image)
        
        self.bridge = CvBridge()

        # 캘리브레이션 데이터 로드
        self.cam_matrix = np.load('/home/messi/camera_ar_marker/camera_ar_marker/calib_data/camMatrix.npy')
        self.dist_coeffs = np.load('/home/messi/camera_ar_marker/camera_ar_marker/calib_data/distCoeffs.npy')

        # ArUco 사전 및 파라미터 정의
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.marker_size = 0.088  # 88mm

        # 웹캠 초기화
        self.cap = cv2.VideoCapture(0)  # 노트북의 기본 카메라 사용

    def process_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image from webcam")
            return
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if marker_corners:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(marker_corners, self.marker_size, self.cam_matrix, self.dist_coeffs)
            for i in range(len(marker_ids)):
                cv2.drawFrameAxes(frame, self.cam_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
                
                distance = np.linalg.norm(tvecs[i][0])
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                y_rotation_angle = np.degrees(np.arctan2(rotation_matrix[0, 2], rotation_matrix[2, 2]))
                
                self.get_logger().info(f"Marker ID: {marker_ids[i][0]} Rotation Angle: {y_rotation_angle:.2f} degrees")
                self.get_logger().info(f"ArUco Distance: {distance:.2f} meters")

        cv2.imshow("Aruco Marker Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_marker_subscriber = TestArucoMarkerSubscriber()
    rclpy.spin(aruco_marker_subscriber)
    aruco_marker_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
