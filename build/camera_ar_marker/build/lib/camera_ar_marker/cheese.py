import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CalibrationImageCapture(Node):

    def __init__(self):
        super().__init__('calibration_image_capture')
        
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/camera/color/image_raw/compressed',  # 압축된 이미지 토픽을 구독
            self.listener_callback,
            10)
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.img_counter = 0

    def listener_callback(self, msg):
        # 압축된 이미지 데이터를 OpenCV 이미지로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if self.latest_image is not None:
            cv2.imshow("Press Space to Save Image, Press Esc to Exit", self.latest_image)
            key = cv2.waitKey(1)

            if key % 256 == 27:  # Esc 키를 눌렀을 때
                self.get_logger().info("Escape hit, closing...")
                rclpy.shutdown()
            elif key % 256 == 32:  # Spacebar를 눌렀을 때
                # 이미지 저장
                img_name = f"chessboard_{self.img_counter}.jpg"
                cv2.imwrite(img_name, self.latest_image)
                self.get_logger().info(f"{img_name} saved!")
                self.img_counter += 1

def main(args=None):
    rclpy.init(args=args)
    image_capture_node = CalibrationImageCapture()
    rclpy.spin(image_capture_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
