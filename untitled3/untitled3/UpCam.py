import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class UpCam(Node):
    def __init__(self):
        super().__init__('UpCam')

        self.cap = self.find_video_capture_device()

        if self.cap is None:
            self.get_logger().error('No video capture device found.')
            return

        # Publisher to /UpCam
        self.publisher = self.create_publisher(Image, '/UpCam', 10)

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.send_webcam_image)

    def find_video_capture_device(self):
        for i in range(10):  # Check the first 10 video devices
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                self.get_logger().info(f'Video capture device found at index {i}')
                return cap
            cap.release()
        return None

    def send_webcam_image(self):
        ret, frame = self.cap.read()
        
        if ret:
            frame = cv2.resize(frame, (640, 480))

            # Flip image horizontally
            frame = cv2.flip(frame, 1)

            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

            # Publish Image message
            self.publisher.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)

    Up_Cam_node = UpCam()

    try:
        rclpy.spin(Up_Cam_node)
    finally:
        Up_Cam_node.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
