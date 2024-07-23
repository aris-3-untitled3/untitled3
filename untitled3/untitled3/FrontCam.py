import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FrontCam(Node):
    def __init__(self):
        super().__init__('FrontCam')

        self.cap = cv2.VideoCapture(0)

        # Publisher to /FrontCam
        self.publisher = self.create_publisher(Image, '/FrontCam', 10)

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.send_webcam_image)

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

    front_cam_node = FrontCam()

    try:
        rclpy.spin(front_cam_node)
    finally:
        front_cam_node.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
