import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ImageNode(Node):

    def __init__(self):
        super().__init__('image_node')
        
        # Subscriber to the image topic
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/color/image_raw',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.last_time = time.time()
        self.frame_count = 0

    def image_callback(self, msg):
        # Convert the ROS image message to an OpenCV image
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        if self.latest_image is not None:
            # Display the image
            cv2.imshow("Image", self.latest_image)
            cv2.waitKey(1)

            # Calculate FPS
            self.frame_count += 1
            current_time = time.time()
            elapsed_time = current_time - self.last_time

            if elapsed_time >= 1.0:  # Every second
                fps = self.frame_count / elapsed_time
                self.get_logger().info(f"FPS: {fps:.2f}")
                
                # Reset counters
                self.last_time = current_time
                self.frame_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = ImageNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()