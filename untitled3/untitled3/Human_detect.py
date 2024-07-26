import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from untitled_msgs.msg import TopicString
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import threading

class HumanDetect(Node):
    def __init__(self):
        super().__init__('HumanDetect')

        # Define callback groups
        self.image_callback_group = ReentrantCallbackGroup()
        self.server_callback_group = ReentrantCallbackGroup()

        # WebCam Topic subscribe
        self.FrontCam_subscription = self.create_subscription(
            Image,
            '/FrontCam',
            self.Webcam_callback,
            10,
            callback_group=self.image_callback_group
        )

        # ROS2 image chage
        self.bridge = CvBridge()

        # Topic publish to Robot_Server
        self.human_detect_publisher = self.create_publisher(TopicString, '/Human_to_Server', 10)

        # Topic subscribe from Robot_Server
        self.robot_server_subscriber = self.create_subscription(
            TopicString,
            '/Server_to_Human',
            self.robot_server_callback,
            10,
            callback_group=self.server_callback_group
        )

        self.model_path = '/home/jchj/Untitled3/src/models/Best.pt'
        self.model = self.load_model()

        self.guest_detected = False
        self.human_detected = False
        self.far_human_detected = False
        self.no_human_detected = False
        self.loop_running = False  
        self.frame = None
        self.annotated_frame = None

        threading.Thread(target=self.display_frames).start()

    # Load YOLOv5 model
    def load_model(self):
        return torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_path, force_reload=True)

    def estimate_distance(self, bbox_height):
        KNOWN_HEIGHT = 1.7  # Human's average height (m)
        FOCAL_LENGTH = 50   # Estimated focal length of the camera (pixels)
        distance = (KNOWN_HEIGHT * FOCAL_LENGTH) / bbox_height
        return distance

    def process_detections(self, detections, annotated_frame):
        closest_distance = float('inf')
        closest_bbox = None

        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            if int(cls) == 0:  # cls = human
                bbox_height = y2 - y1
                distance = self.estimate_distance(bbox_height)

                # Find the closest human
                if distance < closest_distance:
                    closest_distance = distance
                    closest_bbox = (x1, y1, x2, y2)

        # Draw the closest human with bounding box and distance
        if closest_distance < float('inf'):
            x1, y1, x2, y2 = closest_bbox
            cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(annotated_frame, f'Distance: {closest_distance:.2f}m', (int(x1), int(y1)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
            self.detect_human(closest_distance)
        else:
            # No human detected in the frame
            if not self.no_human_detected:
                self.no_human_detect()
                self.no_human_detected = True
                self.guest_detected = False
                self.human_detected = False
                self.far_human_detected = False

    def detect_human(self, distance):
        if distance <= 0.5 and not self.guest_detected:
            self.guest_detect()
            self.guest_detected = True
            self.human_detected = False
            self.far_human_detected = False
            self.no_human_detected = False
            self.loop_running = False  # Stop loop
        elif 0.5 < distance <= 1.5 and not self.human_detected:
            self.human_detect()
            self.human_detected = True
            self.guest_detected = False
            self.far_human_detected = False
            self.no_human_detected = False
        elif distance > 1.5 and not self.far_human_detected:
            self.far_human_detect()
            self.far_human_detected = True
            self.guest_detected = False
            self.human_detected = False
            self.no_human_detected = False

    def guest_detect(self):
        self.get_logger().info('Guest detected!')
        msg = TopicString()
        msg.command = f'guest_detect'
        self.human_detect_publisher.publish(msg)

    def human_detect(self):
        self.get_logger().info('Human detected!')
        msg = TopicString()
        msg.command = f'human_detect'
        self.human_detect_publisher.publish(msg)

    def far_human_detect(self):
        self.get_logger().info('far_human detected!')
        msg = TopicString()
        msg.command = f'far_human'
        self.human_detect_publisher.publish(msg)

    def no_human_detect(self):
        self.get_logger().info('No human detected!')
        msg = TopicString()
        msg.command = 'no_human'
        self.human_detect_publisher.publish(msg)

    def run(self):
        while self.loop_running:
            if self.frame is not None:
                results = self.model(self.frame)
                self.annotated_frame = results.render()[0]
                self.annotated_frame = np.array(self.annotated_frame, copy=True)

                self.process_detections(results.xyxy[0], self.annotated_frame)
            else:
                self.get_logger().warning("No frame!")

    def robot_server_callback(self, msg):
        if msg.command == "start":
            self.get_logger().info('Starting detection!')
            self.loop_running = True  # start loop
            self.run()
            
        elif msg.command == "stop":
            self.get_logger().info('Stopping detection!')
            self.loop_running = False  # Stop loop
        else:
            self.get_logger().error("ERROR")

    def display_frames(self):
        while True:
            if self.annotated_frame is not None:  
                cv2.imshow("Frame", self.annotated_frame)

            else:
                if self.frame is not None:
                    cv2.imshow("Frame", self.frame)
                else:
                    self.get_logger().info("No frame available") 

            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                self.loop_running = False # Stop loop
                break

    def Webcam_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error in Webcam_callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    Human_detect = HumanDetect()

    # MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(Human_detect)

    try:
        executor.spin()
    finally:
        Human_detect.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
