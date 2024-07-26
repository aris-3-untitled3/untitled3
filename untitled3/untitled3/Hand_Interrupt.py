import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from untitled_msgs.msg import TopicString
from cv_bridge import CvBridge
import cv2
import threading
import mediapipe as mp
import numpy as np
import time

class HandDetect(Node):
    def __init__(self):
        super().__init__('HandDetect')

        # Define callback groups
        self.image_callback_group = ReentrantCallbackGroup()
        self.server_callback_group = ReentrantCallbackGroup()

        # WebCam Topic subscribe
        self.UpCam_subscription = self.create_subscription(
            Image,
            '/UpCam',
            self.Webcam_callback,
            10,
            callback_group=self.image_callback_group
        )

        # ROS2 image change
        self.bridge = CvBridge()

        # Topic publish to Robot_Server
        self.hand_detect_publisher = self.create_publisher(TopicString, '/Hand_to_Server', 10)

        # Topic subscribe from Robot_Server
        self.robot_server_subscriber = self.create_subscription(
            TopicString,
            '/Server_to_Hand',
            self.robot_server_callback,
            10,
            callback_group=self.server_callback_group
        )

        # Initialize MediaPipe Hand Detection
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

        self.loop_running = False
        self.frame = None
        self.annotated_frame = None

        # Initialize state tracking
        self.hand_detected = False
        self.previous_hand_detected = False

        threading.Thread(target=self.display_frames, daemon=True).start()

    def robot_server_callback(self, msg):
        if msg.command == "start":
            self.get_logger().info('Starting detection!')
            self.loop_running = True
            self.detect_hands()
        elif msg.command == "stop":
            self.get_logger().info('Stopping detection!')
            self.loop_running = False
        else:
            self.get_logger().error("ERROR")

    def detect_hands(self):
        while self.loop_running:
            if self.frame is not None:
                # Convert the image to RGB
                image_rgb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                # Process the image
                results = self.hands.process(image_rgb)

                # Check if hands are detected
                if results.multi_hand_landmarks:
                    self.hand_detected = True
                    if not self.previous_hand_detected:
                        # Hand was just detected
                        msg = TopicString()
                        msg.command = 'Hand_detected'
                        self.hand_detect_publisher.publish(msg)
                        self.get_logger().info('Hand detected and message published')
                else:
                    self.hand_detected = False
                    if self.previous_hand_detected:
                        # Hand was just lost
                        msg = TopicString()
                        msg.command = 'Hand_not_detected'
                        self.hand_detect_publisher.publish(msg)
                        self.get_logger().info('Hand not detected and message published')

                # Update previous hand detected status
                self.previous_hand_detected = self.hand_detected

                # Annotate frame
                if self.hand_detected:
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.mp_drawing.draw_landmarks(self.frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                self.annotated_frame = self.frame.copy()
            else:
                self.get_logger().info("No frame available")

            # Sleep to reduce CPU usage
            time.sleep(0.1)

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
                self.loop_running = False  # Stop loop
                break

    def Webcam_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error in Webcam_callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    hand_detect = HandDetect()

    # MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(hand_detect)

    try:
        executor.spin()
    finally:
        hand_detect.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
