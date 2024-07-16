import rclpy as rp
from rclpy.node import Node
from untitled_msgs.msg import TopicString
from untitled_msgs.srv import ServiceString
from untitled_msgs.action import ActionString
import time

class Gusetdetect(Node):
    def __init__(self):
        super().__init__('Gusetdetect')

        # Robot_Server로 토픽 퍼블리셔 (사람 / 손님 / 성별,연령대)
        self.robot_server_publisher = self.create_publisher(TopicString, '/Guest_Info', 10)


    def human_detect(self):
        self.get_logger().info('human detected!')

        msg = TopicString()
        msg.command = 'human_detect'

        self.robot_server_publisher.publish(msg)


    def guest_detect(self):
        self.get_logger().info('guest detected!')

        msg = TopicString()
        msg.command = 'guest_detect'

        self.robot_server_publisher.publish(msg)


    def guest_confirm(self):
        self.get_logger().info('guest confirm!')

        msg = TopicString()
        msg.command = 'guest_confirm'
        
        self.robot_server_publisher.publish(msg)


def main(args=None):
    rp.init(args=args)

    Guset_detect = Gusetdetect()

    rp.spin(Guset_detect)

    Guset_detect.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()