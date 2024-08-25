import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
 
class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
 
    def rotate_angle(self, angle_in_degrees, angular_speed):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed  # 회전 속도 (라디안/초)
 
        # 각도에 따른 회전 시간 계산
        rotation_time = abs(angle_in_degrees) * (3.14 / 180.0) / angular_speed
 
        # 회전 명령 발행
        self.publisher_.publish(twist)
        self.get_logger().info(f'Rotating for {rotation_time:.2f} seconds...')
         
        # 지정된 시간 동안 회전
        time.sleep(rotation_time)
         
        # 멈춤 명령 발행
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('Rotation complete, robot stopped.')
 
def main(args=None):
    rclpy.init(args=args)
    node = RotateRobot()
    node.rotate_angle(90, 0.5)  # 원하는 각도와 회전 속도 입력
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
