import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class RotateTurtle(Node):
    def __init__(self):
        super().__init__('rotate_turtle')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    def rotate_angle(self, angle_in_degrees, angular_speed):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = math.radians(angular_speed)  # 회전 속도 (라디안/초)

        # 회전할 각도와 시간 계산
        target_angle = math.radians(abs(angle_in_degrees))
        time_to_rotate = target_angle / math.radians(angular_speed)

        # 회전 방향 설정
        twist.angular.z = twist.angular.z if angle_in_degrees > 0 else -twist.angular.z

        # 회전 명령 발행
        self.get_logger().info(f'Rotating {angle_in_degrees} degrees...')
        
        # 현재 시간 얻기
        start_time = self.get_clock().now().seconds_nanoseconds()
        start_time_in_sec = start_time[0] + start_time[1] * 1e-9

        while (self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9) - start_time_in_sec < time_to_rotate:
            self.publisher_.publish(twist)

        # 회전 멈춤
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('Rotation complete.')

def main(args=None):
    rclpy.init(args=args)
    node = RotateTurtle()
    node.rotate_angle(90, 45)  # 90도 회전, 45도/초 속도
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
