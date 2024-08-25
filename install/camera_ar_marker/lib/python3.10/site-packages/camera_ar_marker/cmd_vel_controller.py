import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg.Twist import Twist

class CmdVelController(Node):
    def __init__(self):
        super().__init__('cmd_vel_controller')
        self.subscription = self.create_subscription(Float32MultiArray, 'aruco_marker_info', self.marker_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist_msg = Twist()

    def marker_callback(self, msg):
        distance, angle = msg.data
        if angle >= 1.0:  # 특정 각도가 되면 회전 멈춤
            self.twist_msg.angular.z = 0.0
        else:
            self.twist_msg.angular.z = 0.3  # 회전 속도

        self.cmd_vel_publisher.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
