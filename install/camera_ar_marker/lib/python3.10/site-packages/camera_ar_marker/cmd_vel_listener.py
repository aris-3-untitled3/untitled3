import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received cmd_vel: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
