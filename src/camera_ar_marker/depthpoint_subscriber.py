import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class DepthPointSubscriber(Node):
    def __init__(self):
        super().__init__('depth_point_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.point_cloud_callback,
            10)
        self.subscription  # prevent unused variable warning

    def point_cloud_callback(self, msg):
        # 포인트 클라우드 데이터를 반복자에서 리스트로 변환
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        if points:
            # 첫 번째 포인트의 좌표 가져오기
            first_point = points[0]
            x, y, z = first_point
            distance = np.sqrt(x**2 + y**2 + z**2)
            self.get_logger().info(f'First point distance: {distance:.2f} meters')
        else:
            self.get_logger().error('No points found in the point cloud.')

def main(args=None):
    rclpy.init(args=args)
    depth_point_subscriber = DepthPointSubscriber()
    rclpy.spin(depth_point_subscriber)
    depth_point_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
