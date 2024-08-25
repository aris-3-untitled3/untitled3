import rclpy
from rclpy.executors import MultiThreadedExecutor
from camera_ar_marker.ar_marker_subscriber import ArucoMarkerSubscriber

def main(args=None):
    rclpy.init(args=args)
    aruco_marker_subscriber = ArucoMarkerSubscriber()

    executor = MultiThreadedExecutor(num_threads=2)  # 두 개의 스레드로 동작
    executor.add_node(aruco_marker_subscriber)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        aruco_marker_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
