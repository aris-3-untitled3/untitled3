import rclpy as rp
from rclpy.node import Node
from untitled_msgs.msg import TopicString
from untitled_msgs.srv import ServiceString
from untitled_msgs.action import ActionString
import time

class RobotControl(Node):
    def __init__(self):
        super().__init__('RobotControl')

        # Robot_Server에서 토픽 받기 ( 호객행위 시작 / 호객행위 중지 / 아이스크림 제조)
        self.robot_server = self.create_subscription(
            TopicString,
            '/Server_to_Robot',
            self.robot_server_callback,
            10
        )

        # prevent unused variable warning
        self.robot_server

        # Robot_Server에서 /Signal_Robot 서비스를 받아옴
        self.robot_control_server = self.create_service(ServiceString, '/Signal_Robot', self.robot_control_callback)

        # Robot_Server로 토픽 퍼블리셔 (아이스크림 감지 / 아이스크림 제조 완료)
        self.robot_server_publisher = self.create_publisher(TopicString, '/Robot_to_Server', 10)



    def robot_server_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.command}')

        if msg.command == "human_detect":
            print("호객행위")
        elif msg.command == "guest_detect":
            print("중지")
        elif "Age" and "Gender" in msg.command:
            print("환영인사")
        elif msg.command == "ice_cream_Bucket_detect":
            print("아이스크림 제조 시작")

        elif "Conclusion" in msg.command:
            print("작별인사")
        else:
            return
        
    def ice_cream_making(self):
        # 로봇이 아이스크림 무게를 감지하면 시그널
        self.ice_cream_Bucket_detect()

    def ice_cream_making_complete(self):
        # 로봇이 아이스크림을 다 만들면 시그널
        self.ice_cream_production_complete()

    
    def ice_cream_Bucket_detect(self):
        self.get_logger().info('ice_cream_Bucket_detect detected!')

        msg = TopicString()
        msg.command = 'ice_cream_Bucket_detect'

        self.robot_server_publisher.publish(msg)


    def ice_cream_production_complete(self):
        self.get_logger().info('ice_cream_production_complete')

        msg = TopicString()
        msg.command = 'ice_cream_production_complete'

        self.robot_server_publisher.publish(msg)


    def robot_control_callback(self, request, response):
        self.get_logger().info(f'Received command: {request.command}')

        print("쓰레기 청소")
        print(request.command)
        time.sleep(3)

        response.success = True
        response.result = f'Command {request.command} received and being processed' 

        print(response)

        return response


def main(args=None):
    rp.init(args=args)

    Robot_Control = RobotControl()

    rp.spin(Robot_Control)

    Robot_Control.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()