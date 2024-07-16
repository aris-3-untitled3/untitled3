import rclpy as rp
from rclpy.node import Node
from untitled_msgs.msg import TopicString
from untitled_msgs.srv import ServiceString
from untitled_msgs.action import ActionString
import time

class PyQt(Node):
    def __init__(self):
        super().__init__('PyQt')

        # Robot_Server에서 토픽 받기 ( 3초 정면 대기 / 인사문구 / 메뉴얼 시작 / 제조완료 / 쓰레기 청소 끝 및 아이스크림 대기 / 작별문구 / 토핑 무게 전달 및 쓰레기 청소 완료)
        self.Robot_Server = self.create_subscription(
            TopicString,
            '/Server_to_UI',
            self.Robot_Server_callback,
            10
        )

        # prevent unused variable warning
        self.Robot_Server

        # Robot_Server에서 Signal_UI 서비스를 받아옴
        self.ui_server = self.create_service(ServiceString, '/Signal_UI', self.ui_callback)

        # Robot_Server로 토픽 퍼블리셔 (제조준비 / 마무리 상황)
        self.robot_server_publisher = self.create_publisher(TopicString, '/UI_to_Server', 10)


    def Robot_Server_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.command}')


    def Pre_production(self):
        self.get_logger().info('UI to Server , Pre-production!')

        msg = TopicString()
        msg.command = 'Pre-production'
        self.robot_server_publisher.publish(msg)


    def Conclusion(self):
        self.get_logger().info('UI to Server , Conclusion!')

        msg = TopicString()
        msg.command = 'Conclusion'
        self.robot_server_publisher.publish(msg)


    def ui_callback(self, request, response):
        self.get_logger().info(f'Received command: {request.command}')

        time.sleep(3)

        response.success = True
        response.result = f'Command {request.command} received and being processed' 

        return response


def main(args=None):
    rp.init(args=args)

    UI = PyQt()

    rp.spin(UI)

    UI.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()