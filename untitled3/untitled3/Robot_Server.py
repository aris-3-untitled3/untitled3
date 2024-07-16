### 대기 상태
    # 사람감지 여부 (사람 or not) : 토픽 : Guest_detect -> Robot_Server 
        # 호객행위 여부 (호객 or stop) : 토픽 : Robot_Server -> Robot_Control

    # 손님 감지 : 토픽 : Guest_detect -> Robot_Server   
        # 3초 정면 대기 : 토픽 : Robot_Server -> UI
        # 호객행위 중지 : 토픽 : Robot_Server -> Robot_Control

### 손님인지 상태
    # 손님 인지 여부 (성별,연령대 or not) : 토픽 : Guest_detect -> Robot_Server 
        # 인사문구 (or 복귀): 토픽 : Robot_Server -> UI
        # 환영인사 : 서비스 : Robot_Server -> Robot_Control
        # 메뉴얼 시작: 토픽 : Robot_Server -> UI
            # 손님 성별,연령대: 서비스 : UI -> DB_Manager
            # 음성 받기: 서비스 : UI -> Voice_Input (or 마우스 클릭)

    # 제조 준비 : 토픽 : UI -> Robot_Server
        # 쓰레기 감지 (ArCUoMarker 좌표) : 서비스 : Robot_Server -> Cup_detect
        # 쓰레기 전달 : 서비스 : Robot_Server -> UI (음성대기)
        # 쓰레기 청소 : 서비스 : Robot_Server -> Robot_Control 
        # 쓰레기 청소 끝 및 아이스크림 대기 : 토픽 : Robot_Server -> UI

### 제조 상태
    # 아이스크림 감지 (무게?) : 토픽 : Robot_Control -> Robot_Server  
        # 아이스크림 준비 : 서비스 : Robot_Server -> UI (음성대기)
        # 아이스크림 제조 시작 : 토픽 : Robot_Server -> Robot_Control

### 완료 상태
    # 아이스크림 제조 완료 : 토픽 : Robot_Control -> Robot_Server
        # 제조 완료 : 토픽 : Robot_Server -> UI 
            # 쿠폰 개수 : 서비스 : UI -> DB_Manager
            # 맛 가격 : 서비스 : UI -> DB_Manager
            # 일 매출 : 서비스 : UI -> DB_Manager

    # 마무리 상황 : 토픽 : UI -> Robot_Server
        # 작별 문구 : 토픽 : Robot_Server -> UI
        # 작별 인사 : 서비스 : Robot_Server -> Robot_Control
        # 쓰레기 감지 (ArCUoMarker 좌표) : 서비스 : Robot_Server -> Cup_detect
        # 쓰레기 전달 : 서비스 : Robot_Server -> UI (음성대기)
        # 쓰레기 청소 및 토핑 무게 확인 : 서비스 : Robot_Server -> Robot_Control  
        # 토핑 무게 전달 및 쓰레기 청소 완료 : 토픽 : Robot_Server -> UI
            # 토핑 재고량 전달 : 서비스 : UI -> DB_Manager

### 통신 정리
    # Robot_Server <(토픽)- Guest_detect (사람감지 , 손님감지 , 손님인지)
    #                      Robot_Control (아이스크림 감지 , 아이스크림 제조 완료)
    #                      UI (제조 준비 , 마무리 상황)
    # Robot_Server -(토픽)> Robot_Control , UI 
    # Robot_Server -(서비스)> - Robot_Control , UI , Cup_detect
    # UI -(서비스)> DB_Manager , Voice_Input


import rclpy as rp
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from untitled_msgs.msg import TopicString
from untitled_msgs.srv import ServiceString
import time


class RobotServer(Node):
    def __init__(self):
        super().__init__('RobotServer')

        # MutuallyExclusiveCallbackGroup 생성
        callback_group = MutuallyExclusiveCallbackGroup()

        # Guest_Detect에서 토픽 받기 (사람 / 손님 / 성별,연령대)
        self.guest_detect = self.create_subscription(
            TopicString,
            '/Guest_Info',
            self.guest_detect_callback,
            10,
            callback_group=callback_group
        )

        # Robot_Control 토픽 받기 (아이스크림 감지 / 아이스크림 제조 완료)
        self.robot_control = self.create_subscription(
            TopicString,
            '/Robot_to_Server',
            self.robot_control_callback,
            10,
            callback_group=callback_group
        )

        # UI에서 토픽 받기 (제조준비 / 마무리 상황)
        self.ui = self.create_subscription(
            TopicString,
            '/UI_to_Server',
            self.ui_callback,
            10,
            callback_group=callback_group
        )

        # prevent unused variable warning
        self.guest_detect
        self.robot_control
        self.ui

        # Robot_Control로 토픽 퍼블리셔
        self.robot_control_publisher = self.create_publisher(TopicString, '/Server_to_Robot', 10)

        # UI로 토픽 퍼블리셔
        self.ui_publisher = self.create_publisher(TopicString, '/Server_to_UI', 10)

        # Cup_detect로 서비스를 요청
        self.Cup_detect_client = self.create_client(ServiceString, '/Cup_Info')

        # Robot_Control로 서비스를 요청
        self.robot_control_client = self.create_client(ServiceString, '/Signal_Robot')

        # UI로 서비스를 요청
        self.ui_client = self.create_client(ServiceString, '/Signal_UI')

    # 사람 / 손님 / 성별,연령대
    async def guest_detect_callback(self, msg):
        # 사람 감지
        if msg.command == "human_detect":
            self.get_logger().info('Processing human_detect')
            # 호객행위 여부 (호객 or stop) : 토픽 : Robot_Server -> Robot_Control
            self.robot_control_publisher.publish(msg)

        # 손님 감지
        elif msg.command == "guest_detect":
            self.get_logger().info('Processing guest_detect')
            # 3초 정면 대기 : 토픽 : Robot_Server -> UI
            self.ui_publisher.publish(msg)
            # 호객행위 중지 : 토픽 : Robot_Server -> Robot_Control
            self.robot_control_publisher.publish(msg)

        # 손님 인지 (성별,연령대)
        elif msg.command == "guest_confirm":
            self.get_logger().info('Processing guest_confirm')
            # 인사문구 (or 복귀): 토픽 : Robot_Server -> UI
            self.ui_publisher.publish(msg)
            time.sleep(10)
            # 환영인사 : 서비스 : Robot_Server -> Robot_Control
            await self.send_robot_control(msg.command)
            time.sleep(3)
            # 메뉴얼 시작: 토픽 : Robot_Server -> UI
            self.ui_publisher.publish(msg)
                # 손님 성별,연령대: 서비스 : UI -> DB_Manager
                # 음성 받기: 서비스 : UI -> Voice_Input (or 마우스 클릭)

        # 예외 처리
        else:
            self.get_logger().error('Unknown command received')

    # 아이스크림 감지 , 아이스크림 제조완료
    async def robot_control_callback(self, msg):
        # 아이스크림 감지
        if msg.command == "ice_cream_Bucket_detect":
            self.get_logger().info('Processing ice_cream_Bucket_detect')
            # 아이스크림 준비 : 서비스 : Robot_Server -> UI (음성대기)
            await self.send_ui(msg.command)
            time.sleep(3)
            # 아이스크림 제조 시작 : 토픽 : Robot_Server -> Robot_Control
            self.robot_control_publisher.publish(msg)
        # 아이스크림 제조완료
        elif msg.command == "ice_cream_production_complete":
            self.get_logger().info('Processing ice_cream_production_complete')
            # 제조 완료 : 토픽 : Robot_Server -> UI
            self.ui_publisher.publish(msg) 
                # 쿠폰 개수 : 서비스 : UI -> DB_Manager
                # 맛 가격 : 서비스 : UI -> DB_Manager
                # 일 매출 : 서비스 : UI -> DB_Manager
        # 예외 처리
        else:
            self.get_logger().error('Unknown command received')

    # 제조 준비 , 마무리 상황
    async def ui_callback(self, msg):
        # 제조 준비
        if msg.command == "Pre-production":
            self.get_logger().info('Processing Pre-production')
            # 쓰레기 감지 (ArCUoMarker 좌표) : 서비스 : Robot_Server -> Cup_detect
            await self.send_cup_detect(msg.command)
            # 쓰레기 전달 : 서비스 : Robot_Server -> UI (음성대기)
            await self.send_ui(msg.command)
            # 쓰레기 청소 : 서비스 : Robot_Server -> Robot_Control
            await self.send_robot_control(msg.command) 
            # 쓰레기 청소 끝 및 아이스크림 대기 : 토픽 : Robot_Server -> UI
            self.ui_publisher.publish(msg)
        # 마무리 상황
        elif msg.command == "Conclusion":
            self.get_logger().info('Processing Conclusion')
            # 작별 문구 : 토픽 : Robot_Server -> UI
            self.ui_publisher.publish(msg)
            # 작별 인사 : 서비스 : Robot_Server -> Robot_Control
            await self.send_robot_control(msg.command)
            # 쓰레기 감지 (ArCUoMarker 좌표) : 서비스 : Robot_Server -> Cup_detect
            await self.send_cup_detect(msg.command)
            # 쓰레기 전달 : 서비스 : Robot_Server -> UI (음성대기)
            await self.send_ui(msg.command)
            # 쓰레기 청소 및 토핑 무게 확인 : 서비스 : Robot_Server -> Robot_Control
            await self.send_robot_control(msg.command)  
            # 토핑 무게 전달 및 쓰레기 청소 완료 : 토픽 : Robot_Server -> UI
            self.ui_publisher.publish(msg)
                # 토핑 재고량 전달 : 서비스 : UI -> DB_Manager
            ## Guest_detect 토픽 다시 받기
        # 예외 처리
        else:
            self.get_logger().error('Unknown command received')

    # 쓰레기 컵 감지 요청 (ArCUoMarker 좌표)
    async def send_cup_detect(self, msg: str):
        if not self.Cup_detect_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/Cup_Info service not available')
            return

        request = ServiceString.Request()
        request.command = msg

        future = self.Cup_detect_client.call_async(request)

        future.add_done_callback(self.handle_response_1)

    def handle_response_1(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Success (Server to Cup): %s' % response.result)
            else:
                self.get_logger().error('Failed (Server to Cup): %s' % response.result)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    # [환영 인사 , 쓰레기 청소 , 작별 인사 , 쓰레기 청소 및 토핑 무게 확인] 요청
    async def send_robot_control(self, msg: str):
        if not self.robot_control_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/Signal_Robot service not available')
            return

        request = ServiceString.Request()
        request.command = msg

        future = self.robot_control_client.call_async(request)  # 비동기 호출

        future.add_done_callback(self.handle_response_2)

    def handle_response_2(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Success (Server to Control): %s' % response.result)
            else:
                self.get_logger().error('Failed (Server to Control): %s' % response.result)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    # [쓰레기 상황 전달 (음성) , 아이스크림 준비 (음성)] 요청
    async def send_ui(self, msg: str):
        if not self.ui_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/Signal_UI service not available')
            return

        request = ServiceString.Request()
        request.command = msg

        future = self.ui_client.call_async(request)

        future.add_done_callback(self.handle_response_3)

    def handle_response_3(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Success (Server to UI): %s' % response.result)
            else:
                self.get_logger().error('Failed (Server to UI): %s' % response.result)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rp.init(args=args)
    Robot_Server = RobotServer()
    rp.spin(Robot_Server)
    Robot_Server.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
