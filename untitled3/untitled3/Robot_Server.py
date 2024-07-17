import rclpy as rp
from rclpy.node import Node
from untitled_msgs.msg import TopicString
from untitled_msgs.srv import ServiceString
import time
import threading


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


class RobotServer(Node):
    def __init__(self):
        super().__init__('RobotServer')

        # Guest_Detect에서 토픽 받기 (사람 / 손님 / 성별,연령대)
        self.guest_detect = self.create_subscription(
            TopicString,
            '/Guest_Info',
            self.guest_detect_callback,
            10
        )

        # Robot_Control 토픽 받기 (아이스크림 감지 / 아이스크림 제조 완료)
        self.robot_control = self.create_subscription(
            TopicString,
            '/Robot_to_Server',
            self.robot_control_callback,
            10
        )

        # UI에서 토픽 받기 (제조준비 / 마무리 상황)
        self.ui = self.create_subscription(
            TopicString,
            '/UI_to_Server',
            self.ui_callback,
            10
        )

        # prevent unused variable warning
        self.guest_detect
        self.robot_control
        self.ui

        # Gender_Age_preditor로 토픽 퍼블리셔
        self.ga_publisher = self.create_publisher(TopicString, '/Server_to_GA', 10)

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
    def guest_detect_callback(self, msg):
        threading.Thread(target=self.handle_guest_detect,args=(msg,)).start()

    def handle_guest_detect(self, msg):
        # 사람 감지
        if msg.command == "human_detect":
            self.get_logger().info('Processing human_detect')
            # 호객행위 여부 (호객 or stop) : 토픽 : Robot_Server -> Robot_Control
            self.robot_control_publisher.publish(msg)

        # 손님 감지
        elif msg.command == "guest_detect":
            self.get_logger().info('Processing guest_detect')
            # 호객행위 중지 : 토픽 : Robot_Server -> Robot_Control
            self.robot_control_publisher.publish(msg)
            # 3초 정면 대기 : 토픽 : Robot_Server -> UI
            self.ui_publisher.publish(msg)
            # 3초동안 얼굴 성별 , 연령대 판별
            self.ga_publisher.publish(msg)

        # 손님 인지 (성별,연령대)
        elif "Age" and "Gender" in msg.command:
            self.get_logger().info('Processing guest_confirm')
            print(msg.command)
            # 환영인사 : 토픽 : Robot_Server -> Robot_Control
            self.robot_control_publisher.publish(msg)
            # 메뉴얼 시작: 토픽 : Robot_Server -> UI
            self.ui_publisher.publish(msg)

        # 예외 처리
        else:
            self.get_logger().error('Unknown command received')


    # 아이스크림 감지 , 아이스크림 제조완료
    def robot_control_callback(self, msg):
        threading.Thread(target=self.handle_robot_control,args=(msg,)).start()
    
    def handle_robot_control(self, msg):
        # 아이스크림 감지
        if msg.command == "ice_cream_Bucket_detect":
            self.get_logger().info('Processing ice_cream_Bucket_detect')
            # 아이스크림 준비 : 서비스 : Robot_Server -> UI (음성대기)
            self.send_ui(msg.command)
            # 아이스크림 제조 시작 : 토픽 : Robot_Server -> Robot_Control
            self.robot_control_publisher.publish(msg)
        # 아이스크림 제조완료
        elif msg.command == "ice_cream_production_complete":
            self.get_logger().info('Processing ice_cream_production_complete')
            # 제조 완료 : 토픽 : Robot_Server -> UI
            self.ui_publisher.publish(msg) 
        # 예외 처리
        else:
            self.get_logger().error('Unknown command received')


    # 제조 준비 , 마무리 상황
    def ui_callback(self, msg):
        threading.Thread(target=self.handle_ui,args=(msg,)).start()
    
    def handle_ui(self, msg):
        # 제조 준비
        if msg.command == "Pre-production":
            self.get_logger().info('Processing Pre-production')
            # 쓰레기 감지 (ArCUoMarker 좌표) : 서비스 : Robot_Server -> Cup_detect
            # 감지 후 상황판다
            response = self.send_cup_detect(msg.command)

            if "," in response.result :

                # 쓰레기 전달 : 서비스 : Robot_Server -> UI (음성대기)
                self.send_ui(msg.command)

                # 쓰레기 청소 : 서비스 : Robot_Server -> Robot_Control
                msg.command = f"Pre-production : {response.result}"
                print(msg.command)
                self.send_robot_control(msg.command) 

                msg.command = "Pre-production"
                # 쓰레기 청소 끝 및 아이스크림 대기 : 토픽 : Robot_Server -> UI
                self.ui_publisher.publish(msg)

            else :
                print("쓰레기 없음")

        # 마무리 상황
        elif "Conclusion" in msg.command :
            self.get_logger().info('Processing Conclusion')
            final = msg.command
            # 작별 인사 : 토픽 : Robot_Server -> Robot_Control
            self.robot_control_publisher.publish(msg)
            time.sleep(5)

            # 쓰레기 감지 (ArCUoMarker 좌표) : 서비스 : Robot_Server -> Cup_detect
            response = self.send_cup_detect(msg.command)

            if "," in response.result :

                # 쓰레기 전달 : 서비스 : Robot_Server -> UI (음성대기)
                self.send_ui(msg.command)

                # 쓰레기 청소 : 서비스 : Robot_Server -> Robot_Control
                msg.command = f"Conclusion : {response.result}"
                print(msg.command)
                self.send_robot_control(msg.command) 

                # 쓰레기 청소 끝 및 아이스크림 대기 : 토픽 : Robot_Server -> UI
                self.ui_publisher.publish(final)
            else :
                print("쓰레기 없음")

            if "ok" in final :
                print("재고량 있음")
                ## Guest_detect 토픽 다시 받기
                self.ga_publisher.publish(msg)
            else :
                print("재고량 없음")
                return
            
        elif msg.command == "start":
            self.get_logger().info('START!')
            self.ga_publisher.publish(msg)

        # 예외 처리
        else:
            self.get_logger().error('Unknown command received')


        
    # 쓰레기 컵 감지 요청 (ArCUoMarker 좌표)
    def send_cup_detect(self, msg: str):
        if not self.Cup_detect_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/Cup_Info service not available')
            return

        request = ServiceString.Request()
        request.command = msg

        future = self.Cup_detect_client.call(request)

        response = future

        if response.success:
            self.get_logger().info('Success (Server to Cup): %s' % response.result)
        else:
            self.get_logger().error('Failed (Server to Cup): %s' % response.result)

        return response


    # [환영 인사 , 쓰레기 청소 , 작별 인사 , 쓰레기 청소 및 토핑 무게 확인] 요청
    def send_robot_control(self, msg: str):
        if not self.robot_control_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/Signal_Robot service not available')
            return

        request = ServiceString.Request()
        request.command = msg

        future = self.robot_control_client.call(request)  # 비동기 호출

        response = future

        if response.success:
            self.get_logger().info('Success (Server to Robot): %s' % response.result)
        else:
            self.get_logger().error('Failed (Server to Robot): %s' % response.result)

    # [쓰레기 상황 전달 (음성) , 아이스크림 준비 (음성)] 요청
    def send_ui(self, msg: str):
        if not self.ui_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/Signal_UI service not available')
            return

        request = ServiceString.Request()
        request.command = msg

        future = self.ui_client.call(request)

        response = future
    
        if response.success:
            self.get_logger().info('Success (Server to UI): %s' % response.result)
        else:
            self.get_logger().error('Failed (Server to UI): %s' % response.result)


def main(args=None):

    rp.init(args=args)

    Robot_Server = RobotServer()

    rp.spin(Robot_Server)

    Robot_Server.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()