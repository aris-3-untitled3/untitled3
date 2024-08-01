import rclpy as rp
from rclpy.node import Node
from untitled_msgs.msg import TopicString
from untitled_msgs.srv import ServiceString
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import threading

# (대기 - ui시작토픽/bgm시작,사람감지시작)
# (사람 - 2m안토픽/로봇호객,음성환영)
# (손님 - 0.5m안토픽/로봇인사,음성인사,ui전환,음성설명,성별&연령대판별(저장),ui전환)

# (사용법 - 음성설명)
# (메뉴 - 음성설명,음성받기,음성설명,음성받기,음성설명)(맛,토핑저장)(1선택,2선택,3선택 시나리오)
# (쿠폰 - 음성출력,번호입력에따른 쿠폰갯수확인) 
# (결제 - 갯수받기,가격받기(맛,토핑),시간받기,음성출력,매출/손님 저장)(쿠폰등록유/무)(사용유/무)
####################################################################
# (준비 - 준비화면토픽/음성설명,쓰레기검출,음성설명,쓰레기처리,캡슐검출시작)(쓰레기유/무)
# (캡슐&실링 - 캡슐검출토픽/음성설명,로봇캡슐,실링확인,음성설명,ui전환,로봇제조(맛,토핑),ui전환)(실링유/무)

# (완료 - 음성출력,화면전환)
# (맛평가 - 음성출력,평가확인,음성출력)(굿/평/뱃)
# (재고 - 재고량확인,음성출력)(재고유/무)

# (마무리 - 마무리토픽/음성출력,쓰레기검출,음성출력,쓰레기처리,음성출력,대기전환토픽)(쓰레기유/무)

class RobotServer(Node):
    def __init__(self):
        super().__init__('RobotServer')

        self.callback_group = ReentrantCallbackGroup()

        # UI에서 토픽 받기 (open_First)
        self.ui = self.create_subscription(
            TopicString,
            '/UI_to_Server',
            self.ui_callback,
            10,
            callback_group=self.callback_group
        )

        # Human_Detect에서 토픽 받기 (human_detect / guest_detect)
        self.human_detect = self.create_subscription(
            TopicString,
            '/Human_to_Server',
            self.human_detect_callback,
            10,
            callback_group=self.callback_group
        )

        # # Robot_Control 토픽 받기 (아이스크림 감지 / 아이스크림 제조 완료)
        # self.robot_control = self.create_subscription(
        #     TopicString,
        #     '/Robot_to_Server',
        #     self.robot_control_callback,
        #     10
        # )

        # prevent unused variable warning
        self.ui
        # self.human_detect
        # self.robot_control

        # UI로 토픽 퍼블리셔
        self.ui_publisher = self.create_publisher(TopicString, '/Server_to_UI', 10)

        # Voice_Out로 토픽 퍼블리셔
        self.Voice_out_publisher = self.create_publisher(TopicString, '/Server_to_Voice_out', 10)

        # Human_detect로 토픽 퍼블리셔
        self.Human_detect_publisher = self.create_publisher(TopicString, '/Server_to_Human', 10)

        # Robot_Control로 토픽 퍼블리셔
        self.robot_control_publisher = self.create_publisher(TopicString, '/Server_to_Robot', 10)

        # Voice_out로 서비스를 요청
        self.Voice_out_client = self.create_client(ServiceString, '/Call_to_Voice_out')

        # Guest_detct로 서비스를 요청
        self.Guest_detect_client = self.create_client(ServiceString, '/Call_to_Guest')

        # Cup_detect 서비스를 요청
        self.Cup_detect_client = self.create_client(ServiceString, '/Call_to_HM')

        # Robot_Control 서비스를 요청
        self.Robot_client = self.create_client(ServiceString, '/Call_to_Robot')

        # Container 서비스를 요청
        self.Container_client = self.create_client(ServiceString, '/Call_to_CS')


    def ui_callback(self, msg):
        # 시작
        if msg.command == "open_First":
            # bgm 시작 - voice_out로 "play_bgm" 토픽 전달
            msg = TopicString()
            # msg.command = f'play_bgm'
            # self.Voice_out_publisher.publish(msg)
            # print(msg.command)

            # 사람감지 시작 - Human_detect로 "start" 토픽 전달
            msg.command = f'start'
            self.Human_detect_publisher.publish(msg)
            print(msg.command)
        # elif msg.command == "open_Direction":
            # self.send_voice_out("사용법 설명입니다")
        elif msg.command == "open_Prepare":
            # self.send_voice_out("쓰레기를 검출하겠습니다")
            self.send_robot_detect("trash_pre")
            xy = self.send_cup_detect("start")
            result, x_y = xy.split(': ')
            x, y = x_y.split(',')
            x = float(x.strip())  # 좌표 값을 실수로 변환
            y = float(y.strip()) 
            print(x)
            print(y)
            self.send_robot_detect("home")

            # if x is not None and y is not None:
            # self.send_voice_out("쓰레기가 검출되었습니다")
            self.send_robot_detect(f"trash_detected,{x},{y}")
            # else:
            #     print("not cleaning")
            # if resulting is "sealing":
            #     self.send_robot_detect(f"return,B")
            # self.send_container_detect("start_sealing")
            # print("1")
            # msg = TopicString()
            # msg.command = "ice_cream_ready"
            # self.ui_publisher.publish(msg)
            # print("2")
            
        elif msg.command == "open_making":
            time.sleep(5)
            self.send_robot_detect(f"pre_making,B")
            time.sleep(5)
            self.send_robot_detect(f"making,1,B")
            print("5")
            msg.command = "ice_cream_completed"
            self.ui_publisher.publish(msg)
        else:
            print("Unkown command")

    def human_detect_callback(self,msg):
        # 사람감지 , 손님인식
        print(msg.command)

        if msg.command == "human_detect":
            # 사람감지 - Robot_control로 "human_detect" 토픽 전달 - 호객행위
            self.robot_control_publisher.publish(msg)
            # Voice_out로 "문구" 서비스 요청 - 호객인사
            # self.send_voice_out("어서오세요 맛있는 아이스크림이 있습니다!")

        elif msg.command == "guest_detect":
            # 손님감지 - Robot_control로 "guest_detect" 토픽 전달 - 환영인사
            self.robot_control_publisher.publish(msg)
            # ui로 "guest_detect" 토픽 전달 - 로딩화면 전환
            self.ui_publisher.publish(msg)  
            # Voice_out로 "문구" 서비스 요청 - 호객인사
            # self.send_voice_out("안녕하세요 손님 3초간 대기해 주세요!")
            # 성별 & 연령대 판별
            self.send_guest_detect(msg.command)
            # ui로 "guest_confirm" 토픽 전달 - 사용법화면 전환
            msg = TopicString()
            msg.command = "guest_confirm"
            print(msg.command)
            self.ui_publisher.publish(msg)
            
        else:
            print("Unkown command")

    # 캡슐 서비스 요청
    def send_container_detect(self, msg: str):
        if not self.Container_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/Call_to_container service not available')
            return

        request = ServiceString.Request()
        request.command = msg

        future = self.Container_client.call(request)  

        response = future

        if response.success:
            self.get_logger().info('Success (Server to container): %s' % response.result)
        else:
            self.get_logger().error('Failed (Server to container): %s' % response.result)

        return response.result

    # 로봇 서비스 요청
    def send_robot_detect(self, msg: str):
        if not self.Robot_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/Call_to_Robot_client service not available')
            return

        request = ServiceString.Request()
        request.command = msg

        future = self.Robot_client.call(request)  

        response = future

        if response.success:
            self.get_logger().info('Success (Server to Robot_client): %s' % response.result)
        else:
            self.get_logger().error('Failed (Server to Robot_client): %s' % response.result)

        return response.result

    # 쓰레기 컴 검출 서비스 요청
    def send_cup_detect(self, msg: str):
        if not self.Cup_detect_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/Call_to_Cup service not available')
            return

        request = ServiceString.Request()
        request.command = msg

        future = self.Cup_detect_client.call(request)  

        response = future

        if response.success:
            self.get_logger().info('Success (Server to Cup): %s' % response.result)
        else:
            self.get_logger().error('Failed (Server to Cup): %s' % response.result)

        return response.result


    # 성별 & 연령대 서비스 요청
    def send_guest_detect(self, msg: str):
        if not self.Guest_detect_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/Call_to_Voice_out service not available')
            return

        request = ServiceString.Request()
        request.command = msg

        future = self.Guest_detect_client.call(request)  

        response = future

        if response.success:
            self.get_logger().info('Success (Server to Voice_out): %s' % response.result)
        else:
            self.get_logger().error('Failed (Server to Voice_out): %s' % response.result)

    # 음성출력 서비스 요청
    def send_voice_out(self, msg: str):
        if not self.Voice_out_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/Call_to_Voice_out service not available')
            return

        request = ServiceString.Request()
        request.command = msg

        future = self.Voice_out_client.call(request)  

        response = future

        if response.success:
            self.get_logger().info('Success (Server to Voice_out): %s' % response.result)
        else:
            self.get_logger().error('Failed (Server to Voice_out): %s' % response.result)


def main(args=None):

    rp.init(args=args)

    Robot_Server = RobotServer()

    # MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(Robot_Server)

    try:
        executor.spin()
    finally:
        Robot_Server.destroy_node()
        rp.shutdown()
if __name__ == '__main__':
    main()