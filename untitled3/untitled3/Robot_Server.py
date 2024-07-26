import rclpy as rp
from rclpy.node import Node
from untitled_msgs.msg import TopicString
from untitled_msgs.srv import ServiceString
import time
import threading

# 0. Human_detect  -(토픽)> Server : No_Human (사람없음) / UI -(토픽)>Server : back_to_home (버튼클릭) 
# 	(- Server -(토픽)> Robot_Control , Voice_out , Hand_Input , Cup_detect , Container_detect : STOP!)
# Server -(토픽)> Human_detect :  사람 감지 시작
# Server -(토픽)> Hand_Input : 손인식 시작
# Server -(토픽)> Voice_out : "잔잔한 bgm"

# ---------------------------------------------------------------------------------------------------------------------
# 1.Human_detect  -(토픽)> Server : human_detect (사람 감지)
# 	(- Server -(토픽)> Robot_Control , Voice_out , Hand_Input , Cup_detect , Container_detect : STOP!)
# Server -(토픽)> Robot_Control : 호객행위
# Server -(토픽)> Voice_out :  "어서오세요 맛있는 아이스크림이 기다려요!" + "밝은 bgm" (multithread)

# ---------------------------------------------------------------------------------------------------------------------
# 2. Human_detect(stop) -(토픽)> Server : guest_detect (손님 감지) / Hand_Input -(토픽)> Server : guest_detect (손인사)
# Server -(토픽)> Voice_out,  Human_detect : STOP 
# Server -(토픽)> Robot_Control : 호객행위 정지 (set_state(3)) (대기상태 복귀)
# Server -(토픽)> UI : 로딩화면
# Server -(서비스)> Voice_out : "안녕하세요 아이스크림가게에 오신걸 환영합니다 ! 3초만 대기해주세요"
# Server -(토픽)> Guest_detect : 성별,연령대 판별 (응답 - 성별,연령대) (변수 저장)

# ------------------------------------------------------------------------------------------------------------------------
# 3. Guest_detect(stop) -(토픽)> Server : guest_confirm (손님 맞이) (성별,연령대) / UI -(토픽)> Server : back_to_manual (버튼클릭)
# 	(- Server -(토픽)> Robot_Control , Voice_out , Cup_detect : STOP!)
# Server  -(토픽)-> UI : 사용법 설명 화면
# Server  -(서비스)-> Voice_out : "사용법은~ , 다음으로 넘어가겠습니까?" + "성별,연령대에 맞는 bgm" (multithread)
# Server -(서비스)> Voice_input : 음성인식 (응답 : 네 or 아니)
# 			if "네" : Server -(서비스)> Voice_out  "다음 페이지로 넘어가겠습니다"
# 				    (next_to_recommend)
# 			else : Server -(토픽)> Voice_out  "마우스를 클릭해주세요"			
# 				  (반복) Server -(서비스)> Human_detect :  (reamain) 20초동안 사람감지 
# 					if "있음" : Server -(토픽)> Voice_out  "마우스를 클리개해주세요!" + return
# 					else : "없음" : No_Human (사람없음)
# --------------------------------------------------------------------------------------------------------
# 4. UI -(토픽)> Server : next_to_recommend (버튼클릭) / UI -(토픽)> Server : back_to_recommend (버튼클릭)  
# 	(- Server -(토픽)> Robot_Control , Voice_out , Cup_detect , Container_detect : STOP!)
#   DB_Manager : 성별,연령대에 따른 맛 선호도 (아이스크림맛,토핑맛)
#   Server -(토픽)> UI : 메뉴 화면 (+맛 선호도)
#   Server -(서비스)> Voice_out : "추천메뉴는 ~ 선택하시겠습니까?"(+ 맛 선호도) 
#    Server -(서비스)> Voice_input : 음성인식 (응답 : 네 or 아니요)
# 			if "네"
#   Server -(토픽)> UI  : 추천메뉴 색깔 진하게
#   Server -(토픽)> Voice_out  "~맛,~맛으로 선택하셨습니다"
#   (next_to_preproduction)
#   else
#   Server -(서비스)> Voice_out : "어떤 메뉴를 선택하겠습니까?"
#   Server -(서비스)> Voice_input : 음성인식 (응답 : 아이스크림맛,토핑맛)
# 		if "맛"
# 		Server -(토픽)> Voice_out  "~맛,~맛으로 선택하셨습니다"
# 		(next_to_preproduction)
# 		else 
# 		Server -(토픽)> Voice_out  "마우스를 클릭하셔서 선택하여 주세요"
# 		(반복) Server -(서비스)> Human_detect :  (reamain) 20초동안 사람감지 
# 				if "있음" : Server -(토픽)> Voice_out  "마우스를 클리개해주세요!" + return
# 				else : "없음" : No_Human (사람없음)
# ----------------------------------------------------------------------------------
# 5. UI -(토픽)> Server : next_to_preproduction (버튼클릭) 
# 	(- Server -(토픽)> Voice_out : STOP!)
# Server -(토픽)> UI : 제조 준비 화면
# Server -(서비스)> Voice_out : "쓰레기를 확인하겠습니다"
# Server -(서비스)> Cup_detect : "쓰레기 감지" (감지 , 좌표 / 노감지)
# 	if "감지" : Server -(서비스)> Voice_out : "쓰레기를 처리하겠습니다."
# 			Server -(서비스?)> Robot_Control : 쓰레기 처리 (x,y 좌표로 이동 , 그리퍼 180도 회전 , z축 내리기 , 집기 , 쓰레기통이동 , 버리기 , 원래상태 복귀) 
# 	else :
# 			Server -(서비스)> Voice_out : "쓰레기가 없습니다."
# Server -(서비스)> Voice_out : "아이스크림을 제조하기 전 아이스크림 통을 가져오기 바랍니다" 
# Server -(서비스)> Container_Sealing_detect : 1분간 아이스크림 통 감지 시작 (감지 or 노감지)
# 	if "감지" : next_to_making
# 	else : No_Human (사람없음)
# --------------------------------------------------------------------------------------
# 6. Container_Sealing_detect -(토픽)> Server : next_to_making (통 감지)
# 	(- Server -(토픽)> Voice_out , Hand_Input , Container_Sealing_detect : STOP!)
# Server -(토픽)> UI : 제조 시작 화면
# Server -(서비스)> Voice_out : "제조를 시작하겠습니다"
# Server -(토픽)> UI : 제조 중 화면
# Server -(서비스)> Robot_Control : 실링 확인
# Server -(서비스)> Voice_out : "실링을 확인하겠습니다"
# Server -(서비스)> Container_Sealing_detect : 3초간 실링 확인 
# 	if "없음" : Server -(서비스)> Voice_out : "실링이 제거 되었습니다"			
# 			Server -(토픽)> Hand_Interrupt : 손감지
# 			Server -(서비스?)> Robot_Control : 아이스크림 제조 (통 놓기 , 컵 올리기 , 컵 가져오기 , 토핑위치에 놓기 , 토핑 뿌리기 , 아이스크림 내리기 , 컵 원위치)
# 			Server -(토픽)> UI : 제조 완료 화면
# 			Server -(서비스)> Voice_out : "아이스크림이 제조완료 되었습니다 , 다음으로 넘어가겠습니까?"
# 			Server -(서비스)> Voice_input : 음성인식 (응답 : 네 or 아니)
# 				if "네"
# 					Server -(서비스)> Voice_out : "다음으로 넘어가겠습니다"
# 					(next_to_Coupon)
# 				else :
# 					Server -(서비스)> Voice_out : "마우스를 클릭해 주세요"
# 	(반복) Server -(서비스)> Human_detect :  (reamain) 20초동안 사람감지 
# 							if "있음" :  Server -(토픽)> Voice_out  "마우스를 클리개해주세요!" + return
# 							else : "없음" : ALART (도둑)
# 	else:
# 		Server -(서비스)> Voice_out : "실링을 제거해주세요"
# 		Server -(서비스)> Robot_Control : 원위치 전달 (다시 반복)
# 6.1 Hand_Interrupt -(토픽)>  Server : 손감지 O
# Server -(토픽)> Robot_Control : set_state(3)
# Server -(서비스)> Voice_out : "방해하지 마라!"
# 6.2 Hand_Interrupt -(토픽)>  Server : 손감지 X (방해 후)
# Server -(토픽)> Robot_Control : set_state(0)
# Server -(서비스)> Voice_out : "다시 시작하겠습니다!"
# --------------------------------------------------------------------------------------
# 7.Robot_Control -(토픽)> Server :  next_to_coupon (아이스크림 제조 후) , UI -(토픽)> Server : back_to_coupon (버튼클릭)
# 	(- Server -(토픽)> Robot_Control , Voice_out , Cup_detect , Container_detect : STOP!)
# Server -(토픽)> UI : 쿠폰 화면
# Server - (서비스) >  Guest_detect ; 3초간 사용자 판단
# 		if "사용자" :
# 			Server -(서비스)> Voice_out : "안녕하세요 00시 , 자동으로 쿠폰 적립하겠습니다."
# 			(next_to_payment)
# 		else : 
# 	Server -(토픽)> Hand_Input : 손인식 시작 (전 화면 , 결제 , 등록X)
# 	Server -(서비스)> Voice_out : "쿠폰을 등록하셔서 10개가 모이면 아이스크림 1개가 공짜입니다 . 사용자 등록 하시겠습니까?"
# 	Server -(서비스)> Voice_input : 음성인식 (응답 : 네 or 아니)
# if "네" : 
# 	Server -(서비스)> Voice_out : "사용자를 등록하겠습니다 , 번호를 등록해 주세요"
# 	(반복) Server -(서비스)> Human_detect :  (reamain) 20초동안 사람감지 
# if "있음" :  Server -(토픽)> Voice_out  "마우스를 클리개해주세요!" + return
# else : "없음" : ALART (도둑)
# else :
# Server -(서비스)> Voice_out : "마우스를 이용해 쿠폰 등록 여부를 설정해주세요"
# (반복) Server -(서비스)> Human_detect :  (reamain) 20초동안 사람감지 
# if "있음" :  Server -(토픽)> Voice_out  "마우스를 클리개해주세요!" + return
# else : "없음" : ALART (도둑)
# ---------------------------------------------------------------------------------------------------------
# 8. UI -(토픽)> Server : next_to_payment (버튼클릭)
# 	(- Server -(토픽)> Robot_Control , Voice_out , Cup_detect , Container_detect : STOP!)
# Server -(토픽) > Guest_registration (사용자에 따라서)
# Server -(토픽)> UI : 결제 화면
# Server -(서비스)> Voice_out : "가위바위보를 해서 이기면 쿠폰 +2 , 비기면 +1 지면 +0 , 하시겠습니까?"
# Server -(서비스)> Voice_input : 음성인식 (응답 : 네 or 아니)
# if "네"
# Server -(토픽)> Hand_Input : stop
# Server -(토픽)> UI : 가위 바위 보 화면
# Server -(서비스)> Voice_out : "가위바위보!!"
#  Server -(서비스)> Hand_Input : 가위바위보 3초간 감지
# if "이김"
# Server -(토픽)> UI : 쿠폰+2 , "환호장면"
# Server -(서비스)> Voice_out : "가위바위보 못하시네요?"
# Server -(토픽)> UI : 원상태 복귀
# elif "비김"
# Server -(토픽)> UI : 쿠폰 +1 , "아쉬움"
# Server -(서비스)> Voice_out : "비기다니 아쉽네요"
# Server -(토픽)> UI : 원상태 복귀
# else "짐"
# Server -(토픽)> ui : ""우울"
# Server -(서비스)> Voice_out : "내가 지다니..."
# Server -(토픽)> UI : 원상태 복귀
# else:
# Server -(서비스)> Voice_out : "재미없어!!!"
# if "쿠폰 10개이상"
# Server -(서비스)> Voice_out : 쿠폰을 사용하시겠습니까?"
# Server -(서비스)> Voice_input : 음성인식 (응답 : 네 or 아니)
# if "네"
# Server -(토픽)> UI : 쿠폰사용
# Server -(서비스)> Voice_out : "쿠폰을 사용하셨습니다. 결제하시겠습니까?
# Server -(서비스)> Voice_input : 음성인식 (응답 : 네 or 아니)
# if "네"
# Server -(토픽)> UI : 결제 장면
# DB_manager : 가격 (아이스크림,토핑)
# Server -(서비스)> Voice_out : 결제를 완료해 주세요"
# else
# Server -(서비스)> Voice_out : 마우스를 클릭해주세요"
# (반복) Server -(서비스)> Human_detect :  (reamain) 20초동안 사람감지 
# if "있음" :  Server -(토픽)> Voice_out  "마우스를 클리개해주세요!" + return
# else : "없음" : ALART (도둑)
# elif "아니요"
# Server -(토픽)> UI : 쿠폰 사용X
# Server -(서비스)> Voice_out : "쿠폰을 사용하지 않으셨습니다. 결제하시겠습니까?
# Server -(서비스)> Voice_input : 음성인식 (응답 : 네 or 아니)
# if "네"
# Server -(토픽)> UI : 결제 장면
# DB_manager : 가격 (아이스크림,토핑)
# Server -(서비스)> Voice_out : 결제를 완료해 주세요"
# else
# Server -(서비스)> Voice_out : 마우스를 클릭해주세요"
# (반복) Server -(서비스)> Human_detect :  (reamain) 20초동안 사람감지 
# if "있음" :  Server -(토픽)> Voice_out  "마우스를 클리개해주세요!" + return
# else : "없음" : ALART (도둑)
# 8.1 UI -(토픽)> Server : next_to_payment2 (버튼클릭) / Hand_Input -(토픽)> Server : next_to_payment2 (손)
# 	(- Server -(토픽)> Robot_Control , Voice_out , Cup_detect , Container_detect : STOP!)
# Server -(서비스)> Voice_out : "결제하시겠습니까?
# Server -(서비스)> Voice_input : 음성인식 (응답 : 네 or 아니)
# if "네"
# Server -(토픽)> UI : 결제 장면
# DB_manager : 가격 (아이스크림,토핑)
# Server -(서비스)> Voice_out : 결제를 완료해 주세요"
# else
# Server -(서비스)> Voice_out : 마우스를 클릭해주세요"
# (반복) Server -(서비스)> Human_detect :  (reamain) 20초동안 사람감지 
# if "있음" :  Server -(토픽)> Voice_out  "마우스를 클리개해주세요!" + return
# else : "없음" : ALART (도둑)
# -------------------------------------------------------------------------------------------------
# DB_Manager : 손님정보 , 일 매출 , 재고량
# Server -(토픽)> UI : 마지막 장면
# Server -(토픽)> Robot_Control : 작별 인사
# Server -(서비스)> Voice_out : "안녕히 가십시오!"
# Server -(토픽)> Robot_Control : 중지
# Server -(서비스)> Voice_out : "마무리 작업을 하겠습니다. 쓰레기를 확인하겠습니다"
# Server -(서비스)> Cup_detect : "쓰레기 감지" (감지 , 좌표 / 노감지)
# 	if "감지" : Server -(서비스)> Voice_out : "쓰레기를 처리하겠습니다."
# 			Server -(서비스?)> Robot_Control : 쓰레기 처리 (x,y 좌표로 이동 , 그리퍼 180도 회전 , z축 내리기 , 집기 , 쓰레기통이동 , 버리기 , 원래상태 복귀) 

# 	else :
# 			Server -(서비스)> Voice_out : "쓰레기가 없습니다."
# DB_Manager : 재고량 확인
# if "있음"
# (back_to_home)
# else:
# Server -(토픽)> UI : 재고량 없는거 알림
# Server -(서비스)> Voice_out : "재고량이 부족합니다. 관리자에게 문의하십시오"
# --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# 10. UI -(토픽)> Server : ALART (도둑)
# Server -(토픽)> UI : 마지막 장면 (+ 도둑놈 얼굴 표시)
# Server -(서비스)> Voice_out : "도둑놈 잡아라!!!" + bgm "알림"
# 2번 반복 후 
# DB_Manager : 손님정보 , 일 매출 , 재고량 
# (back_to_home)


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
                self.send_ui(response)

                # 쓰레기 청소 : 서비스 : Robot_Server -> Robot_Control
                msg.command = f"Pre-production : {response.result}"
                print(msg.command)
                self.send_robot_control(msg.command) 

                msg.command = "Pre-production"
                # 쓰레기 청소 끝 및 아이스크림 대기 : 토픽 : Robot_Server -> UI
                self.ui_publisher.publish(msg)

            else :
                print("쓰레기 없음")
                self.ui_publisher.publish(msg)

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
                self.send_ui(response)
                # Concluson 

                # 쓰레기 청소 : 서비스 : Robot_Server -> Robot_Control
                msg.command = f"Conclusion : {response.result}"
                print(msg.command)
                self.send_robot_control(msg.command) 

                # 쓰레기 청소 끝 및 아이스크림 대기 : 토픽 : Robot_Server -> UI
                self.ui_publisher.publish(final)
            else :
                print("쓰레기 없음")
                self.ui_publisher.publish(final)

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