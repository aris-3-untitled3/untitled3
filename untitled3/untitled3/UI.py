import rclpy as rp
from rclpy.node import Node
from untitled_msgs.msg import TopicString
from untitled_msgs.srv import ServiceString
import time
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QStackedWidget
from PyQt5 import uic
from PyQt5.QtGui import QPixmap, QMovie
from PyQt5.QtCore import pyqtSignal, QThread
from PyQt5 import uic 
import pygame
import os
import threading
import numpy as np
from Voice_Input import Record_API
import datetime
from DB_manager import DB_Manager

# UI 파일 경로 설정
ui_file = os.path.join('/home/jchj/Untitled3/src/untitled3/UI/', "Title.ui")
ui_file2 = os.path.join('/home/jchj/Untitled3/src/untitled3/UI/', "Loading.ui")
ui_Direction = os.path.join('/home/jchj/Untitled3/src/untitled3/UI/', "Direction.ui")
ui_Recommend = os.path.join('/home/jchj/Untitled3/src/untitled3/UI/', "Recommend_kor.ui")
ui_Preparing = os.path.join('/home/jchj/Untitled3/src/untitled3/UI/', "Preparing.ui")
ui_Making = os.path.join('/home/jchj/Untitled3/src/untitled3/UI/', "Making.ui")
ui_Maked = os.path.join('/home/jchj/Untitled3/src/untitled3/UI/', "Maked.ui")
ui_Coupon = os.path.join('/home/jchj/Untitled3/src/untitled3/UI/', "Coupon.ui")
ui_Payment = os.path.join('/home/jchj/Untitled3/src/untitled3/UI/', "Payment.ui")
ui_Bye = os.path.join('/home/jchj/Untitled3/src/untitled3/UI/', "Bye.ui")

from_class = uic.loadUiType(ui_file)[0]
from_class2 = uic.loadUiType(ui_file2)[0]
from_class_Direction = uic.loadUiType(ui_Direction)[0]
from_class_Recommend = uic.loadUiType(ui_Recommend)[0]
from_class_Preparing = uic.loadUiType(ui_Preparing)[0]
from_class_Making = uic.loadUiType(ui_Making)[0]
from_class_Maked = uic.loadUiType(ui_Maked)[0]
from_class_Coupon = uic.loadUiType(ui_Coupon)[0]
from_class_Payment = uic.loadUiType(ui_Payment)[0]
from_class_Bye = uic.loadUiType(ui_Bye)[0]



class PyQt(Node):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    ui_update_signal = pyqtSignal(str)

    def __init__(self , first_window ,loading_window , recommend_window , preparing_window , making_window , maked_window , bye_window):
        super().__init__('PyQt')

        # Robot_Server에서 토픽 받기 ( 3초 정면 대기 / 메뉴얼 시작 / 제조완료 / 쓰레기 청소 끝 및 아이스크림 대기 / 작별문구 / 토핑 무게 전달 및 쓰레기 청소 완료)
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

        # Robot_Server로 토픽 퍼블리셔 (아이스크림 감지 / 제조준비 / 마무리 상황)
        self.robot_server_publisher = self.create_publisher(TopicString, '/UI_to_Server', 10)

        # 시작
        msg = TopicString()
        msg.command = "start"
        self.robot_server_publisher.publish(msg)

        self.first_window = first_window
        self.loading_window = loading_window
        self.recommend_window = recommend_window
        self.preparing_window = preparing_window
        self.making_window = making_window
        self.maked_window = maked_window
        self.bye_window = bye_window


    def Robot_Server_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.command}')

        if msg.command == "guest_detect":
            self.first_window.update_signal.emit(msg.command)


        elif "Age" and "Gender" in msg.command:
            self.loading_window.update_signal.emit(msg.command)


        elif msg.command == "Pre-production":
            self.preparing_window.update_signal.emit(msg.command)


        elif msg.command == "ice_cream_production_complete":
            self.making_window.update_signal.emit(msg.command)
            time.sleep(3)
            self.maked_window.update_signal.emit(msg.command)


        elif "Conclusion" in msg.command :
            
            if "ok" in msg.command :
                self.bye_window.update_signal.emit("restart")
            else :
                print("음성출력 - 재고량 없음")
                return

        else:
            return

    def Pre_production(self):
        self.get_logger().info('UI to Server , Pre-production!')

        msg = TopicString()
        msg.command = 'Pre-production'
        print(msg)
        self.robot_server_publisher.publish(msg)


    def Conclusion(self , stock):
        self.get_logger().info('UI to Server , Conclusion!')

        msg = TopicString()
        msg.command = 'Conclusion , %s'.format(stock)
        print(msg)
        self.robot_server_publisher.publish(msg)


    def ui_callback(self, request, response):
        self.get_logger().info(f'Received command: {request.command}')

        if request.command == "ice_cream_Bucket_detect":
            self.preparing_window.update_signal.emit(request.command)
        elif "Pre-production" in request.command:
            self.preparing_window.update_signal.emit(request.command)
        else:
            self.bye_window.update_signal.emit(request.command)

        response.success = True
        response.result = f'Command {request.command} received and being processed' 

        return response


# DB_test.py 함수 구현 부분
class DB_main:
    def __init__(self):
        self.mydb = DB_Manager()
        self.mydb.DB_connect()
        self.mydb.create_table()

        # 팀원과 임의로 정한 맛 추천 테이블
        self.None_flavor = {'0-6': {'F':'berry', 'M':'choco'},
                    '7-18': {'F':'berry', 'M':'choco'},
                    '19-29': {'F':'choco', 'M':'choco'},
                    '30-49': {'F': 'vanilla', 'M':'vanilla'},
                        '50-': {'F': 'vanilla', 'M':'vanilla'}
                        }

        self.None_topping = {'0-6': {'F':'topA', 'M':'topC'},
                    '7-18': {'F':'topB', 'M':'topA'},
                    '19-29': {'F':'topA', 'M':'topC'},
                    '30-49': {'F': 'topB', 'M':'topB'},
                        '50-': {'F': 'topB', 'M':'topA'}
                        }

    def load_age_gender(self):
        with open('latest_age_gender.txt','r') as f:
            info_txt = f.read()

        infos = info_txt.split('\t')  # 탭 문자로 분리

        if len(infos) >= 2:
            age = infos[0].strip()  # 첫 번째 요소를 age 변수에 저장하고 좌우 공백 제거
            gender = infos[1].strip()  # 두 번째 요소를 gender 변수에 저장하고 좌우 공백 제거
            print(f"{age} {gender} Load Success")
        else:
            print("Failed to load age and gender information from the file.")

        return age,gender

    def recommend_flavor_topping(self):
        age, gender = self.load_age_gender()
        most_flavor, most_topping = self.mydb.load_sales(age, gender)

        if most_flavor is None:
            most_flavor = self.None_flavor[age][gender]
            print(f"맛 추천 정보가 없어서 임의로 출력함 fl  avor = {self.None_flavor[age][gender]}")
        if most_topping is None:
            most_topping = self.None_topping[age][gender]
            print(f"토핑 추천 정보가 없어서 임의로 출력함 topping = {self.None_topping[age][gender]}")

        print (f"맛 추천 {most_flavor}, {most_topping}")

        return age, gender, most_flavor, most_topping

    def API_module(self, audio_file, text_file, response_time):
        # 추천 메뉴에 대한 응답 음성 인식
        if response_time == 3:
            print("추천드린 맛으로 선택하시겠습니까? (네/아니요)")
            time.sleep(0.5)
        if response_time == 5:
            print("메뉴 선택 후 말씀해주세요")
            print("예시) 딸기맛 하나랑 토핑 A로 부탁해")
            time.sleep(0.5)

        record_api = Record_API(audio_file, text_file, response_time)
        flag = record_api.run()

        if text_file:
            print(f"audio_file = {audio_file}. VoicetoText = {text_file} 성공적으로 저장됨")
        else:
            print("API_module --> text_file 생성되지 않음")
            audio_file, response_time = None

        return text_file, response_time, flag
    
    def word_detect(self, text_file, response_time, flag):
        flavor_list = ["딸기", 
                       "초코",
                       "바닐라"]
        
        topping_list = ["A", "에이",
                        "B", "비", "삐",
                        "C", "씨", "시"]

        # 응답 Y/N 판별하는 구문
        if response_time == 3:
            with open(text_file, 'r') as f:
                vtot = f.read()
            if '네' in vtot:
                YorN = 'Y'
            elif "아니요" in vtot or "아니오" in vtot or "아니" in vtot:
                YorN = 'N'
            else:
                print("다시 말씀해주세요")
                YorN = 'E' #error
            return YorN

        # Flavor, Topping 녹화-변환-저장 파일 불러오기(ex. 딸기맛 하나랑 토핑 A로 부탁해)
        if response_time == 5:
            if flag == 1:
                with open(text_file,'r') as f:
                    text_file = f.read()
                text_file = text_file.upper()
                flavor_topping = text_file.split(' ') # (ex. ['딸기맛', '하나랑', '토핑A로', '부탁해'])
                print(f"flavor_topping --> {flavor_topping}")
                flavor = [flavor for flavor in flavor_list 
                    if any(flavor in word for word in flavor_topping)]
            else:
                flavor = None
                
            print(f"인식된 flavor --> {flavor}")


            if flag == 1 and len(flavor) == 1:
                if flavor[0] == flavor_list[0]:
                    Flavor = "berry"
                if flavor[0] == flavor_list[1]:
                    Flavor = "choco"
                if flavor[0] == flavor_list[2]:
                    Flavor = "vanilla"
            elif flag == 1 and len(flavor) > 1:
                print("flavor -- 중복되었습니다.")
                Flavor = "error"
            else:
                print("flavor -- 다시 말씀해주세요")
                Flavor = "error"

            if flag == 1:
                topping = [topping for topping in topping_list
                        if any(topping in word for word in flavor_topping)]
            else:
                topping = None
            print(f"인식된 topping --> {topping}")


            if flag == 1 and len(topping) == 1:
                if topping[0] in [topping_list[0], topping_list[1]]:
                    Topping = "topA"
                if topping[0] in [topping_list[2], topping_list[3], topping_list[4]]:
                    Topping = "topB"
                if topping[0] in [topping_list[5], topping_list[6], topping_list[7]]:
                    Topping = "topC"

            elif flag == 1 and len(topping) > 1:
                print("topping -- 중복되었습니다")
                Topping = "error"
            else:
                print("topping -- 다시 말씀해주세요")
                Topping = "error"

            return Flavor, Topping
        
    def YorN_response(self, most_flavor, most_topping): #(self, response_time, most_flavor, most_topping):
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")

        print(f"{current_time} %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        YorN_audio = f"YorN_response{current_time}.wav"
        YorN_vtot = f"YorN_response{current_time}.txt"

        text_file, response_time, flag = self.API_module(YorN_audio, YorN_vtot, 3)
        YorN = self.word_detect(text_file, response_time, flag)

        cnt = 0 #다시 음성 요청할 때, 리밋하기 위한 카운터
        # 추천 메뉴 Y --> 그대로 return, N --> (1) 음성으로 메뉴 고르기 (2) 클릭으로 메뉴 고르기 값이 return
        if YorN == 'Y':
            Flavor = most_flavor
            Topping = most_topping

        # 만약 음성으로 인식받는다고 하면 아래 파일 실행, 아니면 버튼에서 눌린 데이터값 불러와서 저장
        elif YorN == 'N':
            flavor_audio = f"Flavor_Topping{current_time}.wav"
            flavor_vtot = f"Flavor_Topping{current_time}.txt"
            # error값이 나오면 cnt = 0, 1, 2로 증가하는데 3이 되기 전까지 API_module과 word_detect 반복하는?
            cnt = 0 #다시 음성 요청할 때, 리밋하기 위한 카운터
            while cnt < 2:
                text_file, response_time, flag = self.API_module(flavor_audio, flavor_vtot, 5)
                Flavor, Topping = self.word_detect(text_file, response_time, flag)

                if Flavor == "error" or Topping == "error":
                    Flavor = None
                    Topping = None
                    cnt += 1
                else:
                    break
        else:
            flavor_audio = f"Flavor_Topping{current_time}.wav"
            flavor_vtot = f"Flavor_Topping{current_time}.txt"
            while cnt < 2:
                text_file, response_time, flag = self.API_module(YorN_audio, YorN_vtot, 3)
                YorN = self.word_detect(text_file, response_time, flag)

                if YorN == 'E':
                    Flavor = None
                    Topping = None
                    cnt += 1
                else:
                    break

        print(f"YorN_response return {Flavor}, {Topping} _=====================================================")
        return Flavor, Topping

    def YorN_stamp(self, YorN, phone):
        if YorN == 'Y':
            print("continue coupon")

            Phone = phone
            coupon = self.mydb.load_customer_info(Phone) # 쿠폰 갯수 load

        else:
            Phone = ""
            coupon = None

        return str(Phone), coupon
    
    def YorN_use_coupon(self, coupon, use_coupon):
        if use_coupon == 'Y' and coupon >= 10:
                price = 0
                coupon -= 10
                print(f"{price}, 쿠폰 사용 {use_coupon} --> 선택한 use coupon 값")
        return coupon, price

    def show_price(self, Topping, topping_time):
        flavor_price, toppic_price = self.mydb.load_price(Topping)
        price = flavor_price + (toppic_price*topping_time)

        return price

    def update_infos(self, Flavor, Topping, price, Phone, age, gender, Use_coupon): 
        save_sale = self.mydb.save_sales(Flavor, Topping, price, Phone, age, gender, Use_coupon)

        if Phone:
            user_id, coupon = self.mydb.save_customer_info(Phone, age, gender, Use_coupon)
            print(f"고객정보 업데이트 완료 --> {user_id}")
            return user_id, coupon
        else:
            coupon = None
            print("쿠폰 적립하지 않는다고 해서 손님 저장하지 않음")
            print(f"saved sales ----> {save_sale}")
            return Phone, coupon
    
    def update_stock(self, Flavor, Topping):
        result, stock_dict, flavor_flag, topping_flag = self.mydb.save_stock(Flavor, Topping)

        if stock_dict and flavor_flag and topping_flag is not None:
            print("**********************************재고 리스트****************************************")
            print(f"*-{stock_dict}-*")
            print("************************************************************************************")

            if flavor_flag is not None:
                print(f"Out of stock {flavor_flag}")
            if topping_flag is not None:
                print(f"Out of stock {topping_flag}")

            print("성공적으로 마쳤다!!")
        else:
            print("Final ERROR------------------------------")

        return result, stock_dict, flavor_flag, topping_flag


# 전체 UI 관리   
class MainlWindow(QMainWindow):
    
    def __init__(self):
        super().__init__()
    
        self.stacked_widget = QStackedWidget(self)  # QStackedWidget 인스턴스 생성   
        self.setCentralWidget(self.stacked_widget)  # MainWindow의 중앙 위젯으로 설정 
    
        self.open_FirstWindow()

    def voice_input(self):
        file_name="/home/jchj/Untitled3/src/untitled3/resource/output.wav"
        save_path="/home/jchj/Untitled3/src/untitled3/resource/response.txt"
        VtoT = Record_API(file_name, save_path, respone_time=10)
        text = VtoT.run()
        print(text)

    def open_FirstWindow(self):
        self.FirstWindow = FirstWindow(self)
        self.stacked_widget.addWidget(self.FirstWindow)  # MainWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.FirstWindow)  # MainWindow 페이지를 보여줌

    def open_LoadingWindow(self):
        self.LoadingWindow = LoadingWindow(self)
        self.stacked_widget.addWidget(self.LoadingWindow)  # LoadingWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.LoadingWindow)  # LoadingWindow 페이지를 보여줌

    def open_DirectionWindow(self):
        self.DirectionWindow = DirectionWindow(self)
        self.stacked_widget.addWidget(self.DirectionWindow)  # DirectionWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.DirectionWindow)  # DirectionWindow 페이지를 보여줌

    def open_RecommendWindow(self):
        self.RecommendWindow = RecommendWindow(self)
        self.stacked_widget.addWidget(self.RecommendWindow)  # RecommendWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.RecommendWindow)  # RecommendWindow 페이지를 보여줌

    def open_PreparingWindow(self):
        self.PreparingWindow = PreparingWindow(self)
        self.stacked_widget.addWidget(self.PreparingWindow)  # PreparingWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.PreparingWindow)  # PreparingWindow 페이지를 보여줌

    def open_MakingWindow(self):
        self.MakingWindow = MakingWindow(self)
        self.stacked_widget.addWidget(self.MakingWindow)  # MakingWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.MakingWindow)  # MakingWindow 페이지를 보여줌

    def open_MakedWindow(self):
        self.MakedWindow = MakedWindow(self)
        self.stacked_widget.addWidget(self.MakedWindow)  # MakedWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.MakedWindow)  # MakedWindow 페이지를 보여줌

    def open_CouponWindow(self):
        self.CouponWindow = CouponWindow(self)
        self.stacked_widget.addWidget(self.CouponWindow)  # CouponWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.CouponWindow)  # CouponWindow 페이지를 보여줌
    
    def open_PaymentWindow(self,coupon,number):
        self.PaymentWindow = PaymentWindow(self,coupon,number)
        self.stacked_widget.addWidget(self.PaymentWindow)  # PaymentWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.PaymentWindow)  # PaymentWindow 페이지를 보여줌

    def open_ByeWindow(self):
        self.ByeWindow = ByeWindow(self)
        self.stacked_widget.addWidget(self.ByeWindow)  # ByeWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.ByeWindow)  # ByeWindow 페이지를 보여줌


class FirstWindow(QMainWindow, from_class):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_signal = pyqtSignal(str)
    
    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)
    
        self.load_image()

        self.main_window = main_window
    
        self.pushButton.clicked.connect(self.next)
        self.pushButton.clicked.connect(self.on_click)
    
        self.update_signal.connect(self.next)

    def load_image(self):
        pixmap = QPixmap("/home/jchj/Untitled3/src/untitled3/UI/Title.jpg")
        self.label_2.setPixmap(pixmap)
        self.label_2.setScaledContents(True)

    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        pygame.mixer.init()
        pygame.mixer.music.load("/home/jchj/Untitled3/src/untitled3/UI/cat_like1b.mp3")
        pygame.mixer.music.play()

    def next(self):

        print("음성출력 - 안녕하세요")
        self.main_window.open_LoadingWindow()


class LoadingWindow(QMainWindow, from_class2):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_signal = pyqtSignal(str)

    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window

        # QLabel에 GIF 설정
        self.movie = QMovie("/home/jchj/Untitled3/src/untitled3/UI/loading.gif")
        if self.movie.isValid():
            self.label_pic.setMovie(self.movie)
            self.movie.start()
        else:
            print("GIF를 로드할 수 없습니다.")

        print("음성출력 - 3초간 대기해 주세요")

        self.update_signal.connect(self.next)


    def next(self):
        self.main_window.open_DirectionWindow()

class DirectionWindow(QMainWindow, from_class_Direction):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_signal = pyqtSignal(str)

    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)
        
        self.main_window = main_window

        self.pushButton.clicked.connect(self.next)
        self.pushButton_back.clicked.connect(self.open_loading_window)
        self.pushButton_home.clicked.connect(self.open_Title_window)
        self.pushButton.clicked.connect(self.on_click)
        self.pushButton_back.clicked.connect(self.on_click)
        self.pushButton_home.clicked.connect(self.on_click)

        print("음성출력 - 인사")
        print("음성출력 - 사용법 설명")

    def open_Title_window(self):
        self.main_window.open_FirstWindow()

    def open_loading_window(self):
        self.main_window.open_LoadingWindow()

    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        pygame.mixer.init()
        pygame.mixer.music.load("/home/jchj/Untitled3/src/untitled3/UI/cat_like1b.mp3")
        pygame.mixer.music.play()

    def next(self):
        self.main_window.open_RecommendWindow()

class RecommendWindow(QMainWindow, from_class_Recommend):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_signal = pyqtSignal(str)

    def __init__(self , main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window
        
        self.pushButton_back.clicked.connect(self.open_Direction_window)
        self.pushButton_home.clicked.connect(self.open_Title_window)
        self.pushButton_back.clicked.connect(self.on_click)
        self.pushButton_home.clicked.connect(self.on_click)

        self.setStyleSheet("""
            QPushButton {
                border: 2px solid #1E90FF;  /* 테두리 색상: DodgerBlue */
                border-radius: 10px;        /* 둥근 테두리 */
                padding: 5px;               /* 버튼 내부 여백 */
                background-color: #F0F8FF;  /* 배경 색상: AliceBlue */
            }

            QPushButton:hover {
                border: 2px solid #00BFFF;  /* 마우스 올렸을 때 테두리 색상: DeepSkyBlue */
                background-color: #E6F2FF;  /* 마우스 올렸을 때 배경 색상 */
            }

            /* pushButton_1에 대한 스타일 */
            #pushButton_1 {
                border: 6px solid;
                border-image: linear-gradient(45deg, #FFFF00, #00FFFF) 1;  /* 그라데이션 테두리 */
                border-radius: 10px;        /* 둥근 테두리 */
                padding: 5px;               /* 버튼 내부 여백 */
                background-color: #F0F8FF;  /* 배경 색상: AliceBlue */
                color: #000000;             /* 텍스트 색상: 검정색 */
                font-weight: bold;          /* 텍스트 두껍게 */
            }

            #pushButton_1:hover {
                border: 2px solid;
                border-image: linear-gradient(45deg, #00FFFF, #FFFF00) 1;  /* 마우스 올렸을 때 그라데이션 변경 */
                background-color: #E6F2FF;  /* 마우스 올렸을 때 배경 색상 */
                color: #000000;             /* 텍스트 색상: 검정색 */
                font-weight: bold;          /* 텍스트 두껍게 */
            }

            /* pushButton_4에 대한 스타일 */
            #pushButton_4 {
                border: 6px solid;
                border-image: linear-gradient(45deg, #FFFF00, #00FFFF) 1;  /* 그라데이션 테두리 */
                border-radius: 10px;        /* 둥근 테두리 */
                padding: 5px;               /* 버튼 내부 여백 */
                background-color: #F0F8FF;  /* 배경 색상: AliceBlue */
                color: #000000;             /* 텍스트 색상: 검정색 */
                font-weight: bold;          /* 텍스트 두껍게 */
            }

            #pushButton_4:hover {
                border: 2px solid;
                border-image: linear-gradient(45deg, #00FFFF, #FFFF00) 1;  /* 마우스 올렸을 때 그라데이션 변경 */
                background-color: #E6F2FF;  /* 마우스 올렸을 때 배경 색상 */
                color: #000000;             /* 텍스트 색상: 검정색 */
                font-weight: bold;          /* 텍스트 두껍게 */
            }
        """)

        # 소리
        #uic.loadUi('your_ui_file.ui', self)

        # 맛 추천 표시 두 가지 방법 중 선택하기
        recommended_flavor = "초코"
        recommended_topping = "토핑C"
        self.label_2.setText(f"손님께 추천드리는 맛과 토핑은 {recommended_flavor}와 {recommended_topping}입니다.")

        # QPushButton을 토글 버튼으로 만들기
        self.pushButton_1.setCheckable(True)
        self.pushButton_2.setCheckable(True)
        self.pushButton_3.setCheckable(True)
        self.pushButton_4.setCheckable(True)
        self.pushButton_5.setCheckable(True)
        self.pushButton_6.setCheckable(True)

        # 각 버튼에 대해 토글 이벤트 연결
        self.pushButton_1.clicked.connect(self.toggle_button)
        self.pushButton_2.clicked.connect(self.toggle_button)
        self.pushButton_3.clicked.connect(self.toggle_button)
        self.pushButton_4.clicked.connect(self.toggle_button)
        self.pushButton_5.clicked.connect(self.toggle_button)
        self.pushButton_6.clicked.connect(self.toggle_button)

        # 소리
        self.pushButton_1.clicked.connect(self.on_click)
        self.pushButton_2.clicked.connect(self.on_click)
        self.pushButton_3.clicked.connect(self.on_click)
        self.pushButton_4.clicked.connect(self.on_click)
        self.pushButton_5.clicked.connect(self.on_click)
        self.pushButton_6.clicked.connect(self.on_click)

        # Next 버튼 이벤트 연결
        self.pushButton.clicked.connect(self.check_selection_and_next)

        print("음성출력 - 메뉴선택")

        # 손님 성별,연령대에 따른 맛 선호도 가져오기: 서비스 : UI -> DB_Manager
        #   테투리 - 선호도 (성별,연령대) 
        #   음성 출력 - "추천 메뉴를 선택하시겠습니까?"
        #   음성 인식: 서비스 : UI -> Voice_Input (네 or 아니요)
        # self.main_window.voice_input()
        #       네 - 음성출력 및 다음화면 "선택한 메뉴를 제조하겠습니다"
        #       아니요 - 음성출력 : "메뉴를 선택해 주세요"
        #           음성 인식 : 서비스 : UI -> Voice_Input (아이스크림 맛, 토핑맛)
        # self.main_window.voice_input()
        #               맛 선택 : "선택한 맛으로 제조하겠습니다."
        #               nop : 음성출력 - "메뉴가 부정확합니다 . 마우스로 클릭해 주세요"

        self.update_signal.connect(self.next)

        print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@잘 넘어옴")
        # 맛 추천 표시 두 가지 방법 중 선택하기
        age, gender, recommended_flavor, recommended_topping = DB_main().recommend_flavor_topping()
        print(f"{recommended_flavor}, {recommended_topping}")
        self.label_2.setText(f"손님께 추천드리는 맛과 토핑은 {recommended_flavor}와 {recommended_topping}입니다.")

        self.Flavor, self.Topping = DB_main().YorN_response(recommended_flavor, recommended_topping)

        # print(f"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@{self.Flavor} {self.Topping}")

        # # 인식 못했으면 클릭 활성화 --> 손님 직접 메뉴 선택
        # if self.Flavor is None and self.Topping is None:
        #     # QPushButton을 토글 버튼으로 만들기
        #     self.pushButton_1.setCheckable(True) # choco
        #     self.pushButton_2.setCheckable(True) # vanilla
        #     self.pushButton_3.setCheckable(True) # berry
        #     self.pushButton_4.setCheckable(True) # topC
        #     self.pushButton_5.setCheckable(True) # topB
        #     self.pushButton_6.setCheckable(True) # topA

        #     # 소리
        #     self.pushButton_1.clicked.connect(self.on_click)
        #     self.pushButton_2.clicked.connect(self.on_click)
        #     self.pushButton_3.clicked.connect(self.on_click)
        #     self.pushButton_4.clicked.connect(self.on_click)
        #     self.pushButton_5.clicked.connect(self.on_click)
        #     self.pushButton_6.clicked.connect(self.on_click)
        #     # Next 버튼 이벤트 연결
        #     self.pushButton.clicked.connect(self.check_selection_and_next)
        # else:
        #     Flavor = self.Flavor
        #     Topping = self.Topping
        #     print(f"check_selection_and_next ==========> global4 {Flavor}, {Topping}")
        #     print(f"check_selection_and_next ==========> global5 self.{Flavor}, self.{Topping}")
        #     QTimer.singleShot(30,  self.open_Preparing_window)
    
    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        pygame.mixer.init()
        pygame.mixer.music.load("/home/jchj/Untitled3/src/untitled3/UI/cat_like1b.mp3")
        pygame.mixer.music.play()

    def on_click_warn(self):
        threading.Thread(target=self.play_mp3_warn).start()

    def play_mp3_warn(self):
        pygame.mixer.init()
        pygame.mixer.music.load("/home/jchj/Untitled3/src/untitled3/UI/cat_like1b.mp3")
        pygame.mixer.music.play()
        
    def open_Title_window(self):
        self.main_window.open_FirstWindow()

    def open_Direction_window(self):
        self.main_window.open_DirectionWindow()

    def toggle_button(self):
        button = self.sender()
        if button.isChecked():
            if button == self.pushButton_1:
                button.setStyleSheet('background-color: brown')  # 초코 색
            elif button == self.pushButton_2:
                button.setStyleSheet('background-color: beige')  # 바닐라 색
            elif button == self.pushButton_3:
                button.setStyleSheet('background-color: red')  # 딸기 색
            else:
                button.setStyleSheet('background-color: lightgray')  # 토핑 색상
        else:
            button.setStyleSheet('')  # 초기 상태로 되돌림

    # def toggle_button(self):
    #     global Flavor, Topping
    #     button = self.sender()
    #     if button.isChecked():
    #         if button == self.pushButton_1:
    #             button.setStyleSheet('background-color: brown')  # 초코 색
    #             Flavor = "choco"
    #         elif button == self.pushButton_2:
    #             button.setStyleSheet('background-color: beige')  # 바닐라 색
    #             Flavor = "vanilla"
    #         elif button == self.pushButton_3:
    #             button.setStyleSheet('background-color: red')  # 딸기 색
    #             Flavor = "berry"
    #         elif button == self.pushButton_4:
    #             button.setStyleSheet('background-color: lightgray')  # 토핑 색상
    #             Topping = "topC"
    #         elif button == self.pushButton_5:
    #             button.setStyleSheet('background-color: lightgray')  # 토핑 색상
    #             Topping = "topB"
    #         elif button == self.pushButton_6:
    #             button.setStyleSheet('background-color: lightgray')  # 토핑 색상
    #             Topping = "topA"
    #     else:
    #         button.setStyleSheet('')  # 초기 상태로 되돌림

    def check_selection_and_next(self):
        # 아이스크림 맛이 하나가 선택되었는지 확인
        ice_cream_selected = (
            (self.pushButton_1.isChecked() and not self.pushButton_2.isChecked() and not self.pushButton_3.isChecked()) or
            (not self.pushButton_1.isChecked() and self.pushButton_2.isChecked() and not self.pushButton_3.isChecked()) or
            (not self.pushButton_1.isChecked() and not self.pushButton_2.isChecked() and self.pushButton_3.isChecked())
        )
        topping_selected = (
            (self.pushButton_4.isChecked() and not self.pushButton_5.isChecked() and not self.pushButton_6.isChecked()) or
            (not self.pushButton_4.isChecked() and self.pushButton_5.isChecked() and not self.pushButton_6.isChecked()) or
            (not self.pushButton_4.isChecked() and not self.pushButton_5.isChecked() and self.pushButton_6.isChecked())
        )

        if not ice_cream_selected:
            self.on_click_warn()
            QMessageBox.warning(self, 'Warning', '아이스크림 맛을 한 가지만 골라주세요.')

        if not topping_selected:
            self.on_click_warn()
            QMessageBox.warning(self, 'Warning', '토핑을 한 가지만 골라주세요.')

        if ice_cream_selected and topping_selected:
            self.next()

    # def check_selection_and_next(self):
    #     global Flavor, Topping
    #     print(f"check_selection_and_next ==========> global1 {Flavor}, {Topping}")
    #     # 아이스크림 맛이 하나가 선택되었는지 확인
    #     ice_cream_selected = (
    #         (self.pushButton_1.isChecked() and not self.pushButton_2.isChecked() and not self.pushButton_3.isChecked()) or
    #         (not self.pushButton_1.isChecked() and self.pushButton_2.isChecked() and not self.pushButton_3.isChecked()) or
    #         (not self.pushButton_1.isChecked() and not self.pushButton_2.isChecked() and self.pushButton_3.isChecked())
    #     )
    #     topping_selected = (
    #         (self.pushButton_4.isChecked() and not self.pushButton_5.isChecked() and not self.pushButton_6.isChecked()) or
    #         (not self.pushButton_4.isChecked() and self.pushButton_5.isChecked() and not self.pushButton_6.isChecked()) or
    #         (not self.pushButton_4.isChecked() and not self.pushButton_5.isChecked() and self.pushButton_6.isChecked())
    #     )

    #     if not ice_cream_selected:
    #         self.on_click_warn()
    #         QMessageBox.warning(self, 'Warning', '아이스크림 맛을 한 가지만 골라주세요.')

    #     if not topping_selected:
    #         self.on_click_warn()
    #         QMessageBox.warning(self, 'Warning', '토핑을 한 가지만 골라주세요.')

    #     if ice_cream_selected and topping_selected:
    #         if self.pushButton_1.isChecked():
    #             self.Flavor = "choco"
    #         elif self.pushButton_2.isChecked():
    #             self.Flavor = "vanilla"
    #         else:
    #             self.Flavor = "berry"

    #         if self.pushButton_4.isChecked():
    #             self.Topping = "topC"
    #         elif self.pushButton_5.isChecked():
    #             self.Topping = "topB"
    #         else:
    #             self.Topping = "topA"

    #         Flavor = self.Flavor
    #         Topping = self.Topping

    #         print(f"check_selection_and_next ==========> self.{self.Flavor}, {self.Topping}")
    #         print(f"check_selection_and_next ==========> global2 {Flavor}, {Topping}")
    #         self.open_Preparing_window()

    def play_sound(self):
        if self.mixer_initialized:
            try:
                pygame.mixer.music.load("sound.mp3")  # 소리 파일 로드
                pygame.mixer.music.play()  # 소리 재생
            except pygame.error as e:
                print(f"Error playing sound: {e}")
        else:
            print("Mixer not initialized. Cannot play sound.")

    def next(self):
        self.main_window.open_PreparingWindow()

class PreparingWindow(QMainWindow, from_class_Preparing):

    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_signal = pyqtSignal(str)

    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window
        
        self.pushButton_back.clicked.connect(self.returning)
        self.pushButton_next.clicked.connect(self.next)
        self.pushButton_back.clicked.connect(self.on_click)
        self.pushButton_next.clicked.connect(self.on_click)

        self.update_signal.connect(self.command)

        print("제조 준비")

    def plus_init(self, node):

        self.node = node
        self.node.Pre_production()

    def command(self, command):
        if "," in command:
            # 쓰레기 처리중 화면 강조
            print("음성출력 - 쓰레기를 처리 중입니다")
        elif command == "Pre-production":
            # 쓰레기 없음 - 화면강조
            print("음성출력 - 쓰레기가 없습니다")
        else:
            # 아이스크림 테투리 강조
            print("음성출력 - 아이스크림을 제조 하겠습니다.")
            self.next()


    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        pygame.mixer.init()
        pygame.mixer.music.load("/home/jchj/Untitled3/src/untitled3/UI/cat_like1b.mp3")
        pygame.mixer.music.play()

    def returning(self):
        self.main_window.open_RecommendWindow()

    def next(self):
        self.main_window.open_MakingWindow()

class MakingWindow(QMainWindow, from_class_Making):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_signal = pyqtSignal(str)

    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window

        print("음성출력 - 제조 중")

        # QLabel에 GIF 설정
        self.movie = QMovie("/home/jchj/Untitled3/src/untitled3/UI/loading.gif")
        if self.movie.isValid():
            self.label_pic.setMovie(self.movie)
            self.movie.start()
        else:
            print("GIF를 로드할 수 없습니다.")

        self.update_signal.connect(self.next)
        
    def next(self):
        self.main_window.open_MakedWindow()

class MakedWindow(QMainWindow, from_class_Maked):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_signal = pyqtSignal(str)

    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window

        print("음성출력 - 제조를 완료하였습니다.")

        self.update_signal.connect(self.next)

    def next(self):
        self.main_window.open_CouponWindow()

class CouponWindow(QMainWindow, from_class_Coupon):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_signal = pyqtSignal(str)

    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window

        self.coupon = 1
        self.number = ""

        # 번호 누르기
        self.pushButton0.clicked.connect(lambda: self.add_text("0"))
        self.pushButton1.clicked.connect(lambda: self.add_text("1"))
        self.pushButton2.clicked.connect(lambda: self.add_text("2"))
        self.pushButton3.clicked.connect(lambda: self.add_text("3"))
        self.pushButton4.clicked.connect(lambda: self.add_text("4"))
        self.pushButton5.clicked.connect(lambda: self.add_text("5"))
        self.pushButton6.clicked.connect(lambda: self.add_text("6"))
        self.pushButton7.clicked.connect(lambda: self.add_text("7"))
        self.pushButton8.clicked.connect(lambda: self.add_text("8"))
        self.pushButton9.clicked.connect(lambda: self.add_text("9"))
        # 번호 지우기
        self.pushButton_Back.clicked.connect(self.remove_last_char)
        # 쿠폰 적립 버튼
        self.pushButton.clicked.connect(self.open_Payment_window)
        # 쿠폰 적립 안함 버튼
        self.pushButton_2.clicked.connect(self.open_Payment_window2)

        ## 소리 넣으려고 복사
        # 번호 누르기
        self.pushButton0.clicked.connect(self.on_click)
        self.pushButton1.clicked.connect(self.on_click)
        self.pushButton2.clicked.connect(self.on_click)
        self.pushButton3.clicked.connect(self.on_click)
        self.pushButton4.clicked.connect(self.on_click)
        self.pushButton5.clicked.connect(self.on_click)
        self.pushButton6.clicked.connect(self.on_click)
        self.pushButton7.clicked.connect(self.on_click)
        self.pushButton8.clicked.connect(self.on_click)
        self.pushButton9.clicked.connect(self.on_click)
        self.pushButton_Back.clicked.connect(self.on_click)
        self.pushButton.clicked.connect(self.on_click)
        self.pushButton_2.clicked.connect(self.on_click)

        print("음성출력 - 결제 및 마무리")

    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        pygame.mixer.init()
        pygame.mixer.music.load("/home/jchj/Untitled3/src/untitled3/UI/cat_like1b.mp3")
        pygame.mixer.music.play()

    def add_text(self, text):
        # 입력된 번호 갱신
        self.number += text

        # 현재 텍스트 가져오기
        current_text = self.number

        # 번호를 각각의 textEdit에 배치
        if len(current_text) <= 4:
            self.textEdit_3.setText(current_text)
            self.textEdit_4.setText("")
        else:
            self.textEdit_3.setText(current_text[:4])
            self.textEdit_4.setText(current_text[4:])

    def remove_last_char(self):
        # 입력된 번호에서 마지막 문자 제거
        self.number = self.number[:-1]

        # 현재 텍스트 가져오기
        current_text = self.number

        # 번호를 각각의 textEdit에 배치
        if len(current_text) <= 4:
            self.textEdit_3.setText(current_text)
            self.textEdit_4.setText("")
        else:
            self.textEdit_3.setText(current_text[:4])
            self.textEdit_4.setText(current_text[4:])

    def open_Payment_window(self):
        self.coupon = 1
        self.main_window.open_PaymentWindow(self.coupon,self.number)

    def open_Payment_window2(self):
        self.coupon = 0
        self.main_window.open_PaymentWindow(self.coupon,self.number)

    def open_Maked_window(self):
        self.main_window.MakedWindow()

class PaymentWindow(QMainWindow, from_class_Payment):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_signal = pyqtSignal(str)

    def __init__(self, main_window , coupon_value, number):
        super().__init__()

        global Flavor, Topping, age, gender

        self.setupUi(self)

        self.main_window = main_window

        self.coupon_value = coupon_value
        self.number = number

        # 임의의 금액
        self.price = 3500
        self.priceText()

        # 임의의 쿠폰 수 적용 및 텍스트 출력
        self.coupons = 13
        if self.coupon_value != 0:
            self.set_coupon_text()
        
        # 사용 버튼 시
        self.pushButton_6.clicked.connect(self.use_coupon)
        # 결제 버튼 시
        self.pushButton_8.clicked.connect(self.next)
        # Back 버튼 추가
        self.pushButton_back.clicked.connect(self.returning)
        
        ## 소리!
        # 사용 버튼 시
        self.pushButton_6.clicked.connect(self.on_click)
        # 결제 버튼 시
        self.pushButton_8.clicked.connect(self.on_click)
        # Back 버튼 추가
        self.pushButton_back.clicked.connect(self.on_click)

        print("음성출력 - 결제를 하겠습니다.")

        # 손님정보 : 성별,연령대 저장 및 번호에 따른 쿠폰 갯수 가져오기 
        # 쿠폰 개수 : UI -> DB_Manager
        # 맛 가격 :  UI -> DB_Manager
        # 일 매출 : UI -> DB_Manager

        # self.Use_coupon = 'N' # default 값
        
        # if self.coupon_value == 1:  # stamp 적립 Y
        #     YorN = 'Y'
        # if self.coupon_value == 0:
        #     YorN = 'N'

        # if self.number is not None and self.number != "":
        #     phone = int(self.number)
        #     self.number, self.coupon = DB_main().YorN_stamp(YorN, phone)
        #     print(f"고객 정보 --> {self.number}, {self.coupon}")
        # else:
        #     phone = None



        # topping_time = 5 # topping time은 토크 혹은 나오는 실제 시간으로 판단할 예정

        # # 임의의 금액
        # self.price = DB_main().show_price(Topping, topping_time)
        # self.priceText(self.price)

        # # 임의의 쿠폰 수 적용 및 텍스트 출력
        # if self.coupon_value != 0:
        #     self.set_coupon_text(self.coupon)
        
        # # 사용 버튼 시
        # self.pushButton_6.clicked.connect(self.use_coupon)   
        # # 결제 버튼 시
        # self.pushButton_8.clicked.connect(self.payment)
        # #self.pushButton_8.clicked.connect(self.no_use_coupon)
        # # Back 버튼 추가
        # self.pushButton_back.clicked.connect(self.open_Coupon_window)
        
        # ## 소리!
        # # 사용 버튼 시
        # self.pushButton_6.clicked.connect(self.on_click)
        # # 결제 버튼 시
        # self.pushButton_8.clicked.connect(self.on_click)
        # # Back 버튼 추가
        # self.pushButton_back.clicked.connect(self.on_click)
        

    # def priceText(self, price):
    #     text = str(price) + ' 원'
    #     self.textBrowser_2.setText(text)
        
    # def set_coupon_text(self, coupon):
    #     message = f"회원님의 쿠폰 수는 {coupon}개입니다."
    #     self.textBrowser.setText(message)
    #     message2 = f"회원번호: 010-" + self.number[:4] + '-' + self.number[4:]
    #     self.textBrowser_3.setText(message2)

    # def use_coupon(self):
    #     if self.number is not None and self.number != "":
    #         if self.coupon >= 10 and self.coupon_value == 1: #여기 coupons --> coupon
    #             self.Use_coupon = 'Y' # 사용하면 Y로 바꿈
    #             coupon = self.coupon
    #             coupon -= 10
    #             price = self.price
    #             price = 0
    #             phone = self.number
    #             self.priceText(price)
    #             self.set_coupon_text(coupon)
    #             message2 = f"남은 쿠폰 수는 {coupon}개 입니다."
    #             if self.Use_coupon == 'Y':
    #                 update_info, coupon = DB_main().update_infos(Flavor, Topping, price, int(phone), age, gender, self.Use_coupon)
    #                 print(f"update_info | coupon {update_info}, {coupon}")
    #             QMessageBox.warning(self, '쿠폰 사용 완료', message2)
    #         else:
    #             message2 = f"쿠폰이 10개 미만일 경우에는 사용이 불가합니다."
    #             QMessageBox.warning(self, '쿠폰 사용 불가', message2)
    #         #self.open_Bye_window()

    def payment(self):
        if self.Use_coupon == 'N' and self.coupon_value == 1 and self.number is not None and self.number != "":
            phone = self.number
            update_info, coupon = DB_main().update_infos(Flavor, Topping, self.price, int(phone), age, gender, self.Use_coupon)
            print(f"적립은 하지만 쿠폰 사용 x update_info | coupon {update_info}, {coupon}")
            self.open_Bye_window()
        
        elif self.Use_coupon == 'Y':
            print("쿠폰 사용한 고객은 이미 업데이트 했으니 그냥 바로 넘어감")
            self.open_Bye_window()

        elif self.coupon_value == 0:
            phone = None
            coupon = None
            Use_coupon = 'N'
            update_info, coupon = DB_main().update_infos(Flavor, Topping, self.price, phone, age, gender, Use_coupon)
            print(f"적립 안함 update_info | coupon {update_info}, {coupon}")
            self.open_Bye_window()
        else:
            print("payment Error--------------")   

    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        pygame.mixer.init()
        pygame.mixer.music.load("/home/jchj/Untitled3/src/untitled3/UI/cat_like1b.mp3")
        pygame.mixer.music.play()

    def returning(self):
        self.main_window.open_CouponWindow()

    def next(self):
        self.main_window.open_ByeWindow()


class ByeWindow(QMainWindow, from_class_Bye):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_signal = pyqtSignal(str)
    
    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window

        print("음성출력 - 수고하세요")

        # 토핑 재고량 전달 : 서비스 : UI -> DB_Manager

        self.stock = "ok"

        self.update_signal.connect(self.next)

    #     self.pushButton.clicked.connect(self.showValues)
    #     self.show()

    # def showValues(self):
    #     global Flavor, Topping
    #     print(f"show Values ==> {Flavor}, {Topping}")
    #     result, stock, flavor_flag, topping_flag = DB_main().update_stock(Flavor, Topping)

    #     print(f"Final list ====================== {result}")

    #     self.tableWidget.setItem(0,0,QTableWidgetItem(str(result[0]))) #vanilla
    #     self.tableWidget.setItem(0,1,QTableWidgetItem(str(result[1]))) # choco
    #     self.tableWidget.setItem(0,2,QTableWidgetItem(str(result[2]))) # berry
    #     self.tableWidget.setItem(0,3,QTableWidgetItem(str(result[3]))) # topA
    #     self.tableWidget.setItem(0,4,QTableWidgetItem(str(result[4]))) # topB
    #     self.tableWidget.setItem(0,5,QTableWidgetItem(str(result[5]))) # topC
    #     self.tableWidget.setEditTriggers(QAbstractItemView.NoEditTriggers)

    def plus_init(self, node):
        self.node = node
        self.node.Conclusion(self.stock)

    def command(self,command):
        # topic , Conclusion , ok
        if "restart" in command :
            print("음성출력 - 쓰레기 청소 끝")
            print("음성출력 - 대기상태 복귀")
            self.next()
        # service , Colclusion , 12,13
        elif "," in command:
            print("음성출력 - 쓰레기를 처리 중입니다")
        else:
            print("음성출력 - 쓰레기가 없습니다.")

    def next(self):
        self.main_window.open_FirstWindow()
        
class Ros2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rp.spin(self.node)

def main(args=None):
    rp.init(args=args)

    app = QApplication(sys.argv)

    main_window = MainlWindow()
    first_window =  FirstWindow(main_window)
    loading_window = LoadingWindow(main_window)
    recommend_window = RecommendWindow(main_window)
    preparing_window = PreparingWindow(main_window)
    making_window = MakingWindow(main_window)
    maked_window = MakedWindow(main_window)
    bye_window = ByeWindow(main_window)

    Node = PyQt(first_window , loading_window , recommend_window , preparing_window , making_window , maked_window , bye_window)

    preparing_window.plus_init(Node)
    bye_window.plus_init(Node)

    main_window.showMaximized()

    ros2_thread = Ros2Thread(Node)
    ros2_thread.start()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()