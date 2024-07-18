import rclpy as rp
from rclpy.node import Node
from untitled_msgs.msg import TopicString
from untitled_msgs.srv import ServiceString
from untitled_msgs.action import ActionString
import time
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QDialog, QMessageBox, QStackedWidget, QWidget
from PyQt5 import uic
from PyQt5.QtGui import QPixmap, QMovie
from PyQt5.QtCore import QTimer , pyqtSignal, QObject, QThread
from PyQt5 import QtWidgets, uic 
from pydub import AudioSegment
from pydub.playback import play
from playsound import playsound
import pygame
import os
import threading
import numpy as np

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

    def __init__(self , main_window , loading_window , recommend_window , preparing_window , making_window , maked_window , bye_window):
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

        self.main_window = main_window
        self.loading_window = loading_window
        self.recommend_window = recommend_window
        self.preparing_window = preparing_window
        self.making_window = making_window
        self.maked_window = maked_window
        self.bye_window = bye_window


    def Robot_Server_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.command}')

        if msg.command == "guest_detect":
            self.main_window.update_signal.emit(msg.command)


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
    
class MainWindow(QMainWindow, from_class):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_signal = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.setupUi(self)
    
        self.load_image()
    
        self.stacked_widget = QStackedWidget(self)  # QStackedWidget 인스턴스 생성   
        # self.setCentralWidget(self.stacked_widget)  # MainWindow의 중앙 위젯으로 설정 
    
        # self.stacked_widget.addWidget(self)  # 메인 위젯을 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self)  # 메인 위젯을 보여줌
    
        self.pushButton.clicked.connect(self.open_LoadingWindow)
        self.pushButton.clicked.connect(self.on_click)
    
        self.update_signal.connect(self.open_LoadingWindow)

    def load_image(self):
        pixmap = QPixmap("/home/jchj/Untitled3/src/untitled3/UI/Title.jpg")
        self.label_2.setPixmap(pixmap)
        self.label_2.setScaledContents(True)

    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        song = AudioSegment.from_mp3("/home/messi/Downloads/mp3/cat_like1b.mp3")
        play(song)

    def open_MainWindow(self):
        self.MainWindow = MainWindow()
        self.stacked_widget.addWidget(self.MainWindow)  # MainWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.MainWindow)  # MainWindow 페이지를 보여줌

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
    
    def open_PaymentWindow(self):
        self.PaymentWindow = PaymentWindow(self)
        self.stacked_widget.addWidget(self.PaymentWindow)  # PaymentWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.PaymentWindow)  # PaymentWindow 페이지를 보여줌

    def open_ByeWindow(self):
        self.ByeWindow = ByeWindow(self)
        self.stacked_widget.addWidget(self.ByeWindow)  # ByeWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.ByeWindow)  # ByeWindow 페이지를 보여줌
    

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
        self.Title_window = MainWindow()
        self.Title_window.show()
        self.close()

    def open_loading_window(self):
        self.loading_window = LoadingWindow()
        self.loading_window.show()
        self.close()

    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        song = AudioSegment.from_mp3("/home/messi/Downloads/mp3/cat_like1b.mp3")
        play(song)

    def next(self):
        self.main_window.open_RecommendWindow()

class RecommendWindow(QMainWindow, from_class_Recommend):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_signal = pyqtSignal(str)

    def __init__(self , main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window
        
        self.pushButton_back.clicked.connect(self.next)
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
        #       네 - 음성출력 및 다음화면 "선택한 메뉴를 제조하겠습니다"
        #       아니요 - 음성출력 : "메뉴를 선택해 주세요"
        #           음성 인식 : 서비스 : UI -> Voice_Input (아이스크림 맛, 토핑맛)
        #               맛 선택 : "선택한 맛으로 제조하겠습니다."
        #               nop : 음성출력 - "메뉴가 부정확합니다 . 마우스로 클릭해 주세요"

        self.update_signal.connect(self.next)
    
    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        song = AudioSegment.from_mp3("/home/messi/Downloads/mp3/cat_like1b.mp3")
        play(song)

    def on_click_warn(self):
        threading.Thread(target=self.play_mp3_warn).start()

    def play_mp3_warn(self):
        song = AudioSegment.from_mp3("/home/messi/Downloads/mp3/blip01.mp3")
        play(song)
        
    def open_Title_window(self):
        self.Title_window = MainWindow()
        self.Title_window.show()
        self.close()   

    def open_Direction_window(self):
        self.Direction_window = DirectionWindow()
        self.Direction_window.show()
        self.close()

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


    def returning(self):
        self.Recommend_window = RecommendWindow()
        self.Recommend_window.show()
        self.close()

    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        song = AudioSegment.from_mp3("/home/messi/Downloads/mp3/cat_like1b.mp3")
        play(song)

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
        song = AudioSegment.from_mp3("/home/messi/Downloads/mp3/cat_like1b.mp3")
        play(song)

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
        self.Payment_window = PaymentWindow(self.coupon, self.number)
        self.Payment_window.show()
        self.close()

    def open_Payment_window2(self):
        self.coupon = 0
        self.Payment_window = PaymentWindow(self.coupon, self.number)
        self.Payment_window.show()
        self.close()

    def open_Maked_window(self):
        self.Maked_window = MakedWindow()
        self.Maked_window.show()
        self.close()

class PaymentWindow(QMainWindow, from_class_Payment):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    update_signal = pyqtSignal(str)

    def __init__(self, main_window , coupon_value, number):
        super().__init__()
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
        

    def priceText(self):
        text = str(self.price) + ' 원'
        self.textBrowser_2.setText(text)
        
    def set_coupon_text(self):
        message = f"회원님의 쿠폰 수는 {self.coupons}개입니다."
        self.textBrowser.setText(message)
        message2 = f"회원번호: 010-" + self.number[:4] + '-' + self.number[4:]
        self.textBrowser_3.setText(message2)

    def use_coupon(self):
        if self.coupons >= 10 and self.coupon_value != 0:
            self.coupons -= 10
            self.price = 0
            self.priceText()
            self.set_coupon_text()
            message2 = f"남은 쿠폰 수는 {self.coupons}개 입니다."
            QMessageBox.warning(self, '쿠폰 사용 완료', message2)
        #self.open_Bye_window()

    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        song = AudioSegment.from_mp3("/home/messi/Downloads/mp3/cat_like1b.mp3")
        play(song)

    def next(self):
        self.main_window.open_ByeWindow()

    def returning(self):
        self.Coupon_window = CouponWindow()
        self.Coupon_window.show()
        self.close()

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
        self.main_window.open_MainWindow()
        
class Ros2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rp.spin(self.node)

def main(args=None):
    rp.init(args=args)

    app = QApplication(sys.argv)

    main_window = MainWindow()
    loading_window = LoadingWindow(main_window)
    recommend_window = RecommendWindow(main_window)
    preparing_window = PreparingWindow(main_window)
    making_window = MakingWindow(main_window)
    maked_window = MakedWindow(main_window)
    bye_window = ByeWindow(main_window)

    Node = PyQt(main_window , loading_window , recommend_window , preparing_window , making_window , maked_window , bye_window)
    preparing_window.plus_init(Node)
    bye_window.plus_init(Node)

    main_window.show()

    ros2_thread = Ros2Thread(Node)
    ros2_thread.start()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()