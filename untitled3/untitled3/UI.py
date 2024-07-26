import rclpy as rp
from rclpy.node import Node
from untitled_msgs.msg import TopicString
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
from untitled3.DB_manager import DB_Manager

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

    def __init__(self , main_window , first_window ,loading_window , recommend_window , preparing_window , making_window , maked_window , bye_window):
        super().__init__('PyQt')

        # Robot_Server에서 토픽 받기 (3초 정면 대기 / 설명장면 / 메뉴선택화면 / 추천메뉴선택 / 메뉴선택 / 쓰레기처리 / 제조준비완료 / 제조완료 / 결제장면 / 마지막장면)
        self.Robot_server_subscriber = self.create_subscription(
            TopicString,
            '/Server_to_UI',
            self.Robot_Server_callback,
            10
        )

        # prevent unused variable warning
        self.Robot_server_subscriber

        # Robot_Server로 토픽 퍼블리셔 (화면 전환)
        self.robot_server_publisher = self.create_publisher(TopicString, '/UI_to_Server', 10)

        # 시작
        msg = TopicString()
        msg.command = "start"
        self.robot_server_publisher.publish(msg)

        self.main_window = main_window
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
            self.main_window.open_LoadingWindow()
            # self.first_window.update_signal.emit(msg.command)

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

# 전체 UI 관리  
class MainWindow(QMainWindow, Node):
    def __init__(self):
        QMainWindow.__init__(self)
        Node.__init__(self, 'qt_ros2_node')

        self.stacked_widget = QStackedWidget(self)  # QStackedWidget 인스턴스 생성   
        self.setCentralWidget(self.stacked_widget)  # MainWindow의 중앙 위젯으로 설정

        self.publisher = self.create_publisher(TopicString, '/qt_to_server', 10)
        self.subscription = self.create_subscription(TopicString, '/server_to_qt', self.listener_callback, 10)

        # self.ros_thread = ROS2Thread(self)
        # self.ros_thread.ros_signal.connect(self.handle_ros_message)
        # self.ros_thread.start()

        self.open_FirstWindow()

    def ui_callback(self,msg):
        if msg.command == "loading":
            self.open_LoadingWindow()

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

        # 맛 추천 표시 두 가지 방법 중 선택하기
        # age, gender, recommended_flavor, recommended_topping = DB_main().recommend_flavor_topping()
        # print(f"{recommended_flavor}, {recommended_topping}")
        # self.label_2.setText(f"손님께 추천드리는 맛과 토핑은 {recommended_flavor}와 {recommended_topping}입니다.")

        # self.Flavor, self.Topping = DB_main().YorN_response(recommended_flavor, recommended_topping)

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

        # QLabel에 GIF 설정
        self.movie = QMovie("/home/jchj/Untitled3/src/untitled3/UI/loading.gif")
        if self.movie.isValid():
            self.label_pic.setMovie(self.movie)
            self.movie.start()

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

    # def payment(self):
    #     if self.Use_coupon == 'N' and self.coupon_value == 1 and self.number is not None and self.number != "":
    #         phone = self.number
    #         # update_info, coupon = DB_main().update_infos(Flavor, Topping, self.price, int(phone), age, gender, self.Use_coupon)
    #         print(f"적립은 하지만 쿠폰 사용 x update_info | coupon {update_info}, {coupon}")
    #         self.open_Bye_window()
        
    #     elif self.Use_coupon == 'Y':
    #         print("쿠폰 사용한 고객은 이미 업데이트 했으니 그냥 바로 넘어감")
    #         self.open_Bye_window()

    #     elif self.coupon_value == 0:
    #         phone = None
    #         coupon = None
    #         Use_coupon = 'N'
    #         # update_info, coupon = DB_main().update_infos(Flavor, Topping, self.price, phone, age, gender, Use_coupon)
    #         print(f"적립 안함 update_info | coupon {update_info}, {coupon}")
    #         self.open_Bye_window()
    #     else:
    #         print("payment Error--------------")   

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

    # def plus_init(self, node):
    #     self.node = node
    #     self.node.Conclusion(self.stock)

    def command(self,command):
        # topic , Conclusion , ok
        if "restart" in command :
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

    main_window = MainWindow()
    first_window =  FirstWindow(main_window)
    loading_window = LoadingWindow(main_window)
    recommend_window = RecommendWindow(main_window)
    preparing_window = PreparingWindow(main_window)
    making_window = MakingWindow(main_window)
    maked_window = MakedWindow(main_window)
    bye_window = ByeWindow(main_window)

    Node = PyQt(main_window , first_window , loading_window , recommend_window , preparing_window , making_window , maked_window , bye_window)

    # preparing_window.plus_init(Node)
    # bye_window.plus_init(Node)

    main_window.showMaximized()

    ros2_thread = Ros2Thread(Node)
    ros2_thread.start()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()