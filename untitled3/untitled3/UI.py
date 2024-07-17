import rclpy as rp
from rclpy.node import Node
from untitled_msgs.msg import TopicString
from untitled_msgs.srv import ServiceString
from untitled_msgs.action import ActionString
import time
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QDialog, QMessageBox
from PyQt5 import uic
from PyQt5.QtGui import QPixmap, QMovie
from PyQt5.QtCore import QTimer
from PyQt5 import QtWidgets, uic 
from pydub import AudioSegment
from pydub.playback import play
from playsound import playsound
import pygame
import os
import threading

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
    def __init__(self , main_window , loading_window):
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

        self.main_window = main_window
        self.loading_window = loading_window

        # 시작
        msg = TopicString()
        msg.command = "start"
        self.robot_server_publisher.publish(msg)


    def Robot_Server_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.command}')

        if msg.command == "guest_detect":
            # 3초 정면 대기
            print("3초 정면 대기")
            self.main_window.open_loading_window()


        elif "Age" and "Gender" in msg.command:
            self.loading_window.open_DirectionWindow()

            print("인사 음성출력")
            print("사용법 설명")
            time.sleep(5)

            self.Recommend_window = DirectionWindow()
            self.Recommend_window.open_Recommend_window()

            print("메뉴선택")
            Age_Gender = msg.command
            print(Age_Gender)
            # 손님 성별,연령대에 따른 맛 선호도 가져오기: 서비스 : UI -> DB_Manager
            # 메뉴 선택 화면
            #   테투리 - 선호도 (성별,연령대) 
            #   음성 출력 - "추천 메뉴를 선택하시겠습니까?"
            #   음성 받기: 서비스 : UI -> Voice_Input (네 or 아니요)
            #       네 - 음성출력 및 다음화면 "선택한 메뉴를 제조하겠습니다"
            #       아니요 - 음성출력 : "메뉴를 선택해 주세요"
            #           음성 받기 : 서비스 : UI -> Voice_Input (아이스크림 맛, 토핑맛)
            #               맛 선택 : "선택한 맛으로 제조하겠습니다."
            #               nop : 음성출력 - "메뉴가 부정확합니다 . 마우스로 클릭해 주세요"
            #                
            #               마우스 클리식 -> self.Pre_production()


        elif msg.command == "Pre-production":
            print("쓰레기 청소 끝 및 아이스크림 대기")
            self.Recommend_window = RecommendWindow()
            self.Recommend_window.open_Preparing_window()


        elif msg.command == "ice_cream_production_complete":
            print("결제 및 마무리")
            # 제조 완료 화면 및 음성출력
            self.Making_window = MakingWindow()
            self.Making_window.open_Maked_window()

            time.sleep(5)
            
            # 쿠폰 적립 여부 화면 
            self.Maked_window = MakedWindow()
            self.Maked_window.open_Coupon_window()

            time.sleep(5)
            
            # 쿠폰 사용 및 결제 화면
            self.Coupon_window = CouponWindow()
            self.Coupon_window.open_Payment_window()
            print(Age_Gender)
                # 손님정보 : 성별,연령대 저장 및 번호에 따른 쿠폰 갯수 가져오기 
                # 쿠폰 개수 : UI -> DB_Manager
                # 맛 가격 :  UI -> DB_Manager
                # 일 매출 : UI -> DB_Manager

            # 작별 인사 화면 및 음성출력
            self.Payment_window = PaymentWindow()
            self.Payment_window.open_Bye_window()
            
            # 토핑 재고량 전달 : 서비스 : UI -> DB_Manager
            # 재고량 - > self.Conclusion(stock)

        elif "Conclusion" in msg.command :
            print("쓰레기 청소 끝")

            if "ok" in msg.command :
                self.Bye_window = ByeWindow()
                self.Bye_window.open_Main_window()
            else :
                print("재고량 없음")
                return

        else:
            return

    def Pre_production(self):
        self.get_logger().info('UI to Server , Pre-production!')

        msg = TopicString()
        msg.command = 'Pre-production'
        self.robot_server_publisher.publish(msg)


    def Conclusion(self , stock):
        self.get_logger().info('UI to Server , Conclusion!')

        msg = TopicString()
        msg.command = 'Conclusion , %s'.format(stock)
        self.robot_server_publisher.publish(msg)


    def ui_callback(self, request, response):
        self.get_logger().info(f'Received command: {request.command}')

        if request.command == "ice_cream_Bucket_detect":
            print("제조준비 화면")
            print("음성출력 - 아이스크림을 제조 하겠습니다.")
            self.Preparing_window = PreparingWindow()
            self.Preparing_window.open_Making_window()
        else:
            print("쓰레기를 처리 중입니다")
            time.sleep(3)

        response.success = True
        response.result = f'Command {request.command} received and being processed' 

        return response
    
class MainWindow(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.pushButton.clicked.connect(self.open_loading_window)
        self.pushButton.clicked.connect(self.on_click)

        self.load_image()

    def load_image(self):
        pixmap = QPixmap("/home/jchj/Untitled3/src/untitled3/UI/Title.jpg")
        self.label_2.setPixmap(pixmap)
        self.label_2.setScaledContents(True)

    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        song = AudioSegment.from_mp3("/home/messi/Downloads/mp3/cat_like1b.mp3")
        play(song)

    def open_loading_window(self):
        self.loading_window = LoadingWindow()
        self.loading_window.show()
        self.close()

class LoadingWindow(QMainWindow, from_class2):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # QLabel에 GIF 설정
        self.movie = QMovie("loading.gif")
        if self.movie.isValid():
            self.label_pic.setMovie(self.movie)
            self.movie.start()
        else:
            print("GIF를 로드할 수 없습니다.")

        # 로딩 3초 후
        QTimer.singleShot(300, self.open_DirectionWindow)

    def open_DirectionWindow(self):
        self.DirectionWindow = DirectionWindow()
        self.DirectionWindow.show()
        self.close()

class DirectionWindow(QMainWindow, from_class_Direction):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.pushButton.clicked.connect(self.open_Recommend_window)
        self.pushButton_back.clicked.connect(self.open_loading_window)
        self.pushButton_home.clicked.connect(self.open_Title_window)
        self.pushButton.clicked.connect(self.on_click)
        self.pushButton_back.clicked.connect(self.on_click)
        self.pushButton_home.clicked.connect(self.on_click)

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

    def open_Recommend_window(self):
        self.Recommend_window = RecommendWindow()
        self.Recommend_window.show()
        self.close()

class RecommendWindow(QMainWindow, from_class_Recommend):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
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
            self.open_Preparing_window()

    def play_sound(self):
        if self.mixer_initialized:
            try:
                pygame.mixer.music.load("sound.mp3")  # 소리 파일 로드
                pygame.mixer.music.play()  # 소리 재생
            except pygame.error as e:
                print(f"Error playing sound: {e}")
        else:
            print("Mixer not initialized. Cannot play sound.")

    def open_Preparing_window(self):
        self.Preparing_window = PreparingWindow()
        self.Preparing_window.show()
        self.close()

class PreparingWindow(QMainWindow, from_class_Preparing):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.pushButton_back.clicked.connect(self.open_Recommend_window)
        self.pushButton_next.clicked.connect(self.open_Making_window)
        self.pushButton_back.clicked.connect(self.on_click)
        self.pushButton_next.clicked.connect(self.on_click)

        # 쓰레기 처리, 아이스크림 대기 완료 과정을 3초로 임시 가정 -> 이것도 버그땜에 일단 없앰
        # QTimer.singleShot(3000, self.open_Making_window)

    def open_Recommend_window(self):
        self.Recommend_window = RecommendWindow()
        self.Recommend_window.show()
        self.close()

    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        song = AudioSegment.from_mp3("/home/messi/Downloads/mp3/cat_like1b.mp3")
        play(song)

    def open_Making_window(self):
        self.Making_window = MakingWindow()
        self.Making_window.show()
        self.close()

class MakingWindow(QMainWindow, from_class_Making):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # QLabel에 GIF 설정
        self.movie = QMovie("loading.gif")
        if self.movie.isValid():
            self.label_pic.setMovie(self.movie)
            self.movie.start()
        else:
            print("GIF를 로드할 수 없습니다.")

        # 쓰레기 처리, 아이스크림 대기 완료 과정을 3초로 임시 가정 -> 뒤로가기로 인한 버그로 인해 버튼으로 대체
        QTimer.singleShot(300, self.open_Maked_window)

    def open_Maked_window(self):
        self.Maked_window = MakedWindow()
        self.Maked_window.show()
        self.close()

class MakedWindow(QMainWindow, from_class_Maked):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # 이제 결제
        QTimer.singleShot(300, self.open_Coupon_window)

    def open_Coupon_window(self):
        self.Coupon_window = CouponWindow()
        self.Coupon_window.show()
        self.close()

class CouponWindow(QMainWindow, from_class_Coupon):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
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
    def __init__(self, coupon_value, number):
        super().__init__()
        self.setupUi(self)
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
        self.pushButton_8.clicked.connect(self.open_Bye_window)
        # Back 버튼 추가
        self.pushButton_back.clicked.connect(self.open_Coupon_window)
        
        ## 소리!
        # 사용 버튼 시
        self.pushButton_6.clicked.connect(self.on_click)
        # 결제 버튼 시
        self.pushButton_8.clicked.connect(self.on_click)
        # Back 버튼 추가
        self.pushButton_back.clicked.connect(self.on_click)

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

    def open_Bye_window(self):
        self.Bye_window = ByeWindow()
        self.Bye_window.show()
        self.close()

    def open_Coupon_window(self):
        self.Coupon_window = CouponWindow()
        self.Coupon_window.show()
        self.close()

class ByeWindow(QMainWindow, from_class_Bye):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

    def open_Main_window(self):
        self.Main_window = MainWindow()
        self.Main_window.show()
        self.close()

def main(args=None):
    rp.init(args=args)

    app = QApplication(sys.argv)
    main_window = MainWindow()
    loading_window = LoadingWindow()
    main_window.show()

    UI = PyQt(main_window , loading_window)

    # ROS2 spin을 별도의 스레드에서 실행
    threading.Thread(target=rp.spin, args=(UI,), daemon=True).start()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()