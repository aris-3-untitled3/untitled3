import rclpy as rp
from rclpy.node import Node
from untitled_msgs.msg import TopicString
import sys
import os
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget , QMessageBox , QTableWidgetItem , QAbstractItemView , QWidget , QPushButton ,QLabel,QVBoxLayout,QDialog
from PyQt5 import uic , QtWidgets 
from PyQt5.QtGui import QPixmap, QMovie ,QScreen,QFont
from PyQt5.QtCore import pyqtSignal, QThread, QObject , QTimer , Qt
import pygame
import os
import threading
sys.path.append('/home/jchj/Untitled3/src/untitled3/untitled3')
from DB_manager import DB_main
from Voice_Input import Record_API
from Voice_out import VoiceOut
import time

uipath = '/home/jchj/Untitled3/src/untitled3/UI'

# UI 파일 경로 설정
ui_file = os.path.join(uipath, "Title.ui")
ui_file2 = os.path.join(uipath, "Loading.ui")
ui_Direction = os.path.join(uipath, "Direction.ui")
ui_Recommend = os.path.join(uipath, "Recommend_kor.ui")
ui_Preparing = os.path.join(uipath, "Preparing.ui")
ui_Making = os.path.join(uipath, "Making.ui")
ui_Maked = os.path.join(uipath, "Maked.ui")
ui_Coupon = os.path.join(uipath, "Coupon.ui")
ui_Payment = os.path.join(uipath, "Payment.ui")
ui_Bye = os.path.join(uipath, "Bye.ui")

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

class SignalEmitter(QObject):
    # 신호 정의
    ui_update_signal = pyqtSignal(str)

class PyQtNode(Node):
    def __init__(self, signal_emitter):
        super().__init__('PyQtNode')
        self.signal_emitter = signal_emitter

        self.Robot_server_subscriber = self.create_subscription(
            TopicString,
            '/Server_to_UI',
            self.Robot_Server_callback,
            10
        )

        self.robot_server_publisher = self.create_publisher(TopicString, '/UI_to_Server', 10)

    def Robot_Server_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.command}')
        if msg.command == "guest_detect":
            self.signal_emitter.ui_update_signal.emit("open_Loading")
        elif "guest_confirm" in msg.command :
            self.signal_emitter.ui_update_signal.emit("open_Direction")
        elif msg.command == "ice_cream_ready":
            self.signal_emitter.ui_update_signal.emit("open_Making")
        elif msg.command == "ice_cream_completed":
            self.signal_emitter.ui_update_signal.emit("open_Maked")
        elif msg.command == "open_coupon":
            self.signal_emitter.ui_update_signal.emit("open_Coupon")
        elif msg.command == "return":
            self.signal_emitter.ui_update_signal.emit("open_First")
        else:
            print("NO")

    def publish_message(self, command):
        self.get_logger().info('UI to Server!')
        msg = TopicString()
        msg.command = command
        self.robot_server_publisher.publish(msg)
        print(msg)

class MainWindow(QMainWindow):
    def __init__(self, signal_emitter):
        super().__init__()

        self.stacked_widget = QStackedWidget(self)
        self.setCentralWidget(self.stacked_widget)

        self.node = PyQtNode(signal_emitter)
        self.node.signal_emitter.ui_update_signal.connect(self.handle_ui_update)

        self.open_FirstWindow()

    def handle_ui_update(self, command):
        if command == "open_Loading":
            self.open_LoadingWindow()
        elif command == "open_Direction":
            self.open_DirectionWindow()
        elif command == "open_Making":
            self.open_MakingWindow()
        elif command == "open_Maked":
            self.open_MakedWindow()
        elif command == "open_Coupon":
            self.open_CouponWindow()
        elif command == "open_First":
            self.open_FirstWindow()
        else:
            print("NO")

    def open_FirstWindow(self):
        self.FirstWindow = FirstWindow(self)
        self.stacked_widget.addWidget(self.FirstWindow)
        self.stacked_widget.setCurrentWidget(self.FirstWindow)
        if self.node:
            self.node.publish_message("open_First")

    def open_LoadingWindow(self):
        self.LoadingWindow = LoadingWindow(self)
        self.stacked_widget.addWidget(self.LoadingWindow)
        self.stacked_widget.setCurrentWidget(self.LoadingWindow)
        # if self.node:
        #     self.node.publish_message("open_Loading")

    def open_DirectionWindow(self):
        self.DirectionWindow = DirectionWindow(self)
        self.stacked_widget.addWidget(self.DirectionWindow)  # DirectionWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.DirectionWindow)  # DirectionWindow 페이지를 보여줌
        if self.node:
            self.node.publish_message("open_Direction")

    def open_RecommendWindow(self):
        self.RecommendWindow = RecommendWindow(self)
        self.stacked_widget.addWidget(self.RecommendWindow)  # RecommendWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.RecommendWindow)  # RecommendWindow 페이지를 보여줌
        if self.node:
            self.node.publish_message("open_Recommand")

    def open_PreparingWindow(self):
        self.PreparingWindow = PreparingWindow(self)
        self.stacked_widget.addWidget(self.PreparingWindow)  # PreparingWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.PreparingWindow)  # PreparingWindow 페이지를 보여줌
        if self.node:
            self.node.publish_message("open_Prepare")

    def open_MakingWindow(self):
        self.MakingWindow = MakingWindow(self)
        self.stacked_widget.addWidget(self.MakingWindow)  # MakingWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.MakingWindow)  # MakingWindow 페이지를 보여줌
        if self.node:
            self.node.publish_message("open_making")

    def open_MakedWindow(self):
        self.MakedWindow = MakedWindow(self)
        self.stacked_widget.addWidget(self.MakedWindow)  # MakedWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.MakedWindow)  # MakedWindow 페이지를 보여줌
        if self.node:
            self.node.publish_message("open_maked")

    def open_CouponWindow(self):
        self.CouponWindow = CouponWindow(self)
        self.stacked_widget.addWidget(self.CouponWindow)  # CouponWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.CouponWindow)  # CouponWindow 페이지를 보여줌
        if self.node:
            self.node.publish_message("open_coupon")
    
    def open_PaymentWindow(self,coupon,number):
        self.PaymentWindow = PaymentWindow(self,coupon,number)
        self.stacked_widget.addWidget(self.PaymentWindow)  # PaymentWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.PaymentWindow)  # PaymentWindow 페이지를 보여줌
        if self.node:
            self.node.publish_message("open_payment")

    def open_ByeWindow(self):
        self.ByeWindow = ByeWindow(self)
        self.stacked_widget.addWidget(self.ByeWindow)  # ByeWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.ByeWindow)  # ByeWindow 페이지를 보여줌
        if self.node:
            self.node.publish_message("open_bye")

class FirstWindow(QMainWindow, from_class):
    def __init__(self, main_window):
        super().__init__()
        self.setupUi(self)
        self.load_image()
        self.main_window = main_window

        self.setWindowTitle('Background Image Example')
        self.resize(1920, 1080)

        # 중앙 위젯 설정
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        # 스타일시트를 사용하여 중앙 위젯에 배경 이미지 설정
        central_widget.setStyleSheet(f"""
            QWidget {{
                background-image: url("{uipath}/background.png"); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }}
        """)

        # 버튼 설정
        self.pushButton = QPushButton('주문\n\nOrder Now', self)
        self.pushButton.setGeometry(1400, 600, 360, 360)  # 버튼 위치와 크기 설정

        # 버튼 스타일 설정
        self.pushButton.setStyleSheet("""
            QPushButton {
                background-color: #f1c40f; /* 버튼 배경색 */
                color: white; /* 버튼 글자색 */
                font-size: 26px; /* 글자 크기 */
                font-weight: bold; /* 글자 굵기 */
                border-radius: 15px; /* 둥근 모서리 */
                border: 2px solid #e67e22; /* 테두리 색과 굵기 */
            }
            QPushButton:hover {
                background-color: #e67e22; /* 마우스 오버시 배경색 */
            }
            QPushButton:pressed {
                background-color: #d35400; /* 클릭시 배경색 */
            }
        """)

        # QLabel 수동 생성 및 설정
        self.label = QLabel("온 가족이 함께 즐기는 무제 아이스크림", self)
        self.label.setGeometry(500, 340, 881, 131)
        self.label.setStyleSheet("""
            QLabel {
                color: black; /* 글자 색상 */
                font-size: 50px; /* 글자 크기 */
                font-weight: bold; /* 글자 굵기 */
                background-color: rgba(0, 0, 0, 0); /* 배경 투명하게 */
            }
        """)
        self.label.setAlignment(Qt.AlignCenter)  # 텍스트를 중앙에 정렬
        self.label.raise_()  # QLabel을 최상위로 올림

        self.pushButton.clicked.connect(self.next)
        self.pushButton.clicked.connect(self.on_click)

    def load_image(self):
        pixmap = QPixmap(f"{uipath}/Title.jpg")
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
    def __init__(self, main_window):
        super().__init__()
        self.setupUi(self)
        self.main_window = main_window

        self.resize(1920, 1080)

        # QLabel 객체 접근
        self.label = self.findChild(QtWidgets.QLabel, 'label')

        # QLabel에 배경 이미지 설정
        self.label.setStyleSheet(f"""
            QLabel {{
                background-image: url("{uipath}/background.png"); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }}
        """)

        # self.movie = QMovie("/home/jchj/Untitled3/src/untitled3/UI/loading.gif")
        # if self.movie.isValid():
        #     self.label_2.setMovie(self.movie)
        #     self.movie.start()

    def next(self):
        self.main_window.open_DirectionWindow()

class DirectionWindow(QMainWindow, from_class_Direction):
    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)
        
        self.main_window = main_window

        # QLabel 객체 접근s
        self.widget = self.findChild(QtWidgets.QWidget, 'widget')
        self.pushButton_front = self.findChild(QtWidgets.QPushButton, 'pushButton_front')
        self.pushButton_home = self.findChild(QtWidgets.QPushButton, 'pushButton_home')
        self.pushButton_back = self.findChild(QtWidgets.QPushButton, 'pushButton_back')

        self.pushButton_next1 = self.findChild(QtWidgets.QTextBrowser, 'next_png1')
        self.pushButton_next2 = self.findChild(QtWidgets.QTextBrowser, 'next_png2')
        self.pushButton_front.clicked.connect(self.next)
        self.pushButton_back.clicked.connect(self.open_loading_window)
        self.pushButton_home.clicked.connect(self.open_Title_window)
        self.pushButton_front.clicked.connect(self.on_click)
        self.pushButton_back.clicked.connect(self.on_click)
        self.pushButton_home.clicked.connect(self.on_click)

        # QLabel에 배경 이미지 설정

        self.pushButton_next1.setStyleSheet(f"""
            QTextBrowser {{
                background-image: url({uipath}/button/next.png);
            }}
        """)

        self.pushButton_next2.setStyleSheet(f"""
            QTextBrowser {{
                background-image: url({uipath}/button/next.png);
            }}
        """)

        # QLabel에 배경 이미지 설정
        self.widget.setStyleSheet(f"""
            QWidget {{
                background-image: url("{uipath}/background.png"); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }}
        """)
        self.pushButton_front.setStyleSheet(f"""
            QPushButton {{
                color: #3E2723; /* 다크 브라운 글씨 색 */
                background-image: url({uipath}/button/after.png);  
                font-size: 32px; /* 글씨 크기 */
                border-radius: 45px; /* 둥근 모서리 */
                padding: 10px;
            }}
            QPushButton:hover {{
                background-color: #FFC0CB; /* 마우스 오버시 배경색 */
            }}
        """)

        self.pushButton_home.setStyleSheet(f"""
            QPushButton {{
                color: #3E2723; /* 다크 브라운 글씨 색 */
                background-image: url({uipath}/button/home.png);  
                font-size: 32px; /* 글씨 크기 */
                border-radius: 45px; /* 둥근 모서리 */
                padding: 10px;
            }}
            QPushButton:hover {{
                background-color: #FFC0CB; /* 마우스 오버시 배경색 */
            }}
        """)

        self.pushButton_back.setStyleSheet(f"""
            QPushButton {{
                color: #3E2723; /* 다크 브라운 글씨 색 */
                background-image: url({uipath}/button/before.png);  
                font-size: 32px; /* 글씨 크기 */
                border-radius: 45px; /* 둥근 모서리 */
                padding: 10px;
            }}
            QPushButton:hover {{
                background-color: #FFC0CB; /* 마우스 오버시 배경색 */
            }}
        """)      

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
    def __init__(self , main_window):
        super().__init__()
        global age, gender, taste,top
        self.setupUi(self)

        self.main_window = main_window
        
        self.frame = self.findChild(QtWidgets.QFrame, 'frame')
        self.frame.setStyleSheet(f"""
            QFrame {{
            background-image: url({uipath}/background.png);
                                 }}
            """)

        #맛 추천 표시 두 가지 방법 중 선택하기
        age, gender, recommended_flavor, recommended_topping = DB_main().recommend_flavor_topping()
        print(f"{age},{gender}{recommended_flavor}, {recommended_topping}")
        self.label_2.setText(f"손님께 추천드리는 맛과 토핑은 {recommended_flavor}와 {recommended_topping}입니다.")

        # 디폴트는 추천대로
        taste = recommended_flavor
        top = recommended_topping

        self.pushButton_front = self.findChild(QtWidgets.QPushButton, 'pushButton_front')
        self.pushButton_home = self.findChild(QtWidgets.QPushButton, 'pushButton_home')
        self.pushButton_back = self.findChild(QtWidgets.QPushButton, 'pushButton_back')

        # 홈 버튼, 뒤로가기 버튼
        self.pushButton_back.clicked.connect(self.open_Direction_window)
        self.pushButton_home.clicked.connect(self.open_Title_window)
        self.pushButton_back.clicked.connect(self.on_click)
        self.pushButton_home.clicked.connect(self.on_click)

        # 아이스크림과 토핑을 토글 버튼으로 만들기
        self.pushButton_1.setCheckable(True)
        self.pushButton_2.setCheckable(True)
        self.pushButton_3.setCheckable(True)
        self.pushButton_4.setCheckable(True)
        self.pushButton_5.setCheckable(True)
        self.pushButton_6.setCheckable(True)

        # 버튼 소리
        self.pushButton_1.clicked.connect(self.on_click)
        self.pushButton_2.clicked.connect(self.on_click)
        self.pushButton_3.clicked.connect(self.on_click)
        self.pushButton_4.clicked.connect(self.on_click)
        self.pushButton_5.clicked.connect(self.on_click)
        self.pushButton_6.clicked.connect(self.on_click)

        # Next 버튼 이벤트 연결
        self.pushButton_front.clicked.connect(self.check_selection_and_next)
        # 추천 버튼 눌렀을 때
        # self.pushButton_rec.clicked.connect(self.open_Prepare_window_rec)
        
        self.yesno = 0

        self.pushButton_front.setStyleSheet(f"""
            QPushButton {{
                background-image: url({uipath}/button/after.png);  
                border: 1px solid rgba(0, 0, 0, 0); /* 투명 테두리 */
                background-color: rgba(0, 0, 0, 0);
            }}
            QPushButton:hover {{
                background-color: #FFC0CB; /* 마우스 오버시 배경색 */
            }}
        """)

        self.pushButton_home.setStyleSheet(f"""
            QPushButton {{
                background-image: url({uipath}/button/home.png);  
                border: 1px solid rgba(0, 0, 0, 0); /* 투명 테두리 */
                background-color: rgba(0, 0, 0, 0);
            }}
            QPushButton:hover {{
                background-color: #FFC0CB; /* 마우스 오버시 배경색 */
            }}
        """)

        self.pushButton_back.setStyleSheet(f"""
            QPushButton {{
                background-image: url({uipath}/button/before.png);  
                border: 1px solid rgba(0, 0, 0, 0); /* 투명 테두리 */
                background-color: rgba(0, 0, 0, 0);
                
            }}
            QPushButton:hover {{
                background-color: #FFC0CB; /* 마우스 오버시 배경색 */
            }}
        """)      


        self.pushButton_1.setStyleSheet(f"""
            QPushButton {{
                background-image: url({uipath}/button/choco.png);
                border: 1px solid rgba(0, 0, 0, 0); /* 투명 테두리 */
            }}
            QPushButton:checked {{
                background-image: url({uipath}/button/choco.png);
                border: 4px solid brown; /* 초코 색 */
            }}
        """)
        self.pushButton_2.setStyleSheet(f"""
            QPushButton {{
                background-image: url({uipath}/button/vanilla.png);
                border: 1px solid rgba(0, 0, 0, 0); /* 투명 테두리 */
            }}
            QPushButton:checked {{
                background-image: url({uipath}/button/vanilla.png);
                border: 4px solid yellow; /* 바닐라 색 */
            }}
        """)
        self.pushButton_3.setStyleSheet(f"""
            QPushButton {{
                background-image: url({uipath}/button/berry.png);
                border: 1px solid rgba(0, 0, 0, 0); /* 투명 테두리 */
            }}
            QPushButton:checked {{
                background-image: url({uipath}/button/berry.png);
                border: 4px solid red; /* 딸기 색 */
            }}
        """)
        self.pushButton_4.setStyleSheet(f"""
            QPushButton {{
                background-image: url({uipath}/button/topa.png);
                border: 1px solid lightgray; /* 회색 테두리 */
            }}
            QPushButton:checked {{
                background-image: url({uipath}/button/topa.png);
                border: 4px solid lightgray; /* 회색 */
            }}
        """)
        self.pushButton_5.setStyleSheet(f"""
            QPushButton {{
                background-image: url({uipath}/button/topb.png);
                border: 1px solid lightgray; /* 회색 테두리 */
            }}
            QPushButton:checked {{
                background-image: url({uipath}/button/topb.png);
                border: 4px solid lightgray; /* 녹색 */
            }}
        """)
        self.pushButton_6.setStyleSheet(f"""
            QPushButton {{
                background-image: url({uipath}/button/topc.png);
                border: 1px solid lightgray; /* 회색 테두리 */
            }}
            QPushButton:checked {{
                background-image: url({uipath}/button/topc.png);
                border: 4px solid lightgray; /* 파란색 */
            }}
        """)

        self.yesno = 0

        self.initUI()
        


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
        pygame.mixer.music.load("/home/jchj/Untitled3/src/untitled3/UI/blip01.mp3")
        pygame.mixer.music.play()
        
    def open_Title_window(self):
        self.main_window.open_FirstWindow()

    def open_Direction_window(self):
        self.main_window.open_DirectionWindow()

    def initUI(self):
        self.setGeometry(0, 0, 1920, 1080)
        self.show()
        # 초기 UI 설정 및 음성 인식 시작
        QTimer.singleShot(500, self.show_recommendation_dialog)

    def show_recommendation_dialog(self):
        self.recommendation_dialog = QDialog(self)
        self.recommendation_dialog.setWindowTitle('추천 메뉴')
        self.recommendation_dialog.setGeometry(0, 0, 700, 700)
        self.recommendation_dialog.setStyleSheet("""
            QDialog {
                background-image: url(/home/jchj/Untitled3/src/untitled3/UI/backimg.png); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }
        """)

        layout = QVBoxLayout()

        label = QLabel(f"추천드린 메뉴로 주문하시겠습니까?\n\n추천메뉴는 {taste}와 {top}입니다.\n\n'예' 또는 '아니오'로 대답해주세요.")
        label.setAlignment(Qt.AlignCenter)

        font = QFont()
        font.setPointSize(28)
        label.setFont(font)

        layout.addWidget(label)
        self.recommendation_dialog.setLayout(layout)
        self.recommendation_dialog.setWindowModality(Qt.ApplicationModal)
        self.center_dialog(self.recommendation_dialog)
        self.recommendation_dialog.show()

        QTimer.singleShot(3000, self.show_buffering_dialog)

    def show_buffering_dialog(self):
        self.recommendation_dialog.close()

        self.buffering_dialog = QDialog(self)
        self.buffering_dialog.setWindowTitle('음성 인식 중')
        self.buffering_dialog.setGeometry(0, 0, 700, 700)
        self.buffering_dialog.setStyleSheet("""
            QDialog {
                background-image: url(/home/jchj/Untitled3/src/untitled3/UI/button/record.png); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }
        """)

        layout = QVBoxLayout()

        label = QLabel("\n\n\n\n\n\n\n\n\n\n음성 인식 중")
        label.setAlignment(Qt.AlignCenter)

        font = QFont()
        font.setPointSize(28)
        label.setFont(font)

        layout.addWidget(label)
        self.buffering_dialog.setLayout(layout)
        self.buffering_dialog.setWindowModality(Qt.ApplicationModal)
        self.center_dialog(self.buffering_dialog)
        self.buffering_dialog.show()
        QTimer.singleShot(1000, self.voice_1)

        #음성받기
    def voice_1(self):   
        text = Record_API().run()

        print(text)

        if text == None:
            QTimer.singleShot(1000, self.show_speechorder_dialog)
        elif "선택" in text:
            print("Yes라고 답변 받았을 때")
            QTimer.singleShot(1000, self.buffering_dialog.close)
            QTimer.singleShot(1000, self.open_Prepare_window_rec)
        else:
            print("No라고 답변 받았을 때")
            QTimer.singleShot(1000, self.show_speechorder_dialog)

    def show_speechorder_dialog(self):
        self.buffering_dialog.close()

        self.speechorder_dialog = QDialog(self)
        self.speechorder_dialog.setWindowTitle('음성인식 주문 창')
        self.speechorder_dialog.setGeometry(0, 0, 700, 700)
        self.speechorder_dialog.setStyleSheet("""
            QDialog {
                background-image: url(/home/jchj/Untitled3/src/untitled3/UI/backimg.png); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }
        """)
        layout = QVBoxLayout()

        label = QLabel("음성 인식 중입니다.\n원하시는 메뉴를 말씀해주세요.\n예시문장 (**맛과 **토핑 주세요.)")
        label.setAlignment(Qt.AlignCenter)

        font = QFont()
        font.setPointSize(28)
        label.setFont(font)

        layout.addWidget(label)
        self.speechorder_dialog.setLayout(layout)
        self.speechorder_dialog.setWindowModality(Qt.ApplicationModal)
        self.center_dialog(self.speechorder_dialog)
        self.speechorder_dialog.show()

        self.recognize = 0 # 음성인식 유무 ## 인식 창 안꺼짐
        QTimer.singleShot(1000, self.voice_2)

        #음성받기
    def voice_2(self):   
        text = Record_API().run()

        print(text)

        if text == None:
            flag = 0
        else:
            flag = 1

        text_file = "/home/jchj/Untitled3/src/untitled3/resource/response.txt"

        taste , top = DB_main().word_detect(text_file,5,flag)
        print(f"{taste},{top}--------------------------1")

        ## 인식 기회 1번
        if taste == "error" or top == "error":
            print("답변 못 받았을 때")
            self.speechorder_dialog.close()
        else:
            print("답변 받았을 때")
            self.speechorder_dialog.close()
            print(f"{taste},{top}--------------------------2")
            QTimer.singleShot(1000, self.open_Preparing_window)
        
    def center_dialog(self, dialog):
        # 화면 중앙에 다이얼로그 배치
        screen = QScreen.availableGeometry(QApplication.primaryScreen())
        x = (screen.width() - dialog.width()) // 2
        y = (screen.height() - dialog.height()) // 2
        dialog.move(x, y)

    def open_Prepare_window_rec(self):
        # 맛과 토핑 변경 없음
        self.main_window.open_CouponWindow()
        
    def open_Title_window(self):
        self.main_window.open_FirstWindow()

    def open_Direction_window(self):
        self.main_window.open_DirectionWindow()

    # 한 가지씩 선택하고 다음 눌렀을 때 재확인창
    def showMessage(self):
        message = f'선택하신 메뉴를 확인해주세요. \n아이스크림: {taste}\n토핑: {top}'
        reply = QMessageBox.question(self, 'Message', message, 
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            self.open_Preparing_window()
            print('Yes selected')
        else:
            print('No selected')

    def check_selection_and_next(self):
        # 아이스크림 맛이 하나가 선택되었는지 확인
        global taste,top
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

        # 하나씩 선택하면 DB에 넣을 거 초기화
        if ice_cream_selected:
            if self.pushButton_1.isChecked():
                taste = "choco"
            elif self.pushButton_2.isChecked():
                taste = "vanilla"
            elif self.pushButton_3.isChecked():
                taste = "berry"
            
        if topping_selected:
            if self.pushButton_6.isChecked():
                top = "topA"
            elif self.pushButton_5.isChecked():
                top = "topB"
            elif self.pushButton_4.isChecked():
                top = "topC"

        if not ice_cream_selected:
            self.on_click_warn()
            QMessageBox.warning(self, 'Warning', '아이스크림 맛을 한 가지만 골라주세요.')

        if not topping_selected:
            self.on_click_warn()
            QMessageBox.warning(self, 'Warning', '토핑을 한 가지만 골라주세요.')

        if ice_cream_selected and topping_selected:
            self.showMessage()
            self.setWindowTitle('QMessageBox.question Example')

        # if ice_cream_selected and topping_selected:
        #     self.open_Preparing_window()

    def open_Preparing_window(self):
        print(taste,top)
        self.main_window.open_CouponWindow()

    def next(self):
        print(taste,top)
        self.main_window.open_CouponWindow()

class CouponWindow(QMainWindow, from_class_Coupon):
    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window

        self.setStyleSheet("QMainWindow { background-color: #ADD8E6; }")   
        self.coupon = 1
        self.number = ""

        # QLabel 객체 접근
        self.label_3 = self.findChild(QtWidgets.QLabel, 'label_3')

        # QLabel에 배경 이미지 설정
        self.label_3.setStyleSheet("""
            QLabel {
                background-image: url("/home/jchj/Untitled3/src/untitled3/UI/background.png"); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }
        """)

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

class PaymentWindow(QMainWindow, from_class_Payment):
    def __init__(self, main_window , coupon_value, number):
        super().__init__()

        global taste, top, age, gender 

        print(age,gender,taste,top)

        self.setupUi(self)

        self.main_window = main_window

        self.setStyleSheet("QMainWindow { background-color: #ADD8E6; }")   
        self.coupon_value = coupon_value
        self.number = number

        # QLabel 객체 접근
        self.label_3 = self.findChild(QtWidgets.QLabel, 'label_3')

        # QLabel에 배경 이미지 설정
        self.label_3.setStyleSheet("""
            QLabel {
                background-image: url("/home/jchj/Untitled3/src/untitled3/UI/background.png"); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }
        """)

        #DB
        self.Use_coupon = 'N' # default 값
        
        if self.coupon_value == 1:  # stamp 적립 Y
            YorN = 'Y'
        if self.coupon_value == 0:
            YorN = 'N'

        if self.number is not None and self.number != "":
            phone = int(self.number)
            self.number, self.coupon = DB_main().YorN_stamp(YorN, phone)
            print(f"고객 정보 --> {self.number}, {self.coupon}")
        else:
            phone = None

        topping_time = 5 # topping time은 토크 혹은 나오는 실제 시간으로 판단할 예정

        # 임의의 금액
        self.price = DB_main().show_price(top, topping_time)
        self.priceText(self.price)

        # 임의의 쿠폰 수 적용 및 텍스트 출력
        if self.coupon_value != 0:
            self.set_coupon_text(self.coupon)
        
        # 사용 버튼 시
        self.pushButton_6.clicked.connect(self.use_coupon)   
        # 결제 버튼 시
        self.pushButton_8.clicked.connect(self.payment)
        # Back 버튼 추가
        self.pushButton_back.clicked.connect(self.returning)
        
        ## 소리!
        # 사용 버튼 시
        self.pushButton_6.clicked.connect(self.on_click)
        # 결제 버튼 시
        self.pushButton_8.clicked.connect(self.on_click)
        # Back 버튼 추가
        self.pushButton_back.clicked.connect(self.on_click)
        

    def priceText(self,price):
        text = str(price) + ' 원'
        self.textBrowser_2.setText(text)
        
    def set_coupon_text(self,coupons):
        message = f"회원님의 쿠폰 수는 {coupons}개입니다."
        self.textBrowser.setText(message)
        message2 = f"회원번호: 010-" + self.number[:4] + '-' + self.number[4:]
        self.textBrowser_3.setText(message2)

    def use_coupon(self):
        if self.number is not None and self.number != "":
            if self.coupon >= 10 and self.coupon_value == 1: #여기 coupons --> coupon
                self.Use_coupon = 'Y' # 사용하면 Y로 바꿈
                coupon = self.coupon
                coupon -= 10
                price = self.price
                price = 0
                phone = self.number
                self.priceText(price)
                self.set_coupon_text(coupon)
                message2 = f"남은 쿠폰 수는 {coupon}개 입니다."
                if self.Use_coupon == 'Y':
                    update_info, coupon = DB_main().update_infos(taste, top, price, int(phone), age, gender, self.Use_coupon)
                    print(f"update_info | coupon {update_info}, {coupon}")
                QMessageBox.warning(self, '쿠폰 사용 완료', message2)
            else:
                message2 = f"쿠폰이 10개 미만일 경우에는 사용이 불가합니다."
                QMessageBox.warning(self, '쿠폰 사용 불가', message2)
            #self.open_Bye_window()

    def payment(self):
        if self.Use_coupon == 'N' and self.coupon_value == 1 and self.number is not None and self.number != "":
            phone = self.number
            update_info, coupon = DB_main().update_infos(taste, top, self.price, int(phone), age, gender, self.Use_coupon)
            print(f"적립은 하지만 쿠폰 사용 x update_info | coupon {update_info}, {coupon}")
            self.next()
        
        elif self.Use_coupon == 'Y':
            print("쿠폰 사용한 고객은 이미 업데이트 했으니 그냥 바로 넘어감")
            self.next()

        elif self.coupon_value == 0:
            phone = None
            coupon = None
            Use_coupon = 'N'
            update_info, coupon = DB_main().update_infos(taste, top, self.price, phone, age, gender, Use_coupon)
            print(f"적립 안함 update_info | coupon {update_info}, {coupon}")
            self.next()
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
        self.main_window.open_PreparingWindow()

class PreparingWindow(QMainWindow, from_class_Preparing):
    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window

        self.setupUi(self)
        self.setStyleSheet("QMainWindow { background-color: #ADD8E6; }")   
        self.pushButton_back.clicked.connect(self.returning)
        self.pushButton_next.clicked.connect(self.next)
        self.pushButton_back.clicked.connect(self.on_click)
        self.pushButton_next.clicked.connect(self.on_click)

        # QLabel 객체 접근
        self.label_3 = self.findChild(QtWidgets.QLabel, 'label_3')

        # QLabel에 배경 이미지 설정
        self.label_3.setStyleSheet("""
            QLabel {
                background-image: url("/home/jchj/Untitled3/src/untitled3/UI/background.png"); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }
        """)

    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        pygame.mixer.init()
        pygame.mixer.music.load("/home/jchj/Untitled3/src/untitled3/UI/cat_like1b.mp3")
        pygame.mixer.music.play()

    def returning(self):
        self.main_window.open_PaymentWindow()

    def next(self):
        self.main_window.open_MakingWindow()

class MakingWindow(QMainWindow, from_class_Making):
    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window

        self.setStyleSheet("QMainWindow { background-color: #ADD8E6; }")   

        # QLabel 객체 접근
        self.label_3 = self.findChild(QtWidgets.QLabel, 'label_3')

        # 디버깅 정보 출력
        if self.label_3 is None:
            print("Error: QLabel 'label_3' not found.")
        else:
            # QLabel에 배경 이미지 설정
            self.label_3.setStyleSheet("""
                QLabel {
                    background-image: url("/home/jchj/Untitled3/src/untitled3/UI/background.png"); /* 배경 이미지 설정 */
                    background-position: center; /* 이미지 중앙 정렬 */
                    background-repeat: no-repeat; /* 이미지 반복 안함 */
                }
            """)

        # QLabel에 GIF 설정
        self.movie = QMovie("/home/jchj/Untitled3/src/untitled3/UI/loading.gif")
        if self.movie.isValid():
            self.label_pic.setMovie(self.movie)
            self.movie.start()
        else:
            print("GIF를 로드할 수 없습니다.")

    def next(self):
        self.main_window.open_MakedWindow()

class MakedWindow(QMainWindow, from_class_Maked):
    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window

        self.setStyleSheet("QMainWindow { background-color: #ADD8E6; }")   

        # QLabel 객체 접근
        self.label_3 = self.findChild(QtWidgets.QLabel, 'label_3')

        # QLabel에 배경 이미지 설정
        self.label_3.setStyleSheet("""
            QLabel {
                background-image: url("/home/jchj/Untitled3/src/untitled3/UI/background.png"); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }
        """)

        QTimer.singleShot(3000, self.next)

    def next(self):
        self.main_window.open_ByeWindow()

class ByeWindow(QMainWindow, from_class_Bye):
    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window
        self.stock = "ok"

        # QLabel 객체 접근
        self.label_3 = self.findChild(QtWidgets.QLabel, 'label_3')

        # QLabel에 배경 이미지 설정
        self.label_3.setStyleSheet("""
            QLabel {
                background-image: url("/home/jchj/Untitled3/src/untitled3/UI/background.png"); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }
        """)
        self.pushButton.clicked.connect(self.showValues)
        self.show()

    def showValues(self):
        global taste, top    
        print(f"show Values ==> {taste}, {top}")
        result, stock, flavor_flag, topping_flag = DB_main().update_stock(taste, top)
        print(f"Final list ====================== {result}")
        self.tableWidget.setItem(0,0,QTableWidgetItem(str(result[0]))) #vanilla
        self.tableWidget.setItem(0,1,QTableWidgetItem(str(result[1]))) # choco
        self.tableWidget.setItem(0,2,QTableWidgetItem(str(result[2]))) # berry
        self.tableWidget.setItem(0,3,QTableWidgetItem(str(result[3]))) # topA
        self.tableWidget.setItem(0,4,QTableWidgetItem(str(result[4]))) # topB
        self.tableWidget.setItem(0,5,QTableWidgetItem(str(result[5]))) # topC
        self.tableWidget.setEditTriggers(QAbstractItemView.NoEditTriggers)

        if flavor_flag is None and topping_flag is None:
            self.next()

    def next(self):
        QTimer.singleShot(3000, self.next)

class Ros2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rp.spin(self.node)

def main(args=None):
    rp.init(args=args)

    app = QApplication(sys.argv)

    # Create SignalEmitter instance
    signal_emitter = SignalEmitter()

    main_window = MainWindow(signal_emitter)
    main_window.showMaximized()

    ros2_thread = Ros2Thread(main_window.node)
    ros2_thread.start()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()