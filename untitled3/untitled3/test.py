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

    def __init__(self):
        super().__init__('PyQt')

        # Robot_Server에서 토픽 받기 (손님감지 / 성별,연령대 판별 / 아이스크림 제조 시작 / 아이스크림 제조 완료)
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

    def Robot_Server_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.command}')

        if msg.command == "guest_detect":
            self.main_window.open_LoadingWindow()
        else:
            return

    def publish_message(self,msg):
        self.get_logger().info('UI to Server!')

        msg = TopicString()
        msg.command = msg
        self.robot_server_publisher.publish(msg)

# 전체 UI 관리  
class MainWindow(QMainWindow):
    def __init__(self , node):
        super().__init__(self)

        self.stacked_widget = QStackedWidget(self)  # QStackedWidget 인스턴스 생성   
        self.setCentralWidget(self.stacked_widget)  # MainWindow의 중앙 위젯으로 설정

        self.node = node

        self.open_FirstWindow()

    def open_FirstWindow(self):
        self.FirstWindow = FirstWindow(self)
        self.stacked_widget.addWidget(self.FirstWindow)  # MainWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.FirstWindow)  # MainWindow 페이지를 보여줌
        self.node.publish_message("open_First")

    def open_LoadingWindow(self):
        self.LoadingWindow = LoadingWindow(self)
        self.stacked_widget.addWidget(self.LoadingWindow)  # LoadingWindow 페이지를 stacked widget에 추가
        self.stacked_widget.setCurrentWidget(self.LoadingWindow)  # LoadingWindow 페이지를 보여줌
        self.node.publish_message("open_Loading")


class FirstWindow(QMainWindow, from_class):
    # ROS2에서 수신한 데이터를 업데이트하는 신호 정의
    
    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)
    
        self.load_image()

        self.main_window = main_window
    
        self.pushButton.clicked.connect(self.next)
        self.pushButton.clicked.connect(self.on_click)

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

    def __init__(self,main_window):
        super().__init__()
        self.setupUi(self)

        self.main_window = main_window

        # QLabel에 GIF 설정
        self.movie = QMovie("/home/jchj/Untitled3/src/untitled3/UI/loading.gif")
        if self.movie.isValid():
            self.label_pic.setMovie(self.movie)
            self.movie.start()

    # def next(self):
    #     self.main_window.open_DirectionWindow()

class Ros2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rp.spin(self.node)

def main(args=None):
    rp.init(args=args)

    app = QApplication(sys.argv)

    Node = PyQt()

    main_window = MainWindow(Node)
    first_window =  FirstWindow(main_window)

    main_window.showMaximized()

    ros2_thread = Ros2Thread(Node)
    ros2_thread.start()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()