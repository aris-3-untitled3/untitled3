# git push test 
# 이미지 출처 <a href="https://kr.freepik.com/free-photo/flat-lay-of-raspberries-ice-cream-with-copy-space_5336545.htm#page=4&query=%EC%95%84%EC%9D%B4%EC%8A%A4%ED%81%AC%EB%A6%BC%20%EB%B0%B0%EA%B2%BD&position=1&from_view=keyword&track=ais_user&uuid=79b46e5b-7b1a-4939-963d-12bd4e15b4ee">Freepik</a>

import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import QtWidgets, uic 
from pydub import AudioSegment
from pydub.playback import play

import os
import threading
import time
import urllib.request
import pygame

uipath = '/home/sophie/design_ui/untitled3'

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
ui_Empty = os.path.join(uipath, "Empty.ui")

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
from_class_Empty = uic.loadUiType(ui_Empty)[0]


class MusicThread(QThread):
    def __init__(self):
        super().__init__()
        self.playing = True

    def run(self):
        pygame.mixer.music.load(f"{uipath}/good.mp3")
        pygame.mixer.music.play(-1)  # 무한 반복 재생
        while self.playing:
            time.sleep(1)

    def stop(self):
        self.playing = False
        pygame.mixer.music.stop()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        pygame.mixer.init() # pygame 초기화
        self.music_thread = MusicThread()
        self.music_thread.start()


        self.empty = 0
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

        if self.empty == 0:
            self.pushButton.clicked.connect(self.open_loading_window)
        elif self.empty == 1:
            self.pushButton.clicked.connect(self.open_empty_window)


    def open_empty_window(self):
        # 노래 중지
        self.playing = False
#        self.music_thread.join()

        self.empty_window = EmptyWindow()
        self.empty_window.show()
        self.close()

    def play_audio(self):
        # 오디오 재생을 별도의 스레드에서 실행
        def play_sound():
            song = AudioSegment.from_mp3(f"{uipath}/good.mp3")
            play(song)

        threading.Thread(target=play_sound).start()

        self.load_image()

    def load_image(self):
        pixmap = QPixmap("/home/sophie/design_ui/untitled3/Title.jpg")
        self.label_2.setPixmap(pixmap)
        self.label_2.setScaledContents(True)

    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        song = AudioSegment.from_mp3(f"{uipath}/cat_like1b.mp3")
        play(song)

    def on_click_good(self):
        threading.Thread(target=self.play_mp3_good).start()

    def play_mp3_good(self):
        pygame.mixer.music.load(f"{uipath}/good.mp3")
        pygame.mixer.music.play(-1)  # 무한 반복 재생
        while self.playing:
            time.sleep(1)  # 음악 재생 중

    def open_loading_window(self):
        # 노래 중지
        self.music_thread.stop()
        self.music_thread.wait()
        
        self.loading_window = LoadingWindow()
        self.loading_window.show()
        self.close()


# new ui window
class EmptyWindow(QMainWindow, from_class_Empty):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        

        self.setWindowTitle('Background Image Example')
        self.resize(1920, 1080)

        # 중앙 위젯 설정
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        # 스타일시트를 사용하여 배경 이미지 설정
        self.setStyleSheet("""
            QMainWindow {
                background-image: url("/home/sophie/design_ui/untitled3/dall.jpg"); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }
        """)


class LoadingWindow(QMainWindow, from_class2):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle('Background Image Example')
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

        # 로딩 3초 후
        QTimer.singleShot(100, self.open_DirectionWindow)


    def open_DirectionWindow(self):
        self.DirectionWindow = DirectionWindow()
        self.DirectionWindow.show()
        self.close()

class DirectionWindow(QMainWindow, from_class_Direction):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # QLabel 객체 접근s
        self.widget = self.findChild(QtWidgets.QWidget, 'widget')
        self.pushButton_front = self.findChild(QtWidgets.QPushButton, 'pushButton_front')
        self.pushButton_home = self.findChild(QtWidgets.QPushButton, 'pushButton_home')
        self.pushButton_back = self.findChild(QtWidgets.QPushButton, 'pushButton_back')

        self.pushButton_next1 = self.findChild(QtWidgets.QTextBrowser, 'next_png1')
        self.pushButton_next2 = self.findChild(QtWidgets.QTextBrowser, 'next_png2')
        self.pushButton_front.clicked.connect(self.open_Recommend_window)
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
        song = AudioSegment.from_mp3(f"{uipath}/cat_like1b.mp3")
        play(song)

    def open_Recommend_window(self):
        self.Recommend_window = RecommendWindow()
        self.Recommend_window.show()
        self.close()

class RecommendWindow(QMainWindow, from_class_Recommend):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        # self.setStyleSheet("QMainWindow { background-color: #ADD8E6; }")   

        self.frame = self.findChild(QtWidgets.QFrame, 'frame')
        self.frame.setStyleSheet(f"""
            QFrame {{
            background-image: url({uipath}/background.png);
                                 }}
            """)

        # 디폴트는 추천대로
        self.taste = "초코"
        self.top = "토핑C"

        # 맛 추천 표시 두 가지 방법 중 선택하기
        self.recommended_flavor = "초코"
        self.recommended_topping = "토핑C"
        self.label_2.setText(f"손님께 추천드리는 맛과 토핑은 {self.recommended_flavor}와 {self.recommended_topping}입니다.")


        # 소리
        #uic.loadUi('your_ui_file.ui', self)

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
        self.pushButton_rec.clicked.connect(self.open_Prepare_window_rec)
        
        self.yesno = 0
        
        # self.pushButton_front = self.findChild(QtWidgets.QPushButton, 'pushButton_front')
        # self.pushButton_home = self.findChild(QtWidgets.QPushButton, 'pushButton_home')
        # self.pushButton_back = self.findChild(QtWidgets.QPushButton, 'pushButton_back')


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


        self.initUI()

    def initUI(self):
        self.setGeometry(0, 0, 1920, 1080)
        self.show()
        # 초기 UI 설정 및 음성 인식 시작
        QTimer.singleShot(500, self.show_recommendation_dialog)

    def show_recommendation_dialog(self):
        self.recommendation_dialog = QDialog(self)
        self.recommendation_dialog.setWindowTitle('추천 메뉴')
        self.recommendation_dialog.setGeometry(0, 0, 1920, 1080)
        self.recommendation_dialog.setStyleSheet(f"""
            QDialog {{
                background-image: url({uipath}/background.png); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }}
        """)

        layout = QVBoxLayout()

        label = QLabel(f"추천드린 메뉴로 주문하시겠습니까?\n\n추천메뉴는 {self.recommended_flavor}와 {self.recommended_topping}입니다.\n\n'예' 또는 '아니오'로 대답해주세요.")
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
        self.buffering_dialog.setGeometry(0, 0, 1920, 1080)
        self.buffering_dialog.setStyleSheet(f"""
            QDialog {{
                background-image: url({uipath}/background.png); /* 배경 이미지 설정 */
                background-position: center; /* 이미지 중앙 정렬 */
                background-repeat: no-repeat; /* 이미지 반복 안함 */
            }}
        """)

        layout = QVBoxLayout()

        label = QLabel("음성 인식 중")
        label.setAlignment(Qt.AlignCenter)

        font = QFont()
        font.setPointSize(28)
        label.setFont(font)

        layout.addWidget(label)
        self.buffering_dialog.setLayout(layout)
        self.buffering_dialog.setWindowModality(Qt.ApplicationModal)
        self.center_dialog(self.buffering_dialog)
        self.buffering_dialog.show()

        ## 인식 기회 1번
        if self.yesno == 1:
            print("Yes라고 답변 받았을 때")
            QTimer.singleShot(4000, self.buffering_dialog.close)
            QTimer.singleShot(4000, self.open_Prepare_window_rec)
        else:
            print("No라고 답변 받았을 때")
            QTimer.singleShot(4000, self.show_speechorder_dialog)

    def show_speechorder_dialog(self):
        self.buffering_dialog.close()

        self.speechorder_dialog = QDialog(self)
        self.speechorder_dialog.setWindowTitle('음성인식 주문 창')
        self.speechorder_dialog.setGeometry(0, 0, 1920, 1080)
        self.speechorder_dialog.setStyleSheet("""
            QDialog {
                background-image: url(/home/sophie/design_ui/untitled3/dall.jpg); /* 배경 이미지 설정 */
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

        ## 인식 기회 1번
        if self.recognize == 1:
            print("답변 받았을 때")
            self.taste = "초코"
            self.top = "로투스"
            QTimer.singleShot(4000, self.buffering_dialog.close)
            QTimer.singleShot(4000, self.open_Preparing_window)
        else:
            print("답변 못 받았을 때")
            QTimer.singleShot(4000, self.speechorder_dialog.close)
        

    def center_dialog(self, dialog):
        # 화면 중앙에 다이얼로그 배치
        screen = QScreen.availableGeometry(QApplication.primaryScreen())
        x = (screen.width() - dialog.width()) // 2
        y = (screen.height() - dialog.height()) // 2
        dialog.move(x, y)



    def open_Prepare_window_rec(self):
        # 맛과 토핑 변경 없음
        self.Preparing_window = PreparingWindow(self.taste, self.top)
        self.Preparing_window.show()
        self.close()
    
    def on_click(self):
        threading.Thread(target=self.play_mp3).start()

    def play_mp3(self):
        song = AudioSegment.from_mp3(f"{uipath}/cat_like1b.mp3")
        play(song)

    def on_click_warn(self):
        threading.Thread(target=self.play_mp3_warn).start()

    def play_mp3_warn(self):
        song = AudioSegment.from_mp3(f"{uipath}/blip01.mp3")
        play(song)
        
    def open_Title_window(self):
        self.Title_window = MainWindow()
        self.Title_window.show()
        self.close()   

    def open_Direction_window(self):
        self.Direction_window = DirectionWindow()
        self.Direction_window.show()
        self.close()

    # def toggle_button(self):  
    #     button = self.sender()
    #     # 모든 버튼의 스타일을 초기화

    #     self.pushButton_1.setStyleSheet(f"""
    #         QPushButton {{
    #             background-image: url({uipath}/button/choco.png);
    #             border: 1px solid rgba(0, 0, 0, 0); /* 투명 테두리 */
    #         }}
    #     """)

    #     self.pushButton_3.setStyleSheet(f"""
    #         QPushButton {{
    #             background-image: url({uipath}/button/berry.png);
    #             border: 1px solid rgba(0, 0, 0, 0); /* 투명 테두리 */
    #     }}
    #     """)

    #     self.pushButton_2.setStyleSheet(f"""
    #         QPushButton {{
    #             background-image: url({uipath}/button/vanilla.png);
    #             border: 1px solid rgba(0, 0, 0, 0); /* 투명 테두리 */
    #     }}
    #     """)

    #     self.pushButton_6.setStyleSheet(f"""
    #         QPushButton {{
    #             background-image: url({uipath}/button/topc.png);
    #             border: 1px solid lightgray; /* 회색 테두리 */
    #         }}
    #         QPushButton:checked {{
    #             background-image: url({uipath}/button/topc.png);
    #             border: 4px solid blue; /* 파란색 */
    #         }}
    #     """)

    #     self.pushButton_4.setStyleSheet(f"""
    #         QPushButton {{
    #             background-image: url({uipath}/button/topa.png);
    #             border: 1px solid solid lightgray; /* 투명 테두리 */
    #     }}
    #     """)

    #     self.pushButton_5.setStyleSheet(f"""
    #         QPushButton {{
    #             background-image: url({uipath}/button/topb.png);
    #             border: 1px solid solid lightgray; /* 투명 테두리 */
    #     }}
    #     """)


    #     if button.isChecked():
    #         if button == self.pushButton_1:
    #             button.setStyleSheet('border: 4px solid brown')  # 초코 색
    #         elif button == self.pushButton_2:
    #             button.setStyleSheet('border: 4px solid yellow')  # 바닐라 색
    #         elif button == self.pushButton_3:
    #             button.setStyleSheet('border: 4px solid red')  # 딸기 색
    #         elif button == self.pushButton_4:
    #             button.setStyleSheet('border: 4px solid lightgray')  # 토핑 색상
    #         elif button == self.pushButton_5:
    #             button.setStyleSheet('border: 4px solid green')  # 다른 색상 예시
    #         # elif button == self.pushButton_6:
    #         #     button.setStyleSheet('border: 4px solid blue')  # 다른 색상 예시

    # 한 가지씩 선택하고 다음 눌렀을 때 재확인창
    def showMessage(self):
        message = f'선택하신 메뉴를 확인해주세요. \n아이스크림: {self.taste}\n토핑: {self.top}'
        reply = QMessageBox.question(self, 'Message', message, 
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            self.open_Preparing_window()
            print('Yes selected')
        else:
            print('No selected')

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

        # 하나씩 선택하면 DB에 넣을 거 초기화
        if ice_cream_selected:
            if self.pushButton_1.isChecked():
                self.taste = "초코"
            elif self.pushButton_2.isChecked():
                self.taste = "바닐라"
            elif self.pushButton_3.isChecked():
                self.taste = "딸기"
            
        if topping_selected:
            if self.pushButton_6.isChecked():
                self.top = "토핑A"
            elif self.pushButton_5.isChecked():
                self.top = "토핑B"
            elif self.pushButton_4.isChecked():
                self.top = "토핑C"

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
        # 맛과 토핑 전달
        self.Preparing_window = PreparingWindow(self.taste, self.top)
        self.Preparing_window.show()
        self.close()

class PreparingWindow(QMainWindow, from_class_Preparing):
    def __init__(self, taste, toping):
        super().__init__()
        self.taste = taste
        self.top = toping
        print(self.taste)
        print(self.top)
        self.setupUi(self)
        self.setStyleSheet("QMainWindow { background-color: #ADD8E6; }")   
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
        song = AudioSegment.from_mp3(f"{uipath}/cat_like1b.mp3")
        play(song)

    def open_Making_window(self):
        self.Making_window = MakingWindow()
        self.Making_window.show()
        self.close()

class MakingWindow(QMainWindow, from_class_Making):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setStyleSheet("QMainWindow { background-color: #ADD8E6; }")   

        # QLabel에 GIF 설정
        self.movie = QMovie("loading.gif")
        if self.movie.isValid():
            self.label_pic.setMovie(self.movie)
            self.movie.start()
        else:
            print("GIF를 로드할 수 없습니다.")

        # 쓰레기 처리, 아이스크림 대기 완료 과정을 3초로 임시 가정 -> 뒤로가기로 인한 버그로 인해 버튼으로 대체
        QTimer.singleShot(1000, self.open_Maked_window)

    def open_Maked_window(self):
        self.Maked_window = MakedWindow()
        self.Maked_window.show()
        self.close()

class MakedWindow(QMainWindow, from_class_Maked):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setStyleSheet("QMainWindow { background-color: #ADD8E6; }")   

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
        self.setStyleSheet("QMainWindow { background-color: #ADD8E6; }")   
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
        song = AudioSegment.from_mp3(f"{uipath}/cat_like1b.mp3")
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
        self.setStyleSheet("QMainWindow { background-color: #ADD8E6; }")   
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
        song = AudioSegment.from_mp3(f"{uipath}/cat_like1b.mp3")
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
        self.setStyleSheet("QMainWindow { background-color: #ADD8E6; }")   

if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())










##### 완료

## Recommend_kor.ui에 음성인식과 연결할만한 라벨 추가
## Recommend 전에 qmessage box에 예, 아니오
## 음성 2번 인식 못하면 에러 표시 1번 -> "다시 말해주세요."
# "중복되었습니다."
# ui 화면 추가하기
# 
## Recommend와 Preparing 사이 warning box 표시
# 제조중 박스 두개 카운트, 그리고 색깔 변화
## gif 파일 다른 것 찾기, 자꾸 로드 오류남
