import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QDialog
from PyQt5 import uic
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import QtGui
from PyQt5.QtCore import QTimer

import os
import time

ui_file = os.path.join('/home/messi/ws_amr/qt/', "Title.ui")
ui_file2 = os.path.join('/home/messi/ws_amr/qt/', "Loading.ui")
ui_Direction = os.path.join('/home/messi/ws_amr/qt/', "Direction.ui")
ui_Recommend = os.path.join('/home/messi/ws_amr/qt/', "Recommend_kor.ui")
ui_Preparing = os.path.join('/home/messi/ws_amr/qt/', "Preparing.ui")

from_class = uic.loadUiType(ui_file)[0]
from_class2 = uic.loadUiType(ui_file2)[0]
from_class_Direction = uic.loadUiType(ui_Direction)[0]
from_class_Recommend = uic.loadUiType(ui_Recommend)[0]
from_class_Preparing = uic.loadUiType(ui_Preparing)[0]

class MainWindow(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.pushButton.clicked.connect(self.open_loading_window)

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
        #time.sleep(3)
        #self.open_loading_window_Direction(self)
        QTimer.singleShot(1000, self.open_DirectionWindow)

    def open_DirectionWindow(self):
        self.DirectionWindow = DirectionWindow()
        self.DirectionWindow.show()
        self.close()

class DirectionWindow(QMainWindow, from_class_Direction):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.pushButton.clicked.connect(self.open_Recommend_window)

    ## 연결!
    def open_Recommend_window(self):
        self.Recommend_window = RecommendWindow()
        self.Recommend_window.show()
        self.close()

class RecommendWindow(QMainWindow, from_class_Recommend):
    def __init__(self):
        super().__init__()
        self.setupUi(self)


# 맛 추천 표시 두 가지 방법 중 선택하기
        # 추천1: Text에 전달 
        recommended_flavor = "초코"
        recommended_topping = "토핑A"
        self.label_2.setText(f"손님께 추천드리는 맛과 토핑은 {recommended_flavor}와 {recommended_topping}입니다.")
        

        # 추천2: 맛과 토핑을 작은 메시지박스로 전달
        QMessageBox.information(self, '추천 맛과 토핑', '손님의 연령대와 성별에 맞는\n 맛과 토핑을 추천드립니다. \n 추천 맛: 초코\n 추천 토핑: A')

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

        # Next 버튼 이벤트 연결
        self.pushButton.clicked.connect(self.check_selection_and_next)



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
        
        #  아이스크림 맛이 하나가 선택되었는지 확인!
        self.bt1 = self.pushButton_1.isChecked()
        self.bt2 = self.pushButton_2.isChecked()
        self.bt3 = self.pushButton_3.isChecked()
        self.bt4 = self.pushButton_4.isChecked()
        self.bt5 = self.pushButton_5.isChecked()
        self.bt6 = self.pushButton_6.isChecked()


        self.iceCream_pickOne = ( (self.bt1 == 1 and self.bt2 == 0 and self.bt3 == 0) or
                 (self.bt1 == 0 and self.bt2 == 1 and self.bt3 == 0) or
                 (self.bt1 == 0 and self.bt2 == 0 and self.bt3 == 1) )
        self.toping_pickOne = ( (self.bt4 == 1 and self.bt5 == 0 and self.bt6 == 0) or
                 (self.bt4 == 0 and self.bt5 == 1 and self.bt6 == 0) or
                 (self.bt4 == 0 and self.bt5 == 0 and self.bt6 == 1))

        

        if not ( (self.bt1 == 1 and self.bt2 == 0 and self.bt3 == 0) or
                 (self.bt1 == 0 and self.bt2 == 1 and self.bt3 == 0) or
                 (self.bt1 == 0 and self.bt2 == 0 and self.bt3 == 1)):
            QMessageBox.warning(self, 'Warning', '아이스크림 맛을 한 가지만 골라주세요.')
            

         # 토핑 하나가 선택되었는지 확인!
        if not ( (self.bt4 == 1 and self.bt5 == 0 and self.bt6 == 0) or
                 (self.bt4 == 0 and self.bt5 == 1 and self.bt6 == 0) or
                 (self.bt4 == 0 and self.bt5 == 0 and self.bt6 == 1)):
            QMessageBox.warning(self, 'Warning', '토핑을 한 가지만 골라주세요.')

        # 문제점: Next를 두 번 눌러야 다음 UI가 진행됨
        if(self.iceCream_pickOne == 1 and self.toping_pickOne == 1):
            self.open_Preparing_window()

    def open_Preparing_window(self):
        self.Preparing_window = PreparingWindow()
        self.Preparing_window.show()
        self.close()

class PreparingWindow(QMainWindow, from_class_Preparing):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
    



if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
