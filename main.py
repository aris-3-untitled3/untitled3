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

from_class = uic.loadUiType(ui_file)[0]
from_class2 = uic.loadUiType(ui_file2)[0]
from_class_Direction = uic.loadUiType(ui_Direction)[0]
from_class_Recommend = uic.loadUiType(ui_Recommend)[0]

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
        QTimer.singleShot(3000, self.open_DirectionWindow)

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

        



if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
