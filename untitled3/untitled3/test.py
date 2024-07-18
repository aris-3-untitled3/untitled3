import sys
import os
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget
from PyQt5 import uic

# UI 파일 경로 설정
ui_file = os.path.join("/home/jchj/Untitled3/src/untitled3/UI/", "Title.ui")

# UI 파일 로드
from_class = uic.loadUiType(ui_file)[0]

class MainWindow(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # 기타 초기화 작업
        self.load_image()

        self.stacked_widget = QStackedWidget(self)
        self.setCentralWidget(self.stacked_widget)

    def add(self):
        self.stacked_widget.addWidget(self)

        self.stacked_widget.setCurrentWidget(self)

    def load_image(self):
        # 필요한 경우 이미지 로드 코드 추가
        pass

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    main_window.add()
    sys.exit(app.exec_())
