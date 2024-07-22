import cv2
import mediapipe as mp
from PyQt5.QtCore import QThread, pyqtSignal, QMutex, QMutexLocker
import time

class MediaPipeThread(QThread):
    frame_signal = pyqtSignal(object)  # 카메라 프레임 신호
    hand_wave_signal = pyqtSignal()  # 손 인사 신호 (~2m안)
    point_left_signal = pyqtSignal()  # 왼쪽 가리키기 신호
    point_right_signal = pyqtSignal()  # 오른쪽 가리키기 신호
    number_signal = pyqtSignal(int)  # 숫자 인식 신호

    def __init__(self, camera_index=0):
        super().__init__()
        self.running = True
        self.mutex = QMutex()
        self.camera_index = camera_index  # 카메라 인덱스 초기화

    def run(self):
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            print(f"Error: 카메라를 열 수 없습니다. 인덱스: {self.camera_index}")
            return

        mpHands = mp.solutions.hands
        hands = mpHands.Hands()
        mpDraw = mp.solutions.drawing_utils

        prev_cx, prev_cy = None, None
        move_threshold = 15  # 손이 움직인 거리의 임계값
        move_count = 0
        move_count_threshold = 10  # 손을 흔드는 것으로 인식할 때까지의 움직임 횟수

        pointing_start_time = None
        pointing_duration_threshold = 1  # 1초 이상 지속될 때 인식
        pointing_direction = None

        while self.running:
            success, img = cap.read()
            if not success:
                print("Error: 카메라 프레임을 읽을 수 없습니다.")
                continue

            imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = hands.process(imgRGB)

            if results.multi_hand_landmarks:
                for handLms in results.multi_hand_landmarks:
                    h, w, _ = img.shape
                    landmarks = [lm for lm in handLms.landmark]

                    # 손 인사 감지
                    cx, cy = int(landmarks[12].x * w), int(landmarks[12].y * h)
                    if prev_cx is not None and prev_cy is not None:
                        move_distance = abs(cx - prev_cx)
                        if move_distance > move_threshold:
                            move_count += 4
                        else:
                            move_count = max(0, move_count - 1)

                        if move_count >= move_count_threshold:
                            self.hand_wave_signal.emit()  # 손 인사 신호 보내기
                            move_count = 0

                    prev_cx, prev_cy = cx, cy

                    # 손가락 제스처 인식 (1초 이상 지속)
                    current_time = time.time()
                    if self.is_pointing_left(landmarks):
                        if pointing_direction != 'left':
                            pointing_start_time = current_time
                            pointing_direction = 'left'
                        elif pointing_start_time is not None and current_time - pointing_start_time >= pointing_duration_threshold:
                            self.point_left_signal.emit()
                            pointing_start_time = None
                    elif self.is_pointing_right(landmarks):
                        if pointing_direction != 'right':
                            pointing_start_time = current_time
                            pointing_direction = 'right'
                        elif pointing_start_time is not None and current_time - pointing_start_time >= pointing_duration_threshold:
                            self.point_right_signal.emit()
                            pointing_start_time = None
                    else:
                        pointing_start_time = None
                        pointing_direction = None

                    # 숫자 인식 (1초 이상 지속)
                    raised_fingers = self.count_raised_fingers(landmarks)
                    if raised_fingers in [1, 2, 3]:
                        if pointing_direction != f'number_{raised_fingers}':
                            pointing_start_time = current_time
                            pointing_direction = f'number_{raised_fingers}'
                        elif pointing_start_time is not None and current_time - pointing_start_time >= pointing_duration_threshold:
                            self.number_signal.emit(raised_fingers)
                            pointing_start_time = None

                    mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

            with QMutexLocker(self.mutex):
                if self.running:
                    self.frame_signal.emit(img)

        cap.release()

    def stop(self):
        with QMutexLocker(self.mutex):
            self.running = False
        self.wait()

    def is_pointing_left(self, landmarks):
        # 검지 손가락 끝이 중간 관절보다 왼쪽에 있는지 확인
        return landmarks[8].x < landmarks[6].x

    def is_pointing_right(self, landmarks):
        # 검지 손가락 끝이 중간 관절보다 오른쪽에 있는지 확인
        return landmarks[8].x > landmarks[6].x

    def count_raised_fingers(self, landmarks):
        # 각 손가락이 펴져 있는지 확인
        fingers = []
        fingers.append(landmarks[8].y < landmarks[6].y)  # 검지
        fingers.append(landmarks[12].y < landmarks[10].y)  # 중지
        fingers.append(landmarks[16].y < landmarks[14].y)  # 약지
        fingers.append(landmarks[20].y < landmarks[18].y)  # 새끼
        return fingers.count(True)


