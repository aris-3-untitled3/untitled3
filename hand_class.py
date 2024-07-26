import cv2
import numpy as np
import mediapipe as mp
import pyautogui
import math
import time

class HandMouseControl:
    def __init__(self, detectionCon=0.7, maxHands=2, modelComplexity=1, trackCon=0.5, wCam=640, hCam=480):
        self.wCam, self.hCam = wCam, hCam
        self.frameR = 100
        self.smoothening = 5
        self.pTime = 0
        self.plocX, self.plocY = 0, 0
        self.clocX, self.clocY = 0, 0

        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, self.wCam)
        self.cap.set(4, self.hCam)
        
        self.detector = self.HandDetector(detectionCon, maxHands, modelComplexity, trackCon)
        self.wScr, self.hScr = pyautogui.size()

        self.previous_state = 0
        self.current_state = 0
        self.previous_thumb = 0
        self.current_thumb = 0

    class HandDetector:
        def __init__(self, detectionCon, maxHands, modelComplexity, trackCon):
            self.lmList = []
            self.results = None
            self.mode = False
            self.maxHands = maxHands
            self.modelComplex = modelComplexity
            self.detectionCon = detectionCon
            self.trackCon = trackCon
            
            self.mpHands = mp.solutions.hands
            self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex, self.detectionCon, self.trackCon)
            self.mpDraw = mp.solutions.drawing_utils
            self.tipIds = [4, 8, 12, 16, 20]

        def findHands(self, img, draw=True):
            rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            self.results = self.hands.process(rgb_img)
            if self.results.multi_hand_landmarks:
                for handLms in self.results.multi_hand_landmarks:
                    if draw:
                        self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
            return img

        def findPosition(self, img, handNo=0, draw=True):
            xList = []
            yList = []
            bbox = []
            self.lmList = []
            
            if self.results.multi_hand_landmarks:
                myHand = self.results.multi_hand_landmarks[handNo]
                
                for id, lm in enumerate(myHand.landmark):
                    h, w, c = img.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    xList.append(cx)
                    yList.append(cy)
                    self.lmList.append([id, cx, cy])
                    
                    if draw:
                        cv2.circle(img, (cx, cy), 6, (0, 0, 255), cv2.FILLED)
                        
                xmin, xmax = min(xList), max(xList)
                ymin, ymax = min(yList), max(yList)
                bbox = xmin, ymin, xmax, ymax
                
                if draw:
                    cv2.rectangle(img, (bbox[0] - 20, bbox[1] - 20), (bbox[2] + 20, bbox[3] + 20), (0, 255, 0), 2)
                    
            return self.lmList, bbox

        def fingersUp(self):
            fingers = []
            if self.lmList[self.tipIds[0]][1] < self.lmList[self.tipIds[0] - 1][1]:
                fingers.append(1)
            else:
                fingers.append(0)
            for id in range(1, 5):
                if self.lmList[self.tipIds[id]][2] < self.lmList[self.tipIds[id] - 2][2]:
                    fingers.append(1)
                else:
                    fingers.append(0)
            return fingers

        def findDistance(self, p1, p2, img, draw=True):
            x1, y1 = self.lmList[p1][1:]
            x2, y2 = self.lmList[p2][1:]
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            if draw:
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 255), 3)
                cv2.circle(img, (x1, y1), 6, (0, 255, 255), cv2.FILLED)
                cv2.circle(img, (x2, y2), 6, (0, 255, 255), cv2.FILLED)
                cv2.circle(img, (cx, cy), 6, (0, 255, 255), cv2.FILLED)

            length = math.hypot(x2 - x1, y2 - y1)

            return length, img, [x1, y1, x2, y2, cx, cy]

        def findAngle(self, p1, p2, p3, img, draw=True):
            x1, y1 = self.lmList[p1][1:]
            x2, y2 = self.lmList[p2][1:]
            x3, y3 = self.lmList[p3][1:]
            angle = math.degrees(math.atan2(y3 - y2, x3 - x2) - math.atan2(y1 - y2, x1 - x2))
            if angle < 0:
                angle += 360

            if draw:
                cv2.line(img, (x1, y1), (x2, y2), (255, 255, 255), 3)
                cv2.line(img, (x3, y3), (x2, y2), (255, 255, 255), 3)
                cv2.circle(img, (x1, y1), 15, (0, 0, 255), cv2.FILLED)
                cv2.circle(img, (x1, y1), 15, (0, 0, 255), cv2.FILLED)
                cv2.circle(img, (x2, y2), 15, (0, 0, 255), cv2.FILLED)

            return angle

        def thumbUp(self, sub):
            if sub > 20:
                return 1
            elif sub < -20:
                return 2
            elif sub <= 20 and sub >= -20:
                return 3
            else:
                return 0

        def findSub(self, p1, p2, img, draw=True):
            x1, y1 = self.lmList[p1][1:]
            x2, y2 = self.lmList[p2][1:]
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            if draw:
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 255), 3)
                cv2.circle(img, (x1, y1), 6, (0, 255, 255), cv2.FILLED)
                cv2.circle(img, (x2, y2), 6, (0, 255, 255), cv2.FILLED)
                cv2.circle(img, (cx, cy), 6, (0, 255, 255), cv2.FILLED)

            sub = y1 - y2
            return sub, img, [x1, y1, x2, y2, cx, cy]

    def run(self):
        while True:
            success, img = self.cap.read()
            img = self.detector.findHands(img)
            lmList, bbox = self.detector.findPosition(img)
            output = img.copy()

            if len(lmList) != 0:
                fingers = self.detector.fingersUp()
                sub, img, lineInfo = self.detector.findSub(3, 4, img)
                cv2.circle(img, (lineInfo[4], lineInfo[5]), 6, (0, 255, 0), cv2.FILLED)
                self.current_thumb = self.detector.thumbUp(sub)

                if self.current_thumb == 1:
                    print(" 맛있어요.")
                elif self.current_thumb == 2:
                    print(" 맛없어요.")
                elif self.current_thumb == 3:
                    print(" 그저 그래요")

                count_up = 0
                for i in range(1, 5):
                    if fingers[i] == 1:
                        count_up += 1

                if count_up >= 4:
                    print(" 보 !")
                elif count_up >= 2 and count_up <= 3 or fingers[1] == 1:
                    print(" 가위 !")
                else:
                    print(" 바위 !")

                self.previous_thumb = self.current_thumb

            cTime = time.time()
            fps = 1 / (cTime - self.pTime)
            self.pTime = cTime

            cv2.imshow("Vitual mouse monitor", cv2.flip(img, 1))
            cv2.setWindowProperty("Vitual mouse monitor", cv2.WND_PROP_TOPMOST, 1)
            cv2.waitKey(1)
