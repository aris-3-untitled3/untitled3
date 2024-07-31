import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from untitled_msgs.srv import ServiceString
import cv2
import numpy as np
import mediapipe as mp
import pyautogui
import math
import time
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from collections import Counter

class HandDetector(Node):
    def __init__(self, detectionCon=0.7, maxHands=2, modelComplexity=1, trackCon=0.5):
        super().__init__('HandDetector')
        self.lmList = []
        self.results = None
        self.mode = False
        self.maxHands = maxHands
        self.modelComplex = modelComplexity
        self.detectionCon = detectionCon
        self.trackCon = trackCon
        self.pTime = 0
        
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex, self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]
    
        self.wScr, self.hScr = pyautogui.size()

        self.previous_state = 0
        self.current_state = 0
        self.previous_thumb = 0
        self.current_thumb = 0

        self.frame = None
        self.adjusted_img = None
        self.bridge = CvBridge()

        # Define callback groups
        self.image_callback_group = ReentrantCallbackGroup()
        self.server_callback_group = ReentrantCallbackGroup()

        # Start the display frames thread
        threading.Thread(target=self.display_frames, daemon=True).start()

        # WebCam Topic subscribe
        self.FrontCam_subscription = self.create_subscription(
            Image,
            '/FrontCam',
            self.Webcam_callback,
            10,
            callback_group=self.image_callback_group
        )

        # 서비스 서버 생성
        self.HM = self.create_service(
            ServiceString,
            '/Call_to_HM',
            self.HM_callback,
            callback_group=self.server_callback_group
        )

    def HM_callback(self, request, response):
        self.get_logger().info(f'Starting detection with command: {request.command}')

        if request.command == "start":
            self.get_logger().info(f'Received: {request.command}')
            result = self.run()
            response.result = f'result: {result}'
        else:
            self.get_logger().error(f'ERROR')
            response.result = 'Invalid command'

        response.success = True
        return response
    
    def display_frames(self):
        while True:
            if self.frame is not None:
                cv2.imshow("Frame", self.frame)
            else:
                if self.adjusted_img is not None:
                    cv2.imshow("Frame", self.adjusted_img)
                else:
                    self.get_logger().info("No frame available") 

            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break

    def Webcam_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error in Webcam_callback: {str(e)}')

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
        start_time = time.time()
        thumb_states = []
        
        while time.time() - start_time < 3:
            if self.frame is not None:
                img = self.findHands(self.frame)
                lmList, bbox = self.findPosition(img)
                
                if len(lmList) != 0:
                    sub, img, lineInfo = self.findSub(3, 4, img)
                    self.current_thumb = self.thumbUp(sub)

                    thumb_states.append(self.current_thumb)

        if thumb_states:
            most_common = Counter(thumb_states).most_common(1)[0][0]
            if most_common == 1:
                return "맛있어요"
            elif most_common == 2:
                return "맛없어요"
            elif most_common == 3:
                return "그저 그래요"
        else:
            return "상태 없음"

def main(args=None):
    rclpy.init(args=args)

    HM_node = HandDetector()

    # MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(HM_node)

    try:
        executor.spin()
    finally:
        HM_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()