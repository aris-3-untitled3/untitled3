import cv2
import time
import numpy as np
import hand_detector as hd
import pyautogui
import copy


wCam, hCam = 640, 480 # 너비, 높이
frameR = 100
smoothening = 5
### 오른쪽 위가 (0, 0)
pTime = 0
plocX, plocY = 0, 0
clocX, clocY = 0, 0

cap = cv2.VideoCapture(0)
cap.set(3, wCam) # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(4, hCam) # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
detector = hd.handDetector(detectionCon=0.7)
wScr, hScr = pyautogui.size() # 모니터 해상도 가져오기 (1920, 1080)
print('모니터 해상도는', wScr, hScr)

# 내가 만든 변수를 초기화
previous_state = 0
current_state = 0
pfingers = []

while True:
    success, img = cap.read() # success: 프레임 로드 성공여부, img: 현재 프레임
    img = detector.findHands(img) # 현재 프레임을 findHands 메서드로 보내 랜드마크가 연결된 그림을 반환받음
    lmList, bbox = detector.findPosition(img) # lmList: 각 랜드마크 좌표들, bbox: 손 전체를 둘러싸는 사각형 각 꼭짓점 좌표 
    output = img.copy() # img: 이미지를 저장하는 NumPy 배열, copy: 새로운 메모리 공간에 이미지 데이터 복사 -> 한쪽 이미지에 선을 그려도 다른 이미지에 영향을 주지 않음.

    if len(lmList) != 0:
        # print(lmList[4], lmList[8])
        x1, y1 = lmList[8][1:]  # 검지 끝 x, y
        x2, y2 = lmList[12][1:] # 중지 끝 x, y

        fingers = detector.fingersUp() # fingers[0~4]: 각 손가락이 up 돼있다면 1
        # print(fingers)
        cv2.rectangle(img, (frameR, frameR), (wCam - frameR, hCam - frameR), (205, 250, 255), -1)
        img = cv2.addWeighted(img, 0.5, output, 1 - .5, 0, output) # rectangle이 합쳐진 img와 네모가 없는 output이 0.5의 가중치씩으로 해서 합쳐짐

        # Only Index Finger : Moving Mode
        if fingers[1] == 1 and fingers[2] == 0:
            # 상태값 변화
            current_state = 1
            
            # Convert Coordinates
            x3 = np.interp(x1, (frameR, wCam - frameR), (0, wScr)) # 보간법: x1, y1을 캠 프레임과 노트북 화면 프레임 크기 비율을 통해 계산해줌.
            y3 = np.interp(y1, (frameR, hCam - frameR), (0, hScr))
            # print("x3: ", x3," y3: ", y3)

            # Smoothen Values: previous location, current location
            clocX = plocX + (x3 - plocX) / smoothening # 새로운 위치는 이전 위치와 목표 위치의 차이를 1/7로 줄여 더 부드럽게 이동
            clocY = plocY + (y3 - plocY) / smoothening
            # print("clocX: ", clocX, " clocY: ", clocY)

            # Move Mouse
            pyautogui.moveTo(wScr - clocX, clocY)
            cv2.circle(img, (x1, y1), 6, (255, 28, 0), cv2.FILLED)
            plocX, plocY = clocX, clocY
            # cv2.putText(img, 'Moving Mode', (20, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

        # V 표시일 때도 state를 만들어서 마우스 포인터 고정을 시키자.
        if fingers[1] == 1 and fingers[2] == 1:
            current_state = 2

        # Both Index and middle fingers are up : Clicking Mode
        if (fingers[1] == 0 and fingers[2] == 0 and fingers[3] == 0 and fingers[4] == 0):
            current_state = 3
            print("주먹을 쥐었습니다.")

        # 클릭 방법 2가지 에지 트리거: (1)검지에서 주먹 (2)V에서 주먹
        if (previous_state == 1 and current_state == 3) or (previous_state == 2 and current_state == 3):
            pyautogui.click()
            print("클릭됐습니다.")
            # # Find distance between fingers
            # length, img, lineInfo = detector.findDistance(8, 12, img)

            # # Click mouse if distance short
            # if length < 40:
            #     cv2.circle(img, (lineInfo[4], lineInfo[5]), 6, (0, 255, 0), cv2.FILLED)
            #     # cv2.putText(img, 'Click!!', (20, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
            #     pyautogui.click()

        previous_state = current_state


    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    cv2.imshow("Vitual mouse monitor", cv2.flip(img, 1))
    cv2.setWindowProperty("Vitual mouse monitor", cv2.WND_PROP_TOPMOST, 1)
    cv2.waitKey(1)

    ### https://yunwoong.tistory.com/93
    