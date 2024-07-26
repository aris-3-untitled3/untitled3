import cv2
import torch
from ultralytics import YOLO

# 고정된 영역의 좌표 및 크기 (좌측으로 이동)
regions = [
    (190, 40, 90, 100),   # 첫 번째 영역
    (280, 40, 90, 100),  # 두 번째 영역
    (370, 40, 100, 100)   # 세 번째 영역
]
region_names = ['topping C', 'topping B', 'topping A']
blue = (255, 0, 0)  # 색상 값
font = cv2.FONT_HERSHEY_SIMPLEX

# YOLOv8 모델 파일의 경로 지정
model_path = '/home/jchj/Untitled3/src/AI_models/container_sealing.pt'
# YOLOv8 모델 로드
model = YOLO(model_path)

# 웹캠 캡처 초기화
cap = cv2.VideoCapture(1)  # 웹캠 인덱스 설정

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    img = frame.copy()  # 현재 프레임을 이미지로 저장

    # YOLOv8 모델로 객체 감지
    results = model(img)

    detected_in_region = False  # 객체가 감지된 영역 플래그

    # 각 고정된 영역에 사각형 그림 그리기 및 번호 표시
    for i, (x0, y0, w, h) in enumerate(regions):
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), blue, 2)
        # topping 텍스트 추가
        cv2.putText(img, region_names[i], (x0 + 5, y0 + 15), font, 0.5, blue, 1, cv2.LINE_AA)

    # 감지된 객체 주석 추가 및 위치 판별
    for result in results:
        boxes = result.boxes.xyxy  # 탐지된 객체의 bounding box 좌표
        for box in boxes:
            x1, y1, x2, y2 = map(int, box[:4])
            cup_detected = False
            for i, (x0, y0, w, h) in enumerate(regions):
                if x1 >= x0 and y1 >= y0 and x2 <= (x0 + w) and y2 <= (y0 + h):
                    print(region_names[i])
                    detected_in_region = True
                    cup_detected = True
                    break
            if cup_detected:
                break
        img = result.plot()  # YOLOv8은 plot 메서드를 제공하여 이미지를 주석 처리함

    if not detected_in_region:
        print("no detect")

    cv2.imshow('frame', img)  # 현재 프레임을 화면에 표시

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # Esc 키를 누르면 종료
        break

cap.release()
cv2.destroyAllWindows()