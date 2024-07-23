import cv2
import torch
from ultralytics import YOLO

# YOLOv8 모델 파일의 경로 지정
model_path = '/home/jchj/Untitled3/src/models/container_sealing.pt'

# YOLOv8 모델 로드
model = YOLO(model_path)

# 웹캠 캡처 초기화 (노트북 내장 웹캠 사용)
cap = cv2.VideoCapture(0)

# 채도 및 명도 조절 함수
def adjust_saturation_and_brightness(image, saturation_scale=1.0, brightness_offset=0):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    
    # 채도 조절
    s = cv2.multiply(s, saturation_scale / 100.0)
    s = cv2.min(s, 255)
    
    # 명도 조절
    v = cv2.add(v, brightness_offset - 100)
    v = cv2.min(v, 255)
    v = cv2.max(v, 0)
    
    adjusted_hsv = cv2.merge([h, s, v])
    adjusted_image = cv2.cvtColor(adjusted_hsv, cv2.COLOR_HSV2BGR)
    return adjusted_image

# 슬라이더 콜백 함수 (필요 없지만, OpenCV 슬라이더 생성을 위해 필요)
def nothing(x):
    pass

# 윈도우 생성 및 슬라이더 추가
cv2.namedWindow('Webcam')
cv2.createTrackbar('Saturation', 'Webcam', 100, 200, nothing)  # 초기값 100, 범위 0-200
cv2.createTrackbar('Brightness', 'Webcam', 100, 200, nothing)  # 초기값 100, 범위 0-200

while True:
    ret, img = cap.read()  # 웹캠에서 프레임 읽기
    if not ret:
        print("웹캠에서 프레임을 읽을 수 없습니다.")
        break  # 프레임 읽기 실패 시 루프 종료

    # 슬라이더 값 읽기
    saturation = cv2.getTrackbarPos('Saturation', 'Webcam')
    brightness = cv2.getTrackbarPos('Brightness', 'Webcam')

    # 채도 및 명도 조절
    adjusted_img = adjust_saturation_and_brightness(img, saturation_scale=saturation, brightness_offset=brightness)
    
    # YOLOv8 모델로 객체 감지
    results = model(adjusted_img)

    # 감지된 객체 주석 추가
    for result in results:
        adjusted_img = result.plot()  # YOLOv8은 plot 메서드를 제공하여 이미지를 주석 처리함

    cv2.imshow('Webcam', adjusted_img)  # 화면에 주석이 달린 프레임 표시

    if cv2.waitKey(1) == ord('q'):  # 'q' 키를 누르면 루프 종료
        break

cap.release()  # 웹캠 자원 해제
cv2.destroyAllWindows()  # 모든 OpenCV 창 닫기