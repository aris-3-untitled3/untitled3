import cv2
import torch
import multiprocessing
import time
human_detecting = 0

def yolo_distance_estimation():
    # YOLOv5 모델 로드
    model_path = '/home/messi/Downloads/yolo_test/Age-Gender-Prediction-main/Models/YOLO/Best.pt'
    model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)

    # 웹캡 캡처 초기화 (노트북 그 뭐냐 내장 말고 웹캠 사용)
    cap = cv2.VideoCapture(2)

    def estimate_distance(bbox_height):
        # 카메라의 특정 초점 거리 및 인식된 객체의 실제 높이를 기반으로 거리 추정
        KNOWN_HEIGHT = 1.7  # 사람의 평균 높이(미터)
        FOCAL_LENGTH = 50  # 계산된 카메라의 초점 거리(픽셀 단위로 추정)
        distance = (KNOWN_HEIGHT * FOCAL_LENGTH) / bbox_height
        return distance

    while True:
        ret, frame = cap.read()  # 웹캠에서 프레임 읽기 
        if not ret:
            print("웹캠에서 프레임을 읽을 수 없습니다.")
            break  # 프레임 읽기 실패 시 루프 종료
        
        # YOLOv5 모델로 객체 감지       
        results = model(frame)
        
        # 감지된 객체 주석 추가
        annotated_frame = results.render()[0]
        # 사람의 거리 추정
        for det in results.xyxy[0]:  # 각 검출된 객체에 대해
            x1, y1, x2, y2, conf, cls = det
            if int(cls) == 0:  # 클래스가 '사람'인 경우
                bbox_height = y2 - y1
                distance = estimate_distance(bbox_height)
                cv2.putText(annotated_frame, f'Distance: {distance:.2f}m', (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                
                # 0.5m 내에 사람이 인지되면 터미널에 메시지 출력
                if distance <= 0.5:
                    human_detecting = 1
                    print(1)
                # 0.5 ~ 2m 내에 사람이 인지되면 인사하는 동작
                elif distance > 0.5 and distance <= 2:
                    human_detecting = 2
                    print(2)
                # 그보다 멀면 다른 동작
                else:
                    human_detecting = 0
                    print(0)
        


        # 화면에 주석이 달린 프레임 표시
        cv2.imshow('Webcam', annotated_frame)
        
        if cv2.waitKey(1) == ord('q'):  # 'q' 키를 누르면 루프 종료
            break

    cap.release()  # 웹캠 자원 해제
    cv2.destroyAllWindows()  # 모든 OpenCV 창 닫기

if __name__ == '__main__':
    process1 = multiprocessing.Process(target=yolo_distance_estimation)
    #process2 = multiprocessing.Process(target=yolo_object_detection)

    process1.start()
    #process2.start()

    process1.join()
    #process2.join()


