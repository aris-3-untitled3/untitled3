import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from untitled_msgs.msg import TopicString
import cv2
import torch
import numpy as np
import argparse
import time
import threading

class GuestDetect(Node):
    def __init__(self):
        super().__init__('GuestDetect')

        # Robot_Server로 토픽 퍼블리셔
        self.robot_server_publisher = self.create_publisher(TopicString, '/Guest_Info', 10)

        self.model_path = '/home/jchj/Untitled3/src/models/Best.pt'
        self.model = self.load_model()

        # 상태 변수 초기화
        self.guest_detected = False
        self.human_detected = False
        self.no_human_detected = False
        self.loop_running = True  # 루프 실행 상태 변수 추가

        # Server에서 토픽 받기 (guest_detect)
        self.ui = self.create_subscription(
            TopicString,
            '/Server_to_GA',
            self.ga_callback,
            10
        )

        self.parser = argparse.ArgumentParser()
        self.parser.add_argument('--image')

        self.args = self.parser.parse_args()

        self.faceProto = "/home/jchj/Untitled3/src/models/age_gender/opencv_face_detector.pbtxt"
        self.faceModel = "/home/jchj/Untitled3/src/models/age_gender/opencv_face_detector_uint8.pb"
        self.ageProto = "/home/jchj/Untitled3/src/models/age_gender/age_deploy.prototxt"
        self.ageModel = "/home/jchj/Untitled3/src/models/age_gender/age_net.caffemodel"
        self.genderProto = "/home/jchj/Untitled3/src/models/age_gender/gender_deploy.prototxt"
        self.genderModel = "/home/jchj/Untitled3/src/models/age_gender/gender_net.caffemodel"   

        self.MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
        self.ageList = ['(0-2)', '(3-6)', '(7-12)', '(13-18)', '(19-29)', '(30-43)', '(44-49)', '(50-100)']
        self.genderList = ['M', 'F']

        # self.run()

    def load_model(self):
        return torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_path, force_reload=True)

    def estimate_distance(self, bbox_height):
        KNOWN_HEIGHT = 1.7  # 사람의 평균 높이(미터)
        FOCAL_LENGTH = 50   # 계산된 카메라의 초점 거리(픽셀 단위로 추정)
        distance = (KNOWN_HEIGHT * FOCAL_LENGTH) / bbox_height
        return distance

    def process_detections(self, detections, annotated_frame):
        closest_distance = float('inf')
        closest_bbox = None

        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            if int(cls) == 0:  # 클래스가 '사람'인 경우
                bbox_height = y2 - y1
                distance = self.estimate_distance(bbox_height)

                # 가장 가까운 사람의 거리와 바운딩 박스 저장
                if distance < closest_distance:
                    closest_distance = distance
                    closest_bbox = (x1, y1, x2, y2)

        # 가장 가까운 사람에 대해 박스 그리기 및 메시지 전송
        if closest_distance < float('inf'):
            x1, y1, x2, y2 = closest_bbox
            cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(annotated_frame, f'Distance: {closest_distance:.2f}m', (int(x1), int(y1)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
            self.detect_human(closest_distance)

    def detect_human(self, distance):
        if distance <= 0.5 and not self.guest_detected:
            self.guest_detect()
            self.guest_detected = True
            self.human_detected = False
            self.no_human_detected = False
            self.loop_running = False  # 루프 중지
        elif 0.5 < distance <= 2 and not self.human_detected:
            self.human_detect()
            self.human_detected = True
            self.guest_detected = False
            self.no_human_detected = False
        elif distance > 2 and not self.no_human_detected:
            self.no_human_detect()
            self.no_human_detected = True
            self.guest_detected = False
            self.human_detected = False

    def run(self):
        self.cap = cv2.VideoCapture(0)

        while self.loop_running:
            ret, frame = self.cap.read()
            if not ret:
                print("웹캠에서 프레임을 읽을 수 없습니다.")
                break
            
            results = self.model(frame)
            annotated_frame = results.render()[0]
            annotated_frame = np.array(annotated_frame, copy=True)

            self.process_detections(results.xyxy[0], annotated_frame)

            cv2.imshow('Webcam', annotated_frame)
            
            if cv2.waitKey(1) == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def guest_detect(self):
        self.get_logger().info('guest detected!')

        msg = TopicString()
        msg.command = 'guest_detect'

        self.robot_server_publisher.publish(msg)

    def human_detect(self):
        self.get_logger().info('human detected!')

        msg = TopicString()
        msg.command = 'human_detect'

        self.robot_server_publisher.publish(msg)

    def no_human_detect(self):
        self.get_logger().info('no human detected!')

        msg = TopicString()
        msg.command = 'no_detect'

        self.robot_server_publisher.publish(msg)

    def ga_callback(self, msg):
        if msg.command == "guest_detect":
            # 성별, 연령대 판별
            most_common_age, most_common_gender = self.run_2()
            
            # 결과 메시지 작성 및 발행  
            result_msg = TopicString()
            result_msg.command = f"Age: {most_common_age}, Gender: {most_common_gender}"
            self.robot_server_publisher.publish(result_msg)
        else:
            self.get_logger().info('Restarting detection...')
            self.loop_running = True
            self.run() # 다시 사람검출 시작


    def highlightFace(self, net, frame, conf_threshold=0.7):
        frameOpencvDnn = frame.copy()
        frameHeight = frameOpencvDnn.shape[0]
        frameWidth = frameOpencvDnn.shape[1]
        blob = cv2.dnn.blobFromImage(frameOpencvDnn, 1.0, (300, 300), [104, 117, 123], True, False)

        net.setInput(blob)
        detections = net.forward()
        faceBoxes = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > conf_threshold:
                x1 = int(detections[0, 0, i, 3] * frameWidth)
                y1 = int(detections[0, 0, i, 4] * frameHeight)
                x2 = int(detections[0, 0, i, 5] * frameWidth)
                y2 = int(detections[0, 0, i, 6] * frameHeight)
                faceBoxes.append([x1, y1, x2, y2])
                cv2.rectangle(frameOpencvDnn, (x1, y1), (x2, y2), (0, 255, 0), int(round(frameHeight / 150)), 8)
        return frameOpencvDnn, faceBoxes

    def run_2(self):
        self.faceNet = cv2.dnn.readNet(self.faceModel, self.faceProto)
        self.ageNet = cv2.dnn.readNet(self.ageModel, self.ageProto)
        self.genderNet = cv2.dnn.readNet(self.genderModel, self.genderProto)

        video = cv2.VideoCapture(self.args.image if self.args.image else 0)
        
        Most_Gender = {gender: 0 for gender in self.genderList}
        Most_Age = {age: 0 for age in ['0-6', '7-18', '19-29', '30-49', '50-']}

        retry_limit = 5
        retry_count = 0

        while retry_count < retry_limit:
            Most_Gender, Most_Age = self.loop(video, self.faceNet, self.ageNet, self.genderNet, Most_Gender, Most_Age)
            most_common_gender = max(Most_Gender, key=Most_Gender.get)
            most_common_age = max(Most_Age, key=Most_Age.get)

            gender_max_count = list(Most_Gender.values()).count(Most_Gender[most_common_gender])
            age_max_count = list(Most_Age.values()).count(Most_Age[most_common_age])

            if gender_max_count > 1 or age_max_count > 1:
                print("Warning: Multiple max values found. Retrying...")
                retry_count += 1
                Most_Gender = {gender: 0 for gender in self.genderList}
                Most_Age = {age: 0 for age in ['0-6', '7-18', '19-29', '30-49', '50-']}
            else:
                print("Detection completed")
                break

        print(f'Gender dictionary = {Most_Gender}')
        print(f'Age dictionary = {Most_Age}')

        print(f'----------------------------------------- 최종 데이터 값 | Age is {most_common_age} , Gender is {most_common_gender} | -----------------------------------------')

        if most_common_age and most_common_gender:
            with open('/home/jchj/Untitled3/src/models/age_gender/latest_age_gender.txt', 'w') as save_custom:
                save_custom.write(f'{most_common_age}\t')
                save_custom.write(f'{most_common_gender}')
            print('Success saved\n')
        else:
            print('Failed')

        video.release()
        cv2.destroyAllWindows()

        return most_common_age, most_common_gender

    def loop(self, video, faceNet, ageNet, genderNet, Most_Gender, Most_Age):
        INFO_UPDATE_DURATION = 3  # 업데이트 시간
        face_detected_time = None 
        padding = 20

        while True:
            hasFrame, frame = video.read()
            frame = cv2.flip(frame, 1)
            if not hasFrame:
                print("NO webcam")
                break
            
            resultImg, faceBoxes = self.highlightFace(faceNet, frame)

            if not faceBoxes:
                print("No face detected")
                face_detected_time = None
                Most_Gender = {gender: 0 for gender in self.genderList}
                Most_Age = {age: 0 for age in ['0-6', '7-18', '19-29', '30-49', '50-']}

            else:
                current_time = time.time()
                if face_detected_time is None:
                    face_detected_time = current_time

                if current_time - face_detected_time <= INFO_UPDATE_DURATION:
                    for faceBox in faceBoxes:
                        face = frame[max(0, faceBox[1] - padding):
                                    min(faceBox[3] + padding, frame.shape[0] - 1), max(0, faceBox[0] - padding)
                                    :min(faceBox[2] + padding, frame.shape[1] - 1)]

                        blob = cv2.dnn.blobFromImage(face, 1.0, (227, 227), self.MODEL_MEAN_VALUES, swapRB=False)

                        genderNet.setInput(blob)
                        genderPreds = genderNet.forward()
                        gender = self.genderList[genderPreds[0].argmax()]

                        if gender in Most_Gender:
                            Most_Gender[gender] += 1

                        ageNet.setInput(blob)
                        agePreds = ageNet.forward()
                        age = self.ageList[agePreds[0].argmax()]

                        age_group = ''
                        if age in ['(0-2)', '(3-6)']:
                            age_group = '0-6'
                        elif age in ['(7-12)', '(13-18)']:
                            age_group = '7-18'
                        elif age in ['(19-29)']:
                            age_group = '19-29'
                        elif age in ['(30-43)', '(44-49)']:
                            age_group = '30-49'
                        elif age in ['(50-100)']:
                            age_group = '50-'

                        if age_group in Most_Age:
                            Most_Age[age_group] += 1

                        print(f'Most Gender --> {Most_Gender} | Most Age --> {Most_Age}')
                        print(f'Age: {age_group} Gender: {gender}')

                        cv2.putText(resultImg, f'{gender}, {age_group}',
                                    (faceBox[0], faceBox[1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
                        
                        cv2.imshow("Detecting age and gender", resultImg)
                else:
                    break

            if cv2.waitKey(33) == ord('q'):
                break

        return Most_Gender, Most_Age


def main(args=None):
    rp.init(args=args)

    guest_detect_node = GuestDetect()

    try:
        rp.spin(guest_detect_node)
    finally:
        guest_detect_node.destroy_node()
        rp.shutdown()


if __name__ == '__main__':
    main()
