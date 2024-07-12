#A Gender and Age Detection program by Mahesh Sawant

import cv2
import math
import argparse
import time
import numpy as np

def highlightFace(net, frame, conf_threshold=0.7):
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

parser = argparse.ArgumentParser()
parser.add_argument('--image')

args = parser.parse_args()

faceProto = "opencv_face_detector.pbtxt"
faceModel = "opencv_face_detector_uint8.pb"
ageProto = "age_deploy.prototxt"
ageModel = "age_net.caffemodel"
genderProto = "gender_deploy.prototxt"
genderModel = "gender_net.caffemodel"

MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
#ageList = ['(0-2)', '(4-6)', '(8-12)', '(15-20)', '(25-32)', '(38-43)', '(48-53)', '(60-100)']
ageList = ['(0-2)', '(3-6)', '(7-12)', '(13-18)', '(19-29)', '(30-43)', '(44-49)', '(50-100)']
#['0-6', '7-18', '19-29', '30-49', '50-']

genderList = ['Male', 'Female']

faceNet = cv2.dnn.readNet(faceModel, faceProto)
ageNet = cv2.dnn.readNet(ageModel, ageProto)
genderNet = cv2.dnn.readNet(genderModel, genderProto)

video = cv2.VideoCapture(args.image if args.image else 0)
padding = 20

INFO_UPDATE_DURATION = 3  # 업데이트 시간
face_detected_time = None 

# # Define dictionaries for gender and age counts
Most_Gender = {gender: 0 for gender in genderList}
Most_Age = {age: 0 for age in ['0-6', '7-18', '19-29', '30-49', '50-']}

while True:
    hasFrame, frame = video.read()
    frame = cv2.flip(frame, 1)
    if not hasFrame:
        print("NO webcam")
        break
    
    resultImg, faceBoxes = highlightFace(faceNet, frame)

    if not faceBoxes:
        print("No face detected")
        face_detected_time = None
        Most_Gender = {gender: 0 for gender in genderList}
        Most_Age = {age: 0 for age in ['0-6', '7-18', '19-29', '30-49', '50-']}

    else:
        current_time = time.time()
        if face_detected_time is None:
            face_detected_time = current_time  # 얼굴이 처음 인식된 시간을 기록

        if current_time - face_detected_time <= INFO_UPDATE_DURATION:
            for faceBox in faceBoxes:
                face = frame[max(0, faceBox[1] - padding):
                             min(faceBox[3] + padding, frame.shape[0] - 1), max(0, faceBox[0] - padding)
                             :min(faceBox[2] + padding, frame.shape[1] - 1)]

                blob = cv2.dnn.blobFromImage(face, 1.0, (227, 227), MODEL_MEAN_VALUES, swapRB=False)

                genderNet.setInput(blob)
                genderPreds = genderNet.forward()
                gender = genderList[genderPreds[0].argmax()]

                if gender in Most_Gender:
                    Most_Gender[gender] += 1

                ageNet.setInput(blob)
                agePreds = ageNet.forward()
                age = ageList[agePreds[0].argmax()]

                #['(0-2)', '(3-6)', '(7-12)', '(13-18)', '(19-29)', '(30-43)', '(44-49)', '(50-100)']
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

most_common_gender = max(Most_Gender, key=Most_Gender.get)
most_common_age = max(Most_Age, key=Most_Age.get)

print(f'Gender dictionary = {Most_Gender}')
print(f'Age dictionary = {Most_Age}')

print(f'-----------------------------------------최종 데이터 값 | Age is {most_common_age} , Gender is {most_common_gender} | -----------------------------------------')

video.release()
cv2.destroyAllWindows()
