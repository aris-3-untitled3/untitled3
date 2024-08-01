import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from untitled_msgs.srv import ServiceString
import cv2
from ultralytics import YOLO
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import numpy as np

class Container_Sealing(Node):
    def __init__(self):
        super().__init__('Container_Sealing')

        # 콜백 그룹 정의
        self.image_callback_group = ReentrantCallbackGroup()
        self.CS_callback_group = ReentrantCallbackGroup()
        
        # YOLOv8 모델 파일의 경로 지정
        model_path = '/home/jchj/Untitled3/src/AI_models/CS.pt'
        self.model = YOLO(model_path)
        
        # WebCam Topic subscribe
        self.FrontCam_subscription = self.create_subscription(
            Image,
            '/UpCam',
            self.Webcam_callback,
            10,
            callback_group=self.image_callback_group
        )
        
        # 서비스 서버 생성
        self.CS = self.create_service(
            ServiceString,
            '/Call_to_CS',
            self.Container_Sealing_callback,
            callback_group=self.CS_callback_group
        )

        self.bridge = CvBridge()
        self.frame = None
        self.adjusted_img = None

        # 채도 및 명도 조절
        self.saturation = 70  # 기본값, 필요에 따라 조절
        self.brightness = 30  # 기본값, 필요에 따라 조절

        # 박스 상태 유지
        self.detection_boxes = []  # (x1, y1, x2, y2, class_name, timestamp)

        # # Start the display frames thread
        # threading.Thread(target=self.display_frames, daemon=True).start()

        # 고정된 영역의 좌표 및 크기
        self.regions = [
            (170, 40, 100, 100),   # 첫 번째 영역
            (270, 40, 100, 100),  # 두 번째 영역
            (370, 40, 100, 100)   # 세 번째 영역
        ]
        self.region_names = ['topping C', 'topping B', 'topping A']
        self.blue = (255, 0, 0)  # 색상 값
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.detection_duration = 5  # 박스를 화면에 표시할 시간(초)

        # 각 영역에서의 감지 횟수를 저장할 딕셔너리
        self.region_detection_counts = {name: 0 for name in self.region_names}

    def Webcam_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error in Webcam_callback: {str(e)}')

    def display_frames(self):
        while rclpy.ok():
            if self.adjusted_img is not None:
                # 박스 그리기
                display_img = self.adjusted_img.copy()
                current_time = time.time()
                self.detection_boxes = [box for box in self.detection_boxes if (current_time - box[5]) < self.detection_duration]
                
                for (x1, y1, x2, y2, class_name, _) in self.detection_boxes:
                    color = (0, 255, 0) if class_name == "container" else (0, 0, 255)
                    label = f'{class_name}'
                    display_img = cv2.rectangle(display_img, (x1, y1), (x2, y2), color, 2)
                    display_img = cv2.putText(display_img, label, (x1, y1 - 10), self.font, 0.5, color, 2)

                # 고정된 영역 사각형 및 이름 표시
                for i, (x0, y0, w, h) in enumerate(self.regions):
                    cv2.rectangle(display_img, (x0, y0), (x0 + w, y0 + h), self.blue, 2)
                    cv2.putText(display_img, f'{self.region_names[i]} ({self.region_detection_counts[self.region_names[i]]})',
                                (x0 + 5, y0 + 15), self.font, 0.5, self.blue, 1, cv2.LINE_AA)

                cv2.imshow("Frame", display_img)
            elif self.frame is not None:
                # cv2.imshow("Frame", self.frame)
                continue
            else:
                # self.get_logger().info("No frame available")
                continue

            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break


    def adjust_saturation_and_brightness(self, image, saturation_scale=1.0, brightness_offset=0):
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

    def Container_Sealing_detect_capsulet(self):
        if self.frame is None:
            self.get_logger().warning("No frame available for detection")
            return "none"

        start_time = time.time()
        detection_limit = 30  # 감지 카운트 제한

        ice_cream_bucket_count = 0

        # 각 영역에서 객체 감지 여부 초기화
        self.region_detection_counts = {name: 0 for name in self.region_names}

        while (time.time() - start_time) < 30:  # 60초 동안 감지
            if self.frame is None:
                time.sleep(0.1)  # 프레임이 없을 경우 잠시 대기
                continue

            self.adjusted_img = self.adjust_saturation_and_brightness(self.frame, saturation_scale=self.saturation, brightness_offset=self.brightness)

            results = self.model(self.adjusted_img)
            detected_in_region = False  # 객체가 감지된 영역 플래그

            # 감지된 객체 주석 추가 및 위치 판별
            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy()  # 감지된 박스 정보
                names = result.names  # 클래스 이름

                for box, cls in zip(boxes, result.boxes.cls):
                    class_name = names[int(cls)]
                    x1, y1, x2, y2 = map(int, box)  # Convert box coordinates to integers

                    # Check if the detected object is inside any of the predefined regions
                    for i, (x0, y0, w, h) in enumerate(self.regions):
                        if x1 >= x0 and y1 >= y0 and x2 <= (x0 + w) and y2 <= (y0 + h):
                            # Object is within the predefined region
                            detected_in_region = True
                            self.region_detection_counts[self.region_names[i]] += 1  # Increment detection count for the region
                            self.detection_boxes.append((x1, y1, x2, y2, class_name, time.time()))
                            break
                    else:
                        # Continue to the next box if the current one is not in any region
                        continue

                    # Process detection if inside a region
                    if class_name == "banana":
                        ice_cream_bucket_count += 1
                        self.get_logger().info(f'Detected: {class_name} (ice_cream_bucket Count: {ice_cream_bucket_count})')

                        if ice_cream_bucket_count >= detection_limit:
                            self.get_logger().info(f'ice_cream_bucket detection limit reached ({ice_cream_bucket_count} times)')
                            return self.region_names[i]
                    elif class_name == "chocolate":
                        ice_cream_bucket_count += 1
                        self.get_logger().info(f'Detected: {class_name} (ice_cream_bucket Count: {ice_cream_bucket_count})')

                        if ice_cream_bucket_count >= detection_limit:
                            self.get_logger().info(f'ice_cream_bucket detection limit reached ({ice_cream_bucket_count} times)')
                            return self.region_names[i]

            if not detected_in_region:
                self.get_logger().info("No objects detected in predefined regions")

        self.get_logger().info(f'Detection completed: {ice_cream_bucket_count} ice_cream_bucket')
        return "none"

    def Container_Sealing_detect_sealing(self):
        if self.frame is None:
            self.get_logger().warning("No frame available for detection")
            return "none"

        start_time = time.time()
        detection_limit = 30  # 감지 카운트 제한

        sealing_count = 0

        while (time.time() - start_time) < 30:  # 60초 동안 감지
            if self.frame is None:
                time.sleep(0.1)  # 프레임이 없을 경우 잠시 대기
                continue

            self.adjusted_img = self.adjust_saturation_and_brightness(self.frame, saturation_scale=self.saturation, brightness_offset=self.brightness)

            results = self.model(self.adjusted_img)
            detected_in_region = False  # 객체가 감지된 영역 플래그

            # 감지된 객체 주석 추가 및 위치 판별
            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy()  # 감지된 박스 정보
                names = result.names  # 클래스 이름

                for box, cls in zip(boxes, result.boxes.cls):
                    class_name = names[int(cls)]
                    x1, y1, x2, y2 = map(int, box)  # Convert box coordinates to integers

                    # Process detection
                    if class_name == "seal":
                        sealing_count += 1
                        self.detection_boxes.append((x1, y1, x2, y2, class_name, time.time()))  # Add to detection boxes
                        self.get_logger().info(f'Detected: {class_name} (Sealing Count: {sealing_count})')

                        if sealing_count >= detection_limit:
                            self.get_logger().info(f'Sealing detection limit reached ({sealing_count} times)')
                            return "sealing"

            if not detected_in_region:
                self.get_logger().info("No objects detected")

        self.get_logger().info(f'Detection completed: {sealing_count} sealings')
        return None


    def Container_Sealing_callback(self, request, response):
        self.get_logger().info(f'Starting detection with command: {request.command}')

        if request.command == "start_capsulet":
            self.get_logger().info(f'Received: {request.command}')
            result = self.Container_Sealing_detect_capsulet()
            response.result = f'result: {result}'
        elif request.command == "start_sealing":
            self.get_logger().info(f'Received: {request.command}')
            result = self.Container_Sealing_detect_sealing()
            response.result = f'result: {result}'
        else:
            self.get_logger().error(f'ERROR')
            response.result = 'Invalid command'

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    CS_node = Container_Sealing()

    # MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(CS_node)

    try:
        executor.spin()
    finally:
        CS_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
