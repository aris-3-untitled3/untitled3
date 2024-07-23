import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from untitled_msgs.srv import ServiceString  # 서비스 메시지 정의에 따라 변경 필요
import cv2
from ultralytics import YOLO
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Thread

class Container_Sealing(Node):
    def __init__(self):
        super().__init__('Container_Sealing')

        # 콜백 그룹 정의
        self.image_callback_group = ReentrantCallbackGroup()
        self.CS_callback_group = ReentrantCallbackGroup()
        
        # YOLOv8 모델 파일의 경로 지정
        model_path = '/home/jchj/Untitled3/src/models/container_sealing.pt'
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

    def Webcam_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error in Webcam_callback: {str(e)}')

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
    
    def Container_Sealing_detect(self):

        start_time = time.time()
        detection_duration = 60  # 1분 동안 감지
        
        while (time.time() - start_time) < detection_duration:

            img = self.frame

            while self.frame is None:
                time.sleep(0.1)
            
            # 채도 및 명도 조절
            saturation = 100  # 기본값, 필요에 따라 조절
            brightness = 100  # 기본값, 필요에 따라 조절
            adjusted_img = self.adjust_saturation_and_brightness(img, saturation_scale=saturation, brightness_offset=brightness)
            
            results = self.model(adjusted_img)
            
            for result in results:
                adjusted_img = result.plot()

        #     cv2.imshow('Webcam', adjusted_img)
            
        #     if cv2.waitKey(1) == ord('q'):  
        #         break
        
        # cv2.destroyAllWindows()

    def Container_Sealing_callback(self, request, response):
        self.get_logger().info(f'Starting detection with command: {request.command}')
        
        if request.command == "start":
            self.get_logger().info(f'Received : {request.command}')
            self.Container_Sealing_detect()
        else:
            self.get_logger().error(f'ERROR')
        
        response.success = True
        response.result = 'Detection completed'
        return response

def main(args=None):
    rclpy.init(args=args)

    CS_node = Container_Sealing()

    # MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(CS_node)

    try:
        rclpy.spin(CS_node)
    finally:
        CS_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
