import rclpy as rp
from rclpy.node import Node
from untitled_msgs.msg import TopicString
from untitled_msgs.srv import ServiceString
from untitled_msgs.action import ActionString
import time


class Cupdetect(Node):
    def __init__(self):
        super().__init__('RobotControl')

        # Robot_Server에서 /Cup_Info 서비스를 받아옴
        self.cup_detect_server = self.create_service(ServiceString, '/Cup_Info', self.cup_detect_callback)


    def cup_detect_callback(self, request, response):
        self.get_logger().info(f'Received command: {request.command}')

        print("컵 감지 (?초동안)")
        time.sleep(3)

        result = self.cup_detect()  

        print(result)

        if "," in result:
            response.success = True
            response.result = f'Command {request.command} received and being processed : {result}' 
        else :
            response.success = False
            response.result = f'Command {request.command} Failed! : {result}' 

        return response
    

    def cup_detect(self):
        print("컵 감지중")

        #컵이 감지되었다고 가정
        result = "12,13"

        return result


def main(args=None):
    rp.init(args=args)

    Cup_detect = Cupdetect()

    rp.spin(Cup_detect)

    Cup_detect.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()