import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

FIRST_MARKER = 1
SECOND_MARKER = 3

class ParkingController(Node):
    def __init__(self):
        super().__init__('parking_controller')

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 구독 설정
        self.angle_subscription = self.create_subscription(String, '/ar_marker_angle', self.angle_callback, 10)
        self.distance_subscription = self.create_subscription(String, '/ar_marker_distance', self.distance_callback, 10)
        self.aruco_id_subscription = self.create_subscription(String, '/ar_marker_id', self.id_callback, 10)
        self.center_subscription = self.create_subscription(String, '/ar_marker_center', self.center_callback, 10)  # 마커 중심 구독

        # 상태 초기화
        self.current_direction = 0.2  # z축 각속도 (rad/s)
        self.current_speed = 0.1  # x축 선속도 (m/s)
        self.is_rotating = True  # 회전 상태 플래그
        self.is_adjusting_distance = False  # 거리 조정 플래그
        self.current_scenario = 1  # 현재 시나리오 단계 (goal position 끝나고 마커1 인식하는 첫 시나리오)

        self.marker_id = None  # 마커 아이디 초기화
        self.center_offset = None 

        # 최근 데이터 기록을 위한 리스트
        self.angle_history = []
        self.distance_history = []
        self.offset_history = []

        # 지속적으로 회전하기 위한 타이머 설정 (0.1초 간격)
        self.timer = self.create_timer(0.1, lambda: self.rotate(self.current_scenario))

        # 로깅
        self.get_logger().info(f"goal position flag 켜짐 ! {self.current_scenario}")

    def rotate(self, current_scenario):
        if self.is_rotating:
            twist = Twist()

            # 마커가 감지되었을 때 마커를 화면 중앙으로 맞추기
            if self.center_offset is not None and current_scenario == self.marker_id:
                if self.center_offset > 13 or self.center_offset < -13:
                    angular = -self.center_offset * 0.003  # 중심 오프셋에 비례해 각속도 조정
                    
                    if angular > 0.0:
                        twist.angular.z = 0.15

                    elif angular < -0.0:
                        twist.angular.z = -0.15

                    print(f"조정되는 중: {twist.angular.z}")
                    self.publisher.publish(twist)
                        

                # 오프셋이 작은 경우 회전 멈춤
                else:  # 중심에 충분히 가까워지면 멈춤
                        # print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@2")
                        # print("충분히 가까워 짐, 중단 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@2")
                        # print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@2")
                        self.stop_rotation()

            # 마커가 감지되지 않았을 경우 계속 회전
            else:
                print("마커 감지되지 않음 계속해서 회전 중...")
                if current_scenario == 1:
                    twist.angular.z = 0.2
                elif current_scenario == 3:
                    twist.angular.z = -0.2
                twist.linear.x = 0.0
                self.publisher.publish(twist)

    def stop_rotation(self):
        self.is_rotating = False
        twist = Twist()
        twist.angular.z = 0.0
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        self.get_logger().info(f"회전 정지: 시나리오 {self.current_scenario} 완료")

        self.current_scenario += 1
        self.get_logger().info(f"시나리오 {self.current_scenario} 시작")

        if self.current_scenario == 2:
            self.FIRST_MARKER_adjustment()

        elif self.current_scenario == 3:
            # self.find_marker(SECOND_MARKER, -0.2)  # 두 번째 마커 찾기
            self.Find_SECOND_MARKER()

        elif self.current_scenario == 4:
            self.SECOND_MARKER_adjustment()

        elif self.current_scenario == 5:
            self.Final_adjustment()

        elif self.current_scenario >= 6:
            print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
            print("++++++++++++++++++++++++++++++ Exit ++++++++++++++++++++++++++++++")
            print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")


    def FIRST_MARKER_adjustment(self):
        self.is_adjusting_distance = True
        self.distance_history = []
        self.get_logger().info("첫 번째 마커와의 거리 조정 중...")

    def Find_SECOND_MARKER(self):
        self.is_rotating = True
        self.angle_history = []
        self.offset_history = []
        self.timer = self.create_timer(0.1, lambda: self.rotate(self.current_scenario))
        self.get_logger().info("두 번재 마커 찾는 중...")


    def SECOND_MARKER_adjustment(self):
        self.is_adjusting_distance = True
        self.distance_history = []
        self.get_logger().info("두 번째 마커와의 거리 조정 중...")

    def Final_adjustment(self):
        self.is_adjusting_distance = True
        self.distance_history = []
        self.get_logger().info("주차 중...")

    def distance_callback(self, msg):
        if self.is_adjusting_distance:
            try:
                distance_data = float(msg.data)
                self.distance_history.append(distance_data)
                self.get_logger().info(f"수신된 거리 (시나리오 {self.current_scenario}): {distance_data:.2f}m")

                if len(self.distance_history) > 10:
                    self.distance_history.pop(0)  # 기록된 데이터가 10개를 넘으면 제거

                twist = Twist()

                # 첫 번째 마커와 거리 조정
                if self.current_scenario == 2:
                    in_range_count = sum(0.20 <= d <= 0.22 for d in self.distance_history)
                    if in_range_count >= 5:
                        twist.linear.x = 0.0
                        self.publisher.publish(twist)
                        self.get_logger().info("첫 번째 마커와 거리 맞춤 !!")
                        self.is_adjusting_distance = False
                        self.stop_rotation()
                        return
                    else:
                        if distance_data > 0.22:
                            twist.linear.x = self.current_speed
                        elif distance_data < 0.20:
                            twist.linear.x = -self.current_speed

                # 두 번째 마커와 거리 조정
                if self.current_scenario == 4:
                    in_range_count = sum(2.38 <= d <= 2.40 for d in self.distance_history)
                    if in_range_count >= 5:
                        twist.linear.x = 0.0
                        self.publisher.publish(twist)
                        self.get_logger().info("두 번째 마커와 거리 맞춤 !!")
                        self.is_adjusting_distance = False
                        self.stop_rotation()
                        return
                    else:
                        if distance_data > 2.40:
                            twist.linear.x = self.current_speed
                        elif distance_data < 2.38:
                            twist.linear.x = -self.current_speed

                # 주차 거리 조정
                if self.current_scenario == 5:
                    in_range_count = sum(2.48 <= d <= 2.50 for d in self.distance_history)
                    if in_range_count >= 5:
                        twist.linear.x = 0.0
                        self.publisher.publish(twist)
                        self.get_logger().info("주차 완료")
                        self.is_adjusting_distance = False
                        self.stop_rotation()
                        return
                    else:
                        if distance_data > 2.48:
                            twist.linear.x = self.current_speed
                        elif distance_data < 2.50:
                            twist.linear.x = -self.current_speed

                self.publisher.publish(twist)

            except ValueError:
                self.get_logger().warn("유효하지 않은 거리 데이터 수신")

    def angle_callback(self, msg):
        if not self.is_adjusting_distance:
            try:
                angle_data = float(msg.data)
                # 이거 굳이 리스트형식으로 바꿔야 되나? 그냥 바로 바로 반영되게 하면 안되나?
                self.angle_history.append(angle_data)

                # 만약 리스트 형식으로 저장하게 하면 아래 주석 활성화
                if len(self.angle_history) > 10:
                    self.angle_history.pop(0)  # 기록된 데이터가 10개를 넘으면 제거

            except ValueError:
                self.get_logger().warn("유효하지 않은 거리 데이터 수신")

    def id_callback(self, msg):
        self.marker_id = int(msg.data)
        print(f"marker id: {self.marker_id}")

    def center_callback(self, msg):
        try:
            self.center_offset = float(msg.data)
        except ValueError:
            self.get_logger().warn("유효하지 않은 중심 오프셋 데이터 수신")

def main(args=None):
    rclpy.init(args=args)
    parking_controller = ParkingController()
    rclpy.spin(parking_controller)
    parking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  