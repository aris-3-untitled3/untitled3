import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class RotateController(Node):
    def __init__(self):
        super().__init__('rotate_controller')

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 구독 설정
        self.angle_subscription = self.create_subscription(String, '/ar_marker_angle', self.angle_callback, 10)
        self.distance_subscription = self.create_subscription(String, '/ar_marker_distance', self.distance_callback, 10)
        self.id_subscription = self.create_subscription(String, 'ar_marker_id', self.id_callback, 10)

        # 상태 초기화
        self.current_direction = 0.22  # z축 각속도 (rad/s)
        self.current_speed = 0.05  # x축 선속도 (m/s)
        self.is_rotating = True  # 회전 상태 플래그
        self.is_adjusting_distance = False  # 거리 조정 플래그
        self.current_scenario = 1  # 현재 시나리오 단계

        # 최근 데이터 기록을 위한 리스트
        self.angle_history = []
        self.distance_history = []

        # ID data 처음엔 0으로 초기화
        self.id_data = 0

        # 지속적으로 회전하기 위한 타이머 설정 (0.1초 간격)
        self.timer = self.create_timer(0.1, self.rotate)

        # 로깅
        self.get_logger().info(f"시나리오 {self.current_scenario} 시작: 로봇이 z 축을 중심으로 회전하고 있습니다.")

    def rotate(self):
        if self.is_rotating:  # 회전 중일 때만 퍼블리시
            twist = Twist()

            # 시나리오에 따른 속도 조정
            if self.current_scenario == 1:
                self.current_direction = -0.17  # 각속도 증가
            elif self.current_scenario == 3:
                self.current_direction = 0.30  # 각속도 증가
            elif self.current_scenario == 4:
                self.current_direction = 0.07
            
            elif self.current_scenario == 6 or self.current_scenario == 8:
                self.adjust_rotation_direction()  # 각도에 따른 회전 방향 조정

            twist.angular.z = self.current_direction  # z축으로 회전
            twist.linear.x = 0.0  # 직진/후진 없음
            self.publisher.publish(twist)  # cmd_vel 퍼블리시

    def adjust_rotation_direction(self):
        """Scenario 6에서 각도에 따라 회전 방향을 조정합니다."""
        last_angle = self.angle_history[-1] if self.angle_history else 180
        if last_angle >= 181:
            self.current_direction = -abs(self.current_direction)  # 각속도 음수로 설정 (반시계 방향)
        elif last_angle <= 180:
            self.current_direction = abs(self.current_direction)  # 각속도 양수로 설정 (시계 방향)

    def stop_rotation(self):
        self.is_rotating = False  # 회전 상태 플래그 끄기
        twist = Twist()
        twist.angular.z = 0.0  # 회전 멈춤
        twist.linear.x = 0.0
        self.publisher.publish(twist)  # cmd_vel 퍼블리시
        self.timer.cancel()  # 타이머 취소
        self.get_logger().info(f"회전 정지: 시나리오 {self.current_scenario} 완료")

        # 다음 시나리오로 전환
        self.current_scenario += 1
        self.get_logger().info(f"시나리오 {self.current_scenario} 시작")

        if self.current_scenario == 2:
            self.start_distance_adjustment()
        elif self.current_scenario == 3:
            self.start_marker4_rotation_31()
        elif self.current_scenario == 4:
            self.start_marker4_rotation_32()
        elif self.current_scenario == 5:
            self.start_marker4_approach()
        elif self.current_scenario == 6:
            self.start_angle_adjustment_after_distance()  # 새로운 여섯 번째 시나리오 (각도 조정)
        elif self.current_scenario == 7:
            self.start_distance_adjustment_89_to_91()  # 일곱 번째 시나리오 (거리 조정)
        elif self.current_scenario == 8:
            self.start_final_rotation()  # 기존 여섯 번째 시나리오 (8번으로 이동)
        elif self.current_scenario == 9:
            self.start_final_approach()  # 기존 일곱 번째 시나리오 (9번으로 이동)

    def start_distance_adjustment_89_to_91(self):
        self.is_adjusting_distance = True
        self.distance_history = []
        self.get_logger().info("일곱 번째 시나리오 시작: 거리 0.89~0.91m 조정 중")

    def start_angle_adjustment_after_distance(self):
        self.is_rotating = True
        self.angle_history = []
        self.timer = self.create_timer(0.1, self.rotate)
        self.get_logger().info("여섯 번째 시나리오 시작: 각도 183~184도 조정 중")

    def start_distance_adjustment(self):
        self.is_adjusting_distance = True
        self.distance_history = []  # 거리 기록 초기화
        self.get_logger().info("두 번째 시나리오 시작: 거리 조정 중")

    def start_marker4_rotation_31(self):
        self.is_rotating = True
        self.angle_history = []  # 각도 기록 초기화
        self.timer = self.create_timer(0.1, self.rotate)
        self.get_logger().info("세 번째 시나리오 3.1 시작: 각도 177~179도 조정 중")

    def start_marker4_rotation_32(self):
        self.is_rotating = True
        self.angle_history = []  # 각도 기록 초기화
        self.timer = self.create_timer(0.1, self.rotate)
        self.get_logger().info("세 번째 시나리오 3.2 시작: 각도 181~182도 조정 중")

    def start_marker4_approach(self):
        self.is_adjusting_distance = True
        self.distance_history = []  # 거리 기록 초기화
        self.get_logger().info("네 번째 시나리오 시작: 마커 4번을 향해 직진 중")

    def start_final_rotation(self):
        self.is_rotating = True
        self.angle_history = []  # 각도 기록 초기화
        self.timer = self.create_timer(0.1, self.rotate)
        self.get_logger().info("다섯 번째 시나리오 시작: 각도 179~180도 조정 중")

    def start_final_approach(self):
        self.is_adjusting_distance = True
        self.distance_history = []  # 거리 기록 초기화
        self.current_speed = 0.07  # 선속도 증가
        self.get_logger().info("여섯 번째 시나리오 시작: 거리 0.59~0.60m 조정 중")

    def distance_callback(self, msg):
        if self.is_adjusting_distance:
            try:
                distance_data = float(msg.data)
                self.distance_history.append(distance_data)
                self.get_logger().info(f"수신된 거리 (시나리오 {self.current_scenario}): {distance_data:.2f}m, 현재 선속도: {self.current_speed}")

                if len(self.distance_history) > 10:
                    self.distance_history.pop(0)  # 기록된 데이터가 10개를 넘으면 제거

                twist = Twist()

                if self.current_scenario == 2 and self.id_data == 2:
                    # 두 번째 시나리오: 2번 마커와의 거리 조정
                    in_range_count = sum(1.21 <= d < 1.23 for d in self.distance_history)
                    if in_range_count >= 5:
                        twist.linear.x = 0.0  # 적절한 거리: 정지
                        self.publisher.publish(twist)
                        self.get_logger().info("거리 맞춤 완료: 두 번째 시나리오 완료")
                        self.is_adjusting_distance = False
                        self.stop_rotation()
                        return  # 콜백 종료
                    else:
                        if distance_data < 1.22:
                            twist.linear.x = -self.current_speed  # 너무 가까움: 후진
                            self.get_logger().info("후진 중: 거리가 너무 가깝습니다.")
                        elif distance_data > 1.24:
                            twist.linear.x = self.current_speed  # 너무 멂: 전진
                            self.get_logger().info("전진 중: 거리가 너무 멉니다.")
                
                elif self.current_scenario == 5 and self.id_data == 4:
                    # 네 번째 시나리오: 4번 마커와의 거리 조정
                    in_range_count = sum(1.15 <= d <= 1.18 for d in self.distance_history)
                    if in_range_count >= 5:
                        twist.linear.x = 0.0  # 적절한 거리: 정지
                        self.publisher.publish(twist)
                        self.get_logger().info("거리 맞춤 완료: 네 번째 시나리오 완료")
                        self.is_adjusting_distance = False
                        self.stop_rotation()
                        return  # 콜백 종료
                    else:
                        if distance_data < 1.15:
                            twist.linear.x = -self.current_speed  # 너무 가까움: 후진
                            self.get_logger().info("후진 중: 거리가 너무 가깝습니다.")
                        elif distance_data > 1.18:
                            twist.linear.x = self.current_speed  # 너무 멂: 전진
                            self.get_logger().info("전진 중: 거리가 너무 멉니다.")
                
                elif self.current_scenario == 7 and self.id_data == 4:
                    # 일곱 번째 시나리오: 0.89~0.91m 거리 조정
                    in_range_count = sum(0.89 <= d <= 0.91 for d in self.distance_history)
                    if in_range_count >= 5:
                        twist.linear.x = 0.0
                        self.publisher.publish(twist)
                        self.get_logger().info("거리 맞춤 완료: 여덟 번째 시나리오로 전환")
                        self.is_adjusting_distance = False
                        self.stop_rotation()
                        return
                    else:
                        if distance_data < 0.89:
                            twist.linear.x = -self.current_speed
                            self.get_logger().info("후진 중: 거리가 너무 가깝습니다.")
                        elif distance_data > 0.91:
                            twist.linear.x = self.current_speed
                            self.get_logger().info("전진 중: 거리가 너무 멉니다.")

                elif self.current_scenario == 9 and self.id_data == 4:
                    # 아홉 번째 시나리오: 거리 0.59~0.60m 조정
                    in_range_count = sum(0.59 <= d <= 0.60 for d in self.distance_history)
                    if in_range_count >= 5:
                        twist.linear.x = 0.0  # 적절한 거리: 정지
                        self.publisher.publish(twist)
                        self.get_logger().info("도착 완료: 여섯 번째 시나리오 완료")
                        self.is_adjusting_distance = False
                        return  # 콜백 종료
                    else:
                        if distance_data < 0.59:
                            twist.linear.x = -self.current_speed  # 너무 가까움: 후진
                            self.get_logger().info("후진 중: 거리가 너무 가깝습니다.")
                        elif distance_data > 0.60:
                            twist.linear.x = self.current_speed  # 너무 멂: 전진
                            self.get_logger().info("전진 중: 거리가 너무 멉니다.")

                self.publisher.publish(twist)

            except ValueError:
                self.get_logger().warn("유효하지 않은 거리 데이터 수신")

    def angle_callback(self, msg):
        if not self.is_adjusting_distance:  # 거리 조정 중이 아닐 때만 각도 조정
            try:
                angle_data = float(msg.data)
                self.angle_history.append(angle_data)
                self.get_logger().info(f"수신된 각도 (시나리오 {self.current_scenario}): {angle_data:.2f}도, 현재 각속도: {self.current_direction}")

                if len(self.angle_history) > 10:
                    self.angle_history.pop(0)  # 기록된 데이터가 10개를 넘으면 제거

                if self.current_scenario == 1 and self.id_data == 2:
                    # 첫 번째 시나리오: 각도 조정
                    in_range_count = sum(176 <= a <= 183 for a in self.angle_history)
                    if in_range_count >= 5:
                        self.stop_rotation()

                elif self.current_scenario == 3 and self.id_data == 4:
                    # 세 번째 시나리오 3.1: 각도 177~179도 조정
                    in_range_count = sum(150 <= a <= 170 for a in self.angle_history)
                    if in_range_count >= 5:
                        self.stop_rotation()

                elif self.current_scenario == 4 and self.id_data == 4:
                    # 세 번째 시나리오 3.2: 각도 181~182도 조정
                    in_range_count = sum(180 <= a <= 184 for a in self.angle_history)
                    if in_range_count >= 5:
                        self.stop_rotation()
                
                elif self.current_scenario == 6 and self.id_data == 4:
                    # 여섯 번째 시나리오: 각도 183~184도 조정
                    in_range_count = sum(179 <= a <= 181 for a in self.angle_history)
                    if in_range_count >= 5:
                        self.stop_rotation()

                elif self.current_scenario == 8 and self.id_data == 4:
                    # 여덟 번째 시나리오: 각도 179~180도 조정
                    in_range_count = sum(182 <= a <= 185 for a in self.angle_history)
                    if in_range_count >= 5:
                        self.stop_rotation()

            except ValueError:
                self.get_logger().warn("유효하지 않은 각도 데이터 수신")

    def id_callback(self, msg):
        self.id_data = int(msg.data)
        self.get_logger().info(f"실시간 수신된 마커 ID는 {self.id_data} 입니다.")

def main(args=None):
    rclpy.init(args=args)
    rotate_controller = RotateController()
    rclpy.spin(rotate_controller)
    rotate_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
