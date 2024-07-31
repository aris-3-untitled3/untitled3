import threading
import rclpy as rp
from rclpy.node import Node
from untitled_msgs.srv import ServiceString
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from untitled_msgs.msg import TopicString
from cv_bridge import CvBridge
import cv2 as cv
from cv2 import aruco
import numpy as np
from cv2 import aruco
import numpy as np
import time

class Cupdetect(Node):
    def __init__(self):
        super().__init__('RobotControl')

        self.bridge = CvBridge()

        # 콜백 그룹 정의
        self.image_callback_group = ReentrantCallbackGroup()
        self.server_callback_group = ReentrantCallbackGroup()

        calib_data_path = "/home/jchj/Untitled3/src/AI_models/MultiMatrix.npz"

        # WebCam Topic 구독
        self.FrontCam_subscription = self.create_subscription(
            Image,
            '/UpCam',
            self.Webcam_callback,
            10,
            callback_group=self.image_callback_group
        )

        # 서비스 서버 생성
        self.HM = self.create_service(
            ServiceString,
            '/Call_to_HM',
            self.HM_callback,
            callback_group=self.server_callback_group
        )

        self.calib_data_path = calib_data_path
        self.calib_data = np.load(calib_data_path)

        self.cam_mat = self.calib_data["camMatrix"]
        self.dist_coef = self.calib_data["distCoef"]
        self.r_vectors = self.calib_data["rVector"]
        self.t_vectors = self.calib_data["tVector"]

        self.MARKER_SIZE = 32  #32 centimeters

        self.marker_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.param_markers = aruco.DetectorParameters_create()
        
        self.yaw = np.deg2rad(180)  # Convert 180 degrees to radians
        self.pitch = np.deg2rad(0)  # Convert 0 degrees to radians
        self.roll = np.deg2rad(180)  # Convert 180 degrees to radians
        self.translation = np.array([-56, -55, 0])  # Translation vector
        
        self.x = None
        self.y = None
        
        self.x=None
        self.y=None
        self.cap = None
        self.frame = None

        threading.Thread(target=self.display_frames).start()

    def display_frames(self):
        while True:
            if self.cap is not None:  
                cv.imshow("Frame", self.cap)

            else:
                if self.frame is not None:
                    cv.imshow("Frame", self.frame)
                else:
                    self.get_logger().info("No frame available") 

            if cv.waitKey(1) & 0xFF == ord('q'):
                cv.destroyAllWindows()
                break

    def transformation_matrix(self, yaw, pitch, roll, translation):
        # Yaw (Z-axis rotation)
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Pitch (Y-axis rotation)
        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # Roll (X-axis rotation)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        # Combined rotation matrix
        R = R_z @ R_y @ R_x
        
        # Create 4x4 transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = translation
        
        return T

    def inverse_transformation_matrix(self, T):
        R = T[:3, :3]
        t = T[:3, 3]
        T_inv = np.eye(4)
        T_inv[:3, :3] = R.T
        T_inv[:3, 3] = -R.T @ t
        return T_inv

    def run(self):
            T = self.transformation_matrix(self.yaw, self.pitch, self.roll, self.translation)
            T_inv = self.inverse_transformation_matrix(T)
            
            start_time = time.time()
            success_count = 0
            coordinates = []
            
            while time.time() - start_time < 5:  # Run for 5 seconds
                self.frame = self.cap

                if self.frame is None:
                    self.get_logger().info("No frame available")
                    continue

                gray_frame = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
                marker_corners, marker_IDs, reject = aruco.detectMarkers(
                    gray_frame, self.marker_dict, parameters=self.param_markers
                )

                if marker_corners:
                    rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                        marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef
                    )

                    for ids, corners, i in zip(marker_IDs, marker_corners, range(len(marker_IDs))):
                        cv.polylines(
                            self.frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                        )
                        corners = corners.reshape(4, 2).astype(int)
                        top_right = corners[0].ravel()

                        T_Vec = np.array([tVec[i][0][0], tVec[i][0][1], 1, 1])
                        T_result_Vec = T_inv @ T_Vec

                        self.x = T_result_Vec[0]
                        self.y = T_result_Vec[1]
                        coordinates.append((self.x, self.y))

                        # Draw the pose of the marker
                        distance = np.sqrt(
                            tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                        )
                        point = cv.drawFrameAxes(self.frame, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 4, 4)
                        cv.putText(
                            self.frame,
                            f"id: {ids[0]} Dist: {round(distance, 2)}",
                            top_right,
                            cv.FONT_HERSHEY_PLAIN,
                            1.3,
                            (0, 0, 255),
                            2,
                            cv.LINE_AA,
                        )
                        cv.putText(
                            self.frame,
                            f"x:{round(self.x)} y: {round(self.y)} ",
                            top_right,
                            cv.FONT_HERSHEY_PLAIN,
                            1.0,
                            (0, 0, 255),
                            2,
                            cv.LINE_AA,
                        )

                    success_count += 1

            if success_count > 30:
                # Calculate the average coordinates if there were successful detections
                avg_x = np.mean([coord[0] for coord in coordinates])
                avg_y = np.mean([coord[1] for coord in coordinates])
                print(success_count)
                return f"Success! Average x: {avg_x:.2f}, Average y: {avg_y:.2f}"
            else:
                print(success_count)
                return "Failure: No markers detected"


    def Webcam_callback(self, msg):
        try:
            self.cap = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Webcam_callback에서 오류 발생: {str(e)}')

    def HM_callback(self, request, response):
        self.get_logger().info(f'Starting detection with command: {request.command}')

        if request.command == "start":
            self.get_logger().info(f'Received: {request.command}')
            result = self.run()
            response.result = f'result: {result}'
        else:
            self.get_logger().error(f'ERROR')
            response.result = 'Invalid command'

        response.success = True
        return response

def main(args=None):
    rp.init(args=args)
    node = Cupdetect()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()
