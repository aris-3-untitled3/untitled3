import cv2 as cv
from cv2 import aruco
import numpy as np

calib_data_path = "/home/hyeonuk/python/Basic-Augmented-reality-course-opencv/calib_data/MultiMatrix.npz"

class Mapping:
    
    def __init__(self,calib_data_path):
        self.calib_data_path=calib_data_path
        self.calib_data = np.load(calib_data_path)
        #print(calib_data.files)

        self.cam_mat = self.calib_data["camMatrix"]
        self.dist_coef = self.calib_data["distCoef"]
        self.r_vectors = self.calib_data["rVector"]
        self.t_vectors = self.calib_data["tVector"]

        self.MARKER_SIZE = 32  # centimeters

        self.marker_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

        self.param_markers = aruco.DetectorParameters_create()

        self.cap = cv.VideoCapture(2)
        
        self.yaw = np.deg2rad(180)   # Convert 30 degrees to radians
        self.pitch = np.deg2rad(0) # Convert 45 degrees to radians
        self.roll = np.deg2rad(180)  # Convert 60 degrees to radians

        self.translation=np.array([-27,-55,0])  ##-50 -66  ##43.5 -82
        
        self.x=None
        self.y=None

    def transformation_matrix(self,yaw, pitch, roll, translation):
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

    def inverse_transformation_matrix(self,T):
        R = T[:3, :3]
        t = T[:3, 3]
        T_inv = np.eye(4)
        T_inv[:3, :3] = R.T
        T_inv[:3, 3] = -R.T @ t
        return T_inv

    def run(self):

        T=self.transformation_matrix(self.yaw,self.pitch,self.roll,self.translation)
        T_inv=self.inverse_transformation_matrix(T)

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            marker_corners, marker_IDs, reject = aruco.detectMarkers(
                gray_frame, self.marker_dict, parameters=self.param_markers
            )
            if marker_corners:
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                    marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef
                )
                total_markers = range(0, marker_IDs.size)
                for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                    cv.polylines(
                        frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                    )
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = corners[0].ravel()
                    top_left = corners[1].ravel()
                    bottom_right = corners[2].ravel()
                    bottom_left = corners[3].ravel()
                    
                    #####
                    T_Vec=np.array([tVec[i][0][0],tVec[i][0][1],1,1])
                    T_result_Vec=T_inv @ T_Vec
                    #print(T_result_Vec[0])
                    
                    self.x=T_result_Vec[0]
                    self.y=T_result_Vec[1]
                    print(f"x:{self.x}, \n y: {self.y}")
                    # Since there was mistake in calculating the distance approach point-outed in the Video Tutorial's comment
                    # so I have rectified that mistake, I have test that out it increase the accuracy overall.
                    # Calculating the distance
                    distance = np.sqrt(
                        tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                    )
                    # Draw the pose of the marker
                    point = cv.drawFrameAxes(frame, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 4, 4)
                    cv.putText(
                        frame,
                        f"id: {ids[0]} Dist: {round(distance, 2)}",
                        top_right,
                        cv.FONT_HERSHEY_PLAIN,
                        1.3,
                        (0, 0, 255),
                        2,
                        cv.LINE_AA,
                    )
                    cv.putText(
                        frame,
                        f"x:{round(self.x)} y: {round(self.y)} ",
                        bottom_right,
                        cv.FONT_HERSHEY_PLAIN,
                        1.0,
                        (0, 0, 255),
                        2,
                        cv.LINE_AA,
                    )
                    # print(ids, "  ", corners)
            cv.imshow("frame", frame)
            key = cv.waitKey(1)
            if key == ord("q"):
                break
        
        self.cap.release()
        cv.destroyAllWindows()
            
if __name__=="__main__":
    calib_data_path = "/home/hyeonuk/python/Basic-Augmented-reality-course-opencv/calib_data/MultiMatrix.npz"
    detector=Mapping(calib_data_path)
    
    detector.run()
