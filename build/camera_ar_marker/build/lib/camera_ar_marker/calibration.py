import cv2
import numpy as np
import glob
import os


# 체스보드 패턴 크기
chessboard_size = (8, 5)

# 체스보드 패턴 실제 크기 (mm)
square_size = 25

# 체스보드 패턴의 3D 좌표 준비
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1,2)
objp *= square_size

# 3D 점과 2D 점의 저장소
objpoints = [] # 3D 점
imgpoints = [] # 2D 점

# 체스보드 이미지 불러오기
images = glob.glob('/home/messi/camera_ar_marker/camera_ar_marker/new_calibration_images/chessboard_*.jpg')

for image_file in images:
    img = cv2.imread(image_file)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 체스보드 코너 찾기
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # 코너 그리기 및 표시
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('Chessboard', img)
        cv2.waitKey(100)
    else:
        print(f"Chessboard corners not found in image {image_file}")

cv2.destroyAllWindows()

if len(objpoints) > 0 and len(imgpoints) > 0:
    # 카메라 캘리브레이션 수행
    ret, camMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # 결과 저장 디렉토리 설정
    output_dir = "/home/messi/camera_ar_marker/camera_ar_marker/new_calib_data"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 결과 각각 저장
    np.save(os.path.join(output_dir, "camMatrix.npy"), camMatrix)
    np.save(os.path.join(output_dir, "distCoeffs.npy"), distCoeffs)
    np.save(os.path.join(output_dir, "rvecs.npy"), rvecs)
    np.save(os.path.join(output_dir, "tvecs.npy"), tvecs)

    # 결과 출력
    print("Camera Matrix:\n", camMatrix)
    print("Distortion Coefficients:\n", distCoeffs)
else:
    print("No valid chessboard images found. Calibration cannot be performed.")