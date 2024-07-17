import cv2
import numpy as np

# 棋盘格的尺寸
checkerboard_size = (10, 7)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 准备世界坐标系中的3D点
objp = np.zeros((np.prod(checkerboard_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

# 用于存储3D点和2D点
object_points = []
image_points = []

# 打开摄像头
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# 误差阈值
threshold_error = 0.1
calibration_attempts = 0
calibration_times = 0
found_at_least_once = False

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法从摄像头读取视频帧")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

    if ret:
        found_at_least_once = True
        calibration_attempts += 1
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        object_points.append(objp)
        image_points.append(corners2)

        frame = cv2.drawChessboardCorners(frame, checkerboard_size, corners2, ret)
        cv2.imshow('Chessboard', frame)
        cv2.waitKey(500)
        
        if calibration_attempts >= 10:
            
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, gray.shape[::-1], None, None)
            if ret:
                calibration_times += 1
                mean_error = 0
                for i in range(len(object_points)):
                    imgpoints2, _ = cv2.projectPoints(object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
                    error = cv2.norm(image_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                    mean_error += error

                mean_error /= len(object_points)
                print("当前平均误差:", mean_error)

                if mean_error < threshold_error and calibration_times > 20:
                    print("相机标定完成，误差足够小")
                    print("相机内参矩阵:\n", camera_matrix)
                    print("畸变系数:\n", dist_coeffs)
                    np.savez('camera_calibration_data.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, rvecs=rvecs, tvecs=tvecs)
                    break
    else:
        cv2.imshow('Chessboard', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

if not found_at_least_once:
    print("从未找到棋盘格角点，请检查棋盘格的图像质量、光照条件和摄像头设置")

cap.release()
cv2.destroyAllWindows()

if not ret:
    print("相机标定失败")
