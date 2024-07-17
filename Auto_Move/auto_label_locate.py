import multiprocessing.queues
import cv2
import numpy as np
import sys
import time
import matplotlib.pyplot as plt
import multiprocessing
import json

# 加载相机内参和畸变系数
with np.load('camera_calibration_data.npz') as data:
    camera_matrix = data['camera_matrix']
    dist_coeffs = data['dist_coeffs']

alpha = 0.06  # 低通滤波系数
cam_init_pos_global = np.array([0, 0, 0], dtype=np.float64)
cam_init_rotation_global = np.eye(3)
cam_current_pos_global = np.array([0, 0, 0], dtype=np.float64)
cam_current_rotation_global = np.eye(3)
count = 0
max_count = 500
origin_setted = False
last_time = 0.0
local_filtered_trajectory = []


def sort_corners(corners: np.ndarray) -> np.ndarray:
    """`
    将角点排序为顺时针顺序
    """
    center = np.mean(corners, axis=0)
    angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])
    sorted_corners = corners[np.argsort(angles)]
    return sorted_corners


def detect_label(frame, label_info: dict, bounding_box: tuple = None) -> np.ndarray:
    """
    检测特定颜色的标签并返回其图像平面中的角点
    """
    # 获取当前HSV范围
    lower_h = label_info['hsv_range']['lower'][0]
    lower_s = label_info['hsv_range']['lower'][1]
    lower_v = label_info['hsv_range']['lower'][2]
    upper_h = label_info['hsv_range']['upper'][0]
    upper_s = label_info['hsv_range']['upper'][1]
    upper_v = label_info['hsv_range']['upper'][2]

    lower_color = np.array([lower_h, lower_s, lower_v])
    upper_color = np.array([upper_h, upper_s, upper_v])

    # 获取最小轮廓面积
    min_area = 1000

    # 高斯滤波降噪
    frame = cv2.GaussianBlur(frame, (7, 7), 0)
    
    # 将图像转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 创建颜色掩码
    mask = cv2.inRange(hsv, lower_color, upper_color)
    # 图形学运算以减少噪声
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    cv2.imshow('mask', mask)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    best_contour = None
    max_overlap_area = 0

    for contour in contours:
        # 过滤掉太小的轮廓
        if cv2.contourArea(contour) < min_area:
            continue

        epsilon = 0.05 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        if len(approx) == 4:
            # 检测到四边形，计算重叠面积
            x, y, w, h = cv2.boundingRect(approx)
            
            if bounding_box is not None:
                x1, y1, x2, y2 = bounding_box
                overlap_x1 = max(x1, x)
                overlap_y1 = max(y1, y)
                overlap_x2 = min(x2, x + w)
                overlap_y2 = min(y2, y + h)
                overlap_area = max(0, overlap_x2 - overlap_x1) * max(0, overlap_y2 - overlap_y1)
                if overlap_area > max_overlap_area:
                    max_overlap_area = overlap_area
                    best_contour = approx
            else:
                best_contour = approx

    if best_contour is not None:
        # 返回重叠面积最大的轮廓的角点并排序
        corners = np.array([point[0] for point in best_contour], dtype=np.float32)

        sorted_corners = sort_corners(corners)
        return sorted_corners

    return None

def rotation_matrix_to_euler_angles(R : cv2.Mat) -> np.ndarray:
    """
    将旋转矩阵转换为欧拉角
    """
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def euler_angles_to_rotation_matrix(euler_angles : np.ndarray) -> cv2.Mat:
    """
    将欧拉角转换为旋转矩阵
    """
    x, y, z = euler_angles

    # 计算各个轴的旋转矩阵
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(x), -np.sin(x)],
                    [0, np.sin(x), np.cos(x)]])

    R_y = np.array([[np.cos(y), 0, np.sin(y)],
                    [0, 1, 0],
                    [-np.sin(y), 0, np.cos(y)]])

    R_z = np.array([[np.cos(z), -np.sin(z), 0],
                    [np.sin(z), np.cos(z), 0],
                    [0, 0, 1]])

    # 旋转矩阵的乘积
    R = np.dot(R_z, np.dot(R_y, R_x))

    return R

def low_pass_filter(new_point, prev_filtered_point, alpha):
    """
    简单的低通滤波器
    """
    return alpha * new_point + (1 - alpha) * prev_filtered_point


def is_abnormal_point(trajectory: list, new_point, dt, threshold=3.5):
    trajectory.append(new_point)

    if len(trajectory) < 2:
        return False

    # 计算速度
    velocities = np.diff(trajectory, axis=0) / dt

    if len(velocities) < 2:
        return False

    # 计算加速度
    accelerations = np.diff(velocities, axis=0) / dt

    if len(accelerations) < 1:
        return False

    # 计算加速度变化量
    accel_diffs = np.diff(accelerations, axis=0)

    if len(accel_diffs) < 1:
        return False

    # 获取最近的变化量
    recent_accel_diff = accel_diffs[-1]

    # 计算四分位数和四分位距
    Q1 = np.percentile(accel_diffs, 25, axis=0)
    Q3 = np.percentile(accel_diffs, 75, axis=0)
    IQR = Q3 - Q1
    # print(recent_accel_diff)

    # 判断是否为异常值
    is_abnormal = np.any((recent_accel_diff < Q1 - threshold * IQR)
                         | (recent_accel_diff > Q3 + threshold * IQR))

    trajectory.pop()

    return is_abnormal


def calc_pos_and_trans(frame, label_info: dict, bounding_box: tuple = None, debug=False):
    '''
    返回相机相对于上电原点的位姿，以及标签相对相机的位姿
    '''
    corners = detect_label(frame, label_info, bounding_box)
    geo_info = label_info["geo_info"]
    geo_info = np.array(geo_info, dtype=np.float32)
    label_info["detected"] = False
    if corners is not None:
        # 使用PnP求解相机相对于标签的位姿
        ret, rvec, tvec = cv2.solvePnP(
                geo_info, corners, camera_matrix, dist_coeffs)
        if ret:
            label_info["detected"] = True
            # 计算旋转矩阵
            obj_rotation_to_cam, _ = cv2.Rodrigues(rvec)
            obj_position_to_cam = tvec.flatten()
            # pnp返回的R和t是指目标物体在相机坐标系下的位置和姿态
            # 所以求取逆矩阵和变换才能获得相机位置和姿态

            # 求解标签坐标系下的相机旋转姿态
            camera_rotation_to_label = obj_rotation_to_cam.T
            # 求解标签坐标系下的相机平移向量
            camera_pos_to_label = camera_rotation_to_label @ (-tvec)
            camera_pos_to_label = camera_pos_to_label.flatten()

            if debug:
                # 绘制四个角点
                for corner in corners:
                    cv2.circle(frame, tuple(corner.astype(int)),
                               5, (0, 0, 255), -1)

            # 返回相机在标签坐标系下的位置和姿态
            return camera_pos_to_label, camera_rotation_to_label, \
                obj_position_to_cam, obj_rotation_to_cam

        else:
            return None, None, None, None
    else:
        return None, None, None, None
