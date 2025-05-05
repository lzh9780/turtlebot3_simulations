from math import cos, sin, radians  # 注意：使用弧度转换函数
import numpy as np
import cv2
print(np.__version__)  # 应输出 1.21.0 或更高
print(cv2.__version__)    # 应输出 4.5.0 或更高

# 初始化摄像头
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# ArUco检测器
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector = cv2.aruco.ArucoDetector(aruco_dict)


# 标准相机内参矩阵（假设焦距为图像宽度）
camera_matrix = np.array([
    [530.467, 0, 320.5],  # fx, 0, cx
    [0, 530.467, 240.5], # 0, fy, cy (假设fx ≈ fy)
    [0, 0, 1]
], dtype=np.float32)


# 假设无畸变（实际相机通常有畸变，此处仅测试用）
dist_coeffs = np.zeros((5, 1), dtype=np.float32)

# 定义参数
marker_length = 0.01  # 标记的实际尺寸（米）

# 假设相机安装在机器人基座的前方 0.1 米，高度 0.05 米（需根据实际测量修改）
CAMERA_OFFSET_X = 0.1  # 相机在机器人基座前方的水平偏移
CAMERA_OFFSET_Y = 0.0  # 横向偏移
CAMERA_HEIGHT = 0.05   # 垂直高度

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 检测ArUco标记
    corners, ids, _ = detector.detectMarkers(frame)
    
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # --------------------------
        # 修正：通过 detector 对象调用姿态估计函数
        # --------------------------
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs
        )

        for i in range(len(ids)):
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)

            # 1. 获取标记在相机坐标系中的位置
            x_cam = tvecs[i][0][0]
            y_cam = tvecs[i][0][1]
            z_cam = tvecs[i][0][2]

            # 2. 转换为基座坐标系（考虑安装偏移）
            x_base = x_cam + CAMERA_OFFSET_X
            y_base = y_cam + CAMERA_OFFSET_Y

            # 3. 获取机器人全局位姿（假设已知）
            rob_x_global = 1.0    # 全局X位置
            rob_y_global = 1.5    # 全局Y位置
            rob_rot_deg = 58      # 机器人朝向角度
            rob_rot_rad = radians(rob_rot_deg)

            # 4. 转换到全局坐标系
            x_global = rob_x_global + x_base * cos(rob_rot_rad) - y_base * sin(rob_rot_rad)
            y_global = rob_y_global + x_base * sin(rob_rot_rad) + y_base * cos(rob_rot_rad)

            print(f"MarkID: {ids[i][0]}")
            print(f"Camera coordinate (x, y, z): {x_cam:.2f}, {y_cam:.2f}, {z_cam:.2f}")
            print(f"Global coordinates (x, y): {x_global:.2f}, {y_global:.2f}\n")

    cv2.imshow("ArUco Detection", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
