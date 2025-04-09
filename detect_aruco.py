from math import cos, sin 
import cv2
import numpy as np

# 初始化摄像头
cap = cv2.VideoCapture(0)  # 0表示默认摄像头

# 定义ArUco字典和检测器
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector = cv2.aruco.ArucoDetector(aruco_dict)

# --------------------------
# 设置宽度
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# 设置高度
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# 假设摄像头分辨率为 1280x720
image_width = 1280
image_height = 720



# 标准相机内参矩阵（假设焦距为图像宽度）
camera_matrix = np.array([
    [image_width, 0, image_width/2],  # fx, 0, cx
    [0, image_width, image_height/2], # 0, fy, cy (假设fx ≈ fy)
    [0, 0, 1]
], dtype=np.float32)

# 假设无畸变（实际相机通常有畸变，此处仅测试用）
dist_coeffs = np.zeros((5, 1), dtype=np.float32)



# 定义ArUco标记的实际物理尺寸（单位：米）
marker_length = 0.1  # 例如：10cm x 10cm的标记

# --------------------------
# 主循环
# --------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 检测ArUco标记
    corners, ids, _ = detector.detectMarkers(frame)
    
    if ids is not None:
        # 在图像上绘制检测框和ID
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # --------------------------
        # 姿态估计（需要相机标定参数）
        # --------------------------
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs
        )
        
              

        # 在标记上绘制3D坐标轴（X红，Y绿，Z蓝）
        for i in range(len(ids)):
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)

        # 打印位置和旋转信息
        print(f"MarkID: {ids[i][0]}")
        print(f"Position (x, y, z): {tvecs[i][0]}")
        print(f"Rotation (rx, ry, rz): {rvecs[i][0]}")

        
        #获取turtlebot当前位置
        #暂时设置为x = 1, y = 1.5 angle = 58 degree
        rob_x_pos = 1
        rob_y_pos = 1.5
        rob_rot = 58
        dist = tvecs[i][0][2]
        x_dist = cos(rob_rot)*dist
        y_dist = sin(rob_rot)*dist
        obj_x_pos = rob_x_pos+x_dist
        obj_y_pos = rob_y_pos+y_dist
        print(f"Object Position (x, y): {obj_x_pos,obj_y_pos}\n")
        #前面x,左右y
        
    # 显示画面
    cv2.imshow("ArUco Detection & Pose Estimation", frame)
    if cv2.waitKey(1) == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()