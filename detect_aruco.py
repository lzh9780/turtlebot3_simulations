from math import cos, sin, pi
import cv2
import numpy as np

# 初始化摄像头
# cap = cv2.VideoCapture(0)  # 0表示默认摄像头


cap = rgb_image



# Define ArUco dictionary and detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector = cv2.aruco.ArucoDetector(aruco_dict)

# --------------------------
# 设置宽度
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# 设置高度
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# 假设摄像头分辨率为 1280x720
# image_width = 1280
# image_height = 720



# 标准相机内参矩阵（假设焦距为图像宽度）
camera_matrix = np.array([
    [530.467, 0, 320.5],  # fx, 0, cx
    [0, 530.467, 240.5], # 0, fy, cy (假设fx ≈ fy)
    [0, 0, 1]
], dtype=np.float32)

# 假设无畸变（实际相机通常有畸变，此处仅测试用）
dist_coeffs = np.zeros((5, 1), dtype=np.float32)

# 假设相机安装在机器人基座的前方 0.1 米，高度 0.05 米（需根据实际测量修改）
CAMERA_OFFSET_X = 0.1  # 相机在机器人基座前方的水平偏移
CAMERA_OFFSET_Y = 0.0  # 横向偏移
CAMERA_HEIGHT = 0.05   # 垂直高度

# 定义ArUco标记的实际物理尺寸（单位：米）
marker_length = 0.05  # 例如：10cm x 10cm的标记

# --------------------------
# 主循环
# --------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        break
     
       
    # detect ArUco marker
    corners, ids, _ = detector.detectMarkers(frame)
    global_pose = {}
    
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs
        )

        for i in range(len(ids)):
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)
            id = ids[i][0]
            # --------------------------
            # 1. Get the coordinate of the code in camera
            x_cam = tvecs[i][0][0]
            y_cam = tvecs[i][0][1]
            z_cam = tvecs[i][0][2]

            # 2. Convert the camera coordinate system to the robot base coordinate system (accounting for mounting offset)
            # Assume the camera is facing forward (same direction as the robot base)
            x_base = z_cam + CAMERA_OFFSET_X
            y_base = x_cam + CAMERA_OFFSET_Y
                            
            
            # 3. Get the robot's global position (assuming it is known)
            rob_rot_rad = 1.68    # Global orientation of the robot (angle)
            rob_x_global = 1.0    # The robot's X position in the global coordinate system
            rob_y_global = 1.5    # The robot's Y position in the global coordinate system
            

            # 4. Convert the base coordinate system to the global coordinate system
            obj_x_global = rob_x_global + x_base * cos(rob_rot_rad) - y_base * sin(rob_rot_rad)
            obj_y_global = rob_y_global + x_base * sin(rob_rot_rad) + y_base * cos(rob_rot_rad)
            
            global_pose[id] = (obj_x_global, obj_y_global)

            # print(f"MarkID: {ids[i][0]}")
            # print(f"Camera坐标 (x, y, z): {x_cam:.2f}, {y_cam:.2f}, {z_cam:.2f}")
            # print(f"全局坐标 (x, y): {x_global:.2f}, {y_global:.2f}\n")
    
              



        
        #获取turtlebot当前位置
        #暂时设置为x = 1, y = 1.5 angle = 58 degree
        # rob_x_pos = 1
        # rob_y_pos = 1.5
        # rob_rot = pi/3 
        # dist = tvecs[i][0][2]
        # x_dist = cos(rob_rot)*dist
        # y_dist = sin(rob_rot)*dist
        # obj_x_pos = rob_x_pos+x_dist
        # obj_y_pos = rob_y_pos+y_dist
        # print(f"Object Position (x, y): {obj_x_pos,obj_y_pos}\n")
        #前面x,左右y
        
    # 显示画面
    # cv2.imshow("ArUco Detection & Pose Estimation", frame)
    # if cv2.waitKey(1) == ord('q'):
    #     break

# 释放资源
# cap.release()
# cv2.destroyAllWindows()