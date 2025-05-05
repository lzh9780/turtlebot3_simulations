import cv2
import numpy as np
from scipy.optimize import least_squares
from math import cos, sin, radians  # 

def detect_cube_position(tvecs, cube_size):
    """

    """
    L = cube_size / 2
    local_positions = [
        np.array([0, 0, L]),
        np.array([L, 0, 0]),
        np.array([0, L, 0]),
        np.array([0, 0, -L]),
        np.array([-L, 0, 0]),
        np.array([0, -L, 0])
    ]

    def residual(params, observations):
        R = params[:9].reshape(3, 3)
        T = params[9:12]
        residuals = []
        for i, P_obs in enumerate(observations):
            local_idx = i % 6
            P_local = local_positions[local_idx]
            P_pred = R @ P_local + T
            residuals.extend(P_pred - P_obs)
        return residuals

    if len(tvecs) < 1:
        return None, None  # 不足以优化

    observations = [tvec[0] for tvec in tvecs]
    initial_params = np.eye(3).flatten().tolist() + [0.0, 0.0, 0.0]

    result = least_squares(
        residual,
        initial_params,
        args=(observations,),
        method='trf',
        max_nfev=200
    )

    R_optimized = result.x[:9].reshape(3, 3)
    T_optimized = result.x[9:12]

    # 正交化旋转矩阵
    U, _, Vt = np.linalg.svd(R_optimized)
    R_optimized = U @ Vt

    return T_optimized, R_optimized


# 示例用法
if __name__ == "__main__":
    # 加载相机标定参数
    camera_matrix = np.array([
    [530.4669406576809, 0, 320.5],  # 
    [0, 530.4669406576809, 240.5],  # 
    [0, 0, 1]
    ], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)
    # 假设输入图像为frame
    frame = cv2.imread("3.png")
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)
    detector = cv2.aruco.ArucoDetector(aruco_dict)
    corners, ids, _ = detector.detectMarkers(frame)

    if ids is None:
        print("未检测到任何标记")
        exit()

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.045, camera_matrix, dist_coeffs)

    # 分组：每 6 个 ID 归为一个 cube（你可以根据实际情况调整）
    cube_dict = {}  # cube_idx -> list of (id, tvec)
    for i, marker_id in enumerate(ids.flatten()):
        cube_idx = marker_id // 6
        if cube_idx not in cube_dict:
            cube_dict[cube_idx] = []
        cube_dict[cube_idx].append((marker_id, tvecs[i]))

    # 对每个方块估计位置
    for cube_idx, marker_data in cube_dict.items():
        aruco_ids = [item[0] for item in marker_data]
        tvec_list = [item[1] for item in marker_data]
        T, R = detect_cube_position(tvec_list, cube_size=0.05)
        if T is not None:
            print(f"\n🧊 Cube {cube_idx}")
            print(f"AruCo IDs: {aruco_ids}")
            print(f"中心位置: {T}")
            print(f"旋转矩阵:\n{R}")

            rob_x_global = -0.846661    # 全局X位置
            rob_y_global = 1.217943    # 全局Y位置
            rob_rot_rad = 2.948617      # 机器人朝向角度

            x_cam = T[0]
            y_cam = T[1]
            z_cam = T[2]

            x_base = z_cam + 0.076
            y_base = -x_cam + 0
           

            x_global = rob_x_global + x_base * cos(rob_rot_rad) - y_base * sin(rob_rot_rad)
            y_global = rob_y_global + x_base * sin(rob_rot_rad) + y_base * cos(rob_rot_rad)


            print(f"Global coordinates (x, y): {x_global:.3f}, {y_global:.3f}\n")



        else:
            print(f"\n❌ Cube {cube_idx} ID {aruco_ids} 数据不足，跳过优化。")
