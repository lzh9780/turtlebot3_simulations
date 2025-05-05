import cv2
import numpy as np
from scipy.optimize import least_squares

def detect_cube_position(frame, camera_matrix, dist_coeffs, marker_length, cube_size):
    """
    检测立方体的全局位置和姿态（假设各面使用相同ID的ArUco标记）
    
    参数：
    - frame: 输入图像（BGR格式）
    - camera_matrix: 相机内参矩阵（3x3）
    - dist_coeffs: 畸变系数（1x5）
    - marker_length: 单个ArUco标记的物理边长（米）
    - cube_size: 立方体的物理边长（米）
    
    返回：
    - T: 立方体中心在相机坐标系中的位置（3x1向量）
    - R: 立方体的旋转矩阵（3x3矩阵）
    """
    
    # 初始化ArUco检测器
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict)
    
    # 检测所有标记
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return None, None
    
    # 获取各面的姿态（tvecs和rvecs）
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, marker_length, camera_matrix, dist_coeffs
    )
    
    # 定义立方体各面在本地坐标系中的中心位置
    L = cube_size / 2  # 半边长
    local_positions = [
        np.array([0, 0, L]),   # 前面（假设本地坐标系Z轴向前）
        np.array([L, 0, 0]),    # 右面（X轴向右）
        np.array([0, L, 0]),    # 顶面（Y轴向上）
        np.array([0, 0, -L]),   # 后面
        np.array([-L, 0, 0]),   # 左面
        np.array([0, -L, 0])    # 底面
    ]
    
    def residual(params, observations):
        """
        残差函数：计算预测位置与观测位置的差异
        """
        R = params[:9].reshape(3, 3)
        T = params[9:12]
        residuals = []
        for i, P_obs in enumerate(observations):
            # 获取当前面在本地坐标系中的位置（假设检测顺序对应前面、右面、顶面等）
            # 注意：实际中需要根据面的朝向调整local_position的选择
            local_idx = i % 6  # 简单循环使用本地位置（需根据实际情况优化）
            P_local = local_positions[local_idx]
            # 预测的全局位置：R * P_local + T
            P_pred = R @ P_local + T
            # 计算残差（观测值与预测值的差）
            residuals.extend(P_pred - P_obs)
        return residuals
    
    # 初始化参数（单位矩阵旋转，零平移）
    initial_params = np.eye(3).flatten().tolist() + [0.0, 0.0, 0.0]
    
    # 提取观测数据（tvecs的列表）
    observations = [tvec[0] for tvec in tvecs]
    
    # 运行最小二乘优化
    result = least_squares(
        residual,
        initial_params,
        args=(observations,),
        method='lm',  # Levenberg-Marquardt算法
        max_nfev=200  # 最大迭代次数
    )
    
    # 提取优化后的旋转矩阵和平移向量
    R_optimized = result.x[:9].reshape(3, 3)
    T_optimized = result.x[9:12]
    
    # 正交化旋转矩阵（确保旋转矩阵的性质）
    U, _, Vt = np.linalg.svd(R_optimized)
    R_optimized = U @ Vt
    
    return T_optimized, R_optimized

# 示例用法
if __name__ == "__main__":
    # 加载相机标定参数
    camera_matrix = np.load("camera_matrix.npy")
    dist_coeffs = np.load("dist_coeffs.npy")
    
    # 假设输入图像为frame
    frame = cv2.imread("cube_image.jpg")
    
    # 检测立方体位置
    T, R = detect_cube_position(
        frame,
        camera_matrix,
        dist_coeffs,
        marker_length=0.05,  # 单个标记边长5cm
        cube_size=0.1        # 立方体边长10cm
    )
    
    if T is not None:
        print(f"立方体中心坐标（相机坐标系）: {T}")
        print(f"旋转矩阵:\n{R}")