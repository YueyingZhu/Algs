#!/usr/bin/env python3
"""
根据pose数据计算MSCKF-VIO所需的变换矩阵
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

def pose_to_transform_matrix(x, y, z, roll, pitch, yaw):
    """
    将pose (x,y,z,roll,pitch,yaw) 转换为4x4变换矩阵
    """
    # 创建旋转矩阵
    rotation = R.from_euler('xyz', [roll, pitch, yaw])
    rotation_matrix = rotation.as_matrix()
    
    # 创建变换矩阵
    transform = np.eye(4)
    transform[:3, :3] = rotation_matrix
    transform[:3, 3] = [x, y, z]
    
    return transform

def calculate_cam_transforms():
    """
    计算相机变换矩阵
    """
    # cam0的pose数据 (IMU到cam0的变换)
    cam0_x, cam0_y, cam0_z = 0.0452716, -0.07270481, 0.02461546
    cam0_roll, cam0_pitch, cam0_yaw = -1.56726705684909, 0.00122117029896055, 3.13880406790684
    
    # cam1的pose数据 (IMU到cam1的变换)
    cam1_x, cam1_y, cam1_z = -0.05429731, -0.07303667, 0.02470164
    cam1_roll, cam1_pitch, cam1_yaw = -1.56918030608467, -0.000134300001005714, -3.14042090330584
    
    # 计算变换矩阵 (IMU到相机的变换)
    T_imu_cam0 = pose_to_transform_matrix(cam0_x, cam0_y, cam0_z, cam0_roll, cam0_pitch, cam0_yaw)
    T_imu_cam1 = pose_to_transform_matrix(cam1_x, cam1_y, cam1_z, cam1_roll, cam1_pitch, cam1_yaw)
    
    # MSCKF-VIO需要的是T_cam_imu (camera to IMU)
    T_cam0_imu = np.linalg.inv(T_imu_cam0)
    T_cam1_imu = np.linalg.inv(T_imu_cam1)
    
    # 计算T_cn_cnm1 (cam1 to cam0) = T_cam0_imu @ T_imu_cam1
    T_cam1_cam0 = T_cam0_imu @ T_imu_cam1
    
    print("T_cam0_imu (cam0 to IMU):")
    print_matrix(T_cam0_imu)
    
    print("\nT_cam1_imu (cam1 to IMU):")
    print_matrix(T_cam1_imu)
    
    print("\nT_cam1_cam0 (cam1 to cam0):")
    print_matrix(T_cam1_cam0)
    
    # 验证约束关系
    print("\n验证约束关系:")
    print("T_cam0_imu @ T_imu_cam1 = T_cam1_cam0")
    verification = T_cam0_imu @ T_imu_cam1
    print("计算值:")
    print_matrix(verification)
    print("期望值:")
    print_matrix(T_cam1_cam0)
    print(f"差异: {np.max(np.abs(verification - T_cam1_cam0))}")
    
    # 验证旋转矩阵的正交性
    print(f"\nT_cam0_imu旋转矩阵行列式: {np.linalg.det(T_cam0_imu[:3,:3])}")
    print(f"T_cam1_imu旋转矩阵行列式: {np.linalg.det(T_cam1_imu[:3,:3])}")
    print(f"T_cam1_cam0旋转矩阵行列式: {np.linalg.det(T_cam1_cam0[:3,:3])}")
    
    return T_cam0_imu, T_cam1_imu, T_cam1_cam0

def print_matrix(matrix):
    """
    以MSCKF-VIO YAML格式打印矩阵
    """
    print("    [", end="")
    for i in range(4):
        if i > 0:
            print("    ", end="")
        for j in range(4):
            if i == 0 and j == 0:
                print(f"{matrix[i,j]:.15f}", end="")
            else:
                print(f"{matrix[i,j]:.15f}", end="")
            if j < 3:
                print(", ", end="")
        if i < 3:
            print(",")
        else:
            print("]")

if __name__ == "__main__":
    calculate_cam_transforms()
