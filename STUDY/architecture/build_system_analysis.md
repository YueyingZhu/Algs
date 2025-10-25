# 12个SLAM算法构建系统分析

## 构建系统概览

### 1. AirSLAM
- **构建系统**: CMake + ROS (catkin)
- **C++标准**: C++17
- **主要依赖**:
  - OpenCV 4.2
  - Eigen3
  - CUDA + TensorRT (深度学习加速)
  - Ceres + G2O (优化框架)
  - yaml-cpp, Boost, Gflags, Glog
- **优化框架**: Ceres + G2O (混合使用)
- **特殊功能**: TensorRT加速的深度学习特征提取

### 2. Basalt
- **构建系统**: CMake (无ROS依赖)
- **C++标准**: C++17
- **主要依赖**:
  - Eigen3 3.4.0
  - TBB (并行计算)
  - OpenCV (无版本限制)
  - fmt (格式化库)
  - Pangolin (可视化)
- **优化框架**: 自研优化器 (平方根优化)
- **特殊功能**: 支持多种相机模型，非线性因子恢复

### 3. DM-VIO
- **构建系统**: CMake (基于DSO)
- **C++标准**: C++14
- **主要依赖**:
  - SuiteSparse, Eigen3, Boost
  - GTSAM (优化框架)
  - yaml-cpp
  - Pangolin, OpenCV (可选)
- **优化框架**: GTSAM
- **特殊功能**: 延迟边缘化策略

### 4. DSO
- **构建系统**: CMake (纯C++)
- **C++标准**: C++0x/C++11
- **主要依赖**:
  - SuiteSparse, Eigen3, Boost
  - Pangolin, OpenCV (可选)
  - LibZip (可选)
- **优化框架**: 自研优化器
- **特殊功能**: 直接法，光度标定

### 5. Kimera-VIO
- **构建系统**: CMake (无ROS依赖)
- **C++标准**: C++17
- **主要依赖**:
  - GTSAM (优化框架)
  - OpenCV 4
  - OpenGV (几何视觉)
  - DBoW2 (回环检测)
  - KimeraRPGO (位姿图优化)
  - Pangolin (可选)
- **优化框架**: GTSAM
- **特殊功能**: 度量语义SLAM，3D动态场景图

### 6. MSCKF-VIO
- **构建系统**: CMake + ROS (catkin)
- **C++标准**: C++14
- **主要依赖**:
  - Eigen3, OpenCV, Boost
  - SuiteSparse
  - ROS消息系统
- **优化框架**: 卡尔曼滤波 (SuiteSparse)
- **特殊功能**: 多状态约束卡尔曼滤波

### 7. OpenVINS
- **构建系统**: CMake + ROS (支持ROS1/ROS2)
- **C++标准**: C++14
- **主要依赖**:
  - Eigen3, OpenCV (3或4)
  - Boost
  - Python (matplotlib, numpy)
- **优化框架**: 卡尔曼滤波
- **特殊功能**: 模块化协方差类型系统，5种特征表示

### 8. ORB-SLAM3
- **构建系统**: CMake (无ROS依赖)
- **C++标准**: C++11
- **主要依赖**:
  - OpenCV 4.4
  - Eigen3 3.1.0
  - Pangolin
  - g2o (优化框架)
  - DBoW2 (回环检测)
- **优化框架**: g2o
- **特殊功能**: 多地图系统，Atlas地图管理

### 9. PL-SLAM
- **构建系统**: CMake (无ROS依赖)
- **C++标准**: C++14
- **主要依赖**:
  - OpenCV 3
  - g2o (优化框架)
  - Eigen3
  - DBoW2, line_descriptor
  - MRPT (可选，用于可视化)
- **优化框架**: g2o
- **特殊功能**: 点线特征融合

### 10. SchurVINS
- **构建系统**: CMake + ROS (catkin)
- **C++标准**: C++14
- **主要依赖**:
  - 基于SVO2.0架构
  - Ceres (优化框架)
  - Eigen3, OpenCV
  - yaml-cpp
- **优化框架**: Ceres
- **特殊功能**: Schur补分解，等效残差模型

### 11. SVO2.0
- **构建系统**: CMake + ROS (catkin)
- **C++标准**: C++14
- **主要依赖**:
  - 基于SVO架构
  - Ceres + iSAM2 (优化框架)
  - Eigen3, OpenCV
  - DBoW2 (回环检测)
- **优化框架**: Ceres + iSAM2
- **特殊功能**: 半直接法，主动曝光控制

### 12. VINS-Fusion
- **构建系统**: CMake + ROS (catkin)
- **C++标准**: C++11
- **主要依赖**:
  - Ceres (优化框架)
  - OpenCV, Eigen3
  - ROS消息系统
- **优化框架**: Ceres
- **特殊功能**: 多传感器融合，在线时空标定

## 优化框架分类

### 1. 自研优化器
- **DSO**: 自研直接法优化器
- **Basalt**: 自研平方根优化器

### 2. Ceres Solver
- **AirSLAM**: Ceres + G2O混合
- **SchurVINS**: 纯Ceres
- **SVO2.0**: Ceres + iSAM2
- **VINS-Fusion**: 纯Ceres

### 3. GTSAM
- **DM-VIO**: 纯GTSAM
- **Kimera-VIO**: 纯GTSAM

### 4. g2o
- **ORB-SLAM3**: 纯g2o
- **PL-SLAM**: 纯g2o

### 5. 卡尔曼滤波
- **MSCKF-VIO**: SuiteSparse + 卡尔曼滤波
- **OpenVINS**: 卡尔曼滤波

## 构建系统特点

### 1. ROS依赖
- **有ROS依赖**: AirSLAM, MSCKF-VIO, OpenVINS, SchurVINS, SVO2.0, VINS-Fusion
- **无ROS依赖**: Basalt, DSO, Kimera-VIO, ORB-SLAM3, PL-SLAM, DM-VIO

### 2. C++标准演进
- **C++11**: DSO, ORB-SLAM3, VINS-Fusion
- **C++14**: DM-VIO, MSCKF-VIO, OpenVINS, PL-SLAM, SchurVINS, SVO2.0
- **C++17**: AirSLAM, Basalt, Kimera-VIO

### 3. 深度学习支持
- **AirSLAM**: TensorRT + CUDA支持
- **其他**: 传统计算机视觉方法

### 4. 可视化支持
- **Pangolin**: 大部分算法支持
- **MRPT**: PL-SLAM可选支持
- **OpenCV**: 所有算法都支持

## 依赖复杂度分析

### 轻量级依赖
- **DSO**: 最少依赖，仅需SuiteSparse, Eigen3, Boost
- **MSCKF-VIO**: 相对简单，主要依赖ROS生态

### 中等复杂度
- **ORB-SLAM3**: 标准依赖，g2o + DBoW2
- **PL-SLAM**: 标准依赖，g2o + 线特征库
- **VINS-Fusion**: 标准依赖，Ceres + ROS

### 高复杂度
- **AirSLAM**: 深度学习 + 传统优化混合
- **Kimera-VIO**: 多个专业库 (GTSAM, OpenGV, DBoW2)
- **Basalt**: 自研优化器 + 多种相机模型
- **SVO2.0**: Ceres + iSAM2 + 回环检测

## 总结

1. **优化框架选择**: 从自研到成熟库的演进
2. **C++标准**: 从C++11到C++17的现代化
3. **ROS依赖**: 从强依赖到可选依赖的灵活性
4. **深度学习**: 开始融入传统SLAM系统
5. **模块化**: 从单一系统到模块化设计

