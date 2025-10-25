# 12个SLAM算法核心架构分析

## 架构设计模式

### 1. ORB-SLAM3 (多线程模块化架构)
```cpp
class System {
    // 核心模块
    Tracking* mpTracker;           // 跟踪线程
    LocalMapping* mpLocalMapper;   // 局部建图线程  
    LoopClosing* mpLoopCloser;     // 回环检测线程
    Atlas* mpAtlas;               // 多地图管理
    
    // 支持模块
    KeyFrameDatabase* mpKeyFrameDatabase;  // 关键帧数据库
    ORBVocabulary* mpVocabulary;          // 词袋模型
    Viewer* mpViewer;                     // 可视化
}
```
**特点**: 经典的多线程SLAM架构，分离跟踪、建图、回环检测

### 2. DSO (直接法单线程架构)
```cpp
class FullSystem {
    // 核心组件
    EnergyFunctional* ef;                    // 能量函数
    std::vector<FrameHessian*> frameHessians; // 帧管理
    CoarseTracker* coarseTracker;            // 粗跟踪
    CoarseInitializer* coarseInitializer;    // 初始化
    
    // 线程管理
    boost::thread mappingThread;             // 建图线程
    boost::mutex trackMutex, mapMutex;       // 线程同步
}
```
**特点**: 直接法，基于光度误差的能量函数优化

### 3. VINS-Fusion (滑动窗口架构)
```cpp
class Estimator {
    // 滑动窗口状态
    Vector3d Ps[(WINDOW_SIZE + 1)];     // 位置
    Vector3d Vs[(WINDOW_SIZE + 1)];     // 速度  
    Matrix3d Rs[(WINDOW_SIZE + 1)];     // 旋转
    Vector3d Bas[(WINDOW_SIZE + 1)];    // 加速度偏置
    Vector3d Bgs[(WINDOW_SIZE + 1)];    // 陀螺仪偏置
    
    // 优化相关
    MarginalizationInfo *last_marginalization_info;  // 边缘化信息
    FeatureManager f_manager;                        // 特征管理
}
```
**特点**: 滑动窗口优化，边缘化策略

### 4. Kimera-VIO (模块化管道架构)
```cpp
class Pipeline {
    // 数据流模块
    MonoDataProviderModule::UniquePtr data_provider_module_;
    VisionImuFrontendModule::UniquePtr vio_frontend_module_;
    VioBackendModule::UniquePtr vio_backend_module_;
    
    // 高级功能模块
    MesherModule::UniquePtr mesher_module_;           // 网格化
    LcdModule::UniquePtr lcd_module_;                // 回环检测
    VisualizerModule::UniquePtr visualizer_module_;  // 可视化
    
    // 线程管理
    std::unique_ptr<std::thread> frontend_thread_;
    std::unique_ptr<std::thread> backend_thread_;
}
```
**特点**: 高度模块化，支持语义建图

### 5. MSCKF-VIO (卡尔曼滤波架构)
```cpp
class MsckfVio {
    struct StateServer {
        IMUState imu_state;              // IMU状态
        CamStateServer cam_states;       // 相机状态
        Eigen::MatrixXd state_cov;       // 状态协方差
    };
    
    // 滤波器组件
    MapServer map_server;                // 地图服务器
    std::vector<sensor_msgs::Imu> imu_msg_buffer;  // IMU缓冲区
}
```
**特点**: 多状态约束卡尔曼滤波，状态扩增策略

### 6. AirSLAM (深度学习+传统优化混合架构)
```cpp
class MapBuilder {
    // 深度学习组件
    FeatureDetectorPtr _feature_detector;    // 深度学习特征检测
    PointMatcherPtr _point_matcher;          // 特征匹配
    
    // 传统优化组件  
    MapPtr _map;                             // 地图管理
    CameraPtr _camera;                       // 相机模型
    
    // 多线程处理
    std::thread _feature_thread;             // 特征提取线程
    std::thread _tracking_thread;            // 跟踪线程
}
```
**特点**: 深度学习特征提取 + 传统几何优化

### 7. SchurVINS (Schur补分解架构)
```cpp
class SchurVINS {
    class ImuState {
        Eigen::Quaterniond quat;         // 四元数
        Eigen::Vector3d pos, vel;        // 位置、速度
        Eigen::Vector3d ba, bg;          // 偏置
        Eigen::Quaterniond quat_fej;     // FEJ四元数
    };
    
    // Schur补相关
    svo::StateMap states_map;            // 状态映射
    std::set<svo::PointPtr> schur_pts_;  // Schur点集合
    Eigen::MatrixXd cov;                 // 协方差矩阵
}
```
**特点**: 基于Schur补的轻量级优化

### 8. SVO2.0 (半直接法架构)
```cpp
class FrameHandlerBase {
    // 核心模块
    SparseImgAlignBasePtr sparse_img_align_;    // 稀疏图像对齐
    std::vector<ReprojectorPtr> reprojectors_;  // 重投影器
    PoseOptimizerPtr pose_optimizer_;           // 位姿优化器
    DepthFilterPtr depth_filter_;               // 深度滤波器
    InitializerPtr initializer_;                // 初始化器
    
    // 后端优化
    AbstractBundleAdjustmentPtr bundle_adjustment_;  // 束调整
}
```
**特点**: 半直接法，稀疏图像对齐 + 特征点跟踪

## 架构分类

### 按线程模型分类

#### 1. 多线程架构
- **ORB-SLAM3**: 跟踪、建图、回环检测分离
- **Kimera-VIO**: 前端、后端、网格化、回环检测分离
- **AirSLAM**: 特征提取、跟踪分离

#### 2. 单线程架构  
- **DSO**: 单线程直接法优化
- **MSCKF-VIO**: 单线程卡尔曼滤波
- **SchurVINS**: 单线程Schur补优化

#### 3. 混合架构
- **VINS-Fusion**: 滑动窗口 + 多线程
- **SVO2.0**: 前端单线程 + 后端多线程

### 按优化策略分类

#### 1. 基于图优化
- **ORB-SLAM3**: g2o图优化
- **Kimera-VIO**: GTSAM图优化
- **PL-SLAM**: g2o图优化

#### 2. 基于卡尔曼滤波
- **MSCKF-VIO**: 多状态约束卡尔曼滤波
- **OpenVINS**: 扩展卡尔曼滤波
- **SchurVINS**: 基于Schur补的卡尔曼滤波

#### 3. 基于直接法
- **DSO**: 光度误差直接优化
- **SVO2.0**: 半直接法优化

#### 4. 基于滑动窗口
- **VINS-Fusion**: 滑动窗口BA
- **DM-VIO**: 延迟边缘化滑动窗口

### 按特征类型分类

#### 1. 点特征
- **ORB-SLAM3**: ORB特征
- **VINS-Fusion**: 角点特征
- **MSCKF-VIO**: 角点特征

#### 2. 点+线特征
- **AirSLAM**: 深度学习点特征 + 结构线
- **PL-SLAM**: ORB点特征 + 线特征

#### 3. 直接法
- **DSO**: 像素级直接法
- **SVO2.0**: 半直接法

### 按传感器融合分类

#### 1. 纯视觉
- **DSO**: 单目直接法
- **PL-SLAM**: 双目点线特征

#### 2. 视觉+IMU
- **VINS-Fusion**: 单目/双目+IMU
- **Kimera-VIO**: 双目+IMU
- **MSCKF-VIO**: 双目+IMU
- **AirSLAM**: 单目/双目+IMU

#### 3. 多传感器
- **VINS-Fusion**: 支持GPS
- **Kimera-VIO**: 支持外部里程计

## 架构演进趋势

### 1. 从单线程到多线程
- **早期**: DSO单线程直接法
- **中期**: ORB-SLAM多线程分离
- **现代**: Kimera-VIO高度模块化

### 2. 从传统到深度学习
- **传统**: 手工特征 (ORB, SIFT)
- **混合**: AirSLAM深度学习+传统优化
- **未来**: 端到端学习

### 3. 从局部到全局
- **局部**: 滑动窗口优化
- **全局**: 多地图系统 (ORB-SLAM3 Atlas)
- **语义**: 度量语义建图 (Kimera-VIO)

### 4. 从单一到多模态
- **单一**: 纯视觉SLAM
- **多模态**: 视觉+IMU+GPS
- **语义**: 几何+语义信息

## 性能特点对比

### 计算复杂度
- **低**: MSCKF-VIO (卡尔曼滤波)
- **中**: ORB-SLAM3 (图优化)
- **高**: Kimera-VIO (多模块)

### 精度
- **高**: VINS-Fusion (滑动窗口BA)
- **中**: ORB-SLAM3 (多地图系统)
- **低**: DSO (直接法，尺度漂移)

### 鲁棒性
- **高**: ORB-SLAM3 (回环检测)
- **中**: Kimera-VIO (多传感器)
- **低**: DSO (光照敏感)

### 实时性
- **高**: MSCKF-VIO (滤波器)
- **中**: VINS-Fusion (滑动窗口)
- **低**: Kimera-VIO (复杂模块)

