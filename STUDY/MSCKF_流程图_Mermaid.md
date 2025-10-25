# MSCKF 数据流和逻辑流程图 (Mermaid格式)

## 1. 整体系统架构

```mermaid
graph TB
    A[IMU数据] --> C[ImageProcessor前端]
    B[双目图像] --> C
    C --> D[特征数据]
    D --> E[MsckfVio后端]
    E --> F[位姿估计结果]
    
    subgraph "前端处理"
        C1[imuCallback] --> C2[imu_msg_buffer]
        C3[stereoCallback] --> C4[trackFeatures]
        C4 --> C5[特征跟踪]
        C5 --> C6[publish features]
    end
    
    subgraph "后端处理"
        E1[featureCallback] --> E2[batchImuProcessing]
        E2 --> E3[stateAugmentation]
        E3 --> E4[addFeatureObservations]
        E4 --> E5[removeLostFeatures]
        E5 --> E6[pruneCamStateBuffer]
        E6 --> E7[publish odometry]
    end
```

## 2. 前端特征跟踪流程

```mermaid
graph TD
    A[双目图像到达] --> B[stereoCallback]
    B --> C{是否第一帧?}
    C -->|是| D[提取初始特征]
    C -->|否| E[trackFeatures]
    
    E --> F[1. 左图前后帧跟踪]
    F --> F1[integrateImuData]
    F1 --> F2[predictFeatureTracking]
    F2 --> F3[LK光流跟踪]
    
    F3 --> G[2. 左右图跟踪]
    G --> G1[相机外参预测]
    G1 --> G2[LK光流跟踪]
    
    G2 --> H[3. RANSAC剔除外点]
    H --> H1[左图前后帧RANSAC]
    H --> H2[右图前后帧RANSAC]
    
    H1 --> I[addNewFeatures]
    H2 --> I
    I --> J[pruneGridFeatures]
    J --> K[publish features]
    
    D --> K
```

## 3. 后端主处理流程

```mermaid
graph TD
    A[特征数据到达] --> B[featureCallback]
    B --> C{重力是否已设置?}
    C -->|否| D[返回]
    C -->|是| E[batchImuProcessing]
    
    E --> E1[处理IMU数据]
    E1 --> E2[IMU预积分]
    E2 --> E3[状态预测]
    
    E3 --> F[stateAugmentation]
    F --> F1[计算相机位姿]
    F1 --> F2[扩充状态向量]
    F2 --> F3[扩充协方差矩阵]
    
    F3 --> G[addFeatureObservations]
    G --> G1[更新map_server]
    G1 --> G2[计算跟踪比例]
    
    G2 --> H[removeLostFeatures]
    H --> H1{特征是否跟丢?}
    H1 -->|是| H2[特征三角化]
    H1 -->|否| I[pruneCamStateBuffer]
    H2 --> H3[观测更新]
    H3 --> I
    
    I --> I1{相机状态数量是否超限?}
    I1 -->|否| J[publish]
    I1 -->|是| I2[移除冗余状态]
    I2 --> I3[观测更新]
    I3 --> J
```

## 4. 特征跟踪详细流程

```mermaid
graph TD
    A[trackFeatures开始] --> B[计算网格大小]
    B --> C[integrateImuData]
    C --> C1[收集IMU数据]
    C1 --> C2[计算平均角速度]
    C2 --> C3[计算相对旋转]
    
    C3 --> D[组织上一帧特征]
    D --> E[前后帧跟踪]
    E --> E1[预测特征点位置]
    E1 --> E2[LK光流跟踪]
    E2 --> E3[极线约束检查]
    
    E3 --> F[左右图跟踪]
    F --> F1[预测右图特征点]
    F1 --> F2[LK光流跟踪]
    F2 --> F3[极线约束检查]
    
    F3 --> G[RANSAC剔除外点]
    G --> G1[2-point RANSAC]
    G1 --> G2[剔除外点]
    
    G2 --> H[更新特征状态]
    H --> I[返回跟踪结果]
```

## 5. 观测更新流程

```mermaid
graph TD
    A[观测更新触发] --> B{触发条件}
    B -->|特征跟丢| C[removeLostFeatures]
    B -->|相机状态修剪| D[pruneCamStateBuffer]
    
    C --> C1[遍历map_server]
    C1 --> C2{特征是否跟丢?}
    C2 -->|是| C3{观测数量≥3?}
    C2 -->|否| C1
    C3 -->|是| C4[特征三角化]
    C3 -->|否| C5[移除特征]
    C4 --> C6[计算观测雅可比]
    C6 --> C7[EKF更新]
    C7 --> C5
    
    D --> D1[检查相机状态数量]
    D1 --> D2{是否超限?}
    D2 -->|否| E[结束]
    D2 -->|是| D3[findRedundantCamStates]
    D3 --> D4[选择要移除的状态]
    D4 --> D5[处理相关特征]
    D5 --> D6[观测更新]
    D6 --> E
```

## 6. 状态扩增流程

```mermaid
graph TD
    A[stateAugmentation] --> B[获取IMU状态]
    B --> C[计算相机位姿]
    C --> C1[R_w_c = R_i_c * R_w_i]
    C1 --> C2[t_c_w = t_w_i + R_w_i^T * t_c_i]
    
    C2 --> D[创建新相机状态]
    D --> D1[设置位置: t_c_w]
    D1 --> D2[设置姿态: R_w_c]
    D2 --> D3[设置时间戳]
    
    D3 --> E[扩充协方差矩阵]
    E --> E1[计算新相机自身协方差]
    E1 --> E2[计算与IMU的交叉协方差]
    E2 --> E3[更新state_cov]
    
    E3 --> F[添加到cam_states]
    F --> G[完成状态扩增]
```

## 7. 初始化流程

```mermaid
graph TD
    A[系统启动] --> B[等待IMU数据]
    B --> C[imuCallback]
    C --> D{IMU数据数量<200?}
    D -->|是| E[存储到buffer]
    D -->|否| F[initializeGravityAndBias]
    
    F --> F1[计算平均加速度]
    F1 --> F2[计算平均角速度]
    F2 --> F3[计算重力向量]
    F3 --> F4[计算初始旋转]
    F4 --> F5[设置陀螺仪bias]
    F5 --> F6[设置重力方向]
    F6 --> G[初始化完成]
    
    E --> C
    G --> H[开始正常处理]
```

## 8. 数据流时序图

```mermaid
sequenceDiagram
    participant IMU as IMU传感器
    participant CAM as 双目相机
    participant IP as ImageProcessor
    participant MV as MsckfVio
    participant OUT as 输出
    
    IMU->>IP: IMU数据
    CAM->>IP: 双目图像
    
    IP->>IP: 特征跟踪
    IP->>MV: 特征数据
    
    MV->>MV: batchImuProcessing
    MV->>MV: stateAugmentation
    MV->>MV: addFeatureObservations
    MV->>MV: removeLostFeatures
    MV->>MV: pruneCamStateBuffer
    
    MV->>OUT: 位姿估计
```

这些流程图清晰地展示了MSCKF算法的数据流和逻辑结构，帮助你理解代码的整体架构和各个模块之间的交互关系。

