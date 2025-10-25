# DROID-SLAM: 单目、双目、RGBD相机的端到端视觉SLAM

 **Author:** [方川]

 **Link:** [https://zhuanlan.zhihu.com/p/434471738]

标题：DROID-SLAM: Deep Visual SLAM for Monocular Stereo, and RGB-D Cameras

作者：Zachary Teed, Jis Deng

来源：arXiv 2021

大家好，今天给大家带来的Princeton VL的SLAM工作: DROID-SLAM: Deep Visual SLAM for Monocular Stereo, and RGB-D Cameras. 这篇工作把Visual-SLAM问题使用深度神经网络直接端到端的实现了，并且取得了比以往传统SLAM方案更高的精度和鲁棒性。

  


## 摘要  
本文提出了DROID-SLAM, 一个全新的基于深度学习的SLAM系统. DROID-SLAM通过一个深度BA层来循环迭代的更新相机位姿和像素深度值. 实验证明, DROID-SLAM比传统SLAM取得了更高的精度和鲁棒性, 在实验场景中几乎不会失败. 尽管我们只在单目视频上训练了我们的网络, 但是在测试阶段，这个网络仍然可以在双目和RGB-D视频上取得很好的表现.

DROID-SLAM主要贡献:

* High Accuracy: 在TartanAir、ETH-3D、EuRoc、TUM-RGBD数据集上处于领先地位, 并且极大地超越原先方法;
* High Robustness: 非常稳定的跟踪能力, 除了ETH-3D, 其他数据集上没有出现fail情况;
* Strong Generalization: 虽然本文只使用单目视频训练, 但是在测试阶段直接使用双目视频和RGB-D作为输入, 仍然得到了很好的预测结果;

  


## 算法流程:  
**Notation**:

 算法的输入 $\{\mathbf I_t\}^N_{t=0}$ 是一个视频或者图像序列. 每一张图像, 算法的目标是求解相机位姿 $\mathbf G_t \in SE(3)$ 和 逆深度 $\mathbf d_t \in \mathbb R ^{H \times W}_{+}$ . 相机位姿序列 $\{\mathbf G_t\}^N_{t=0}$ 和逆深度序列 $\{\mathbf d_t\}^N_{t=0}$ 是未知的的系统状态变量, 在输入的图像序列中迭代的更新最终收敛. 同时, 本文设计了一个frame-graph结构 $(\mathcal V, \varepsilon)$ 表示图像之间的共视关系, 边 $(i,j) \in \varepsilon$ 表示 $\mathbf I_i$ 和 $\mathbf I_j$ 之间存在共视关系. frame-graph随着位姿和深度的更新也会动态更新, 对于回环检测, 我们在frame-graph中添加长的回环边.

  


### 一、特征提取和关联:  
特征提取和关联这一部分与RAFT中的特征网络和相关信息体correlation volume完全一致.

**特征提取网络**: feature network由6个残差模块和3个下采样层组成, 生成输入图像1/8分辨率的feature map; context network提取图像全局信息, 作为下文更新算子的一个输入;

**Correlation Pyramid**: 对于frame-graph中的每条边, 本文使用点积计算一个4D的correlation volume:

![]((20211118)DROID-SLAM_单目双目RGBD相机的端到端视觉SLAM_方川/v2-6174ae293df26184456605178dabf3a8_1440w.png)  
**Correlation Lookup**: 根据像素坐标和查找半径, 查找算子会在不同分辨率的correlation volume上查找对应的相关信息张量，最后将他们拼接为一个feature vector;

  


### 二、更新算子:  
更新模块是本文网络的核心模块, 整体结构如下图所示:

![]((20211118)DROID-SLAM_单目双目RGBD相机的端到端视觉SLAM_方川/v2-7d858e9ea87c2586869811ad2b51c751_1440w.jpg)  
$\mathbf C_{ij}$ 是图像 $I_i$ 与 $I_j$ 之间的相关信息张量, $\mathbf h_{ij}$ 是隐藏状态. 每轮更新之后, $\mathbf h_{ij}$ 会被更新, 并且输出位姿变化 $\Delta \xi^{(k)}$ 和深度变化 $\Delta d^{(k)}$ , 然后更新下一帧的位姿和深度:

![]((20211118)DROID-SLAM_单目双目RGBD相机的端到端视觉SLAM_方川/v2-cc0babf186cebe1bb77c239ee00a1c67_1440w.png)  
**Correspondece**: 在每次更新之前, 根据当前的pose和depth, 对于图像 $\mathbf I_i$ 中的每个网格, 网格中的像素坐标集合 $p_i \in \mathbb R^{H \times W \times 2}$ , 那么它在图像 $\mathbf I_j$ 上的对应网格像素集合 $p_{ij}$ 可以表示为:

![]((20211118)DROID-SLAM_单目双目RGBD相机的端到端视觉SLAM_方川/v2-2c57aaca5bda7c6dd86d74bd59ef743d_1440w.png)  
  


**Input**: 根据上一步计算得到的grid correspondence, 查找两张图像的correlation volume $\mathbf C_{ij}$ ； 同时可以根据 $p_i$ 和 $p_{ij}$ 计算两张图像之间的光流场flow fileds. $\mathbf C_{ij}$ 表征的是两张图像之间的视觉一致程度, 更新模块的目标是计算相对位姿来对齐两张图像使之一致性最大化. 但是因为视觉一致性会存在奇异, 因此我们同时利用光流场来提高位姿估计的鲁棒性.

**Update**:

与RAFT网络一样, correlation features 和 flow features经过两层卷积之后, 与context feature一同送入GRU模块. 在GRU模块中, 本文对隐藏状态 $h_{ij}$ 作average pooling来提取global context, global context对于提高剧烈运动物体的光流估计鲁棒性有帮助.

GRU模块同时更新隐藏状态得到 $\mathbf h^{(k+1)}$ , 我们利用这个隐藏状态张量得到光流场的变化量 $\mathbf r_{ij} \in \mathbb R^{H \times W \times 2}$ 和对应的置信度 $\mathbf w_{ij} \in \mathbb R^{H \times W \times 2}$ , 则修正后的网格 $p^*_{ij} = \mathbf r_{ij}  + p_{ij}$ . 

再利用 $\mathbf h^{(k+1)}$ 得到pixel-wise的阻尼系数矩阵 $\lambda$ 和 用于深度估计过程中的8x8的上采样mask;

  


**Dense Bundle Adjustment Layer**:

首先, DBA层将更新模块输出的稠密光流场变化量转换为相机位姿和稠密深度值: 这里的相机位姿可以使用传统方法计算得到, 深度值则是根据下面的目标函数和舒尔补公式, 迭代优化的方式得到的.

![]((20211118)DROID-SLAM_单目双目RGBD相机的端到端视觉SLAM_方川/v2-ab64b9248f71ec00f6d4c6818c54dbef_1440w.png)  
这里仍然是使用Guass-Newton法计算位姿和深度的变化量, 利用舒尔补先计算位姿变化量, 再计算深度变化量

![]((20211118)DROID-SLAM_单目双目RGBD相机的端到端视觉SLAM_方川/v2-7a580953447d175645996e863f678da8_1440w.png)  
​ 分别代表相机位姿和深度的梯度方向.

DBA 层的实现和反向传播是基于LieTorch的, 是一个李群李代数的pytorch实现的package.

### 三、训练过程  
单目尺度问题: 为了解决单目SLAM尺度问题, 我们把前两帧图像位姿固定为ground truth.

采样视频/图像序列: 为了使我们的网络泛化能力更好, 我们通过计算图像序列中任意两张图像之间的光流场距离, 对这样的 $N_i \times N_i$ flow distance matrix进行采样，得到采样后的新的图像序列形成的视频来输入网络进行训练.

监督和loss: 监督信息是ground truth pose和ground truth flow. loss是 网络预测的flow fileds与ground truth flow fileds的 loss. 图像位姿loss则是 $\mathcal L_{pose} = \Sigma_i \left \|  Log_{SE3}(\mathbf T^{-1}_i \cdot \mathbf G_i)\right \|_2$ 

  


### 四、SLAM system  
和以前的SLAM系统一样, 本文方法实现的SLAM系统也包括前端和后端两个线程. 前端线程提取特征、选取关键帧、局部优化. 后端线程对所有关键帧进行全局优化.

**初始化**: 选择视频中前部的12帧图像来完成初始化, 相邻两帧图像之间的光流必须大于16px, 12帧图像集齐后, 我们建立起一个frame-graph, 并且运行10次更新算子的迭代.

**视觉前端**: 前端部分选取并维护着关键帧序列. 新的图像到来时, 进行特征提取、计算光流场, 并根据光流场计算3个与之共视程度最大的关键帧, 然后根据共视关系来迭代更新当前关键帧的pose和depth. 同时, 前端部分也负责边缘化操作.

**后端优化**: 后端部分将所有关键帧进行BA优化. 每次进行更新迭代时，都会对所有关键帧重新构建frame-graph, 这是根据所有关键帧之间的光流场距离矩阵来构建的. 接着在frame-graph上运行更新算子, 在BA缓解我们使用的是LieTorch. 后端网络只在关键帧上运行BA优化, 对于视频中的普通帧, 只优化相机pose.

**Stereo and RGB-D**:

为了使我们设计的这个SLAM系统能够很好的应用到双目和RGB-D的场景中， 我们会在Stereo和RGB-D的情景中对公式(4)做一些修改. 比如在RGB-D场景中, 公式4添加一个残差项: 估计的depth map与测量的depth map之间的距离平方和. 在Stereo场景中, 公式4改为左右两个相机的位姿重投影误差.、

  


## 五、实验结果  
本文提出的方法在多个数据集上进行了充分的实验, 并且和其他深度学习方法以及经典的SLAM算法做了比较. 

实验部分着重比较了相机轨迹的绝对轨迹误差ATE.

本文中的网络在合成的数据集TartanAir上面训练了250k次, 图像分辨率为384x512, 在4块 RTX-3090上训练了1周时间.

![]((20211118)DROID-SLAM_单目双目RGBD相机的端到端视觉SLAM_方川/v2-831dbc50ead1c8737961b90ceec58df7_1440w.jpg)  
![]((20211118)DROID-SLAM_单目双目RGBD相机的端到端视觉SLAM_方川/v2-73a37df421ae89f0c4ed15d0689a8b26_1440w.jpg)  
![]((20211118)DROID-SLAM_单目双目RGBD相机的端到端视觉SLAM_方川/v2-a913145de278a6936696be0890a9f0ec_1440w.jpg)  
 在EuRoc和TUM-RGB-D数据集上也做了充分的实验, 实验正面,本文网络可以很好的泛化到Stereo和RGB-D上，同时取得很高的精度和鲁棒性.

![]((20211118)DROID-SLAM_单目双目RGBD相机的端到端视觉SLAM_方川/v2-1de1d800a157ca6872b380aad6218752_1440w.jpg)  
![]((20211118)DROID-SLAM_单目双目RGBD相机的端到端视觉SLAM_方川/v2-f96919b368a45d25261cb12421f0982e_1440w.jpg)  
![]((20211118)DROID-SLAM_单目双目RGBD相机的端到端视觉SLAM_方川/v2-76188daf640b8b282987ec02bbac807e_1440w.jpg)  
