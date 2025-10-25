# MSCKF那些事（二）S-MSCKF试用与源码解析

 **Author:** [紫薯萝卜]

 **Link:** [https://zhuanlan.zhihu.com/p/76347723]

S-MSCKF是宾大Vijay Kumar实验室开源的双目版本MSCKF算法，Kumar无人机领域应该家喻户晓了，它们开源的S-MSCKF精度挺高，代码质量也很高，非常适合入门学习。

源码地址：

[KumarRobotics/msckf\_vio](https://github.com/KumarRobotics/msckf\_vio)## 1. 安装测试  
S-MSCKF是在ROS下运行的package，依赖与ROS，这里假设已经安装了ROS并创建的catkin\_ws。（笔者的测试环境是Ubuntu16.04+ROS Kinetic）

首先下载源码并编译，执行


```
sudo apt-get install libsuitesparse-dev
cd ~/catkin_ws/src
git clone KumarRobotics/msckf_vio
cd ..
catkin_make --pkg msckf_vio --cmake-args -DCMAKE_BUILD_TYPE=Release
```
**注意: catkin\_make时一定要加”-DCMAKE\_BUILD\_TYPE=Release”, 不然后面运行会得不到想要的效果 轨迹会迅速发散**

然后测试，下载EuROC数据集(rosbag版的)，执行：


```
 roslaunch msckf_vio msckf_vio_euroc.launch
 rosrun rviz rviz -d rviz/rviz_euroc_config.rviz 
 rosbag play ~/data/euroc/MH_04_difficult.bag(改成你自己的rosbag文件)
```
可以看到MSCKF的运行图如下，rviz中没有显示轨迹, 可以自行修改代码显示轨迹，运行结束对比和起点的位置，累积误差还是比较小的。

![]((20190802)MSCKF那些事二S-MSCKF试用与源码解析_紫薯萝卜/v2-3c15b06fe7bbda983ef7c0ab15e0a637_1440w.jpg)  


S-MSCKF运行示意图

  
  
**F&Q**

* 运行后Rviz中只有特征跟踪的图像, 没有地图点或者有一些很乱的地图点.

原因是catkin\_make没有打开Release, 需要加上’-DCMAKE\_BUILD\_TYPE=Release’, 参见[http://how to get good performance on euroc dataset · issue /#6 · KumarRobotics/msckf\_vio](http://how to get good performance on euroc dataset · issue /#6 · KumarRobotics/msckf\_vio)

* 代码编译成功，但是跑EUROC数据会很快就发散了。

测试发现是观测更新耗时太长，导致IMU数据丢失。原因是编译时没有添加release，同问题1.

* 运行msckf\_vio\_euroc.launch会报错如下, 原因没去查, 但貌似不影响使用.


> [ERROR] [1518059161.545163249]: Skipped loading plugin with error: XML Document ‘/opt/ros/kinetic/share/gmapping/nodelet\_plugins.xml’ has no Root Element. This likely means the XML is malformed or missing..

## 2. 代码解析  
### 2.1 前端 ImageProcessor  
ImageProcessor中两个回调函数作为数据入口:

* imuCallback：接收IMU数据，将IMU数据存到imu\_msg\_buffer中，这里并不处理数据。
* stereoCallback：接收双目图像，进行双目特征跟踪，先光流跟踪上一帧的特征点，然后提取新的特征点。将跟踪到的特征点publish出去，供后端接收使用。

前端中比较关键的操作是双目特征点跟踪trackFeatures()，它分成三步：

1. 左图特征点前后帧跟踪，得到当前帧左图特征点
2. 当前帧左右图跟踪，得到当前帧右图特征点
3. 左右图分别做前后帧RANSAC剔除外点

前后帧跟踪和左右图跟踪都是用的LK光流，前后帧跟踪会用IMU积分的相对旋转预测特征点在当前帧的位置作为初值(integrateImuData, predictFeatureTracking)；左右图跟踪会用相机外参预测右图特征点位置作为初值。

代码将图像分成了4\*5个网格(grid)，每个网格中最多4个特征点，这样能够使特征点均匀分布在图像上。

### 2.2 后端 MsckfVio  
msckf\_vio中两个回调函数作为数据入口:

* imuCallback：接收IMU数据，将IMU数据存到imu\_msg\_buffer中，这里只会利用开头200帧IMU数据进行静止初始化，不做其他处理。
* featureCallback：接收双目特征，进行后端处理。利用IMU进行EKF Propagation，利用双目特征进行EKF Update。

静止初始化(initializeGravityAndBias)：将前200帧加速度和角速度求平均, 平均加速度的模值g作为重力加速度, 平均角速度作为陀螺仪的bias, 计算重力向量(0,0,-g)和平均加速度之间的夹角(旋转四元数), 标定初始时刻IMU系与world系之间的夹角. **因此MSCKF要求前200帧IMU是静止不动的**

后端代码流程如下：

* batchImuProcessing: 首先对两帧观测之间的所有IMU数据做预积分
* stateAugmentation: 然后对MSCKF的估计状态进行扩充, 将当前的imu\_state加入到状态向量cam\_states中, 并扩充6\*6的covariance matrix
* addFeatureObservations: 将特征添加到map\_server, 将特征添加到对应feature.id的observations(std::map)中, 并计算跟踪已有特征的比例.
* removeLostFeatures: 对于那些lost(当前帧未track)的特征, 剔除掉观测小于3个的特征, 如果没有初始化尝试进行初始化, 剔除掉初始化失败的特征. 对于剩下的特征进行观测更新(measurementUpdate), 然后从map\_server中移除.也就是说只有跟丢后的特征才会用于观测更新
* pruneCamStateBuffer: 当state\_server中cam\_states超过最大个数时, 需要移除冗余的states，也会调用measurementUpdate
+ findRedundantCamStates:取倒数第四个状态左右key state, 如果倒数第3,2个cam\_state与key state之间距离和角度都很小, 则将倒数第3,2个cam\_state移除, 否则移除最历史的cam\_state。移除2个cam states存在rm\_cam\_state\_ids中。
+ map\_server中所有的feature，查找rm\_cam\_state\_ids中的观测，放在involved\_cam\_state\_ids中，当待移除观测为1时，直接将该观测移除，否则，尝试初始化feature，如果初始化失败，则移除所有待删除的观测。
+ 构建Jacobian，进行measurementUpdate。

measurementUpdate在两种情况下触发：

* 特征跟丢了需要移除特征时（removeLostFeatures）
* 相机状态数量达到最大限制需要剔除掉相机状态时（pruneCamStateBuffer）

### 2.3 双目特征观测模型  
* **单个特征 $f_j$ 的对单个相机 $cam_i$ 的观测模型**（measurementJacobian）  
$$
\overbrace{r_i^{(j)}}^{4} = \underbrace{H_{X_i}^{(j)}}_{4\times 6}\overbrace{\tilde{X}_{cam_i}}^{6}+\underbrace{H_{f_i}^{(j)}}_{4\times3}\overbrace{p_{f_j}}^{3}+n_i^{(j)}
$$

其中 $r_i^{(j)}=[\delta u_1,\delta v_1,\delta u_2,\delta v_2]^T$ 是特征 $f_j$ 在相机 $cam_i$ 双目图像下的重投影残差， $X_{cam_i}=[p_{cam_i},q_{cam_i}]$ 是单个相机状态， $p_{f_j}$ 是特征点在世界系下的坐标。

* **单个特征 $f_j$ 对所有观测到该特征的相机的观测模型**（featureJacobian）  
$$
\overbrace{r^{(j)}}^{4n\times1} = \underbrace{H_X^{(j)}}_{4n\times m}\overbrace{\tilde{X}}^{m\times1}+\underbrace{H_f^{(j)}}_{4n\times3}\overbrace{p_{f_j}}^{3\times1}+n^{(j)}
$$

其中 $X$ 是整个状态向量，长度是 $21+6n_{\text{cameras}}$， $n_{\text{cameras}}$ 为sliding windows中相机的个数。21是 $X_{\text{IMU}}$ 的维度，S-MSCKF中IMU状态除了包含常规的15维状态向量 $(p,q,v,b_g,b_a)$ 以外，还包含左右相机外参 $p_{lr},q_{lr}$，所以是15+6=21。为了将特征点从观测模型中移除，需要在观测模型的等式两边乘以 $H_f^{(j)}$ 的左零空间（left nullspaces），得到

$$
\overbrace{r_o^{(j)}}^{\{4n-3\}\times1} = \underbrace{H_{X,o}^{(j)}}_{\{4n-3\}\times m}\overbrace{\tilde{X}}^{m\times1}+n_o^{(j)}
$$

由于 $H_f^{(j)}$ 列满秩情况下的左零空间的大小为 $4n\times\{4n-3\}$，所以剔除特征以后的观测模型的行数减少了3行。

* **所有特征 $\{f_j\}_{j=1,...,M}$ 的观测模型合并**

$$
\left[\begin{matrix}r_o^{(1)}\\r_o^{(2)}\\\vdots\\r_o^{(M)}\end{matrix}\right]=\left[\begin{matrix}H_o^{(1)}\\H_o^{(2)}\\\vdots\\H_o^{(M)}\end{matrix}\right]\tilde{X}+n_o=H_X \tilde{X}+n_o
$$

$H_X$ 的大小是 $\left\{\sum_j^M(4n_j-3)\right\}\times \left\{21+6N_{\text{cam}}\right\}$。当 $H_x$ 的行数大于列数时，进行QR分解减少 $H_X$ 的行数。

$$
\begin{aligned}
H_X&=\left[\begin{array}{ll}{Q_1} & {Q_2}\end{array}\right]\left[\begin{array}{c}{{T}_{H}}\\0\end{array}\right]\\ 
{r}_{o} &=\left[\begin{array}{cc}{{Q}_{1}}&{{Q}_{2}}\end{array}\right]\left[\begin{array}{c}{{T}_{H}}\\ {{0}}\end{array}\right] \tilde{{X}}+{n}_{o}\\ 
\left[\begin{array}{c}{{Q}_{1}^{T} r_o} \\ {{Q}_{2}^{T} r_o}\end{array}\right]&=\left[\begin{array}{c}{T_H} \\ 0\end{array}\right] \tilde{X}+\left[\begin{array}{c}{Q_1^Tn_o} \\ {Q_2^{T} n_o}\end{array}\right]\\ 
r_n&=Q_1^Tr_o=T_H\tilde{X}+n_n 
\end{aligned}
$$ 

$r_n=T_H\tilde{X}$ 就是最终用于EKF更新的观测模型。

