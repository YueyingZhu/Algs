# MSCKF那些事（六）算法详解4：State Augmentation

 **Author:** [紫薯萝卜]

 **Link:** [https://zhuanlan.zhihu.com/p/76894345]

MSCKF的状态向量：

$$
\tilde{X}_k=\left[\begin{matrix}\tilde{X}_{\text{IMU}_k}^T&\delta \theta_{C_1}^T&^{G}_{C_1}\tilde{p}^T&...&&\delta \theta_{C_N}^T&^{G}_{C_N}\tilde{p}^T\end{matrix}\right]^T
$$

它包括IMU状态和N个相机状态。那究竟维护哪些相机状态呢？与基于关键帧的方法不同的是，MSCKF将每一帧都加入到状态向量中，每来一帧图像后，先根据IMU Propogation对IMU状态进行预测，然后根据相机外参计算当前帧的相机位姿，将最新相机状态加入到状态向量中并扩展协方差矩阵，这就是状态扩增（State Augmentation）。

## 1. 状态向量扩增  
根据预测的当前IMU位姿 $^{I}_{G}{\hat{q}}, ^{G}_{I}{\hat{p}}$ 以及相机外参 $^{C}_{I}{q}, ^{I}_{C}{p}$ 来计算当前相机的位姿：

$$
^{C}_{G}{\hat{q}}=^{C}_{I}{q} \otimes^{I}_{G}{\hat{q}} \\
^{G}_{C}{\hat{p}}=^{G}_{I}{\hat{p}}+{C}\left(^{I}_{G}{\hat{q}}\right)^T^{I}_{C}{p}
$$

然后将计算的当前相机状态 $^{C}_{G}{\hat{q}}, ^{G}_{C}{\hat{p}}$ 加入到状态向量中。

## 2.协方差扩增  
协方差扩增

$$
P_{k | k} \leftarrow\left[\begin{array}{c}{I_{6 N+15}} \\ {J_{6\times(6N+15)}}\end{array}\right] P_{k | k}\left[\begin{array}{c}{I_{6 N+15}} \\ {J_{6\times(6N+15)}}\end{array}\right]^{T}=\left[\begin{matrix}P_{k|k}&P_{k|k}J^T\\JP_{k|k}&JP_{k|k}J^T\end{matrix}\right]_{(6N+21) \times (6N+21)}
$$

其中 $J$ 是新增相机状态对原状态向量之间的Jacobian：

$$
J=\left[\begin{matrix} \frac{\partial ^{C_{\text{new}}}_{G}\delta \theta}{\partial \tilde{X}_{\text{imu}}}_{3\times15}&\frac{\partial ^{C_{\text{new}}}_{G}\delta \theta}{\partial \tilde{X}_{c}} _{3\times6N}\\ \frac{\partial ^{G}_{C_{\text{new}}} {\tilde{p}}}{\partial \tilde{X}_{\text{imu}}}_{3\times15} &\frac{\partial ^{G}_{C_{\text{new}}} {\tilde{p}}}{\partial \tilde{X}_{c}}_{3\times6N} \end{matrix}\right]_{6\times(15+6N)}=\left[\begin{array}{cccc}{C\left(^{I}_{C}{\hat{q}}\right)} & {{0}_{3 \times 9}} & {{0}_{3 \times 3}} & {{0}_{3 \times 6 N}} \\ -{C}\left(^{I}_{G}{\hat{q}}\right)^T\left\lfloor ^{I}_{C}{p}\times\right\rfloor & {{0}_{3 \times 9}} &I_{3\times3}& {{0}_{3 \times 6 N}}\end{array}\right]
$$ 

推导如下（这里顺便推导了相机状态对外参的Jacobian，目前未将外参加入了状态向量，可以暂时先忽略）：

![]((20191031)MSCKF那些事六算法详解4State_Augmentation_紫薯萝卜/v2-07fde4bdc6026dad32582f47b435a583_1440w.jpg)  
![]((20191031)MSCKF那些事六算法详解4State_Augmentation_紫薯萝卜/v2-ba99c74a2c7fd76d35cb5819242d68b3_1440w.jpg)  

> 求导思路：根据IMU位姿转Camera位姿的关系式，对IMU状态中的相关量求导，根据 $x=\hat x+\tilde x$对位置量进行展开，根据 $R=\delta R\cdot \hat R=(I-\lfloor\delta \theta_{\times}\rfloor) \cdot \hat R$对旋转量进行展开，等式两边相约得到误差量的关系式，然后根据叉乘的性质进行等式变形，最终得到Jacobian。  
>  **注意 $\delta R$的左乘和右乘，这里所有推导用的左乘，而MSCKF的原论文中用的是右乘，所以推导出来的Jacobian会不一样。**

## 3. 相机状态维护  
相机状态扩增

* 扩增时机：IMU Propagation之后，Measurement Update之前。
* 扩增条件：每一帧新的图像观测都进行相机状态扩增。
* 扩增操作：计算最新相机状态并插入到状态向量中；扩展新状态的自身协方差以及与原状态向量的关联协方差。

相机状态移除

* 移除时机：Measurement Update之后
* 移除条件：当相机状态个数超过最大限制 $N$时移除。正常是移除最老的相机状态，但当相机运动较小时也可以移除最近的相机状态。
* 移除操作：将待移除相机状态从状态向量中删除；删除协方差中对应的行和列。

## 4. 状态扩增的意义  
MSCKF中，IMU Propagation只改变IMU状态向量和其对应的协方差，与相机无关；而Measurement Updata的观测模型是残差相对于相机状态的观测模型，与IMU状态没有直接关联。状态扩增就相当于相机和IMU状态之间的桥梁，通过关联协方差 $P_{IC}$描述相机和IMU状态之间的关系，每一个相机状态都与IMU状态形成关联，这样在观测更新相机状态的同时，会间接更新IMU状态。

EKF观测更新计算如下：

$z=\left[\begin{matrix}0&H_{X_C}\end{matrix}\right]\left[\begin{matrix}\tilde X_I\\\tilde X_C\end{matrix}\right]\\S=H_{X_C}P_{CC}H_{X_C}^T+R\\K=PH^TS^{-1}=\left[\begin{matrix}P_{IC}H_{X_C}^TS^{-1}\\P_{CC}H_{X_C}^TS^{-1}\end{matrix}\right]=\left[\begin{matrix}K_{X_I}\\K_{X_C}\end{matrix}\right]\\\Delta X=Kr=\left[\begin{matrix}K_{X_I}r\\K_{X_C}r\end{matrix}\right]=\left[\begin{matrix}\Delta X_I\\\Delta X_C\end{matrix}\right]\\ P'=(I-KH)P=\left[\begin{matrix}I&-K_{X_I}H_{X_C}\\0&I-K_{X_C}H_{X_C}\end{matrix}\right]\left[\begin{matrix}P_{II}&P_{IC}\\P_{IC}^T&P_{CC}\end{matrix}\right]$ 

如果令IMU与相机之间的关联协方差 $P_{IC}=0$，那么

$K_{X_I}=0\Rightarrow\Delta X_I=0\\ P'=\left[\begin{matrix}P_{II}&0\\0&(I-K_{X_C}H_{X_C})P_{CC}\end{matrix}\right]$

可以看到，观测更新只会改变相机状态和相机对应的协方差，而不会改变IMU状态及其协方差，EKF的预测 $X_I$而更新 $X_C$，二者完全独立，没有任何融合。只有当 $P_{IC}\neq0$时，在观测更新时， $X_I$和对应的协方差 $P_{II},P_{IC}$才会同步被更新。

