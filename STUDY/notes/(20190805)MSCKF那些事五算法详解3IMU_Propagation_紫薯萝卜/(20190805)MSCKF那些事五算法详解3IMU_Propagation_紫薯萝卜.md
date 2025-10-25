# MSCKF那些事（五）算法详解3：IMU Propagation

 **Author:** [紫薯萝卜]

 **Link:** [https://zhuanlan.zhihu.com/p/76794930]

IMU Propagation过程：利用两帧图像之间的所有IMU观测数据（加速度 $a_m$和角速度 $\omega_m$），对MSCKF的状态向量和协方差进行迭代预测。它相当于EKF中的预测过程。

本节主要探讨的是如何根据状态量之间的物理关系推导出状态的线性化运动模型，以及如何根据运动模型对状态向量及其协方差进行迭代预测。

## 1. 运动模型推导  
### 1.1 状态真实值的系统模型  
各状态量真实值之间的物理关系如下：

$$
\begin{aligned}
{}^{I}_{G}\dot{q}(t) &= \tfrac{1}{2}\,\Omega\!\big(\omega(t)\big)\,{}^{I}_{G}q(t), \quad && \dot b_g(t)= n_{wg}(t) \\
{}^{G}_{I}\dot v(t) &= {}^{G}_{I} a(t), \quad && \dot b_a(t)= n_{wa}(t), \quad {}^{G}\dot p_I(t) = {}^{G} v_I(t)
\end{aligned}
$$


其中， $^{G}_{I}{a}$ 是Global系下的加速度， $\omega=[\omega_x,\omega_y,\omega_z]^T$ 是IMU系的角速度，

$$
\Omega(\omega)=\left[\begin{array}{cc}{-\lfloor\omega_\times\rfloor} & \omega \\ {-\omega^T} & {0}\end{array}\right], \quad\lfloor{\omega}_\times\rfloor=\left[\begin{array}{ccc}{0} & {-\omega_{z}} & {\omega_{y}} \\ {\omega_{z}} & {0} & {-\omega_{x}} \\ {-\omega_{y}} & {\omega_{x}} & {0}\end{array}\right]
$$
其中，$\Omega(\omega)$ 表示四元数左乘矩阵，
IMU观测量的定义如下：

$$
\begin{aligned}
\omega_m &= \omega + C\!\left({}^{I}_{G}q\right)\,\omega_G + b_g + n_g \\
a_m &= C\!\left({}^{I}_{G}q\right)\!\left({}^{G}_{I}a - {}^{G}g + 2\left[\omega_G\right]_\times\,{}^{G}_{I}v + \left[\omega_G\right]_\times^{2}\,{}^{G}_{I}p\right) + b_a + n_a
\end{aligned}
$$


其中， $C(.)$ 表示四元数转旋转矩阵， $n_g,n_a$ 为IMU测量误差（高斯白噪声）。注意：原始MSCKF论文中考虑了地球自旋速度 $\omega_G$，S-MSCKF中将这部分忽略，观测量可以化简为

$$
\begin{aligned}
\omega_m&=\omega+{b}_{g}+{n}_{g} \\ 
a_m&={C}\left(^{I}_{G}{q}\right)\left(^{G}_{I}{a}-^{G}{g}\right) +b_a+{n}_{a} 
\end{aligned}
$$ 

### 1.2 状态估计量的系统模型  
状态估计量的系统模型：

$$
\begin{aligned}
{}^{I}_{G}\dot{\hat q} &= \tfrac{1}{2}\,\Omega(\hat\omega)\,{}^{I}_{G}\hat q, && \dot{\hat b}_g = 0_{3\times 1} \\
{}^{G}_{I}\dot v &= C\!\left({}^{I}_{G}\hat q\right)^{\!T}\hat a + {}^{G}g, && \dot{\hat b}_a = 0_{3\times 1}, \quad {}^{G}_{I}\dot{\hat p} = {}^{G}_{I}\hat v
\end{aligned}
$$


其中，加速度和角速度估计值定义如下：

$$
\begin{aligned} 
\hat{\omega}&=\omega_m-\hat{b}_g \Rightarrow \omega = \hat{\omega}-\tilde{b}_g -n_g\\  
\hat{a}&=a_m-\hat{b}_a\Rightarrow ^{G}_{I}a=^{I}_{G}{^T}R(\hat{a}-\tilde{b}_a-n_a)+^{G}{g} 
\end{aligned}
$$ 

### 1.3 运动模型推导  
根据状态估计量的系统模型推导IMU误差状态的运动模型

$$
\dot{\tilde{X}}_{\text{IMU}}=F\tilde{X}_{\text{IMU}}+G n_{\text{IMU}}
$$

先上结论：

$$
\begin{aligned}
\underbrace{\left[\begin{matrix}\dot{\delta \theta}_I\\\dot{\tilde{b}}_g\\^{G}_{I}\dot{\tilde{v}}\\\dot{\tilde{b}}_a\\^{G}_{I}\dot{\tilde{p}}\end{matrix}\right]}_{\dot{\tilde{X}}_{\text{IMU}}}=&\quad\underbrace{\left[\begin{matrix}-\lfloor\hat{\omega}_{\times}\rfloor&-I_3&0_{3\times3}&0_{3\times3}&0_{3\times3}\\0_{3\times3}&0_{3\times3}&0_{3\times3}&0_{3\times3}&0_{3\times3}\\-{C}\left(^{I}_{G}{\hat{q}}\right)^T\lfloor \hat{a}_{\times}\rfloor&0_{3\times3}&0_{3\times3}&-{C}\left(^{I}_{G}{\hat{q}}\right)^T&0_{3\times3}\\0_{3\times3}&0_{3\times3}&0_{3\times3}&0_{3\times3}&0_{3\times3}\\0_{3\times3}&0_{3\times3}&I_3&0_{3\times3}&0_{3\times3}\end{matrix}\right]}_F\underbrace{\left[\begin{matrix}\delta \theta_I\\\tilde{b}_g\\^{G}_{I}{\tilde{v}}\\\tilde{b}_a\\^{G}_{I}{\tilde{p}}\end{matrix}\right]}_{\tilde{X}_{\text{IMU}}}\\
&+\quad\underbrace{\left[\begin{matrix}-I_3&0_{3\times3}&0_{3\times3}&0_{3\times3}\\0_{3\times3}&I_3&0_{3\times3}&0_{3\times3}\\0_{3\times3}&0_{3\times3}&-{C}\left(^{I}_{G}{\hat{q}}\right)^T&0_{3\times3}\\0_{3\times3}&0_{3\times3}&0_{3\times3}&I_3\\0_{3\times3}&0_{3\times3}&0_{3\times3}&0_{3\times3}\end{matrix}\right]}_G\underbrace{\left[\begin{matrix}n_g\\n_{wg}\\n_a\\n_{wa}\end{matrix}\right]}_{n_{\text{IMU}}}
\end{aligned}
$$

其中 $n_{\text{IMU}}$ 向量中依次是陀螺仪误差、陀螺仪bias误差、加速计误差、加速计bias误差。具体推导细节放在最后。

## 2. 状态向量预测  
这里直接对状态估计量进行迭代预测，而不是误差状态向量，但协方差是根据误差状态的运动模型进行迭代。每来一帧IMU观测数据，需要对 $X_{\text{IMU}}$ 中的姿态 $^{I}_{G}{^T}{q}$、速度 $^{G}_{I}{^T}{v}$ 和位置 $^{G}_{I}{^T}{p}$ 进行预测， $b_a,b_g$ 保持不变。

### 姿态预测  
 姿态四元数采用的0阶积分(假设角速度在单位时间内恒定不变)，推导过程见[1]中的1.6.1小节。

$$
\begin{aligned} 
\text{当 } |\hat{\omega}|>10^{-5} \text{ 时：} ^{I}_{G}{\hat{q}}(t+\Delta t)&=\left(\cos\left(\frac{|\hat{\omega}|}{2}\Delta t\right)\cdot I_{4\times4} + \frac{1}{|\hat{\omega}|}\sin\left(\frac{|\hat{\omega}|}{2}\Delta t\right) \cdot \Omega(\hat{\omega})\right)^{I}_{G}{\hat{q}}(t)\\ 
\text{当 } |\hat{\omega}|\leq10^{-5} \text{ 时：} ^{I}_{G}{\hat{q}}(t+\Delta t)&=\left(I_{4\times4}-\frac{\Delta t}{2}\Omega(\hat{\omega})\right)^{I}_{G}{\hat{q}}(t) 
\end{aligned}
$$ 

其中 $\hat{\omega} = \omega_m-\hat{b}_g$。

### 速度和位置预测  
速度和位置可以采用的4阶Runge-Kutta积分, 其计算过程如下： $\hat a=a_m-\hat b_a$

$$
\hat{v}(t+\Delta t) = \hat{v}(t)+\frac{\Delta t}{6}(k_{v_1}+2k_{v_2}+2k_{v_3}+k_{v_4}) \\ 
\begin{aligned} 
k_{v_1} &= ^{I}_{G}{\hat{R}}(t)\hat{a}+g\\ 
k_{v_2} &= ^{I}_{G}{\hat{R}}\left(t+\Delta t/2\right)\hat{a}+g\\ 
k_{v_3} &= ^{I}_{G}{\hat{R}}\left(t+\Delta t/2\right)\hat{a}+g\\ 
k_{v_4} &= ^{I}_{G}{\hat{R}}\left(t+\Delta t\right)\hat{a}+g\\ 
\end{aligned}\\ 
\hat{p}(t+\Delta t) = \hat{p}(t)+\frac{\Delta t}{6}(k_{p_1}+2k_{p_2}+2k_{p_3}+k_{p_4}) \\ 
\begin{aligned} 
k_{p_1} &=  \hat{v}(t)\\ 
k_{p_2} &=  \hat{v}(t)+k_{v_1}\Delta t/2\\ 
k_{p_3} &=  \hat{v}(t)+k_{v_2}\Delta t/2\\ 
k_{p_4} &=  \hat{v}(t)+k_{v_3}\Delta t\\ 
\end{aligned}
$$ 

常用的三种数值积分：Euler积分、Mid-Point积分、Runge-Kutta积分(4阶)。它们的精度依次从低到高、计算量依次从小到大。令 $k_2$ 的权重为1，其他为0，RK4就退化为Mid-Point法；令 $k_1$ 的权重为1，其他为0，RK4就退化为Euler积分。

## 3. 状态协方差预测  
状态协方差预测公式如下：

![]((20190805)MSCKF那些事五算法详解3IMU_Propagation_紫薯萝卜/v2-3ad83e671cca220c20547493eff708e5_1440w.jpg)  


IMU Propagation协方差预测公式

  
  
单帧IMU数据的迭代公式如下：

$$
\begin{aligned}
\Phi_n&=\exp(F\delta t)\approx I+F\delta t+\frac{1}{2}(F\delta t)^2+\frac{1}{6}(F\delta t)^3\\ 
P_{II_{n+1}}&=\Phi_nP_{II_n}\Phi_n^T+\Phi_nG(Q\tau)G^T\Phi_n^T\\ 
P_{IC_{n+1}}&=\Phi_nP_{IC_n}
\end{aligned}
$$ 

其中， $n$ 为IMU迭代的帧数， $\delta t$ 为两帧IMU数据的时间间隔，上式中假设了 $F(\tau)$ 在 $\delta t$ 时间段内恒定。

## 参考文献  
1. [http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf)
2. Mourikis A I, Roumeliotis S I. A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation[C]// IEEE International Conference on Robotics and Automation. IEEE, 2007:3565-3572

## 附录  
运动模型推导（手写的，懒得敲公式了，可以参照文献[1]自行推导）

![]((20190805)MSCKF那些事五算法详解3IMU_Propagation_紫薯萝卜/v2-4ae185cdf272afd987a92db66dff20d5_1440w.jpg)  
![]((20190805)MSCKF那些事五算法详解3IMU_Propagation_紫薯萝卜/v2-ea1e61f6e54c3eb0c7d3e5a833f4ceee_1440w.jpg)  