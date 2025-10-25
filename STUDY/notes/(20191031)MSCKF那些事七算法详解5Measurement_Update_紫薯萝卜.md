# MSCKF那些事（七）算法详解5：Measurement Update

 **Author:** [紫薯萝卜]

 **Link:** [https://zhuanlan.zhihu.com/p/77040286]

MSCKF观测更新的原理：一个静止的特征点被多个相机位姿观测到，从而对观测到该特征点的多个相机产生约束（即多个相机对该特征点的观测射线应该汇聚在一点）。MSCKF以此为出发点，先根据多个观测三角化特征点的空间坐标，然后定义特征点到各相机的重投影误差（残差 $r$）作为观测量，并推导残差与状态向量之间的线性化关系，即残差观测模型

$r=H\tilde X+n\\$

其中， $H$为残差对误差状态向量 $\tilde X$的Jacobian， $n$为特征的高斯白噪声，最后，直接套用EKF公式对状态向量和协方差进行更新。

## 1. 特征点三角化  
特征点三角化保持当前Sliding Windows中的相机位姿不变，根据特征点在多个相机中的观测，利用最小二乘优化估计特征点的3D坐标，为了提高数值计算的稳定性，特征点坐标采用逆深度来表示。设特征点在相机坐标系下的坐标为 $[x_C,y_C,z_C]^T$，其逆深度表示为 $[\alpha,\beta,\rho]^T=[x_C/z_C, y_C/z_C, 1/z_C]^T$。对特征点 $f_j$，选取 $C_n$为参照器，根据相机之间的相对位姿关系可以转到其他相机坐标系：

$\sideset{^{C_i}}{_{f_j}}{p}=C\left(\sideset{_{C_n}^{C_i}}{}{q}\right) \sideset{^{C_n}}{_{f_j}}{p}+\sideset{^{C_i}}{_{C_n}}{p}\\$

将逆深度表示带入：

$\begin{aligned}\sideset{^{C_i}}{_{f_j}}{p}&=\sideset{^{C_n}}{_j}{z}\left(C\left(\sideset{_{C_n}^{C_i}}{}{q}\right) \left[\begin{array}{c}\frac{\sideset{^{C_n}}{_j}{x}}{\sideset{^{C_n}}{_j}{z}}\\ \frac{\sideset{^{C_n}}{_j}{y}}{\sideset{^{C_n}}{_j}{z}} \\1\end{array}\right]+\frac1{\sideset{^{C_n}}{_j}{z}}\sideset{^{C_i}}{_{C_n}}{p}\right)\\ &=\sideset{^{C_n}}{_j}{z}\left(C\left(\sideset{_{C_n}^{C_i}}{}{q}\right) \left[\begin{array}{c}\alpha_j\\\beta_j \\1\end{array}\right]+\rho_j\sideset{^{C_i}}{_{C_n}}{p}\right)\\ &=\sideset{^{C_n}}{_j}{z}\left[\begin{matrix}h_{i1}(\alpha_j,\beta_j,\rho_j)\\h_{i2}(\alpha_j,\beta_j,\rho_j)\\h_{i3}(\alpha_j,\beta_j,\rho_j)\end{matrix}\right] \end{aligned}\\$ 

$\sideset{^{C_i}}{_{f_j}}{p}$投影相机 $C_i$中的观测为(**注意：这里忽略的相机内参，默认观测是在相机的Normalize平面**)：

$\mathbf{z}_i^{(j)}=\frac1{h_{i3}(\alpha_j,\beta_j,\rho_j)}\left[\begin{matrix}h_{i1}(\alpha_j,\beta_j,\rho_j)\\h_{i2}(\alpha_j,\beta_j,\rho_j)\end{matrix}\right]+\mathbf{n}_i^{(j)}\\$

设 $f_j$在 $C_1,...,C_n$相机下的观测分别为 $\mathbf{z}_1^{(j)},...,\mathbf{z}_n^{(j)}$，则最小二乘优化问题为：

$\min \frac12\sum_{i=1}^n\left|\left|e_i(\alpha_j,\beta_j,\rho_j)\right|\right|^2 = \min\frac12\sum_{i=1}^n\left|\left|\mathbf{z}_i^{(j)}-\hat{\mathbf{z}}_i^{(j)}\right|\right|^2\\$

推导误差对优化参数的Jacobian如下： 

$\begin{aligned} J_f =\frac{\partial e_i}{\partial (\alpha,\beta,\rho)}&=-\frac{\partial z_i}{\partial h}\frac{\partial h}{\partial (\alpha,\beta,\rho)}\\ &=- \frac{\partial z_i}{\partial h} \left[\begin{matrix}\frac{\partial  h}{\partial \alpha}&\frac{\partial  h}{\partial \beta}&\frac{\partial  h}{\partial \rho}\end{matrix}\right]\\ &=-\left[\begin{matrix} \frac1{h_2} & 0 & -\frac{h_0}{h_2^2}\\ 0&\frac1{h_2}&-\frac{h_1}{h_2^2} \end{matrix}\right] \left[\begin{matrix} \sideset{_{C_N}^{C_i}}{}R\left(\begin{matrix}1\\0\\0\end{matrix}\right),&\sideset{_{C_N}^{C_i}}{}R\left(\begin{matrix}0\\1\\0\end{matrix}\right), &\sideset{^{C_i}}{_{C_n}}p \end{matrix}\right] \end{aligned}$ 

推导出Jacobian后，可以通过Gauss-Newton法或Levenberg-Marquardt法进行最小二乘求解 $(\hat\alpha_j,\hat\beta_j,\hat\rho_j)$，最后得到世界坐标系下特征点坐标：

$^{G} \hat p_{f_j}=\frac1{\hat \rho_j} {C}\left(\sideset{_{G}^{C_n}}{}{\hat q}\right)^T\left[\begin{array}{c}{\hat{\alpha}_j} \\ {\hat{\beta}_j} \\ {1}\end{array}\right]+^{G} \hat p_{C_n}\\$

## 2. 观测模型  
MSCKF中的观测是根据特征进行聚集，即每个特征点保存所有观测相机及对应观测。单个特征点对单个相机有一个残差模型，残差定义为

$r_i^{(j)}=z_i^{(j)}-\hat z_i^{(j)}\\ \hat{z}_i^{(j)}=\frac1{C_{i} \hat{Z}_{j}}\left[\begin{array}{c}{^{C_{i}} \hat{X}_{j}} \\ {^{C_{i}} \hat{Y}_{j}}\end{array}\right],\left[\begin{array}{c}{^{C_{i}} \hat{X}_{j}} \\ {C_{i} \hat{Y}_{j}} \\ {^{C_{i}} \hat{Z}_{j}}\end{array}\right]=C\left(_{G}^{C_{i}} \hat{q}\right)\left(^{G} \hat{p}_{f_{j}}-^{G} \hat{p}_{C_{i}}\right)$

其中 $z_i^{(j)}$为特征 $f_j$在相机 $C_i$的观测值， $\hat z_i^{(j)}$为特征 $f_j$在相机 $C_i$的2D投影。将所有观测到该特征点的相机的残差模型聚集起来后，再将所有特征点的观测模型合并得到完整的观测模型。

### 2.1 单个特征点对单个相机的观测模型  
对残差进行线性化，得到线性模型：

$r_i^{(j)}\approx H_{X_i}^{(j)}\tilde X + H_{f_i}^{(j)}\sideset{^G}{_{f_j}}{\tilde p}+n_i^{(j)}\\$

其中， $H_{X_i}^{(j)}， H_{f_i}^{(j)}$分别是残差 $r_i^{(j)}$对状态向量和特征点的Jacobian，计算公式如下：

$H_{X_i}^{(j)}=\left[\begin{matrix}0& H_{X_{C_i}}^{(j)}& 0\end{matrix}\right],\quad 	H_{X_{C_i}}^{(j)}=\frac{\partial z_i^{(j)}}{\partial \sideset{^{C_i}}{_j}{p}}\frac{\partial \sideset{^{C_i}}{_j}{p}}{\partial X_{C_i}},\quad 	H_{f_i}^{(j)}=\frac{\partial z_i^{(j)}}{\partial \sideset{^{C_i}}{_j}{p}}\frac{\partial \sideset{^{C_i}}{_j}{p}}{\partial \sideset{^G}{_j}{p}}\\ 	\frac{\partial z_i^{(j)}}{\partial \sideset{^{C_i}}{_j}{p}}=\left[\begin{matrix}\frac1{\sideset{^{C_i}}{_j}{\hat Z}}&0&-\frac{\sideset{^{C_i}}{_j}{\hat X}}{\sideset{^{C_i}}{_j^2}{\hat Z}}\\0&\frac1{\sideset{^{C_i}}{_j}{\hat Z}}&-\frac{\sideset{^{C_i}}{_j}{\hat Y}}{\sideset{^{C_i}}{_j^2}{\hat Z}}\end{matrix}\right]\\ 	\frac{\partial \sideset{^{C_i}}{_j}{p}}{\partial X_{C_i}}=\left(\left\lfloor\sideset{^{C_i}}{_j}{\hat p}{}_{\times}\right\rfloor\quad-C\left(\sideset{_G^{C_i}}{}{\hat q}\right)\right)\\ \frac{\partial \sideset{^{C_i}}{_j}{p}}{\partial \sideset{^G}{_j}{p}}=C\left(\sideset{_G^{C_i}}{}{\hat q}\right)$ 

Jacobian推导：

$\frac{\partial \sideset{^{C_i}}{_j}p}{\partial X_{C_i}}=\left[\begin{matrix} \frac{\partial \sideset{^{C_i}}{_j}p}{\partial \sideset{_G^{C_i}}{}{\theta}} & \frac{\partial \sideset{^{C_i}}{_j}p}{\partial \sideset{^G}{_{C_i}}p} \end{matrix}\right]\\ \begin{aligned} 求\frac{\partial \sideset{^{C_i}}{_j}p}{\partial \sideset{_G^{C_i}}{}{\theta}}：&\sideset{^{C_i}}{_j}{\hat p}+\sideset{^{C_i}}{_j}{\tilde p} = (I-\lfloor{\sideset{_G^{C_i}}{}{\delta \theta}}_{\times}\rfloor)\sideset{_G^{C_i}}{}{\hat R}(\sideset{^G}{_{f_j}}{\hat p}-\sideset{^G}{_{C_i}}{\hat p})\\ \Rightarrow &\sideset{^{C_i}}{_j}{\hat p}+\sideset{^{C_i}}{_j}{\tilde p} = (I-\lfloor{\sideset{_G^{C_i}}{}{\delta \theta}}_{\times}\rfloor)\sideset{^{C_i}}{_j}{\hat p}\\ \Rightarrow &\sideset{^{C_i}}{_j}{\tilde p} = -\lfloor{\sideset{_G^{C_i}}{}{\delta R}}_{\times}\rfloor\sideset{^{C_i}}{_j}{\hat p}=\lfloor{{\sideset{^{C_i}}{_j}{\hat p}}_{\times}\rfloor\sideset{_G^{C_i}}{}{\delta \theta}}\\ 求\frac{\partial \sideset{^{C_i}}{_j}p}{\partial \sideset{^G}{_{C_i}}p}：&\sideset{^{C_i}}{_j}{\hat p}+\sideset{^{C_i}}{_j}{\tilde p} = \sideset{_G^{C_i}}{}{\hat R}(\sideset{^G}{_{f_j}}{\hat p}-(\sideset{^G}{_{C_i}}{\hat p}+\sideset{^G}{_{C_i}}{\tilde p}))\\ \Rightarrow &\sideset{^{C_i}}{_j}{\tilde p} = -\sideset{_G^{C_i}}{}{\hat R}\sideset{^G}{_{C_i}}{\tilde p}\\ 求\frac{\partial \sideset{^{C_i}}{_j}p}{\partial \sideset{^G}{_{f_j}}p}：&\sideset{^{C_i}}{_j}{\hat p}+\sideset{^{C_i}}{_j}{\tilde p} = \sideset{_G^{C_i}}{}{\hat R}(\sideset{^G}{_{f_j}}{\hat p}+\sideset{^G}{_{f_j}}{\tilde p}-\sideset{^G}{_{C_i}}{\hat p})\\ \Rightarrow &\sideset{^{C_i}}{_j}{\tilde p} = \sideset{_G^{C_i}}{}{\hat R}\sideset{^G}{_{f_j}}{\tilde p}\\ \end{aligned}\\$ 

### 2.2 单个特征点对所有相机的观测模型合并  
将单个特征点对所有相机的观测模型按行堆叠，得到单个特征点的观测模型：

$\underbrace{r^{(j)}}_{2M_j\times1}\approx \underbrace{H_X^{(j)}}_{2M_j\times(15+6N)}\tilde X+\underbrace{H_f^{(j)}}_{2M_j\times3}\sideset{^G}{_{f_j}}{\tilde p}+n^{(j)}\\ r^{(j)}=\left[\begin{matrix}r^{(j)}_1\\r^{(j)}_2\\...\\r^{(j)}_{M_j}\end{matrix}\right],\quad H_X^{(j)}=\left[\begin{matrix}H_{X_1}^{(j)}\\H_{X_2}^{(j)}\\...\\H_{X_{M_j}}^{(j)}\end{matrix}\right],\quad H_f^{(j)}=\left[\begin{matrix}H_{f_1}^{(j)}\\H_{f_2}^{(j)}\\...\\H_{f_{M_j}}^{(j)}\end{matrix}\right]\\$

其中， $M_j$为 $f_j$的观测数量。注意到上述观测模型与特征点 $\sideset{^G}{_{f_j}}{\tilde p}$相关，而 $\sideset{^G}{_{f_j}}{\tilde p}$并未在状态向量中，所以希望将特征点从观测模型中移除，移除方式是观测模型两边乘以 $H_f^{(j)}$的左零空间 $V^T$，左零空间的定义为 $V^TH_f^{(j)}=0$。由于 ${H_f^{(j)}}_{2M_j\times3}$列满秩为3，其左零空间有 $2M_j-3$维， $V$的尺寸为 $2M_j\times(2M_j-3)$，观测模型两边同时乘以 $V^T$，得到只与状态向量相关，与特征点坐标无关的观测模型：

$r_o^{(j)}=V^Tr^{(j)}\approx \underbrace{V^TH_X^{(j)}}_{H_o^{(j)}} \tilde X+\underbrace{V^Tn^{(j)}}_{n_o^{(j)}}= H_o^{(j)}\tilde X+n_o^{(j)}\\$

值得注意的是左零空间 $V$并不需要显示计算出来， $r_o^{(j)},H_o^{(j)}$相当于原始的 $r^{(j)}和H_X^{(j)}$在 $H_f^{(j)}$零空间上的投影，这个用Givens rotaions可以高效的计算。 

为什么左零空间消除特征项这一步是在将单个特征点的所有重投影模型聚合之后？个人认为原因有几点：

1. 单个特征点对单个相机的重投影模型中 $H_{f_i}^{(j)}$的维度为 $2\times 3$，左零空间不存在，因为 ${A^T}_{3\times2}x=0$的解不存在，当 $A$的行数大于 $A$的列数时，才存在左零空间。所以特征点至少要被两个相机观测到 $(2*2>3)$才可以用于观测更新。
2. 对于单个特征点来说，到所有相机的残差模型中都包含 $\sideset{^G}{_{f_j}}{\tilde p}$项，所以可以合并后用左零空间消除；而不同特征点的观测模型中 $\sideset{^G}{_{f_j}}{\tilde p}$项不一样，所以要在不同特征点的观测模型合并之间进行消除。

### 2.3 所有特征点的观测模型合并  
对所有特征点的残差模型进行合并，得到整体残差模型：

$r_o=H_X\tilde X+n_o\\ r_o=\left[\begin{matrix}r_o^{(1)}\\r_o^{(2)}\\...\\r_o^{(N)}\end{matrix}\right],\quad H_X=\left[\begin{matrix}H_X^{(1)}\\H_X^{(2)}\\...\\H_X^{(N)}\end{matrix}\right], \quad n_o=\left[\begin{matrix}n_o^{(1)}\\n_o^{(2)}\\...\\n_o^{(N)}\end{matrix}\right]$

其中， $H_X$ 的大小为 $\left(\sum_{j=1}^N(2M_j-3)\right)\times L$， $L=15+6N_C$为状态向量的总长度。通常 $\left(\sum_{j=1}^N(2M_j-3)\right)> L$，MSCKF对 $H_X$进行QR分解来减少观测模型的规模(行数)，计算过程如下：

$\begin{aligned}H_X=\left[\begin{array}{cc}Q_1 & Q_2\end{array}\right]\left[\begin{array}{c}T_H\\0\end{array}\right]\quad&\Rightarrow\quad r_o =\left[Q_1 \quad Q_2\right]\left[\begin{array}{c}T_H \\ 0\end{array}\right] \tilde{X}+n_o \\ 等式两边同时乘以Q^{-1}=Q^T\quad&\Rightarrow\quad\left[\begin{array}{c}Q_1^Tr_o \\Q_2^Tr_o\end{array}\right]=\left[\begin{array}{c}T_H \\ 0\end{array}\right] \tilde{X}+\left[\begin{array}{c}{Q_1^T n_o} \\ {Q_2^T n_o}\end{array}\right]\\ &\Rightarrow\quad r_n=Q_1^Tr_o=T_H\tilde X+n_n\end{aligned}$ 

通过QR分解，观测矩阵大小从 ${H_X}$的 $\left(\sum_{j=1}^N(2M_j-3)\right)\times L$降到了 $T_H$的 $L\times L$，从而减少了EKF更新的计算量。

关于噪声：

* 特征点对各个观测之间的噪声相互独立，所以 $n^{(j)}$的协方差为： $R^{(j)}=\sigma^2 I_{2M_j}$
* $n_o^{(j)}=V^Tn^{(j)},n_n=Q_1^Tn_o$，从而 $n_o^{(j)}$的协方差为： $R^{(j)}_o=\sigma^2A^TA=\sigma^2I_{2M_j-3}$
* $n_n=Q_1^Tn_o$，从而 $n_n$的协方差为： $R_n=Q_1^TR_oQ_1=\sigma^2I_r$

可以看到，经过左零空间、QR分解等变化， $n_n$的噪声协方差仍然是 $sigma^2$乘以单位阵，所以在构建观测模型时，可以先不考虑噪声，只计算 $r$和 $H$，在EKF更新中在加上特征噪声 $R_n$即可。

## 3. 观测更新策略  
观测更新直接套用EKF更新公式即可，计算如下：

$K=PT_H^T(T_HPT_H^T+R_n)^{-1}\\ \Delta X=Kr_n\\ P_{k+1|k+1}=(I-KT_H)P_{k+1|k}(I-KT_H)^T+KR_nK^T$ 

这里主要讨论一下：**哪些特征被用到观测更新？何时进行更新？** 观测模型的推导中假设特征点坐标已知，因此需要提前计算特征点坐标，MSCKF通过观测三角化来估计特征点坐标，三角化的精度直接影响观测模型的精度。而三角化的精度又与哪些因素有关呢？

* **特征观测的数量**：理论上观测越多，三角化精度越高。为了让三角化获得足够多的观测，MSCKF等到特征被跟丢后再用于观测更新，特征跟丢意味着该特征点的观测不会再继续增加。
* **特征观测的视差**：特征视差越大，三角化精度越高。但是特征点的视差取决于相机的运动方式，而相机运动是算法主动控制不了的，所以只能被动地判断特征点的视差，如果视差不够的话，则将其丢弃掉，不进行三角化和观测更新。
* **相机位姿的精度**：相机位姿的精度会直接影响三角化的精度，MSCKF在特征跟丢后再进行三角化，也是因为历史的相机位姿都经过了EKF更新过程，相对来说比较准。相机位姿和三角化是典型的SLAM中“鸡生蛋蛋生鸡”的关系，只有三角化足够准，才能准确估计相机的位姿，只有相机位姿足够准，才能准确地三角化。这个问题的根源在于MSCKF将特征点估计(Mapping)和位姿估计(Localization)分离开来，像很多SLAM算法都会通过BA来同时优化特征点和位姿，但BA优化的计算量又会很大。MSCKF虽然将二者分开，但实际精度也能做到挺高。

还有一个细节值得考虑：当Slidingwindow满了以后历史相机状态会被移除，假如此时特征点仍未跟丢，就不会触发三角化和观测更新，如果直接丢弃历史相机状态对应的观测会造成信息丢失，这种情况应该如何处理呢？

参考开源算法S-MSCKF中的做法：移除历史相机状态时，如果特征点尚未三角化，则先进行三角化，再将其对待移除的观测相机（注意不是对全部观测相机）的观测模型用于更新。另外，需要注意的是观测模型约束的是相机之间的相对位姿关系，单个特征点至少需要被两个相机观测到（两个残差模型）才能用于观测更新，所以S-MSCKF中每次移除两个相机状态，如果特征点被两个待移除相机状态观测到，则计算特征点对两个待移除相机状态的残差模型进行更新，剩下的观测待到特征点跟丢后再进行更新。

