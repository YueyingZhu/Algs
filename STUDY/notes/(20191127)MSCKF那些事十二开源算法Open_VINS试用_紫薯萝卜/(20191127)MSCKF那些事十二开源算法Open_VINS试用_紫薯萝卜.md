# MSCKF那些事（十二）开源算法Open VINS试用

 **Author:** [紫薯萝卜]

 **Link:** [https://zhuanlan.zhihu.com/p/93814423]

**1. Open VINS简介**

Open VINS是Huang Guoquan老师团队在2019年8月份开源的一套基于MSCKF的VINS算法，黄老师曾是Tango项目的核心成员，在MSCKF这块非常的权威。

* Github地址:

[rpng/open\_vins](https://github.com/rpng/open\_vins)* 官方文档:

[Open VINS](https://docs.openvins.com/pages.html)Open VINS的特色官网上有介绍，个人比较关注的有以下几点:

* 详细的官方文档和理论推导，写的非常好，强烈推荐
* 五种特征表征形式: 全局 XYZ，全局逆深度，Anchored XYZ，Anchored 逆深度，Anchored MSCKF版本逆深度
* 融合了两种特征SLAM: 分别是ARUCO二维码特征和常规的稀疏特征. Hybird SLAM代码实现(特征加入剔除状态向量，VIO和SLAM切换)非常具有参考意义

open\_vins运动初始化代码没有开源，只有静止初始化. 所以直接跑EUROC和TUM数据集，有些数据可能会跑飞。

## 2. 安装测试  
参考[https://docs.openvins.com/gs-installing.html](https://docs.openvins.com/gs-installing.html)，安装非常简单

* **Step0: 准备工作**

安装ROS和OpenCV，如果要测试EUROC或TUM数据集，需要提前下载rosbag数据。

* **Step1: 编译代码**


```
mkdir -p ~/workspace/catkin_ws_ov/src/

cd ~/workspace/catkin_ws_ov/src/

git clone https://github.com/rpng/open_vins/

cd ..

catkin build
```
* **Step2: 修改rosbag路径**

ov\_msckf/launch目录下已经有了EUROC(pgeneva\_eth.launch)和TUM数据集(pgeneva\_tum.launch)的launch文件，修改'path\_bag'，'path\_gt'，分别是rosbag的路径和对应的ground truth文件，其中gt文件在ov\_data中已经有了,只需要修改bag对应的gt文件即可。有的launch文件名带'ros'有的不带，不带'ros'的是顺序执行rosbag，带'ros'的是用ros bag play，算法处理不过来的时候会有数据丢失，可以自行选择。


```
<!-- bag parameters -->
<param name="path_bag"    type="string" value="/home/symao/data/euroc/rosbag/MH_04_difficult.bag" />
<param name="path_gt"     type="string" value="$(find ov_data)/euroc_mav/MH_04_difficult.csv" />
```
* **Step3: 测试程序**


```
cd catkin_ws_ov
source devel/setup.bash
roslaunch ov_msckf pgeneva_eth.launch
# 新开一个终端
rosrun rviz rviz -d ov_msckf/launch/display.rviz
```
成功运行图如下:

![]((20191127)MSCKF那些事十二开源算法Open_VINS试用_紫薯萝卜/v2-092c70a809eb5ad800a401653123673d_1440w.jpg)  


open\_vins运行效果图

  
  
## 3. 精度测试  
在MH\_04\_difficult.bag数据上测试了下不同特征表征以及单双目对精度的影响，修改launch文件中的'feat\_representation'和'max\_cameras'参数，测试结果如下：

![]((20191127)MSCKF那些事十二开源算法Open_VINS试用_紫薯萝卜/v2-28f7addadd4ecc2d44f62d701335b36e_1440w.jpg)  
对比发现，单目VIO时，逆深度相对于XYZ的精度提升比较明显，ANCHOR坐标系相对于GLOABL坐标系的提升并不是很明显。双目VIO反而最naive的GLOABL XYZ精度最好。这里只测了一组数据，并不能完全说明问题，只能粗略参考。

最后，再次强烈推荐官方文档[https://docs.openvins.com/pages.html](https://docs.openvins.com/pages.html). 有代码有文档，理论推导又详细又全面，绝对的良心巨作，给黄老师团队点赞。

