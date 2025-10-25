#!/bin/bash

source /opt/ros/noetic/setup.bash

# 清理旧的编译结果（可选）
# rm -rf build devel

# 设置 OpenCV 环境为 3
export OpenCV_DIR=/opt/opencv-3.4.11/share/OpenCV
export PKG_CONFIG_PATH=/opt/opencv-3.4.11/lib/pkgconfig
export LD_LIBRARY_PATH=/opt/opencv-3.4.11/lib:$LD_LIBRARY_PATH
export LD_PRELOAD=/opt/opencv-3.4.11/lib/libopencv_core.so.3.4:/opt/opencv-3.4.11/lib/libopencv_imgproc.so.3.4
# export CMAKE_PREFIX_PATH=/opt/opencv-3.4.11:$CMAKE_PREFIX_PATH
# export CMAKE_PREFIX_PATH=/opt/opencv-3.4.11:/opt/ros/noetic:$CMAKE_PREFIX_PATH
export CMAKE_IGNORE_PATH=/usr/lib/x86_64-linux-gnu

# 编译
#catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR=/opt/opencv-3.4.11/share/OpenCV \

source devel/setup.bash

