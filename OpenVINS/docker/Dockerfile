FROM osrf/ros:noetic-desktop-full

ARG USERNAME=dev8
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# 基础与常用依赖（OpenVINS + GUI/RVIZ）
RUN apt-get update && apt-get install -y --no-install-recommends \
    nano git cmake build-essential gdb clang rsync \
    python3-dev python3-numpy python3-matplotlib python3-psutil python3-tk \
    python3-catkin-tools python3-osrf-pycommon \
    libeigen3-dev libatlas-base-dev libsuitesparse-dev libgoogle-glog-dev \
    libgflags-dev libceres-dev \
    ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-message-filters \
    ros-noetic-tf ros-noetic-rviz mesa-utils \
 && rm -rf /var/lib/apt/lists/*

# 用户
RUN groupadd -g ${GROUP_ID} ${USERNAME} \
 && useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ${USERNAME}
USER ${USERNAME}
WORKDIR /ws

# 方便交互：进入容器即自动 source ROS
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc

