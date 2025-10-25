# Ubuntu 20.04 + ROS Noetic（含 rviz）
FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=dev6
ARG USER_ID=1000
ARG GROUP_ID=1000

# 基础依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    git wget curl ca-certificates sudo tzdata locales \
    build-essential cmake ninja-build \
    python3-pip python3-catkin-tools python3-vcstool python3-osrf-pycommon \
    # SVO2 系统依赖
    libglew-dev libopencv-dev libyaml-cpp-dev \
    # Ceres 依赖
    libblas-dev liblapack-dev libsuitesparse-dev \
    # Eigen + 常用 ROS 组件
    libeigen3-dev \
    ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-tf ros-noetic-tf2-ros ros-noetic-message-generation \
    ros-noetic-rqt ros-noetic-rqt-common-plugins \
    libtool autoconf automake m4 pkg-config \
    # 可视化/调试用
    tmux vim less tree \
 && rm -rf /var/lib/apt/lists/*

# 本地化（可选）
RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8

# 创建与宿主机一致的用户/组，避免权限问题
RUN groupadd -g ${GROUP_ID} ${USERNAME} && \
    useradd -m -s /bin/bash -u ${USER_ID} -g ${GROUP_ID} ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-${USERNAME}

# 先用 root 创建并授权
RUN mkdir -p /ws/src && chown -R ${USER_ID}:${GROUP_ID} /ws

USER ${USERNAME}
ENV USER=${USERNAME} HOME=/home/${USERNAME}
WORKDIR /home/${USERNAME}

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

WORKDIR /ws

# 默认保持容器常驻
CMD ["/bin/bash","-lc","sleep infinity"]

