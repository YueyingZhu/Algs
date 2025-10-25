FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=dev7
ARG USER_ID=1000
ARG GROUP_ID=1000

# 1) 基础工具 & ROS Noetic
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata locales sudo git curl wget ca-certificates gnupg lsb-release \
    build-essential cmake pkg-config \
    python3 python3-pip python3-venv python3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8

# ROS sources
RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common && rm -rf /var/lib/apt/lists/*
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
    && rm -rf /var/lib/apt/lists/*

# rosdep 初始化（容器内）
RUN rosdep init || true && rosdep update

# 2) SchurVINS 依赖（与你笔记一致）
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools python3-vcstool \
    libglew-dev libopencv-dev libyaml-cpp-dev \
    libblas-dev liblapack-dev libsuitesparse-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# 3) 创建非 root 用户
RUN groupadd -g ${GROUP_ID} ${USERNAME} && \
    useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

# 4) 工作区
USER ${USERNAME}
ENV HOME=/home/${USERNAME}
WORKDIR /ws

# 5) 预置 bashrc：ROS & catkin 环境
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc && \
    echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc

# 6) 默认命令
CMD ["/bin/bash"]

