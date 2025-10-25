FROM ros:kinetic-ros-base-xenial

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=kinetic \
    ROS_MASTER_URI=http://localhost:11311 \
    ROS_HOSTNAME=localhost

# 换源到 NUS（在新加坡最快）；若慢可改 mirrors.ustc.edu.cn 或 mirrors.aliyun.com
RUN bash -lc "printf '%s\n' \
'deb http://mirrors.aliyun.com/ubuntu/ xenial main restricted universe multiverse' \
'deb http://mirrors.aliyun.com/ubuntu/ xenial-updates main restricted universe multiverse' \
'deb http://mirrors.aliyun.com/ubuntu/ xenial-backports main restricted universe multiverse' \
'deb http://mirrors.aliyun.com/ubuntu/ xenial-security main restricted universe multiverse' \
> /etc/apt/sources.list" \
 && apt-get update

# 基础工具 & 编译链
RUN apt-get install -y --no-install-recommends \
      locales ca-certificates gnupg2 dirmngr curl wget sudo \
      build-essential cmake git pkg-config \
      python python-dev python-pip python-setuptools \
      python-rosdep python-rosinstall python-rosinstall-generator python-wstool \
      python-catkin-tools \
      libx11-6 libxext6 libxrender1 libxft2 \
 && rm -rf /var/lib/apt/lists/* \
 && locale-gen en_US.UTF-8

# VIO 依赖（msckf_vio 所需）
RUN apt-get update && apt-get install -y --no-install-recommends \
      libeigen3-dev libopencv-dev libboost-all-dev libsuitesparse-dev \
      ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-tf ros-kinetic-pcl-ros \
 && rm -rf /var/lib/apt/lists/*

# rosdep（容器内）
RUN rosdep init || true && rosdep update || true

# 用户（与 compose 里的 UID/GID 对齐）
ARG USERNAME=dev9
ARG USER_ID=1000
ARG GROUP_ID=1000
RUN groupadd -g ${GROUP_ID} ${USERNAME} \
 && useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ${USERNAME} \
 && usermod -aG sudo ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/${USERNAME}

# 工作区
RUN mkdir -p /ws/src
WORKDIR /ws
SHELL ["/bin/bash", "-lc"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

USER ${USERNAME}
CMD ["bash", "-lc", "sleep infinity"]

