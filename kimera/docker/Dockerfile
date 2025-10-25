FROM osrf/ros:noetic-desktop-full
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    build-essential cmake git python3-catkin-tools \
    libeigen3-dev libopencv-dev libtbb-dev \
    libgoogle-glog-dev libgflags-dev \
    ros-noetic-pcl-ros ros-noetic-cv-bridge ros-noetic-image-transport \
 && rm -rf /var/lib/apt/lists/*

# GTSAM 4.1.x（Kimera更稳）
RUN git clone --depth 1 --branch 4.1.1 https://github.com/borglab/gtsam.git /opt/gtsam && \
    cmake -S /opt/gtsam -B /opt/gtsam/build -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES=OFF && \
    cmake --build /opt/gtsam/build -j4 && cmake --install /opt/gtsam/build

# OpenGV
RUN git clone --depth 1 https://github.com/laurentkneip/opengv.git /opt/opengv && \
    cmake -S /opt/opengv -B /opt/opengv/build && \
    cmake --build /opt/opengv/build -j4 && cmake --install /opt/opengv/build

# --- Pangolin (v0.6) ---
RUN apt-get update && apt-get install -y \
    git cmake build-essential \
    libgl1-mesa-dev libegl1-mesa-dev libgles2-mesa-dev \
    libglew-dev libxkbcommon-dev libxrandr-dev libxi-dev libxxf86vm-dev \
    libwayland-dev wayland-protocols \
    libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
    libjpeg-dev libpng-dev zlib1g-dev \
    libopenexr-dev libilmbase-dev \
 && git clone --depth 1 --branch v0.6 https://github.com/stevenlovegrove/Pangolin.git /tmp/Pangolin \
 && cmake -S /tmp/Pangolin -B /tmp/pango-build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/pangolin-0.6 \
      -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF \
      -DCMAKE_CXX_FLAGS="-Wno-error -Wno-deprecated-copy" \
 && cmake --build /tmp/pango-build -j4 \
 && cmake --install /tmp/pango-build \
 && echo /opt/pangolin-0.6/lib > /etc/ld.so.conf.d/pangolin.conf \
 && ldconfig \
 && rm -rf /tmp/Pangolin /tmp/pango-build

# --- create a non-root user matching the host uid/gid ---
ARG USERNAME=dev
ARG USER_ID=1000
ARG GROUP_ID=1000

RUN if ! getent group ${GROUP_ID} >/dev/null; then groupadd -g ${GROUP_ID} ${USERNAME}; fi && \
    if ! id -u ${USER_ID} >/dev/null 2>&1; then useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ${USERNAME}; fi

USER ${USER_ID}:${GROUP_ID}
ENV HOME=/home/${USERNAME}

WORKDIR /ws

