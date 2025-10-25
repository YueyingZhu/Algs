# syntax=docker/dockerfile:1.7

FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
ARG OPENCV_VERSION=3.4.11
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USERNAME=dev4

# 1) 换源（可注释掉恢复官方）
RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://sg.archive.ubuntu.com/ubuntu/|g' /etc/apt/sources.list \
 && sed -i 's|http://security.ubuntu.com/ubuntu|http://sg.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list || true

# 2) 基础依赖 + BuildKit 缓存（注意：无 libcsparse3）
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential cmake git pkg-config ca-certificates wget curl \
      gdb unzip tar xz-utils sudo \
      libeigen3-dev libyaml-cpp-dev libboost-all-dev \
      libsuitesparse-dev libcholmod3 \
      libglew-dev freeglut3-dev libgl1-mesa-dev mesa-utils \
      libxext-dev libxrender-dev libxrandr-dev \
      libwxgtk3.0-gtk3-dev \
      libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev \
      libjpeg-dev libpng-dev libtiff-dev zlib1g-dev \
      python3 python3-pip python3-dev python3-numpy && \
    rm -rf /var/lib/apt/lists/*

# OpenCV 3.4.11 + contrib（全克隆 + 全量主模块 + Python3 绑定）
RUN if [ ! -d "/opt/opencv-${OPENCV_VERSION}" ]; then \
      set -eux; \
      mkdir -p /opt && cd /opt && \
      git clone https://github.com/opencv/opencv opencv-${OPENCV_VERSION} && \
      cd opencv-${OPENCV_VERSION} && git checkout ${OPENCV_VERSION} && cd .. && \
      git clone https://github.com/opencv/opencv_contrib opencv_contrib-${OPENCV_VERSION} && \
      cd opencv_contrib-${OPENCV_VERSION} && git checkout ${OPENCV_VERSION} && cd .. && \
      mkdir -p /opt/opencv-${OPENCV_VERSION}/build && cd /opt/opencv-${OPENCV_VERSION}/build && \
      cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/opencv-${OPENCV_VERSION} \
        -DOPENCV_ENABLE_NONFREE=ON \
        -DOPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib-${OPENCV_VERSION}/modules \
        -DBUILD_opencv_xfeatures2d=ON \
        -DBUILD_opencv_python3=ON \
        -DPYTHON3_EXECUTABLE=/usr/bin/python3 \
        -DPYTHON3_INCLUDE_DIR=/usr/include/python3.8 \
        -DPYTHON3_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so \
        -DPYTHON3_NUMPY_INCLUDE_DIRS=/usr/lib/python3/dist-packages/numpy/core/include \
        -DPYTHON3_PACKAGES_PATH=/opt/opencv-${OPENCV_VERSION}/lib/python3.8/dist-packages \
      && make -j"$(nproc)" \
      && make install \
      && echo "/opt/opencv-${OPENCV_VERSION}/lib" > /etc/ld.so.conf.d/opencv.conf \
      && ldconfig ; \
    fi


# 让后续调试/重编不再遇到权限问题（把 /opt 下相关目录交给你的用户）
# 也顺便保留源码，便于你二次 cmake
RUN mkdir -p /opt && \
    chown -R ${USER_ID}:${GROUP_ID} /opt/opencv-${OPENCV_VERSION} /opt/opencv_contrib-${OPENCV_VERSION} || true

# 4) 环境变量（固定到 3.4.11；Python 绑定可直接 import）
ENV OpenCV_DIR=/opt/opencv-3.4.11/share/OpenCV
ENV LD_LIBRARY_PATH=/opt/opencv-3.4.11/lib
ENV PKG_CONFIG_PATH=/opt/opencv-3.4.11/lib/pkgconfig
ENV PYTHONPATH=/opt/opencv-3.4.11/lib/python3.8/dist-packages

# 5) g2o（20160424_git；含历史符号链接修复）
RUN if [ ! -d "/opt/g2o" ]; then \
      git clone https://github.com/RainerKuemmerle/g2o.git /opt/g2o && \
      cd /opt/g2o && git checkout 20160424_git && \
      mkdir build && cd build && \
      cmake .. -DCMAKE_BUILD_TYPE=Release -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF && \
      make -j"$(nproc)" && make install ; \
    fi && \
    cd /usr/local/lib && ln -sf libg2o_csparse_extension.so libg2o_ext_csparse.so || true && ldconfig

# 6) 创建非 root 用户，并把 /opt 的写权限交给他（见上 chown）
RUN groupadd -g ${GROUP_ID} ${USERNAME} && \
    useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ${USERNAME} && \
    usermod -aG video,render ${USERNAME} || true && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /ws
CMD ["/bin/bash"]

