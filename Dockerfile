ARG ROS_DISTRO=noetic
FROM ros:$ROS_DISTRO
# set deb non-interactive
ARG DEBIAN_FRONTEND=noninteractive
# set time zone
ENV TZ=Asia/Shanghai

# set proxy from host
ENV http_proxy=http://192.168.1.2:20171
ENV https_proxy=http://192.168.1.2:20171

# install basic tools
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y software-properties-common apt-utils \
                          bash-completion sudo wget curl zstd pv vim git tmux \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*
# install gcc-13
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test \
    && apt-get update \
    && apt-get install -y gcc-13 g++-13 gdb build-essential ninja-build \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 13 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 13 \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*
# install llvm-18
RUN wget https://apt.llvm.org/llvm.sh \
    && bash llvm.sh 18 \
    && rm llvm.sh \
    && rm -rf /var/lib/apt/lists/* 
ENV PATH="/usr/lib/llvm-18/bin:${PATH}"
# install gtsam-4.0
RUN add-apt-repository -y ppa:borglab/gtsam-release-4.0 \
  && apt-get -y install libgtsam-dev libgtsam-unstable-dev
# install gps-common
RUN apt-get install -y ros-noetic-gps-common

# install cmake
RUN wget https://apt.kitware.com/kitware-archive.sh \
    && bash kitware-archive.sh \
    && apt-get install -y cmake \
    && rm kitware-archive.sh \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*
# install libraries
RUN apt-get update \
    && apt-get install -y ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-vision-opencv \
                          ros-${ROS_DISTRO}-rosbridge-suite ros-${ROS_DISTRO}-foxglove-bridge \
                          ros-${ROS_DISTRO}-tf2-ros \
                          libyaml-cpp-dev \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

# install onetbb
# RUN apt-get install -y libtbb-dev

# install onetbb, sophus, robin-map, reflcpp
RUN wget https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.tar.gz -O /tmp/Sophus.tar.gz \
    && tar -xzf /tmp/Sophus.tar.gz -C /tmp/ \
    && wget https://github.com/Tessil/robin-map/archive/refs/tags/v1.2.1.tar.gz -O /tmp/robin-map.tar.gz \
    && tar -xzf /tmp/robin-map.tar.gz -C /tmp/ \
    && git clone https://github.com/oneapi-src/oneTBB.git /tmp/oneTBB

WORKDIR /tmp/Sophus-1.22.10
RUN mkdir build && cd build
WORKDIR /tmp/Sophus-1.22.10/build
RUN cmake -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF .. \
    && cmake --build . \
    && cmake --install .

WORKDIR /tmp/robin-map-1.2.1
RUN mkdir build && cd build
WORKDIR /tmp/robin-map-1.2.1/build
RUN cmake .. \
    && cmake --build . \
    && cmake --install .

WORKDIR /tmp/oneTBB
RUN mkdir build && cd build
WORKDIR /tmp/oneTBB/build
RUN cmake -DTBB_TEST=OFF .. \
    && cmake --build . \
    && cmake --install .

RUN rm -rf /tmp/*

# unset proxy
ENV http_proxy=
ENV https_proxy=

# create user
ARG USERNAME=loc
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME
WORKDIR /home/$USERNAME

ENV SHELL /bin/bash

# initialize ros environment
RUN mkdir -p loc_ws/src
WORKDIR /home/$USERNAME/loc_ws
RUN echo "source /opt/ros/noetic/setup.bash\nsource /home/$USERNAME/loc_ws/devel/setup.bash\nexport ROS_MASTER_URI=http://192.168.0.140:11311\nexport ROS_IP=192.168.0.20" >> /home/$USERNAME/.bashrc
