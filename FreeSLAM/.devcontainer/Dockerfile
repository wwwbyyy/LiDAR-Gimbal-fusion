ARG ROS_DISTRO=noetic
FROM osrf/ros:$ROS_DISTRO-desktop
# install basic tools
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y software-properties-common apt-utils bash-completion \
                          openssh-client wget curl zip unzip jq vim tmux git sudo \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
# install cmake
RUN wget https://apt.kitware.com/kitware-archive.sh \
    && bash kitware-archive.sh \
    && apt-get install -y cmake \
    && rm kitware-archive.sh \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
# install gcc
ARG GCC_VERSION=11
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test \
    && apt-get update \
    && apt-get install -y gcc-$GCC_VERSION g++-$GCC_VERSION gdb build-essential ninja-build \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-$GCC_VERSION $GCC_VERSION \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-$GCC_VERSION $GCC_VERSION \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
# install pcl
RUN apt-get update \
    && apt-get install -y ros-$ROS_DISTRO-pcl-ros \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
# install fmtlib
RUN export VERSION=9.1.0 \
    && wget https://github.com/fmtlib/fmt/archive/refs/tags/$VERSION.tar.gz \
    && tar -zxvf $VERSION.tar.gz \
    && cd fmt-$VERSION \
    && cmake -B build -DCMAKE_BUILD_TYPE=Release -DFMT_TEST=OFF \
    && cmake --build build --parallel $(nproc) \
    && cmake --install build \
    && cd ../ && rm -rf fmt-$VERSION && rm $VERSION.tar.gz
# install spdlog
RUN export VERSION=1.11.0 \
    && wget https://github.com/gabime/spdlog/archive/refs/tags/v$VERSION.tar.gz \
    && tar -zxvf v$VERSION.tar.gz \
    && cd spdlog-$VERSION \
    && cmake -B build -DCMAKE_BUILD_TYPE=Release -DSPDLOG_BUILD_EXAMPLE=OFF \
    && cmake --build build --parallel $(nproc) \
    && cmake --install build \
    && cd ../ && rm -rf spdlog-$VERSION && rm v$VERSION.tar.gz
# install yaml-cpp
RUN export VERSION=0.6.3 \
    && wget https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-$VERSION.tar.gz \
    && tar -zxvf yaml-cpp-$VERSION.tar.gz \
    && cd yaml-cpp-yaml-cpp-$VERSION \
    && cmake -B build -DCMAKE_BUILD_TYPE=Release -DYAML_CPP_BUILD_TESTS=OFF -DYAML_CPP_BUILD_TOOLS=OFF \
    && cmake --build build --parallel $(nproc) \
    && cmake --install build \
    && cd ../ && rm -rf yaml-cpp-yaml-cpp-$VERSION && rm yaml-cpp-$VERSION.tar.gz
# create user
ARG USERNAME=xinyang
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME
WORKDIR /home/$USERNAME
# install gtsam
RUN sudo add-apt-repository -y ppa:borglab/gtsam-release-4.0 \
    && sudo apt-get -y install libgtsam-dev libgtsam-unstable-dev \
    && sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
# install project requirements
RUN sudo apt-get update \
    && sudo apt-get install -y ros-noetic-gps-common \
    && sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*