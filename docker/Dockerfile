# @author Alberto Soragna (alberto dot soragna at gmail dot com)
# @2018 

FROM ubuntu:16.04
MAINTAINER Alberto Soragna alberto dot soragna at gmail dot com

# working directory
ENV HOME /root
WORKDIR $HOME

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# general utilities
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    git \
    vim \
    nano \
    python-dev \
    python3-pip \
    ipython

# pip
RUN pip3 install --upgrade pip


#### ROS2 SETUP


# Locale options
RUN apt-get install -y locales
RUN locale-gen en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# setup sources
RUN apt-get install -y lsb-release
RUN curl http://repo.ros2.org/repos.key | apt-key add -
RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# ROS setup requirements
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget
  
RUN python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures

# install additional ubuntu 16.04 requirements
RUN python3 -m pip install -U \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools

# install Fast-RTPS dependencies
RUN apt-get install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev \  
  libopensplice67

# create ros2 sdk workspace
RUN mkdir -p $HOME/ros2_sdk/src
WORKDIR $HOME/ros2_sdk
RUN wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
RUN vcs import src < ros2.repos

# initialize rosdep and install dependencies
RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"

# build the workspace
RUN colcon build --symlink-install


##### OPENCV3


# install opencv dependencies
RUN apt-get install -y libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

# create a workspace for opencv
RUN git clone -b 3.4 https://github.com/Itseez/opencv.git $HOME/opencv
RUN mkdir $HOME/opencv/build
WORKDIR $HOME/opencv/build

# add build script to docker image
COPY scripts/build_opencv.sh $HOME/scripts/build_opencv.sh

RUN chmod +x $HOME/scripts/build_opencv.sh
RUN bash $HOME/scripts/build_opencv.sh


######## ORB_SLAM2


# create a workspace for orbslam
ADD https://api.github.com/repos/alsora/ORB_SLAM2/git/refs/heads/master version.json
RUN git clone https://github.com/alsora/ORB_SLAM2.git $HOME/ORB_SLAM2
WORKDIR $HOME/ORB_SLAM2

# install orbslam depenendencies
RUN apt-get install -y \
  libboost-system-dev

# build orbslam
RUN /bin/bash -c 'source $HOME/ros2_sdk/install/setup.sh; \
  bash build.sh'


######## ORB_SLAM2 ROS2


# create a ros2 workspace
RUN mkdir -p $HOME/ws/src
WORKDIR $HOME/ws

# get some dependencies
RUN apt-get install -y libcanberra-gtk-module libboost-all-dev
RUN pip install numpy

# get source code
RUN git clone -b ros2 https://github.com/ros-perception/vision_opencv.git src/vision_opencv
RUN git clone https://github.com/ros2/message_filters src/message_filters
RUN ln -s $HOME/ORB_SLAM2/ros2_ORB_SLAM2 src/

# build the workspace
RUN /bin/bash -c 'source $HOME/ros2_sdk/install/setup.sh; \
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH; \
  colcon build'