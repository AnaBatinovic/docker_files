FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu18.04

LABEL maintainer ana.milas@fer.hr

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=melodic
ARG USER=developer 

# Install all the things to stop docker build from breaking
RUN ln -fs /usr/share/zoneinfo/Europe/Zagreb /etc/localtime && \
    apt-get update && apt-get install -q -y \
    git \
    nano \
    tmux \
    git-gui \
    sudo \
    lsb-release \
    gnupg2 \
    apt-utils \
    dialog \
    curl \
    ca-certificates \
    bzip2 \
    libx11-6 \
    # Base tools
    cmake \
    build-essential \
    unzip \
    pkg-config \
    python-dev \
    # OpenCV dependencies
    python-numpy \
    # Pangolin dependencies
    libgl1-mesa-dev \
    libglew-dev \
    libpython2.7-dev \
    libeigen3-dev \
    apt-transport-https \
    software-properties-common \
    tzdata && \
    dpkg-reconfigure --frontend noninteractive tzdata


# Install ROS
RUN curl https://raw.githubusercontent.com/larics/uav_ros_stack/main/installation/dependencies/ros.sh | bash

# Install General ROS things
RUN curl https://raw.githubusercontent.com/larics/uav_ros_stack/main/installation/dependencies/general.sh | bash

# Install Realsense specific dependencies
RUN apt-get update && \
    apt-get install -y \
        openssh-server \
        libmysqlclient-dev \
        ros-$ROS_DISTRO-realsense2-camera \
        ros-$ROS_DISTRO-rgbd-launch \
        ros-$ROS_DISTRO-realsense2-description \
        libglew-dev \
        libglfw3 \
        libglfw3-dev \
        software-properties-common \
        libnvidia-gl-440 

# Install Realsense Viewer
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get update && \
    apt-get install -y \
        librealsense2-dkms \
        librealsense2-utils

# Set realsense rules
RUN curl https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules >> /etc/udev/rules.d/99-realsense-libusb.rules

# Agent forwarding during docker build https://stackoverflow.com/questions/43418188/ssh-agent-forwarding-during-docker-build
# install ssh client and git
RUN apt-get install openssh-client git

# Download public key for github.com
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Build OpenCV (3.0 or higher should be fine)
RUN apt-get install -y python3-dev python3-numpy 
RUN apt-get install -y python-dev python-numpy
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev
RUN apt-get install -y libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
RUN apt-get install -y libgtk-3-dev

RUN cd /tmp && git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    git checkout 3.2.0 && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D BUILD_EXAMPLES=OFF  -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$nproc && make install && \
    cd / && rm -rf /tmp/opencv

# Build Pangolin
RUN cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && git checkout v0.6 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11 .. && \
    make -j$nproc && make install && \
    cd / && rm -rf /tmp/Pangolin

# Create catkin workspace
WORKDIR /root
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                 mkdir -p ~/catkin_ws/src && \
                 cd ~/catkin_ws/src && \
                 catkin_init_workspace && \
                 cd ~/catkin_ws/ && \
                 catkin_make"
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Clone and build ORB_SLAM3
WORKDIR /root/
RUN git clone -b docker_opencv3.2_fix https://github.com/jahaniam/ORB_SLAM3 
RUN cd /root/ORB_SLAM3 \
    chmod +x build.sh \
    ./build.sh

# Clone and build ORB_SLAM3_ROS
RUN touch ~/.bashrc
RUN echo "ROS_PACKAGE_PATH=/opt/ros/melodic/share:/ORB_SLAM3/Examples/ROS" >> ~/.bashrc
RUN bash -c "source ~/.bashrc"
RUN cd /root/ORB_SLAM3 \
    chmod +x build_ros.sh \
    ./build_ros.sh

# Clone and build orbslam3 ros wrapper
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c "git clone https://github.com/zhaozhongch/orbslam3_ros && \
                 cd ~/catkin_ws/src/orbslam3_ros && \
                 ./build_thrid_party.sh && \
                 cd ~/catkin_ws/ && \
                 source /opt/ros/melodic/setup.bash && \
                 catkin_make"