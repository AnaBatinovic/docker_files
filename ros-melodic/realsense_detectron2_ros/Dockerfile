FROM nvidia/cuda:10.2-devel-ubuntu18.04
# Use ARG - persists only during docker build
# https://github.com/moby/moby/issues/4032#issuecomment-192327844
ARG CATKIN_WORKSPACE=catkin_ws
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=melodic
ARG USER=developer 


# Install all the things to stop docker build from breaking
RUN ln -fs /usr/share/zoneinfo/Europe/Zagreb /etc/localtime && \
    apt-get update && apt-get install -q -y \
    git \
    sudo \
    lsb-release \
    gnupg2 \
    apt-utils \
    dialog \
    curl \
    ca-certificates \
    bzip2 \
    libx11-6 \
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
        ros-$ROS_DISTRO-ros-numpy \
        libglew-dev \
        libglfw3 \
        libglfw3-dev \
        software-properties-common \
        libnvidia-gl-440 \
        nano \
        tmux \
        git-gui

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

#  Create a non-root user
RUN adduser --disabled-password \
    --gecos '' ${USER}
RUN adduser ${USER} sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to non-root user 
USER developer

# Install Miniconda (the latest vesion !!! hahhaha) and Python 3.8
ENV CONDA_AUTO_UPDATE_CONDA=false
ENV PATH=/home/developer/miniconda/bin:$PATH
RUN curl -sLo ~/miniconda.sh https://repo.continuum.io/miniconda/Miniconda3-py38_4.12.0-Linux-x86_64.sh | bash \
 && chmod +x ~/miniconda.sh \
 && ~/miniconda.sh -b -p ~/miniconda \
 && rm ~/miniconda.sh \
 && conda install -y python==3.8.1 \
 && conda clean -ya

# CUDA 10.2-specific steps
RUN conda install pytorch torchvision torchaudio cudatoolkit=10.2 -c pytorch
# Install packages neccessary for training and testing neural networks
RUN conda install -c anaconda \
                     scipy \
                     configargparse \
                     cython \
                     pyyaml
 
RUN conda install -c conda-forge \
                     progress \
                     json_tricks \
                     tensorboardx \
                     easydict \
                     opencv \
                     pycocotools
RUN pip install -U rospkg \
                    yacs \
                    pyrealsense2 \
                    scikit-learn==0.22.2 \
                    scikit-image \
                    numba \
                    filterpy

USER root
# Install workspace
RUN curl https://raw.githubusercontent.com/larics/uav_ros_stack/main/installation/workspace_setup.sh | bash -s $CATKIN_WORKSPACE

# Clone and build other packages
WORKDIR /root/catkin_ws/src
RUN git clone git@github.com:AnaBatinovic/detectron2_ros.git
RUN python -m pip install -e detectron2
RUN git clone https://github.com/SfTI-Robotics/ROS-label-node.git
RUN bash -c "catkin build"
RUN cp ROS-label-node/sort.py detectron2/
RUN cp ROS-label-node/ros_colour_node.py detectron2/
RUN cp -r ROS-label-node/ros_numpy detectron2
