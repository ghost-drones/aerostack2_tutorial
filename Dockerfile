# Ghost 2025

# Ubuntu 22.04
# ROS2 Humble

###########################################
# Base image
###########################################
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y --no-install-recommends \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y --no-install-recommends tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ENV DEBIAN_FRONTEND=

# Agent forwarding during docker build https://stackoverflow.com/questions/43418188/ssh-agent-forwarding-during-docker-build

ENV DOCKER_BUILDKIT=1
RUN apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v"
USER root

RUN mkdir -p -m 0600 ~/.ssh/ && ssh-keyscan github.com >> ~/.ssh/known_hosts
ARG HOME=/root

## ROS2 utils

RUN apt-get update
RUN apt-get install apt-utils software-properties-common -y

RUN apt install git tmux tmuxinator -y

RUN apt-get install python3-rosdep  \
                python3-pip     \
                python3-colcon-common-extensions \
                python3-colcon-mixin \
                ros-dev-tools -y

RUN apt-get install python3-flake8 \
                python3-flake8-builtins  \
                python3-flake8-comprehensions \
                python3-flake8-docstrings \
                python3-flake8-import-order \
                python3-flake8-quotes -y

RUN pip3 install pylint
RUN pip3 install flake8==4.0.1
RUN pip3 install pycodestyle==2.8
RUN pip3 install cmakelint cpplint

RUN apt-get install cppcheck lcov -y

RUN colcon mixin update default
RUN rm -rf log # remove log folder

RUN pip3 install colcon-lcov-result cpplint cmakelint
RUN pip3 install PySimpleGUI-4-foss

RUN pip install "numpy<1.24"

WORKDIR $HOME
# PX4 Offboard dependencies
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR $HOME/Micro-XRCE-DDS-Agent
RUN mkdir build
WORKDIR $HOME/Micro-XRCE-DDS-Agent/build
RUN cmake ..
RUN make
RUN sudo make install
RUN sudo ldconfig /usr/local/lib/

WORKDIR $HOME/tutorials/src/
RUN git clone -b release/1.15 https://github.com/PX4/px4_msgs.git
RUN git clone https://github.com/PX4/px4_ros_com.git
RUN git clone https://github.com/ros/executive_smach.git
WORKDIR $HOME/tutorials
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install
RUN echo "source /root/tutorials/install/setup.bash" >> ~/.bashrc
RUN echo "export AEROSTACK2_TUTORIAL_PATH=/root/tutorials/src/aerostack2_tutorial" >> ~/.bashrc
RUN sudo apt-get install nano