# Ghost Maio 2025

# Ubuntu 22.04
# ROS2 Humble
# Aerostack2, 

FROM aerostack2/nightly-humble:latest

WORKDIR /root/

# Gazebo Fortress clean uninstall
RUN apt remove ignition* -y
RUN apt autoremove -y

# Install Gazebo Harmonic
RUN apt-get install lsb-release wget gnupg && apt-get update
RUN apt-get install nano && apt-get update
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y -q gz-harmonic

# Install ros-gazebo dependencies
RUN apt update && apt install ros-humble-ros-gzharmonic -y

RUN apt-get update
RUN apt-get install apt-utils software-properties-common -y


RUN apt install git tmux tmuxinator -y
## ROS2 utils

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

RUN mkdir -p /root/aerostack2_ws/src/
WORKDIR /root/aerostack2_ws/src/

# Agent forwarding during docker build https://stackoverflow.com/questions/43418188/ssh-agent-forwarding-during-docker-build

ENV DOCKER_BUILDKIT=1
RUN apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v"
USER root

RUN mkdir -p -m 0600 ~/.ssh/ && ssh-keyscan github.com >> ~/.ssh/known_hosts
ARG HOME=/root

RUN rm -rf aerostack2
RUN --mount=type=ssh git clone git@github.com:ghost-drones/aerostack2.git

RUN export GZ_VERSION=harmonic
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --parallel-workers 3 --cmake-args -DCMAKE_BUILD_TYPE=Release

# mavros
RUN sudo apt install ros-humble-mavros ros-humble-mavros-extras -y 
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo 'export AEROSTACK2_PATH=/root/aerostack2_ws/src/aerostack2' >> ~/.bashrc
RUN echo 'source $AEROSTACK2_PATH/as2_cli/setup_env.bash' >> ~/.bashrc

# Ghost AS2 tutorial
WORKDIR $HOME
RUN mkdir -p tutorials/src
WORKDIR $HOME/tutorials/src
RUN --mount=type=ssh git clone git@github.com:ghost-drones/aerostack2_tutorial.git

COPY to_copy/tmux $HOME/.tmux.conf
COPY to_copy/aliases $HOME/.bash_aliases
RUN echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/tutorials/src/aerostack2_tutorial/models' >> ~/.bashrc

# Ghost PX4
WORKDIR /root/
RUN --mount=type=ssh git clone -b release/1.15 git@github.com:ghost-drones/PX4-Autopilot.git --recursive
WORKDIR /root/PX4-Autopilot/

# Upgrading the system
RUN apt update && apt upgrade -y
RUN apt-get install wget -y

# Install dependencies
RUN ./Tools/setup/ubuntu.sh --no-nuttx
RUN make px4_sitl

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
WORKDIR $HOME/tutorials
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install
RUN echo "source /root/tutorials/install/setup.bash" >> ~/.bashrc
RUN echo "export AEROSTACK2_TUTORIAL_PATH=/root/tutorials/src/aerostack2_tutorial" >> ~/.bashrc
RUN sudo apt-get install nano

# AS2 Aerial Platforms
WORKDIR $HOME/aerostack2_ws/src/aerostack2/as2_aerial_platforms

# PX4 Platform
RUN --mount=type=ssh git clone git@github.com:ghost-drones/as2_platform_pixhawk.git

# PX4 Platform
RUN --mount=type=ssh git clone git@github.com:ghost-drones/as2_platform_mavlink.git

# Mavlink Platform
WORKDIR /root/aerostack2_ws
RUN pip3 install numpy==1.24.4

CMD ["bash"]