FROM aerostack2/nightly-humble:latest

ARG HOME=/root

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

# Clone project gazebo
RUN git clone https://github.com/aerostack2/project_gazebo.git ../project_gazebo

# Recompile Aerostack2
WORKDIR /root/aerostack2_ws
RUN rm -rf build/ install/ log/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --parallel-workers 3 --cmake-args -DCMAKE_BUILD_TYPE=Release

# Agent forwarding during docker build https://stackoverflow.com/questions/43418188/ssh-agent-forwarding-during-docker-build

ENV DOCKER_BUILDKIT=1
RUN apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v"
USER root

RUN mkdir -p -m 0600 ~/.ssh/ && ssh-keyscan github.com >> ~/.ssh/known_hosts

WORKDIR $HOME
RUN mkdir -p tutorials/src
WORKDIR $HOME/tutorials/src
RUN --mount=type=ssh git clone git@github.com:ghost-drones/aerostack2_tutorial.git
RUN --mount=type=ssh git clone git@github.com:mgonzs13/yolo_ros.git
RUN pip3 install -r yolo_ros/requirements.txt
WORKDIR $HOME/tutorials
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --parallel-workers 3
RUN pip3 install numpy==1.24.4

RUN bash -c "source /root/tutorials/install/setup.bash" 
RUN bash -c "export AEROSTACK2_TUTORIAL_PATH=/root/tutorials/src/aerostack2_tutorial" 

COPY to_copy/tmux $HOME/.tmux.conf
COPY to_copy/aliases $HOME/.bash_aliases

CMD ["bash"]