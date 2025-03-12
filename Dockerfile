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

WORKDIR $HOME

COPY to_copy/tmux $HOME/.tmux.conf
COPY to_copy/aliases $HOME/.bash_aliases

CMD ["bash"]