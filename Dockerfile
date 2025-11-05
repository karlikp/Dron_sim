FROM ros:humble-ros-base
ARG USERNAME=karol
ARG GAZEBO_VERSION=garden
ARG ROS_DISTRO=humble
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Install general dependencies
RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    build-essential \
    cmake \
    ros-dev-tools \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user with sudo privileges
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}

# Install Gazebo + Integration with ROS2
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN sudo apt-get update --allow-releaseinfo-change && \
    sudo apt-get install -y gz-${GAZEBO_VERSION} \
    ros-${ROS_DISTRO}-ros-gz${GAZEBO_VERSION} \
    ros-humble-rviz2 \
    && sudo rm -rf /var/lib/apt/lists/*

# Install Gazebo Garden + integration with ROS 2 Humble
# RUN sudo apt-get update && \
#     sudo apt-get install -y \
#     gz-garden \
#     ros-humble-ros-gz-sim \
#     ros-humble-ros-gz-bridge \
#     ros-humble-ros-gz-interfaces \
#     ros-humble-ros-gz-image \
#     ros-humble-gz-ros2-control \
#     ros-humble-rviz2 && \
#     sudo rm -rf /var/lib/apt/lists/*

# Install QGroundControl
WORKDIR /home/${USERNAME}
RUN sudo usermod -aG dialout "${USERNAME}" && \
    sudo apt-get update && \
    sudo apt install -y gstreamer1.0-plugins-bad \
        gstreamer1.0-libav \
        gstreamer1.0-gl \
        libqt5gui5 libfuse2 fuse libpulse-mainloop-glib0 \
        libmosquitto-dev mosquitto \
    && sudo rm -rf /var/lib/apt/lists/*

RUN wget -q -O QGroundControl-x86_64.AppImage \
      https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage && \
    chmod +x ./QGroundControl-x86_64.AppImage

# Add PX4
WORKDIR /home/${USERNAME}
RUN git clone "https://github.com/PX4/PX4-Autopilot.git" --branch v1.15.2 --recursive && \
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
ENV PX4_PATH=/home/${USERNAME}/PX4-Autopilot
WORKDIR ${PX4_PATH}
RUN make "-j$(nproc)" px4_sitl

# Clone repositories to workspace
ENV ROS_WORKSPACE=/home/${USERNAME}
WORKDIR ${ROS_WORKSPACE}
RUN git clone "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git" --branch v2.4.2 && \
    git clone "https://github.com/PX4/px4_msgs.git" --branch "release/1.15"

# Install ROS dependencies
WORKDIR $ROS_WORKSPACE
RUN rosdep update && \
    rosdep install --from-paths ${ROS_WORKSPACE} -r -y --ignore-src

# Build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR $ROS_WORKSPACE
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build

# Copy simulation package to workspace
WORKDIR $ROS_WORKSPACE/src
RUN pwd
COPY . dron_sim

WORKDIR $ROS_WORKSPACE

COPY ./scripts ./scripts
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    ./scripts/sim_build.sh

RUN echo "source \"/opt/ros/${ROS_DISTRO}/setup.bash\"" >> "/home/${USERNAME}/.bashrc" && \
    echo "source \"${ROS_WORKSPACE}/install/setup.bash\"" >> "/home/${USERNAME}/.bashrc"

RUN sudo sed -i '$i source $ROS_WORKSPACE/install/setup.bash' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]