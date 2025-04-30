FROM ros:humble-ros-base

ARG USERNAME=highflyers
ARG ROS_DISTRO=humble
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Install general dependencies
RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    openssh-client \
    build-essential \
    cmake \
    ros-dev-tools \
    python3-pip \
    ros-humble-geographic-msgs \
    ros-humble-vision-msgs \
    ros-humble-std-msgs \
    ros-humble-image-geometry \
    ros-humble-launch \
    ros-humble-launch-xml \
    ros-humble-launch-ros \
    ros-humble-rviz2 \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install ultralytics dill pyrr shapely transitions matplotlib opencv-contrib-python cv_bridge
RUN pip3 install -U numpy

# Create a non-root user with sudo privileges
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Clone repositories to workspace
ENV ROS_WORKSPACE=/home/${USERNAME}/ws
WORKDIR ${ROS_WORKSPACE}/src
RUN git clone "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git" --branch v2.4.2 && \
    git clone "https://github.com/PX4/px4_msgs.git" --branch "release/1.15"

# Install ROS dependencies
WORKDIR $ROS_WORKSPACE
RUN rosdep update && \
    rosdep install --from-paths ${ROS_WORKSPACE} -r -y --ignore-src

# Build
WORKDIR $ROS_WORKSPACE
# hadolint ignore=SC1091
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build

# Copy simulation package to workspace
WORKDIR $ROS_WORKSPACE/src
RUN pwd
COPY . martian_mines_ros2

WORKDIR $ROS_WORKSPACE
# hadolint ignore=SC1091
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" 

RUN echo "source \"/opt/ros/${ROS_DISTRO}/setup.bash\"" >> "/home/${USERNAME}/.bashrc" && \
    echo "source \"${ROS_WORKSPACE}/install/setup.bash\"" >> "/home/${USERNAME}/.bashrc"

RUN sudo sed -i '$i source $ROS_WORKSPACE/install/setup.bash' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]