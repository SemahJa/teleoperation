FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends software-properties-common \
    && add-apt-repository --yes universe \
    && apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake git python3-colcon-common-extensions python3-rosdep python3-vcstool \
    libpoco-dev \
    ros-humble-desktop \
    ros-humble-controller-interface ros-humble-pluginlib ros-humble-geometry-msgs ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
    ros-humble-joint-state-publisher ros-humble-joint-state-broadcaster ros-humble-robot-state-publisher ros-humble-joint-trajectory-controller \
    ros-humble-rviz2 ros-humble-moveit ros-humble-moveit-ros-planning-interface \
    ros-humble-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*
RUN rosdep init 2>/dev/null || true
WORKDIR /ws
COPY . /ws
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
CMD ["bash"]
