# Optional camera/mock recording image; never installs a second RealSense SDK.
ARG CAMERA_IMAGE=franka-dual-realsense:4.56.4-sdk2.56.5
FROM ${CAMERA_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbag2-storage-mcap nlohmann-json3-dev python3-numpy \
    && rm -rf /var/lib/apt/lists/*
COPY src/quest_teleop_interfaces /opt/demonstration_ws/src/quest_teleop_interfaces
COPY src/quest_teleop /opt/demonstration_ws/src/quest_teleop
COPY src/camera_bringup /opt/demonstration_ws/src/camera_bringup
COPY src/demonstration_timing /opt/demonstration_ws/src/demonstration_timing
COPY src/recording /opt/demonstration_ws/src/recording
COPY src/teleop_bringup /opt/demonstration_ws/src/teleop_bringup
RUN source /opt/camera_ws/install/setup.bash && cd /opt/demonstration_ws \
    && colcon build --cmake-args -DBUILD_TESTING=OFF
ENTRYPOINT ["/bin/bash", "-lc", "source /opt/demonstration_ws/install/setup.bash; exec \"$@\"", "recording-entrypoint"]
CMD ["ros2", "launch", "recording", "demonstration.launch.py", "output_root:=/recordings"]
