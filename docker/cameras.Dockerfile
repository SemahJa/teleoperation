# Separate image: do not install/mount a second SDK into the robot image.
ARG TELEOP_IMAGE=panda-ros2-fci-4.2.2:dev
FROM ${TELEOP_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# Release tags deliberately fixed; rebuild only after reviewing version changes.
ARG SDK_VERSION=v2.56.5
ARG WRAPPER_VERSION=4.56.4
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates git cmake build-essential libusb-1.0-0-dev libudev-dev libssl-dev \
    ros-humble-cv-bridge ros-humble-image-transport ros-humble-diagnostic-updater \
    ros-humble-rclcpp-components ros-humble-sensor-msgs ros-humble-tf2-ros \
    ros-humble-nav-msgs python3-yaml python3-pytest \
    && rm -rf /var/lib/apt/lists/*
# Refuse an ambiguous SDK installation in a changed base image.
RUN ! dpkg-query -W -f='${Status}\n' librealsense2 ros-humble-librealsense2 2>/dev/null | grep -q 'install ok installed'
RUN git clone --depth 1 --branch ${SDK_VERSION} https://github.com/realsenseai/librealsense.git /opt/librealsense-src \
    && cmake -S /opt/librealsense-src -B /tmp/sdk-build \
       -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
       -DBUILD_UNIT_TESTS=OFF -DFORCE_RSUSB_BACKEND=ON -DBUILD_WITH_CUDA=OFF \
    && cmake --build /tmp/sdk-build -j2 && cmake --install /tmp/sdk-build && ldconfig \
    && rm -rf /tmp/sdk-build
RUN git clone --depth 1 --branch ${WRAPPER_VERSION} https://github.com/realsenseai/realsense-ros.git /opt/camera_ws/src/realsense-ros
COPY src/camera_bringup /opt/camera_ws/src/camera_bringup
RUN source /opt/ros/humble/setup.bash \
    && cd /opt/camera_ws && colcon build --packages-select realsense2_camera_msgs realsense2_camera camera_bringup \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -Drealsense2_DIR=/usr/local/lib/cmake/realsense2
ENTRYPOINT ["/bin/bash", "-lc", "source /opt/camera_ws/install/setup.bash; exec \"$@\"", "camera-entrypoint"]
CMD ["ros2", "launch", "camera_bringup", "cameras.launch.py"]

# Verify release tags against audited commits, including cached build layers.
RUN test "$(git -C /opt/librealsense-src rev-parse HEAD)" = 38a41441971387197193ad3aeae3cefe6a11f2cb \
    && test "$(git -C /opt/camera_ws/src/realsense-ros rev-parse HEAD)" = bafc21080c5c8e259dadbb309797949aee0dd950 \
    && for package in librealsense2 ros-humble-librealsense2; do \
         if dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'install ok installed'; then exit 1; fi; \
       done
