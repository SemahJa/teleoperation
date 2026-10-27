source /opt/ros/humble/setup.bash
source /opt/camera_ws/install/local_setup.bash
source /camera_ws/install/hardware_demo/local_setup.bash
export LD_LIBRARY_PATH=/ws/install/libfranka/lib:$LD_LIBRARY_PATH
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/camera_ws/src/camera_bringup/config/fastdds_local_large_frames.xml
export ROS_LOG_DIR=/camera_ws/log/data_collection/ros
mkdir -p "$ROS_LOG_DIR"
