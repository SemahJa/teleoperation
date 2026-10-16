# Atomic impedance-controller input. Header timestamp is the mapped source sample time.
std_msgs/Header header
geometry_msgs/Pose target_pose
geometry_msgs/Twist desired_twist
# Mapped velocity before the mapper's low-pass filter. This is a blend of
# native/pose-derived velocity, not raw IMU data. Older/custom producers may
# leave requested_twist_valid false; consumers must not interpret zeros as data.
geometry_msgs/Twist requested_twist
bool requested_twist_valid
bool motion_enabled
string session_id
uint64 sequence
uint64 source_acquisition_time_ns
uint64 openxr_location_time_ns
bool source_time_valid
# PC CLOCK_MONOTONIC, shared with the controller in the host-network container.
uint64 source_time_pc_monotonic_ns
uint64 clock_uncertainty_ns
bool clock_sync_valid
uint64 reference_space_generation
# A disarmed message is not necessarily a valid physical trigger release.
bool input_valid
bool trigger_released
