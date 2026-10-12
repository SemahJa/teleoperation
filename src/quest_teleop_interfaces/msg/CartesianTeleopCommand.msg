# Atomic impedance-controller input. Header timestamp is the mapped source sample time.
std_msgs/Header header
geometry_msgs/Pose target_pose
geometry_msgs/Twist desired_twist
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
