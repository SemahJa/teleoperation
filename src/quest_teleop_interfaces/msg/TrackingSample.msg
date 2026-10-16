# One coherent control cycle. ROS publication happens outside realtime control.
# Full stream: tracking_samples. Latest-only live view: tracking_status.
std_msgs/Header header
uint64 control_cycle
uint64 state_time_pc_monotonic_ns
uint64 control_time_pc_monotonic_ns
uint64 command_issue_time_pc_monotonic_ns
uint64 command_source_time_pc_monotonic_ns
uint64 command_receipt_time_pc_monotonic_ns
uint64 openxr_location_time_ns
uint64 sequence
uint64 reference_space_generation
string session_id
string reason
bool hardware_backend
bool motion_enabled
bool requested_pose_valid
bool requested_twist_valid
bool cartesian_reference_valid
bool homing_active
float64 period_seconds
float64 command_age_ms
float64 lambda_translation
float64 lambda_rotation
float64 gain_translation
float64 gain_rotation

# Requested pose is mapped into the robot base frame before any reference
# shaping. Requested twist is the mapper's pre-low-pass blend; reference twist
# is the filtered twist actually supplied to the selected Cartesian law.
geometry_msgs/Pose requested_pose
geometry_msgs/Pose reference_pose
geometry_msgs/Pose measured_pose
geometry_msgs/Twist requested_twist
geometry_msgs/Twist reference_twist
geometry_msgs/Twist measured_twist
float64[3] position_error
float64[3] rotation_error
float64 position_error_norm
float64 rotation_error_angle
float64 linear_velocity_error_norm
float64 angular_velocity_error_norm
float64[7] joint_position
float64[7] joint_velocity
float64[7] homing_joint_position
float64[7] homing_joint_velocity

# Current callback output, without robot-side gravity compensation.
float64[7] nominal_torque
float64[7] commanded_torque
# Readbacks belong to the incoming RobotState, before this callback's output
# has taken effect. robot_desired_torque excludes gravity; measured_torque is
# the link-side sensor reading and includes gravity. Neither is motor current.
bool robot_torque_valid
float64[7] robot_desired_torque
float64[7] measured_torque
bool gravity_valid
float64[7] gravity_torque
# Previous callback output is supplied for time alignment. It is NOT an
# acknowledgement that this exact command produced the current readback.
bool previous_command_valid
uint64 previous_control_cycle
uint64 previous_command_issue_time_pc_monotonic_ns
float64[7] previous_nominal_torque
float64[7] previous_commanded_torque
float64[7] torque_limits
float64 maximum_torque_rate
bool torque_rate_limited
bool torque_magnitude_limited
bool cbf_enabled
bool safety_fallback
bool solver_succeeded
float64 normalized_torque_intervention
uint64 monitor_queue_drops
