# Raw controller sample. All poses/twists are expressed in the declared OpenXR reference space.
std_msgs/Header header
string session_id
uint64 sequence
uint64 acquisition_time_ns
uint64 openxr_location_time_ns
# Estimate converts Quest CLOCK_MONOTONIC into PC CLOCK_MONOTONIC.
bool clock_sync_valid
int64 clock_offset_pc_minus_quest_ns
uint64 clock_uncertainty_ns
uint64 clock_sync_age_ns
uint64 source_time_pc_monotonic_ns
bool source_time_valid
string reference_space
uint64 reference_space_generation
string controller_id
bool connection_valid
bool session_focused
bool pose_action_active
geometry_msgs/Pose grip_pose
bool position_valid
bool orientation_valid
bool position_tracked
bool orientation_tracked
geometry_msgs/Twist native_twist
bool linear_velocity_valid
bool angular_velocity_valid
float32 trigger_value
bool trigger_active
float32 grip_value
bool grip_active
# Only this controller's physical buttons: right thumbstick/a/b; left thumbstick/x/y/menu.
# Disconnect notifications carry an empty array with connection_valid=false.
QuestButtonState[<=4] buttons
