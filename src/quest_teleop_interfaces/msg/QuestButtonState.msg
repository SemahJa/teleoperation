# One physical button on the controller identified by QuestControllerState.
# Names: right thumbstick/a/b; left thumbstick/x/y/menu. System/Meta is not exposed.
string name
# Physical value, separate from action availability. No debounce/filter applied.
bool pressed
# OpenXR isActive: false means unavailable, not a confirmed release.
bool active
