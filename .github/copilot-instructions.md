# Workspace instructions

- This workspace targets a legacy Panda FCI 4.2.2 only.
- Keep `libfranka` pinned to LCAS `lcas_0.9.2` and the LCAS ROS 2 driver pinned in `third_party.repos`.
- Do not add an interface allowing policy code to publish joint torque, velocity, position, or controller switch commands.
- Default launches must not command physical hardware.
- All physical robot commands require bounded workspace, speed, watchdog, and explicit operator action.
- Build with ROS 2 Humble on Ubuntu 22.04.
