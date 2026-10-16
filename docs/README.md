# Documentation guide

Start with the [repository README](../README.md) to build the workspace, configure
an arm and launch teleoperation. ROS commands run inside the ROS container;
Docker commands run on the host PC. The host repository is mounted at `/ws`.

| What you want to do | Read |
| --- | --- |
| Understand packages and add a controller | [Package architecture](PACKAGE_STRUCTURE.md) |
| Understand trigger release, homing, Hand behavior or startup errors | [Runtime guide](TELEOPERATION_RUNTIME.md) |
| Check mapping without a robot; calibrate the coordinate basis | [Robot-free mapping](ROBOT_FREE_MAPPING.md) |
| Plot live data, record an experiment or create MATLAB reports | [Trajectory monitoring](TRAJECTORY_MONITORING.md) |
| Change torque caps or understand CBF constraints and timing | [Panda safety filter](CBF_PANDA.md) |
| Format code, build and run tests | [Development guide](DEVELOPMENT.md) |
| Understand USB messages and timestamps | [Quest USB protocol](quest-usb-protocol.md) |
| Understand the native headset application | [Quest app](VR_APP_PORT.md) |
| Review implemented features and validation limits | [Implementation record](IMPLEMENTATION_PROGRESS.md) |
| Review proposed asynchronous CBF and reference smoothing work | [Milestone 2 plan](MILESTONE_2.md) |

The combined-launch configuration files are
[`single_panda.yaml`](../src/teleop_bringup/config/single_panda.yaml) and
[`dual_panda.yaml`](../src/teleop_bringup/config/dual_panda.yaml). Inspect the
selected file before running it: these are editable installation settings, and
the single-arm file may already select hardware. Package-local YAML files apply
to standalone launches.

Current behavior includes joint position holding after normal trigger release,
35% default actuator caps even with CBF disabled, full-cycle tracking telemetry,
and MATLAB reports with explicit legends. The CBF owner still waits for the
current cycle's result. Asynchronous result reuse and pose interpolation remain
planned features.

`IMPLEMENTATION_PROGRESS.md` records earlier stages as history. Its older paths,
test counts and behavior descriptions are not current operating instructions.
`AGENTS.md` contains contributor/agent policy and historical decisions.
