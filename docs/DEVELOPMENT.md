# Development and verification

Use the existing Ubuntu 22.04 / ROS 2 Humble container described in the
[main README](../README.md). The repository is mounted at `/ws`; builds go to
`build/` and `install/`. Docker shares the host kernel, so the deployment host
must supply PREEMPT_RT for physical FCI control.

## Build the maintained pipeline

In the ROS container, start with a shell sourced from Humble:

```bash
cd /ws
source /opt/ros/humble/setup.bash
bash scripts/fetch_teleop_dependencies.sh
colcon build --base-paths src --packages-up-to teleop_bringup --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DBUILD_EXAMPLES=OFF
source install/setup.bash
```

The dependency helper imports pinned sources and initializes their submodules.
The build selects the teleoperation dependency graph; it excludes the legacy
Panda demos. Install dependencies as described in the main README on first
setup. A message saying libfranka did not use `BUILD_TESTING` is harmless;
the option controls tests in the ROS packages.

## Run robot-free tests

In the same container shell:

```bash
export ROS_DOMAIN_ID=137 ROS_LOCALHOST_ONLY=1
colcon test --base-paths src --executor sequential \
  --packages-select quest_teleop recording safety_filter teleop_controllers teleop_gripper teleop_bringup \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --test-result-base build --verbose
```

These tests use synthetic input, mock backends and temporary recordings. The
isolated domain keeps their publishers separate from a running operator session.
They cover mapping, command gates, torque laws, hold transitions, homing, Hand
decisions, safety/coordinator behavior, launch configuration and CSV export.
Mocks do not establish physical tracking performance, grasp quality, collision
clearance or worst-case realtime timing. Test MATLAB separately in a licensed
session using the [monitoring guide](TRAJECTORY_MONITORING.md).

## Keep formatting consistent

Optional developer tools are pinned in `requirements-dev.txt`. In a host or
development container with Python 3.10+ and clang-format 14, from the repository
root:

```bash
python3 -m venv /tmp/teleop-dev
source /tmp/teleop-dev/bin/activate
python -m pip install -r requirements-dev.txt
python scripts/format_code.py
python scripts/format_code.py --check
```

Install `clang-format-14` through your system package manager if needed and
ensure `clang-format` resolves to version 14. C++ uses `.clang-format`; Python
uses Black settings in `pyproject.toml`; MATLAB uses
`analysis/matlab/miss_hit.cfg`. MISS_HIT parses the supported R2022a syntax subset;
the report runtime remains MATLAB R2024b. Fix mode expands compressed statements
and normalizes indentation and spacing. Check mode reports drift without edits.

The script selects the seven maintained teleoperation packages, `scripts/` and
`analysis/matlab/`. It excludes Android, third-party sources, legacy
`panda_bringup`/`panda_demos`, generated build files and installed overlays.
Shell scripts use two-space indentation; check them with
`for script in scripts/*.sh; do bash -n "$script"; done`.
CMake and ROS interface definitions retain their package conventions.

Formatting improves readability; it is not evidence of faster control. Preserve
control equations and fixed-size realtime handoffs when optimizing. Torque
laws must not allocate, block, call ROS or perform I/O during `compute()` or
`reset()`. Build and run the affected tests after a coherent implementation change.

## Generated data and cleanup

`log/` is ignored by Git and contains disposable build/test logs, diagnostic
captures, raw episode bags, CSV exports and MATLAB reports. The 2026-10-16
maintenance cleanup removed the existing contents at the owner's request,
including raw recordings. New runs create directories as needed.
`analysis/matlab/` is maintained source and must remain when removing reports.
Keep experiments you need to reproduce, together with their actual runtime
YAML, outside a cleanup target.

To keep development logs outside the repository, put the colcon global option
before the verb: `colcon --log-base /tmp/teleop-build-logs build ...` or
`colcon --log-base /tmp/teleop-test-logs test ...`.

## Android and containers

Android build/ADB commands are in the main README and
[container guide](../docker/README.md). Reuse the validated APK unless Android
source or its wire contract changes. ROS-only tracking fields do not require
a headset reinstall. The native app uses OpenXR, JDK 17, Gradle 8.7, API 34,
build tools 34.0.0 and NDK 26.3.11579264. Host networking lets USB ADB reverse
reach the loopback ROS listener; the ADB server remains bound to loopback.

The optional MATLAB container has independent licensing and no robot/USB access.
See [MATLAB setup and handover](TRAJECTORY_MONITORING.md#first-time-matlab-docker-setup-and-licensing).
