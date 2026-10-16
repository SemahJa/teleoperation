# PlotJuggler tracking dashboard

`plotjuggler_tracking.xml` is the generated single-arm `/panda_1/tracking_status`
layout. It contains Pose, Velocity, Four torques and Diagnostics tabs.

Run `bash scripts/plotjuggler_tracking.sh /panda_1` inside the ROS container,
then subscribe to `/panda_1/tracking_status` using the ROS2 Topic Subscriber.
For multiple/custom namespaces, pass them to the helper; it generates a temporary
layout without modifying this checked-in default.

To regenerate the checked-in single-arm layout from its source:

```bash
python3 scripts/create_tracking_layout.py --namespace /panda_1 \
  --output config/plotjuggler_tracking.xml
```

Run this from the repository root. Runtime configuration belongs in
`src/teleop_bringup/config/`, not this dashboard directory.

See [the monitoring and MATLAB guide](../docs/TRAJECTORY_MONITORING.md) for signal
validity, timestamps, recording/export, MATLAB Docker licensing and reports.
