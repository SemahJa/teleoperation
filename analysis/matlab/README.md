# MATLAB experiment analysis

Run in the optional official MATLAB Docker container. Base MATLAB R2024b is
the default; no ROS or Signal Processing Toolbox is required.

This directory contains reusable analysis source. Experiment outputs belong in
`log/analysis_reports/`; removing old reports does not require deleting these
scripts. Existing trial data was removed in the requested maintenance cleanup.

See [the monitoring guide](../../docs/TRAJECTORY_MONITORING.md) for first-use
account/network licensing, CSV export, batch/browser execution and handover.

`analyze_teleop(csv, output)` analyzes one arm. `analyze_teleop_export(exportDir,
outputDir)` analyzes every CSV in the manifest. Both accept `DerivativeWindow`,
`MaximumGap`, `MakePlots`, and `DPI` name/value options. Run
`runtests('test_teleop_analysis.m')` in a licensed MATLAB session before first use.

Figures are vector PDFs, configurable-DPI PNGs and editable `.fig` files. Metrics
and estimated derivatives are also exported as JSON/CSV. Raw torque readbacks
retain their gravity conventions and time offsets. Acceleration and jerk are
bandwidth-dependent supporting estimates from velocity, not sensor readings.
Every subplot includes a visible legend with explicit signal labels; joint
residuals identify Joint 1--7 with consistent colors. Regenerate old reports to
apply the updated legends. The report round-trip self-test verifies legends
in all 38 saved plot panels.

For formatting, use `python scripts/format_code.py` from the repository root
after installing the optional tools in `requirements-dev.txt`. The local
`miss_hit.cfg` controls MATLAB style. Runtime self-tests require licensed MATLAB;
source parsing alone does not validate numerical results or figure rendering.
