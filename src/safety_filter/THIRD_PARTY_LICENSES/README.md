# Source attribution

- Joint motion generator: adapted from thesis_ws/quest_fr3_controllers,
  Copyright (c) 2023 Franka Robotics GmbH and (c) 2026 Jiachun, Apache-2.0.
- Homing and CBF filter: adapted from thesis_ws/quest_fr3_controllers,
  Copyright (c) 2026 Jiachun, Apache-2.0. Changes include Panda limits/geometry,
  single-arm masking, actual torque anchors, invalid model/constraint checks,
  fixed-size runtime statuses and controlled per-arm cancellation.
- OSQP 0.6.3: commit 0dd00a578cf1c2691c5c379965d504c75bf6cfad,
  Apache-2.0. See OSQP_NOTICE and OSQP_APACHE_2_0.
- QDLDL: pinned OSQP submodule commit 7d16b70a10a152682204d745d814b6eb63dc5cd2,
  Apache-2.0. See QDLDL_APACHE_2_0.
- AMD ordering code bundled with OSQP: see AMD_LICENSE for its copyright and
  license terms. All original dependency source headers remain intact.
