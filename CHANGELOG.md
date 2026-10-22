# Changelog

All notable changes to the AERO project are documented here, organized by milestone.

## [v1.0] - 2026-10-22
### Milestone M9: Polish, one-command launch, demo
- Added one-command launcher `scripts/run_all.sh` and `full.launch.py`.
- Formatted and unit-tested all modules (25/25 pytest passing).
- Comprehensive project README and documentation added.

## [m8-pass] - 2026-10-22
### Milestone M8: Evaluation and advanced extras
- Implemented `aero_eval` package with `metrics_logger`, `run_trials.py`, and `report.py`.
- Added automated benchmark tracking: exploration coverage, path length, collision events, localization error.
- Integrated Extra B: Claude Vision scene description (`describe_scene` tool).

## [m7-pass] - 2026-10-22
### Milestone M7: Web dashboard
- Implemented `aero_dashboard` with FastAPI and WebSocket server.
- Built single-file vanilla HTML5/JS canvas frontend.
- Added live occupancy grid, robot arrow, semantic object markers, MJPEG debug camera stream, chat interface, and hardware E-Stop button.

## [m6-pass] - 2026-10-22
### Milestone M6: LLM Commander and Safety Gate
- Created `llm_commander` with Claude API tool use (5-turn limit, 20s timeout).
- Implemented robust regex-based `offline_parser` fallback when `ANTHROPIC_API_KEY` is not set.
- Created `safety_gate` action server (`SafeGoTo`) implementing 6-layer goal validation: bounds check, costmap clearance, 15m distance limit, 2s rate limit, LiDAR watchdog (1s timeout), and E-stop handler.
- Strictly enforced single-publisher rule for `/cmd_vel`.

## [m5-pass] - 2026-10-22
### Milestone M5: Autonomous frontier exploration
- Implemented `frontier_utils.py` with 8-connected cluster detection, minimum cell size filter (8 cells), and snapping to free space.
- Implemented `frontier_explorer` node with Nav2 `NavigateToPose` client, 60s timeout, failed goal blacklisting, and automatic `DONE` state transition after 5 empty cycles.

## [m4-pass] - 2026-10-22
### Milestone M4: Semantic map
- Built `object_localizer` node projecting 2D bounding boxes to 3D map coordinates via camera intrinsics + median LiDAR beam fusion and TF2 lookup.
- Built `semantic_map` node with Euclidean distance association (0.6m threshold), confidence-weighted spatial averaging, and 3-observation confirmation filter.
- Added RViz `MarkerArray` visualization and `QueryObjects` service.
- Added JSON persistence to `~/.aero/semantic_map.json`.

## [m3-pass] - 2026-10-22
### Milestone M3: Perception (YOLOv8)
- Created `yolo_detector` node wrapping Ultralytics YOLOv8n.
- Implemented rate throttling (5 Hz) and CPU/GPU fallback.
- Published `vision_msgs/Detection2DArray` and `/aero/debug_image`.
- Created custom `aero_house.sdf` Gazebo world populated with Fuel 3D models (chair, table, bottle, potted plant).

## [m2-pass] - 2026-10-22
### Milestone M2: SLAM Toolbox and Nav2
- Created `nav_slam.launch.py` integrating `slam_toolbox` (online async) and `nav2_bringup`.
- Tuned `nav2_params.yaml` for TurtleBot3 waffle_pi (radius 0.22m, inflation 0.55m, speed 0.26 m/s).
- Configured collision monitor safety polygons.
- Created `scripts/save_map.sh` for map serialization.

## [m1-pass] - 2026-10-22
### Milestone M1: Simulation baseline and reactive obstacle guard
- Created `sim.launch.py` supporting `house` and `world` Gazebo environments with headless mode support.
- Built `aero_controller` package with `obstacle_guard.py` reactive controller.
- Discovered and verified `/cmd_vel` Twist message specifications in `topics.yaml`.

## [m0-pass] - 2026-10-22
### Milestone M0: Environment check and repository bootstrap
- Created `scripts/env_check.sh` and `scripts/env.sh`.
- Created `.gitignore` and `AGENTS.md`.
- Defined ROS 2 custom message interfaces in `aero_interfaces`.
