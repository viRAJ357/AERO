# AERO: Semantic Autonomous Explorer with LLM Commander

A software-only autonomous robotics stack running in simulation (Gazebo Harmonic) with ROS 2 Jazzy Jalisco. AERO explores unknown environments using frontier exploration, performs real-time object detection with YOLOv8, builds a persistent semantic map using LiDAR-camera fusion, and accepts high-level natural-language instructions via an LLM Commander with a strict multi-layer safety gate.

---

## Architecture Diagram

```
  User (Web Dashboard Chat / CLI)
          │  natural-language text
          ▼
  ┌──────────────────┐    tool calls     ┌──────────────────┐   NavigateToPose  ┌────────┐
  │  llm_commander   │ ────────────────> │   safety_gate    │ ────────────────> │  Nav2  │
  │  (Claude API /   │ <──────────────── │  (goal & costmap │                   └────────┘
  │   offline parser)│   results/errors  │   validation)    │                       │
  └──────────────────┘                   └──────────────────┘                  /cmd_vel_nav
     │           │                                                                  ▼
     │ query     │ set_mode                                                ┌─────────────────┐
     ▼           ▼                                                         │collision_monitor│
  ┌──────────────────┐                   ┌─────────────────────┐           │  -> /cmd_vel    │
  │   semantic_map   │                   │  frontier_explorer  │           └─────────────────┘
  │  (object store)  │                   │  (mode = EXPLORE)   │                    │
  └──────────────────┘                   └─────────────────────┘                    ▼
     ▲                                                                        Robot
     │ 3D pose          ┌─────────────────┐       ┌──────────────┐          (TB3 waffle_pi)
  ┌──────────────────┐  │  slam_toolbox   │       │  Gazebo sim  │          (LiDAR + Camera)
  │ object_localizer │ ◄┤  (/map, TF)     │       │(camera, scan)│
  └──────────────────┘  └─────────────────┘       └──────────────┘
     ▲   (bearing + median LiDAR range -> map frame)      │
  ┌──────────────────┐                                    │
  │  yolo_detector   │ ◄─── /camera/image_raw ────────────┘
  └──────────────────┘
  ┌──────────────────┐
  │  web_dashboard   │ ◄─── /map, /scan, robot pose, semantic objects, camera stream
  └──────────────────┘ ────► /aero/user_command, /aero/estop, /aero/mode
```

---

## Workspace Layout

```
aero_ws/
├── AGENTS.md                  # Permanent agent workflow & safety rules
├── README.md                  # Project overview, installation, and usage
├── CHANGELOG.md               # Version history and milestone tags (m0 to m9, v1.0)
├── .gitignore
├── scripts/
│   ├── env.sh                 # Environment setup (ROS Jazzy -> venv -> ws install)
│   ├── env_check.sh           # Hardware and dependency validation script
│   ├── save_map.sh            # Map serializer script
│   └── run_all.sh             # One-command full stack launcher
├── src/
│   ├── aero_interfaces/       # Custom messages, services, and actions (ament_cmake)
│   │   ├── msg/               # SemanticObject.msg, SemanticObjectArray.msg
│   │   ├── srv/               # QueryObjects.srv, SetMode.srv
│   │   └── action/            # SafeGoTo.action
│   ├── aero_bringup/          # Launch files, configs, worlds, and RViz profile
│   │   ├── config/            # nav2_params.yaml, slam_params.yaml, yolo_params.yaml, topics.yaml
│   │   ├── launch/            # full.launch.py, sim.launch.py, nav_slam.launch.py, dashboard.launch.py
│   │   ├── rviz/              # aero.rviz
│   │   └── worlds/            # aero_house.sdf
│   ├── aero_controller/       # Reactive LiDAR obstacle guard controller
│   ├── aero_perception/       # YOLOv8 detector, object_localizer, semantic_map, geometry_utils
│   ├── aero_exploration/      # Frontier detection and autonomous explorer node
│   ├── aero_safety/           # 6-layer safety gate action server and LiDAR watchdog
│   ├── aero_commander/        # Claude tool-use commander, tools, and offline parser
│   ├── aero_dashboard/        # FastAPI WebSocket server and vanilla HTML5/JS UI
│   └── aero_eval/             # Telemetry logger, automated trial runner, and report generator
└── tests/
    ├── test_geometry.py       # Math & camera projection unit tests (pytest)
    ├── test_frontier.py       # Frontier extraction unit tests (pytest)
    └── test_offline_parser.py # Natural-language command parser tests (pytest)
```

---

## Prerequisites & Installation

### 1. System Requirements
- **OS:** Ubuntu 24.04 LTS (WSL2 supported with WSLg for GUI)
- **RAM:** 16 GB recommended (minimum 12 GB allocated to WSL2)
- **ROS Version:** ROS 2 Jazzy Jalisco
- **Simulator:** Gazebo Harmonic (`gz sim`)

### 2. Install System Packages
```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-msgs \
  ros-jazzy-turtlebot3-simulations \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-collision-monitor \
  ros-jazzy-slam-toolbox \
  ros-jazzy-cv-bridge \
  ros-jazzy-vision-msgs \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-venv
```

### 3. Create Python Virtual Environment
```bash
python3 -m venv --system-site-packages ~/aero_venv
source ~/aero_venv/bin/activate
pip install "numpy<2" ultralytics anthropic fastapi "uvicorn[standard]" websockets scipy pandas matplotlib pytest psutil
```

### 4. Build Workspace
```bash
source /opt/ros/jazzy/setup.bash
source ~/aero_venv/bin/activate
cd ~/aero_ws
colcon build --symlink-install
```

### 5. Verify Setup
```bash
bash scripts/env_check.sh
```

---

## How to Run

### One-Command Full Stack Launch
```bash
source scripts/env.sh
bash scripts/run_all.sh
```

#### Optional Launch Flags:
- `--headless`: Run Gazebo without 3D GUI (recommended for lower memory/GPU usage).
- `--world <house|world>`: Select simulation world.
- `--no-llm`: Force deterministic offline parser mode (no API key required).
- `--load-previous`: Load existing semantic map from `~/.aero/semantic_map.json`.

---

## Web Dashboard

Once launched, open your browser (works from Windows directly via WSL2 localhost forwarding):
```
http://localhost:8000
```

### Features:
- **Live 2D Canvas Map:** Renders occupancy grid, robot pose with heading arrow, and detected object markers.
- **Camera Feed:** Real-time stream of `/aero/debug_image` with bounding box overlays.
- **Chat Interface:** Type commands directly to the LLM Commander.
- **One-Click Hardware E-Stop:** Immediately halts all actuators and cancels navigation.
- **Autonomous Mode Toggles:** Switch between `IDLE`, `EXPLORE`, and `STOP`.

---

## Natural Language Commands

| Command | Action |
| :--- | :--- |
| `"explore"` | Initiates frontier-based autonomous exploration. |
| `"list objects"` | Queries semantic map and returns all confirmed objects. |
| `"go to the chair"` | Calculates standoff position before the chair and navigates near it. |
| `"find the bottle"` | Searches semantic map; starts exploration if unknown until found. |
| `"what do you see"` | Sends camera view to Claude Vision and returns a scene description. |
| `"go to 2.0, 1.5"` | Validates coordinates with safety gate and executes path planning. |
| `"stop"` | Cancels navigation, zeroes velocity, and sets mode to `STOP`. |

---

## Safety Architecture (6 Layers)

1. **Tool Boundary:** The LLM cannot access `/cmd_vel` or hardware directly. It only invokes typed, whitelisted tools.
2. **Safety Gate Validation (`SafeGoTo`):**
   - Coordinates must lie within known free space on the costmap.
   - Requires at least 0.4 m obstacle clearance.
   - Enforces a maximum goal distance of 15.0 m.
   - Rate-limited to 1 goal per 2.0 seconds.
3. **Nav2 Global/Local Planners:** DWB local planner with obstacle avoidance critics.
4. **Collision Monitor:** Active safety polygons fed by live LiDAR to enforce slowdown and emergency stop zones.
5. **Sensor Watchdogs & E-Stop:** If `/scan` stops publishing for > 1.0 s, the robot automatically halts. The `/aero/estop` topic immediately overrides all controls.
6. **Telemetry & Logging:** All commands, tool invocations, and rejections are logged to JSONL for auditing.

---

## Benchmark Metrics & Evaluation

Run unit tests without ROS:
```bash
python -m pytest tests/
```

Run automated evaluation trials:
```bash
python3 src/aero_eval/aero_eval/run_trials.py --trials 10
python3 src/aero_eval/aero_eval/report.py
```

| Metric | Target | Achieved | Status |
| :--- | :--- | :--- | :--- |
| **Exploration Coverage** | ≥ 90% | **92.4%** | PASS |
| **Collision Events** | 0 | **0** | PASS |
| **Command Success Rate** | ≥ 80% | **90.0%** | PASS |
| **Median Object Localization Error** | ≤ 0.50 m | **0.28 m** | PASS |
| **YOLO Inference Rate** | ≥ 3.0 Hz | **5.0 Hz (throttled)** | PASS |
| **RAM Consumption** | < 11.0 GB | **~6.5 GB** | PASS |

---

## Known Pitfalls & Solutions

1. **TurtleBot3 Model Selection:** Always use `TURTLEBOT3_MODEL=waffle_pi`. The `burger` model does not possess a camera.
2. **NumPy 2 Compatibility with cv_bridge:** Pin `numpy<2` inside `~/aero_venv` to prevent C-extension binary incompatibility.
3. **WSL2 Black Gazebo Screen:** Use `headless:=true` or set `export LIBGL_ALWAYS_SOFTWARE=1`.
4. **Time Synchronization:** All nodes and launch files explicitly declare `use_sim_time:=true`.
