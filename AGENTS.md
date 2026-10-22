# AERO agent rules

## Environment (fixed, never change)
- Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, Gazebo Harmonic via `gz sim` and `ros_gz`.
- Never use Humble/Foxy/Iron commands, Gazebo Classic (`gazebo`), or `ros1_bridge`.
- Robot: TurtleBot3 `waffle_pi` (the `burger` has NO camera). Verify with `echo $TURTLEBOT3_MODEL`.
- Machine: WSL2, 16 GB RAM. Be frugal: yolov8n, 5 Hz inference, optional headless Gazebo.

## Workflow rules
1. Work on ONE milestone at a time. Do not start the next until its acceptance test passes
   and the user confirms.
2. Use git. Branch per milestone (`m3-perception`). After the acceptance test passes, merge
   to main and tag (`m3-pass`). Never break a tagged milestone.
3. VERIFY BEFORE ASSUMING. Before coding against a topic, message type, package, or launch
   file, check it: `ros2 topic list`, `ros2 topic info -v <topic>`, `ros2 interface show`,
   `ros2 pkg list`, `apt-cache policy`. Topic names and types differ between releases.
   Example: check whether /cmd_vel is Twist or TwistStamped and adapt.
4. Every node in simulation uses `use_sim_time:=true`.
5. One responsibility per node. Parameters declared and loaded from YAML in
   `aero_bringup/config`. No hard-coded absolute paths. No secrets in code or git.
6. Final `/cmd_vel` must have exactly ONE publisher (the collision monitor or mux).
   Teleop, obstacle_guard and Nav2 must never publish to it at the same time.
7. Pure logic (geometry, frontier detection, command parsing) goes in plain Python modules
   with pytest tests. ROS node files stay thin.
8. Each node: logs at INFO for key events, handles Ctrl+C, and publishes a zero velocity
   command on shutdown if it controls motion.
9. The LLM NEVER publishes velocity. It only calls validated tools (see spec section 8).
10. After each milestone: update README.md, CHANGELOG.md, and package READMEs.
11. If something fails 3 times, STOP. Report the exact error, your hypothesis, and what you
    tried. Ask the user instead of looping.
12. Do not install global pip packages. Use the venv `~/aero_venv` (created with
    `--system-site-packages`). Build with the venv ACTIVE so entry-point shebangs use it.

## Definition of done for any milestone
- Code builds with `colcon build --symlink-install` with no warnings you introduced.
- `colcon test` / pytest pass for the pure-logic modules.
- The acceptance test from the spec is executed and its result is pasted in the reply.
- README and CHANGELOG updated, git tag created.
