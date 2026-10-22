#!/usr/bin/env bash
# Run map saver - call after SLAM has explored the world
set -e
source "$(dirname "$0")/env.sh"
mkdir -p ~/aero_ws/src/aero_bringup/maps
ros2 run nav2_map_server map_saver_cli -f ~/aero_ws/src/aero_bringup/maps/house
echo "[save_map] Saved to ~/aero_ws/src/aero_bringup/maps/house.{pgm,yaml}"
