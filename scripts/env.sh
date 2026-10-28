#!/usr/bin/env bash
# AERO environment setup - source this before every terminal session
# Order matters: ROS first, then venv, then workspace install

source /opt/ros/jazzy/setup.bash
source ~/aero_venv/bin/activate
[ -f ~/aero_ws/install/setup.bash ] && source ~/aero_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="/home/nikhil/aero_venv/lib/python3.12/site-packages:${PYTHONPATH:-}"

echo "[env.sh] ROS_DISTRO=$ROS_DISTRO  TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL  VIRTUAL_ENV=$VIRTUAL_ENV"
