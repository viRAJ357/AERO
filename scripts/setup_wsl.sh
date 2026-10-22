#!/usr/bin/env bash
set -e

echo "=== 1. Adding ROS 2 Jazzy Repository ==="
apt-get update -y
apt-get install -y software-properties-common curl gnupg lsb-release

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
ARCH=$(dpkg --print-architecture)
CODENAME=$(lsb_release -cs)
echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" > /etc/apt/sources.list.d/ros2.list

echo "=== 2. Updating package lists ==="
apt-get update -y

echo "=== 3. Installing ROS 2 and Gazebo stack ==="
apt-get install -y \
  ros-jazzy-ros-base \
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
  python3-venv \
  python3-pip

echo "=== 4. Setting up Python venv for user nikhil ==="
su - nikhil -c "
python3 -m venv --system-site-packages ~/aero_venv
source ~/aero_venv/bin/activate
pip install 'numpy<2' ultralytics anthropic fastapi 'uvicorn[standard]' websockets scipy pandas matplotlib pytest psutil
"

echo "=== 5. Linking project workspace ==="
su - nikhil -c "
mkdir -p ~/aero_ws
ln -sfn /mnt/c/Users/nikhi/OneDrive/Desktop/Roboticsadvanceproject/src ~/aero_ws/src
ln -sfn /mnt/c/Users/nikhi/OneDrive/Desktop/Roboticsadvanceproject/scripts ~/aero_ws/scripts
ln -sfn /mnt/c/Users/nikhi/OneDrive/Desktop/Roboticsadvanceproject/tests ~/aero_ws/tests
ln -sfn /mnt/c/Users/nikhi/OneDrive/Desktop/Roboticsadvanceproject/AGENTS.md ~/aero_ws/AGENTS.md
ln -sfn /mnt/c/Users/nikhi/OneDrive/Desktop/Roboticsadvanceproject/README.md ~/aero_ws/README.md
ln -sfn /mnt/c/Users/nikhi/OneDrive/Desktop/Roboticsadvanceproject/CHANGELOG.md ~/aero_ws/CHANGELOG.md
"

echo "=== 6. Initializing rosdep ==="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init || true
fi
su - nikhil -c "rosdep update || true"

echo "=== SETUP COMPLETE ==="
