#!/bin/bash
set -e

# Source ROS2 underlay
source "/opt/ros/jazzy/setup.bash"

# Source probation_ws overlay if built
if [ -f "$HOME/probation_ws/install/setup.bash" ]; then
  source "$HOME/probation_ws/install/setup.bash"
fi

set -e 

PACKAGE_NAME="task"

source install/setup.bash

echo "[entrypoint] Step 1: switching to GUIDED mode..."
ros2 run "$PACKAGE_NAME" mode_switcher

echo "[entrypoint] Step 2: descending to flare gate depth..."
ros2 run "$PACKAGE_NAME" depth_controller

#DEBUGGERSCRIPT.
echo "[entrypoint] Step 3: starting navigation debugger..."
ros2 run "$PACKAGE_NAME" nav_debugger   # ← NEW: use debugger instead of nav_controller

#echo "[entrypoint] Step 3: starting main navigation..."
#ros2 run "$PACKAGE_NAME" nav_controller

echo "[entrypoint] Mission sequence finished."
exec "$@"
