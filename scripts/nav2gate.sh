#!/bin/bash

set -e


source ~/probation_ws/install/setup.bash

PACKAGE_NAME="task"

echo "[entrypoint] Step 1: switching to GUIDED mode..."
/opt/ros/jazzy/bin/ros2 run "$PACKAGE_NAME" mode_switcher

echo "[entrypoint] Step 2: descending to flare gate depth..."
/opt/ros/jazzy/bin/ros2 run "$PACKAGE_NAME" depth_controller

#only un=comment if u want to debug...
#echo "[entrypoint] Step 3: starting main navigation..."
#/opt/ros/jazzy/bin/ros2 run "$PACKAGE_NAME" nav_debugger

echo "[entrypoint] Step 3: starting main navigation..."
/opt/ros/jazzy/bin/ros2 run "$PACKAGE_NAME" nav_controller


echo "[entrypoint] Mission sequence finished."
exec "$@"