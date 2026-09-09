#!/bin/bash

set -e


source ~/probation_ws/install/setup.bash

PACKAGE_NAME="probation_bringup"



# Give everyhign a few seconds to initialize its services
sleep 5


echo "[entrypoint] Step 1: switching to GUIDED mode..."
ros2 run "$PACKAGE_NAME" set_guide


echo "[entrypoint] Step 2: starting main navigation..."
ros2 run "$PACKAGE_NAME" solution_template


echo "[entrypoint] Mission sequence finished."
exec "$@"