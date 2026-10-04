#!/bin/bash
# yaw.sh [YAW] -- turn the sim base to world yaw YAW (default -0.85) with odom feedback (runs yaw_to.py in the container)
C=${ROS_CONTAINER:-jazzy_sobit_sciurus_kachaka_ws}
ENVS=$(grep -v '^#' "$(dirname "$0")/../ros_env.sh" | tr '\n' ';' | sed 's/;$//')
docker exec -i $C bash -lc "$ENVS python3 - ${1:--0.85}" < "$(dirname "$0")/yaw_to.py"
