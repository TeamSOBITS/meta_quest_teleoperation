#!/bin/bash
# home.sh -- stop the sim base and put it back at its spawn point facing the room (wheel odom drifts
# after a collision, so this teleports through the gz set_pose service instead of driving back)
C=${ROS_CONTAINER:-jazzy_sobit_sciurus_kachaka_ws}
NS=$(echo "${VERIFY_ROBOT:-SOBIT_HOME}" | tr A-Z a-z)
ENVS=$(grep -v '^#' "$(dirname "$0")/../ros_env.sh" | tr '\n' ';' | sed 's/;$//')
CMDVEL=/$NS/cmd_vel; [ "$NS" = sobit_light ] && CMDVEL=/$NS/manual_control/cmd_vel
docker exec $C bash -lc "$ENVS; ros2 topic pub -w 1 --once $CMDVEL geometry_msgs/msg/Twist '{}'" > /dev/null 2>&1
"$(dirname "$0")/teleport.sh" -0.85
