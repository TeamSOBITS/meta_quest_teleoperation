#!/bin/bash
# cmdvel.sh lin|ang|both|stop SECONDS -- cmd_vel burst at 10 Hz
C=${ROS_CONTAINER:-jazzy_sobit_sciurus_kachaka_ws}
NS=$(echo "${VERIFY_ROBOT:-SOBIT_HOME}" | tr A-Z a-z)
ENVS=$(grep -v '^#' "$(dirname "$0")/../ros_env.sh" | tr '\n' ';' | sed 's/;$//')
case "$1" in lin) M="{linear: {x: 0.15}}";; ang) M="{angular: {z: 0.5}}";; both) M="{linear: {x: 0.15}, angular: {z: 0.5}}";; stop) M="{}";; *) exit 2;; esac
[ "$1" = stop ] && docker exec $C pkill -f "cmd_vel geometry_msgs/msg/Twist"
CMDVEL=/$NS/cmd_vel; [ "$NS" = sobit_light ] && CMDVEL=/$NS/manual_control/cmd_vel   # Twist; wheel_controller/cmd_vel is TwistStamped
docker exec $C bash -lc "$ENVS; timeout $2 ros2 topic pub -r 10 $CMDVEL geometry_msgs/msg/Twist \"$M\"" > /dev/null 2>&1
echo "cmdvel $1 done"
