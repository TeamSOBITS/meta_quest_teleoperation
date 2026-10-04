#!/bin/bash
# targets.sh left|right|both|arm SECONDS -- publish base_footprint -> *_target_link on /tf at 10 Hz
# (SOBIT LIGHT, env VERIFY_ROBOT=SOBIT_LIGHT: its single arm_target_link, any argument)
C=${ROS_CONTAINER:-jazzy_sobit_sciurus_kachaka_ws}
NS=$(echo "${VERIFY_ROBOT:-SOBIT_HOME}" | tr A-Z a-z)
ENVS=$(grep -v '^#' "$(dirname "$0")/../ros_env.sh" | tr '\n' ';' | sed 's/;$//')
L="{header: {frame_id: base_footprint}, child_frame_id: left_target_link, transform: {translation: {x: 0.4, y: 0.2, z: 0.9}, rotation: {w: 1.0}}}"
R="{header: {frame_id: base_footprint}, child_frame_id: right_target_link, transform: {translation: {x: 0.4, y: -0.2, z: 0.9}, rotation: {w: 1.0}}}"
A="{header: {frame_id: base_footprint}, child_frame_id: arm_target_link, transform: {translation: {x: 0.4, y: 0.2, z: 0.9}, rotation: {w: 1.0}}}"
[ "$NS" = sobit_light ] && { L="$A"; R="$A"; set -- left "$2"; }
case "$1" in left) T="$L";; right) T="$R";; both) T="$L, $R";; *) exit 2;; esac
docker exec $C bash -lc "$ENVS; timeout $2 ros2 topic pub -r 10 /tf tf2_msgs/msg/TFMessage \"{transforms: [$T]}\"" > /dev/null 2>&1
echo "targets $1 done"
