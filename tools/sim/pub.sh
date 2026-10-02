#!/bin/bash
# (helper for the live Editor suites; they find it via env TELEOP_TOOLS, default <repo>/tools/sim)
# pub.sh head PAN TILT | lift Q | armleft  -- one JointTrajectory to the sim controllers
C=${ROS_CONTAINER:-jazzy_sobit_sciurus_kachaka_ws}
ENVS=$(grep -v '^#' "$(dirname "$0")/../ros_env.sh" | tr '\n' ';' | sed 's/;$//')
case "$1" in
  head) T=head_position_controller; M="{joint_names: [head_pan_joint, head_tilt_joint], points: [{positions: [$2, $3], time_from_start: {sec: 1}}]}";;
  lift) T=body_position_controller; M="{joint_names: [body_lift_joint], points: [{positions: [$2], time_from_start: {sec: 2}}]}";;
  armleft) T=arm_left_position_controller; M="{joint_names: [arm_left_shoulder_tilt_joint, arm_left_upper_roll_joint, arm_left_upper_flex_joint, arm_left_elbow_joint, arm_left_lower_flex_joint, arm_left_wrist_tilt_joint, arm_left_wrist_roll_joint], points: [{positions: [-0.75, -1.22, -0.2, 2.5, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}";;
  armhome) T=arm_left_position_controller; M="{joint_names: [arm_left_shoulder_tilt_joint, arm_left_upper_roll_joint, arm_left_upper_flex_joint, arm_left_elbow_joint, arm_left_lower_flex_joint, arm_left_wrist_tilt_joint, arm_left_wrist_roll_joint], points: [{positions: [-0.55, -1.5707, 0.0, 1.5709, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}";;
  *) echo "usage"; exit 2;;
esac
W=1; case "$T" in arm_*) W=2;; esac   # servo_target_bridge also subscribes to the arm topics
docker exec $C bash -lc "$ENVS; ros2 topic pub -w $W --once /sobit_home/$T/joint_trajectory trajectory_msgs/msg/JointTrajectory \"$M\"" > /dev/null 2>&1
echo "pub $1 rc=$?"
