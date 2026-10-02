#!/bin/bash
# (helper for the live Editor suites; they find it via env TELEOP_TOOLS, default <repo>/tools/sim)
# pub.sh head PAN TILT | lift Q | arm | armhome | hand open|close  -- one JointTrajectory to the sim controllers
# Robot: env VERIFY_ROBOT (SOBIT_HOME default, SOBIT_LIGHT). `arm` = a pose away from `armhome` (armleft = alias), no lift on SOBIT LIGHT.
C=${ROS_CONTAINER:-jazzy_sobit_sciurus_kachaka_ws}
ENVS=$(grep -v '^#' "$(dirname "$0")/../ros_env.sh" | tr '\n' ';' | sed 's/;$//')
NS=$(echo "${VERIFY_ROBOT:-SOBIT_HOME}" | tr 'A-Z' 'a-z')
traj() { echo "{joint_names: [$1], points: [{positions: [$2], time_from_start: {sec: ${3:-2}}}]}"; }
if [ "$NS" = sobit_light ]; then
  LARM="arm_shoulder_roll_joint, arm_shoulder_pitch_joint, arm_elbow_pitch_joint, arm_forearm_roll_joint, arm_wrist_pitch_joint, arm_wrist_roll_joint"
  case "$1" in
    head) T=head_position_controller; M=$(traj "head_yaw_joint, head_pitch_joint" "$2, $3" 1);;
    arm|armleft) T=arm_position_controller; M=$(traj "$LARM" "0.0, -0.5, -0.2, 0.0, 0.7, 0.0");;      # floor_ready_pose
    armhome) T=arm_position_controller; M=$(traj "$LARM" "0.0, -1.1, -0.6, 0.0, 0.8, 0.0");;          # servo_ready_pose
    hand) T=hand_position_controller; case "$2" in open) P=0.029;; close) P=-0.013;; *) echo "usage"; exit 2;; esac; M=$(traj "hand_joint" "$P");;
    *) echo "usage"; exit 2;;
  esac
else
  LARM="arm_left_shoulder_tilt_joint, arm_left_upper_roll_joint, arm_left_upper_flex_joint, arm_left_elbow_joint, arm_left_lower_flex_joint, arm_left_wrist_tilt_joint, arm_left_wrist_roll_joint"
  case "$1" in
    head) T=head_position_controller; M=$(traj "head_pan_joint, head_tilt_joint" "$2, $3" 1);;
    lift) T=body_position_controller; M=$(traj "body_lift_joint" "$2");;
    arm|armleft) T=arm_left_position_controller; M=$(traj "$LARM" "-0.75, -1.22, -0.2, 2.5, 0.0, 0.0, 0.0");;
    armhome) T=arm_left_position_controller; M=$(traj "$LARM" "-0.55, -1.5707, 0.0, 1.5709, 0.0, 0.0, 0.0");;
    *) echo "usage"; exit 2;;
  esac
fi
W=1; case "$T" in arm_*) W=2;; esac   # servo_target_bridge also subscribes to the arm topics
docker exec $C bash -lc "$ENVS; ros2 topic pub -w $W --once /$NS/$T/joint_trajectory trajectory_msgs/msg/JointTrajectory \"$M\"" > /dev/null 2>&1
echo "pub $1 rc=$?"
