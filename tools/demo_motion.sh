#!/bin/bash
# ~62 s SOBIT HOME choreography for the on-device demo recording (see DemoRecorder.cs).
# Run inside the container after `docker cp`:
#   C=$(tools/ros_container.sh); docker cp tools/demo_motion.sh $C:/tmp/demo_motion.sh
#   docker exec $C bash -lc 'source ~/colcon_ws/install/setup.bash; bash /tmp/demo_motion.sh'
# Start it together with the app (launch extra record=60, switchat=30); first person starts at t=30.

NS=/sobit_home
ARM_JOINTS="shoulder_tilt upper_roll upper_flex elbow lower_flex wrist_tilt wrist_roll"
INIT="-0.75, -1.22, -0.2, 2.5, 0.0, 0.0, 0.0"
RAISED="-1.5, -1.22, -0.2, 1.5, 0.0, 0.0, 0.0"
# controllable hand joints, in controller order
HAND_JOINTS="l_mcp l_dip l_pip c_mcp c_ip r_dip r_pip"
HAND_OPEN="0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0"
HAND_CLOSED="0.0, -0.8, 0.8, -0.5, 0.8, 0.8, -0.8"

# pub <group> <wait> <secs> <joint prefix> <"name ..."> <"p, p, ..."> ; joint = prefix + name + _joint
# Backgrounded so the timeline is not stretched by the ros2 CLI start-up time.
pub() {
  local group=$1 wait=$2 secs=$3 prefix=$4 names=$5 pos=$6 n json=""
  for n in $names; do json+="\"${prefix}${n}_joint\","; done
  json=${json%,}
  ros2 topic pub -w "$wait" --once "$NS/${group}_position_controller/joint_trajectory" \
    trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [$json], points: [{positions: [$pos], time_from_start: {sec: $secs}}]}" >/dev/null 2>&1 &
}
head_to() { pub head 1 "$1" head_ "pan tilt" "$2, $3"; }
lift_to() { pub body 1 "$1" body_ "lift" "$2"; }
arm_to()  { pub "arm_$1" 2 "$2" "arm_$1_" "$ARM_JOINTS" "$3"; }   # servo_target_bridge also subscribes: -w 2
hand_to() { pub "hand_$1" 1 "$2" "hand_$1_finger_" "$HAND_JOINTS" "$3"; }

# at <seconds since start>: wait for the next step of the timeline
at() { while (( SECONDS < $1 )); do sleep 0.1; done; echo "[t=${SECONDS}s] $2"; }

SECONDS=0
at 0  "start: arms to initial_pose, head/lift to 0"
arm_to left 3 "$INIT"; arm_to right 3 "$INIT"; head_to 3 0.0 0.0; lift_to 3 0.0
at 3  "head pan left";   head_to 2 -0.6 0.0
at 5  "head pan right";  head_to 2 0.6 0.0
at 7  "head pan centre"; head_to 2 0.0 0.0
at 12 "head tilt down";  head_to 3 0.0 0.4
at 15 "head tilt up";    head_to 3 0.0 -0.5
at 18 "head tilt level"; head_to 2 0.0 0.0
at 21 "lift up";         lift_to 3 0.55
at 24 "lift down";       lift_to 3 0.3
at 27 "lift to 0.3 hold"
at 30 "first person: slow head pan left";  head_to 4 -0.6 0.0
at 34 "slow head pan right";               head_to 4 0.6 0.0
at 38 "slow head pan centre; left arm raise"; head_to 4 0.0 0.0; arm_to left 3 "$RAISED"
at 42 "left arm back";   arm_to left 3 "$INIT"
at 46 "right arm raise"; arm_to right 3 "$RAISED"
at 50 "right arm back";  arm_to right 3 "$INIT"
at 52 "hands close";     hand_to left 2 "$HAND_CLOSED"; hand_to right 2 "$HAND_CLOSED"
at 55 "hands open";      hand_to left 2 "$HAND_OPEN";   hand_to right 2 "$HAND_OPEN"
at 58 "all back to initial"
arm_to left 3 "$INIT"; arm_to right 3 "$INIT"; head_to 3 0.0 0.0; lift_to 3 0.0
hand_to left 2 "$HAND_OPEN"; hand_to right 2 "$HAND_OPEN"
at 62 "done"
wait
