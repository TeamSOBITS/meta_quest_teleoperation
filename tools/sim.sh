#!/bin/bash
# sim.sh status [--robot home|light] | home | light | stop   -- manage the Gazebo sim + sobits_teleop in the ROS container.
#   status  topic rates of the active robot (detected from the running launch arguments, or --robot / env ROBOT)
#           (/clock, joint_states, head image) + endpoint port
#   home    docker restart, launch sobit_home_bringup sim_minimal (Gazebo), wait for /clock + head image (180 s),
#           launch sobits_teleop (robot_name:=sobit_home device:=quest ...), wait for port 10000, adb reverse
#   light   same for SOBIT LIGHT (sim_minimal with enable_tf_prefix:=false headless:=true)
#   stop    docker restart only (everything gone, nothing relaunched)
# Never uses `docker compose`. Env: ROS_CONTAINER (default: found by tools/ros_container.sh), ROBOT (status override).
D=$(cd "$(dirname "$0")" && pwd)
C=$("$D/ros_container.sh") || exit 1
ENVS=$(grep -v '^#' "$D/ros_env.sh" | tr '\n' ';' | sed 's/;$//')

rexec() { docker exec "$C" bash -lc "$ENVS; $*"; }
rdetach() { docker exec -d "$C" bash -lc "$ENVS; $*"; }

# topic_alive TOPIC [timeout]: one message received?
topic_alive() { rexec "timeout ${2:-8} ros2 topic echo --once --no-arr --qos-profile sensor_data $1" >/dev/null 2>&1; }

wait_topics() {  # wait_topics ROBOT -> /clock and head image publish (180 s)
  local r=$1 end=$((SECONDS + 180))
  while [ $SECONDS -lt $end ]; do
    if topic_alive /clock 5 && topic_alive "/$r/head_camera/color/image_raw/compressed" 6; then echo "sim up: /clock + /$r head image"; return 0; fi
    sleep 3
  done
  echo "TIMEOUT: /clock or /$r/head_camera/color/image_raw/compressed not publishing after 180 s" >&2; return 1
}

wait_port() {
  local end=$((SECONDS + 90))
  while [ $SECONDS -lt $end ]; do ss -ltn 2>/dev/null | grep -q ':10000 ' && { echo "port 10000 listening"; return 0; }; sleep 2; done
  echo "TIMEOUT: nothing listens on :10000" >&2; return 1
}

adb_reverse() { adb kill-server; adb start-server; adb reverse tcp:10000 tcp:10000 || echo "adb reverse failed (headset not attached/authorized?)" >&2; }

start() {  # start ROBOT GZ_PACKAGE EXTRA_GZ_ARGS...
  local robot=$1 pkg=$2; shift 2
  docker restart "$C" >/dev/null || exit 1
  sleep 5
  rdetach "exec ros2 launch $pkg sim_minimal.launch.py $* > /tmp/sim_gz.log 2>&1"
  wait_topics "$robot" || { echo "see: docker exec $C tail -40 /tmp/sim_gz.log" >&2; exit 1; }
  rdetach "exec ros2 launch sobits_teleop sobits_teleop.launch.py robot_name:=$robot device:=quest use_sim_time:=true use_moveit:=true use_servo:=true > /tmp/sim_teleop.log 2>&1"
  wait_port || { echo "see: docker exec $C tail -40 /tmp/sim_teleop.log" >&2; exit 1; }
  adb_reverse
  echo "$robot ready"
}

# active_robot: sobit_home | sobit_light from the running launch arguments (empty when no sim is launched)
active_robot() {
  local pl; pl=$(rexec "pgrep -af 'ros2 launch'" 2>/dev/null)
  case "$pl" in
    *sobit_light_bringup*|*robot_name:=sobit_light*) echo sobit_light;;
    *sobit_home_bringup*|*robot_name:=sobit_home*) echo sobit_home;;
  esac
}

status() {
  local r=${ROBOT:-}
  [ -z "$r" ] && r=$(active_robot)
  if [ -z "$r" ]; then echo "robot: none (no ros2 launch running in $C)"; r=sobit_home; fi
  echo "robot: $r"
  for t in /clock /$r/joint_states /$r/head_camera/color/image_raw/compressed; do
    printf '%-52s ' "$t"
    rexec "timeout 6 ros2 topic hz $t 2>&1 | grep 'average rate' | tail -1" 2>/dev/null | sed 's/^ *//' | grep . || echo "NO DATA"
  done
  printf '%-52s ' "endpoint :10000"; ss -ltn 2>/dev/null | grep -q ':10000 ' && echo listening || echo "NOT listening"
}

case "$1" in
  status) shift; [ "$1" = --robot ] && ROBOT=${2#sobit_} && ROBOT=sobit_$ROBOT; status;;
  home)   start sobit_home sobit_home_bringup;;
  light)  # sobit_light is not built in every workspace
          rexec "ros2 pkg prefix sobit_light_bringup" >/dev/null 2>&1 || { echo "sobit_light_bringup is not built in $C (set ROS_CONTAINER)" >&2; exit 1; }
          start sobit_light sobit_light_bringup enable_tf_prefix:=false headless:=true;;
  stop)   docker restart "$C" >/dev/null && echo "container restarted, nothing running";;
  *) sed -n 2,11p "$0"; exit 2;;
esac
