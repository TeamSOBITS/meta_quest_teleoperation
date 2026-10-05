#!/bin/bash
# ros_container.sh [--src]: print the ROS container the tools drive (or, with --src, its workspace src dir on the host).
#   It is the container that bind-mounts a ~/colcon_ws/src containing sobits_teleop; running ones win over stopped ones.
#   Env ROS_CONTAINER overrides the search. Exits 1 (message on stderr) when none or several match.
mount_src() {  # host dir mounted on the container's ~/colcon_ws/src
  docker inspect "$1" --format '{{range .Mounts}}{{.Source}} {{.Destination}}{{"\n"}}{{end}}' 2>/dev/null \
    | awk '$2 ~ /\/colcon_ws\/src$/ {print $1; exit}'
}

pick() {
  if [ -n "${ROS_CONTAINER:-}" ]; then echo "$ROS_CONTAINER"; return; fi
  local state c found
  for state in running exited; do
    found=()
    for c in $(docker ps -a --filter "status=$state" --format '{{.Names}}'); do
      local s; s=$(mount_src "$c")
      [ -n "$s" ] && [ -d "$s/sobits_teleop" ] && found+=("$c")
    done
    if [ ${#found[@]} -eq 1 ]; then echo "${found[0]}"; return; fi
    if [ ${#found[@]} -gt 1 ]; then
      echo "ros_container: several $state containers have sobits_teleop (${found[*]}); set ROS_CONTAINER" >&2; return 1
    fi
  done
  echo "ros_container: no container mounts a ~/colcon_ws/src with sobits_teleop; set ROS_CONTAINER" >&2; return 1
}

c=$(pick) || exit 1
if [ "${1:-}" = --src ]; then
  s=$(mount_src "$c"); [ -n "$s" ] || { echo "ros_container: $c has no ~/colcon_ws/src mount" >&2; exit 1; }
  echo "$s"
else
  echo "$c"
fi
