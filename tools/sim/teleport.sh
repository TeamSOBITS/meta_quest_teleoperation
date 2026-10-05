#!/bin/bash
# teleport.sh [YAW] -- (SOBIT HOME; no-op for SOBIT LIGHT) put the sim robot back at its spawn point (-6.0, 1.5) facing YAW (world, default -0.85 = the room)
C=$("$(dirname "$0")/../ros_container.sh") || exit 1
NS=$(echo "${VERIFY_ROBOT:-SOBIT_HOME}" | tr A-Z a-z)
ENVS=$(grep -v '^#' "$(dirname "$0")/../ros_env.sh" | tr '\n' ';' | sed 's/;$//')
Y=${1:--0.85}
[ "$NS" = sobit_light ] && { echo "teleport skipped: SOBIT LIGHT spawn point not mapped"; exit 0; }
docker exec $C bash -lc "$ENVS; python3 - <<PY
import math
y=$Y
print('{entity: {name: $NS, type: 2}, pose: {position: {x: -6.0, y: 1.5, z: 0.0}, orientation: {z: %f, w: %f}}}'%(math.sin(y/2),math.cos(y/2)))
PY" > /tmp/tp_req.txt
REQ=$(cat /tmp/tp_req.txt)
docker exec $C bash -lc "$ENVS; ros2 service call /world/rcjo2026_arena/set_pose ros_gz_interfaces/srv/SetEntityPose \"$REQ\"" | tail -1
