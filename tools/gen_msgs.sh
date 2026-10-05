#!/bin/bash
# gen_msgs.sh [SOBITS_INTERFACES_DIR]
#   Regenerates UnityProject/Assets/RosMessages/ (C# classes of the sobits_interfaces messages the app uses) from the .msg files.
#   Default dir: env SOBITS_INTERFACES_DIR, else sobits_interfaces in the ROS container's workspace (tools/ros_container.sh --src).
#   The Unity Editor must not have the project open (batch mode cannot share it).
# Env: UNITY (editor binary), PROJECT (default <repo>/UnityProject), SCRATCH (log dir, default ~/.cache/teleop-verify)
set -u
R=$(cd "$(dirname "$0")/.." && pwd)
UNITY=${UNITY:-$HOME/Unity/Hub/Editor/6000.0.69f1/Editor/Unity}
PROJECT=${PROJECT:-$R/UnityProject}
SCRATCH=${SCRATCH:-$HOME/.cache/teleop-verify}
[ $# -gt 0 ] && export SOBITS_INTERFACES_DIR=$1
if [ -z "${SOBITS_INTERFACES_DIR:-}" ]; then
  src=$("$R/tools/ros_container.sh" --src) && export SOBITS_INTERFACES_DIR=$src/sobits_interfaces
fi
mkdir -p "$SCRATCH/logs"
log=$SCRATCH/logs/gen_msgs.log
"$UNITY" -batchmode -quit -nographics -projectPath "$PROJECT" -executeMethod RosMessageGen.Run -logFile "$log"; rc=$?
grep '\[RosMessageGen\]\|\[Verify\]' "$log"
echo "log: $log (rc=$rc)"
exit $rc
