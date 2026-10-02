#!/bin/bash
# verify.sh [--sync] [--suite NAME ...] [--all] [--build] [--shots]
#   Runs the Editor verification harnesses (UnityProject/Assets/Editor/Verify) headlessly on a scratch COPY of the project.
#   --sync        rsync UnityProject/ -> $VERIFY_DIR (excludes Library Temp Logs Builds UserSettings obj .utmp *.csproj *.slnx)
#   --suite NAME  run one suite (repeatable); --all = the default list:
#                 LayoutVerifier AddRobotTest SelectionShot FirstPersonVerify ExperimentsVerify ExperimentsVerify2 ExperimentsVerify3 ExperimentsVerify4
#   --build       BuildApk.Build on the copy (-nographics -buildTarget Android); APK copied to UnityProject/Builds/
#   --shots       SceneShots: baseline pictures + scene_stats.json in <copy-parent>/shots/baseline/
# Env: SCRATCH (work dir, default ~/.cache/teleop-verify), VERIFY_DIR (default $SCRATCH/verify/UnityProject), UNITY (editor binary),
#      TELEOP_TOOLS (default <repo>/tools/sim). Live suites need the sim (tools/sim.sh home) and no other ROS client
#      (adb shell am force-stop com.unity.template.vr). A flock on $SCRATCH/verify.lock serialises runs. Exit code != 0 if anything failed.
set -u
R=$(cd "$(dirname "$0")/.." && pwd)
SCRATCH=${SCRATCH:-$HOME/.cache/teleop-verify}
VERIFY_DIR=${VERIFY_DIR:-$SCRATCH/verify/UnityProject}
UNITY=${UNITY:-$HOME/Unity/Hub/Editor/6000.0.69f1/Editor/Unity}
PARENT=$(dirname "$VERIFY_DIR")
DEFAULT="LayoutVerifier AddRobotTest SelectionShot FirstPersonVerify ExperimentsVerify ExperimentsVerify2 ExperimentsVerify3 ExperimentsVerify4"
sync=0; build=0; shots=0; suites=()
while [ $# -gt 0 ]; do
  case "$1" in
    --sync) sync=1;; --build) build=1;; --shots) shots=1;;
    --all) suites+=($DEFAULT);;
    --suite) suites+=("$2"); shift;;
    -h|--help) sed -n 2,13p "$0"; exit 0;;
    *) echo "unknown option $1" >&2; exit 2;;
  esac; shift
done
if [ $sync = 0 ] && [ $build = 0 ] && [ $shots = 0 ] && [ ${#suites[@]} = 0 ]; then sed -n 2,13p "$0"; exit 2; fi

mkdir -p "$SCRATCH" "$PARENT/logs" "$PARENT/shots"
exec 9>"$SCRATCH/verify.lock"
flock -w 3600 9 || { echo "could not get $SCRATCH/verify.lock" >&2; exit 1; }

if [ $sync = 1 ]; then
  echo "== rsync $R/UnityProject -> $VERIFY_DIR"
  mkdir -p "$VERIFY_DIR"
  rsync -a --delete \
    --exclude=/Library --exclude=/Temp --exclude=/Logs --exclude=/Builds --exclude=/UserSettings --exclude=/obj --exclude=.utmp \
    --exclude='*.csproj' --exclude='*.slnx' --exclude='*_BurstDebugInformation_DoNotShip' --exclude='*.apk' --exclude='mono_crash*' \
    "$R/UnityProject/" "$VERIFY_DIR/" || exit 1
fi
[ -d "$VERIFY_DIR" ] || { echo "no project at $VERIFY_DIR (use --sync)" >&2; exit 1; }
# The copy keeps its own product name: it isolates the Editor PlayerPrefs from the real app's.
sed -i 's/^  productName: .*/  productName: SOBITS Quest Teleoperation VerifyCopy/' "$VERIFY_DIR/ProjectSettings/ProjectSettings.asset"

export VERIFY_SHOTS="$PARENT/shots" EXP_SHOTS="$PARENT/shots/exp" SHOTS_DIR="$PARENT/shots/model"
export TELEOP_TOOLS=${TELEOP_TOOLS:-$R/tools/sim}
mkdir -p "$EXP_SHOTS" "$SHOTS_DIR"
rc_all=0
rows=()

run_unity() {  # run_unity LOGNAME args...
  local name=$1; shift
  timeout 2400 "$UNITY" -batchmode -projectPath "$VERIFY_DIR" "$@" -logFile "$PARENT/logs/$name.log"
}

for s in "${suites[@]}"; do
  echo "== suite $s"
  t0=$SECONDS
  run_unity "$s" -executeMethod "$s.Run"; rc=$?
  log="$PARENT/logs/$s.log"
  pass=$(grep -c '\[Verify\] PASS' "$log"); fail=$(grep -c '\[Verify\] FAIL' "$log")
  [ $rc != 0 ] || [ "$fail" != 0 ] && rc_all=1
  rows+=("$(printf '%-20s %5s pass %4s fail  rc=%-3s %4ss' "$s" "$pass" "$fail" "$rc" "$((SECONDS - t0))")")
  [ "$fail" != 0 ] && grep '\[Verify\] FAIL' "$log" | head -5 | sed 's/^/     /'
done

if [ $shots = 1 ]; then
  echo "== SceneShots"
  run_unity SceneShots -executeMethod SceneShots.Run; rc=$?
  n=$(ls "$PARENT/shots/baseline"/*.png 2>/dev/null | wc -l)
  [ $rc != 0 ] || [ "$n" != 4 ] && rc_all=1
  rows+=("$(printf '%-20s %5s png  rc=%s -> %s' SceneShots "$n" "$rc" "$PARENT/shots/baseline")")
fi

if [ $build = 1 ]; then
  echo "== BuildApk"
  ver=$(grep -m1 '^  bundleVersion:' "$VERIFY_DIR/ProjectSettings/ProjectSettings.asset" | awk '{print $2}')
  apk="$PARENT/build/SOBITS-Quest-Teleoperation-$ver.apk"
  mkdir -p "$PARENT/build" "$R/UnityProject/Builds"; rm -f "$apk"
  BUILD_APK="$apk" run_unity BuildApk -nographics -buildTarget Android -executeMethod BuildApk.Build; rc=$?
  if [ $rc = 0 ] && [ -f "$apk" ]; then
    cp "$apk" "$R/UnityProject/Builds/"
    rows+=("$(printf '%-20s ok  -> %s' BuildApk "$R/UnityProject/Builds/$(basename "$apk")")")
  else rc_all=1; rows+=("$(printf '%-20s FAILED rc=%s (see %s)' BuildApk "$rc" "$PARENT/logs/BuildApk.log")"); fi
  grep -m1 'productName' "$VERIFY_DIR/ProjectSettings/ProjectSettings.asset" | sed 's/^ */copy after build: /'
fi

echo; echo "==== summary"; printf '%s\n' "${rows[@]}"
[ $rc_all = 0 ] && echo "OVERALL: OK" || echo "OVERALL: FAILED"
exit $rc_all
