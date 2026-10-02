#!/bin/bash
# device.sh launch [--robot SOBIT_HOME|SOBIT_LIGHT] [--viewmode blocks|model|firstperson] [--capture] [--record N --fps N --switchat N]
#          capture DEST.png   pull the debug capture (persistentDataPath/fpv_capture.png)
#          logs [-f]          logcat filtered: FPV: RTT: Targets: Exception Fatal lowmemorykiller(unity)
#          awake | release    keep the headset awake (prox_close) / hand back to the proximity sensor (automation_disable)
#          reverse            adb kill-server/start-server + reverse tcp:10000
#          stop               force-stop the app
# `launch` force-stops the app first so the intent extras apply; without options it is a plain launch.
PKG=com.unity.template.vr
ACT=$PKG/com.unity3d.player.UnityPlayerGameActivity
FILES=/sdcard/Android/data/$PKG/files
cmd=$1; shift
case "$cmd" in
  launch)
    extra=()
    while [ $# -gt 0 ]; do
      case "$1" in
        --robot) extra+=(--es robot "$2"); shift 2;;
        --viewmode) extra+=(--es viewmode "$2"); shift 2;;
        --capture) extra+=(--es capture 1); shift;;
        --record) extra+=(--ei record "$2"); shift 2;;
        --fps) extra+=(--ei fps "$2"); shift 2;;
        --switchat) extra+=(--ei switchat "$2"); shift 2;;
        *) echo "unknown option $1" >&2; exit 2;;
      esac
    done
    adb shell am force-stop $PKG
    adb logcat -c
    adb shell am start -n $ACT "${extra[@]}";;
  capture) [ -n "$1" ] || { echo "usage: device.sh capture DEST.png" >&2; exit 2; }; adb pull $FILES/fpv_capture.png "$1";;
  logs)
    f=-d; [ "$1" = -f ] && f=
    adb logcat $f -v time | grep -E 'FPV:|RTT:|Targets:|Exception|Fatal|lowmemorykiller.*unity';;
  awake) adb shell am broadcast -a com.oculus.vrpowermanager.prox_close;;
  release) adb shell am broadcast -a com.oculus.vrpowermanager.automation_disable;;
  reverse) adb kill-server; adb start-server; adb reverse tcp:10000 tcp:10000;;
  stop) adb shell am force-stop $PKG;;
  *) sed -n 2,9p "$0"; exit 2;;
esac
