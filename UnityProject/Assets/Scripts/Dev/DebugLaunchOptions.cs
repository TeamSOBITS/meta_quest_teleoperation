using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Autonomous tests start the app with intent extras, e.g.
///   am start -n &lt;pkg&gt;/&lt;activity&gt; --es dev 1 --es robot SOBIT_HOME --es viewmode firstperson --es capture 1
/// dev = 1 turns <see cref="DevTools"/> on for this run (without it a release build ignores the other extras);
/// robot = profile asset name (opens it); viewmode = firstperson (model + first-person layout) |
/// model (model + blocks layout) | blocks (no model), this robot screen only, not saved;
/// capture = 1 (save a screenshot of the robot screen, see <see cref="DebugCapture"/>);
/// record = seconds, fps, switchat = seconds (see <see cref="DemoRecorder"/>);
/// recstatus = 1 | deploy (fake VLA status feed of the recorder / the deploy node, see <see cref="VlaStatusDemo"/>;
/// not `record`, that is screen recording).
/// Read once per app run, so "Back to robots" does not open the robot again. Android only.
/// </summary>
public static class DebugLaunchOptions
{
    static bool _handled;

    /// <summary>Set by the `recstatus` extra (VlaStatusDemo.ModeCollection "1" / ModeDeploy "deploy", null = none): the
    /// next robot screen plays that fake VLA status feed once.</summary>
    public static string VlaStatusDemoMode;

    /// <summary>Applies the launch extras (session only, nothing is saved except the cleared capture flag)
    /// and returns the robot the intent asks to open, or null.</summary>
    public static RobotProfile Apply(IReadOnlyList<RobotProfile> robots)
    {
#if UNITY_ANDROID && !UNITY_EDITOR
        if (_handled) return null;
        _handled = true;
        string robot = null, viewMode = null, capture = null, dev = null, recStatus = null;
        int record = 0, fps = 15, switchAt = -1;
        try
        {
            using (var player = new AndroidJavaClass("com.unity3d.player.UnityPlayer"))
            using (var activity = player.GetStatic<AndroidJavaObject>("currentActivity"))
            using (var intent = activity?.Call<AndroidJavaObject>("getIntent"))
            {
                if (intent != null)
                {
                    dev = intent.Call<string>("getStringExtra", "dev");
                    robot = intent.Call<string>("getStringExtra", "robot");
                    viewMode = intent.Call<string>("getStringExtra", "viewmode");
                    capture = intent.Call<string>("getStringExtra", "capture");
                    recStatus = intent.Call<string>("getStringExtra", "recstatus");
                    record = intent.Call<int>("getIntExtra", "record", 0);          // --ei record 60
                    fps = intent.Call<int>("getIntExtra", "fps", 15);
                    switchAt = intent.Call<int>("getIntExtra", "switchat", -1);
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"FPV: could not read intent extras: {e.Message}");
            return null;
        }
        if (dev == "1") DevTools.Session = true;
        DevLog.Log("FPV", $"intent robot={robot} viewmode={viewMode} capture={capture} record={record} fps={fps} switchat={switchAt} dev={DevTools.Enabled}");

        VlaStatusDemoMode = DevTools.Enabled && (recStatus == VlaStatusDemo.ModeCollection || recStatus == VlaStatusDemo.ModeDeploy) ? recStatus : null;

        // DebugCapture must not linger: set only by this launch, cleared when no extra is present.
        Settings.DebugCapture = DevTools.Enabled && capture == "1";
        if (!DevTools.Enabled || string.IsNullOrEmpty(robot)) { Settings.Save(); return null; }

        var profile = FindRobot(robots, robot);
        if (profile == null)
        {
            Debug.LogWarning($"FPV: intent robot '{robot}' not found");
            Settings.Save();
            return null;
        }
        if (viewMode == FirstPersonView.LayoutFirstPerson || viewMode == FirstPersonView.LayoutBlocks || viewMode == "model")
            FirstPersonView.ViewModeOverride = viewMode;
        Settings.Save();
        if (record > 0)   // session only, never saved
            DemoRecorder.Request = new DemoRecorder.Settings
            {
                seconds = record, fps = Mathf.Clamp(fps, 1, 60), switchAt = switchAt >= 0 ? switchAt : record / 2f,
            };
        return profile;
#else
        return null;
#endif
    }

    static RobotProfile FindRobot(IReadOnlyList<RobotProfile> robots, string name)
    {
        foreach (var r in robots)
            if (r.name == name) return r;
        return null;
    }
}
