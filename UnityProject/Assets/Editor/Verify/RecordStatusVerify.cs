// Verify harness: recorder status feed (RecordStatusRule + RecordStatus data layer). No sim needed (the ROS IP is isolated).
// Run: tools/verify.sh --suite RecordStatusVerify
using System;
using System.Collections;
using System.Reflection;
using RosMessageTypes.SobitsInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Object = UnityEngine.Object;

public static class RecordStatusVerify
{
    static IEnumerator _run; static double _until; static int _fail, _pass;
    static RecordStatus _rs;
    static uint _seq;

    public static void Run()
    {
        EditorSettings.enterPlayModeOptionsEnabled = true;
        EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.DisableDomainReload;
        _run = Main(); EditorApplication.update += Tick;
    }
    static void Tick()
    {
        if (EditorApplication.timeSinceStartup < _until) return;
        try
        {
            if (!_run.MoveNext())
            {
                EditorApplication.update -= Tick;
                Debug.Log(_fail == 0 ? $"[Verify] ALL RECORD STATUS CHECKS PASSED ({_pass})" : $"[Verify] {_fail} RECORD STATUS CHECK(S) FAILED");
                EditorApplication.Exit(_fail == 0 ? 0 : 1);
            }
        }
        catch (Exception e) { Debug.LogException(e); EditorApplication.Exit(3); }
    }
    static object Wait(double s) { _until = EditorApplication.timeSinceStartup + s; return null; }
    static void Check(bool ok, string what) { if (ok) _pass++; else _fail++; Debug.Log("[Verify] " + (ok ? "PASS " : "FAIL ") + what); }

    static void Inject(byte state, byte evt, float elapsed, string detail = "", bool taskSet = true, string task = "pick cup")
    {
        if (evt != 0) _seq++;
        var m = new VlaRecordStatusMsg
        {
            state = state, @event = evt, event_seq = _seq, task_set = taskSet, task_name = task, episode_name = "episode_test",
            elapsed_sec = elapsed, detail = detail, message = "test",
        };
        typeof(RecordStatus).GetMethod("OnMessage", BindingFlags.Instance | BindingFlags.NonPublic).Invoke(_rs, new object[] { m });
    }

    static void StaticChecks()
    {
        Check(RecordStatusRule.StaleS == 3f && RecordStatusRule.ToastS == 4f && RecordStatusRule.PulseHz == 1f, "constants StaleS 3, ToastS 4, PulseHz 1");
        var L = RecordStatusRule.Look.Hidden;
        Check(RecordStatusRule.Evaluate(false, 0f, 1) == RecordStatusRule.Look.Hidden, "Evaluate: no message -> Hidden");
        Check(RecordStatusRule.Evaluate(true, 3f, 1) == RecordStatusRule.Look.Hidden && RecordStatusRule.Evaluate(true, 3.1f, 1) == RecordStatusRule.Look.Hidden, "Evaluate: age >= 3 -> Hidden (stale)");
        Check(RecordStatusRule.Evaluate(true, -1f, 1) == RecordStatusRule.Look.Hidden, "Evaluate: negative age -> Hidden");
        Check(RecordStatusRule.Evaluate(true, 0.5f, 0) == RecordStatusRule.Look.Idle, "Evaluate: STOPPED -> Idle");
        Check(RecordStatusRule.Evaluate(true, 0.5f, 1) == RecordStatusRule.Look.Recording, "Evaluate: RECORDING -> Recording");
        Check(RecordStatusRule.Evaluate(true, 0.5f, 2) == RecordStatusRule.Look.Paused, "Evaluate: PAUSED -> Paused");
        Check(RecordStatusRule.Evaluate(true, 0.5f, 4) == RecordStatusRule.Look.Error, "Evaluate: ERROR -> Error");
        Check(RecordStatusRule.Evaluate(true, 2.9f, 1) == RecordStatusRule.Look.Recording, "Evaluate: age 2.9 still alive");
        Check(RecordStatusRule.IsAlive(2.9f) && !RecordStatusRule.IsAlive(3.1f) && !RecordStatusRule.IsAlive(-1f), "IsAlive: 2.9 true, 3.1 false, -1 false");
        Check(RecordStatusRule.FormatElapsed(3661f) == "01:01:01" && RecordStatusRule.FormatElapsed(-5f) == "00:00:00" && RecordStatusRule.FormatElapsed(65.9f) == "00:01:05", "FormatElapsed 3661 -> 01:01:01, negative -> 00:00:00, 65.9 -> 00:01:05");
        Check(RecordStatusRule.Label(RecordStatusRule.Look.Recording, 65f) == "REC 00:01:05", "Label Recording");
        Check(RecordStatusRule.Label(RecordStatusRule.Look.Paused, 65f) == "PAUSED 00:01:05", "Label Paused");
        Check(RecordStatusRule.Label(RecordStatusRule.Look.Idle, 0f) == "IDLE" && RecordStatusRule.Label(RecordStatusRule.Look.Error, 0f) == "ERROR" && RecordStatusRule.Label(L, 0f) == "", "Label Idle / Error / Hidden");
        Check(RecordStatusRule.Colour(RecordStatusRule.Look.Recording) == HudTheme.Record && RecordStatusRule.Colour(RecordStatusRule.Look.Paused) == HudTheme.Warn
              && RecordStatusRule.Colour(RecordStatusRule.Look.Error) == HudTheme.Bad && RecordStatusRule.Colour(L).a == 0f, "Colour: Record / Warn / Bad / clear");
        Check(RecordStatusRule.Toast(4, "", 65f) == "Saved · 00:01:05", "Toast SAVED");
        Check(RecordStatusRule.Toast(5, "too_short", 0f) == "Discarded: too short" && RecordStatusRule.Toast(5, "integrity_failed", 0f) == "Discarded: integrity failed"
              && RecordStatusRule.Toast(5, "other", 0f) == "Discarded: other", "Toast DISCARDED (too_short / integrity_failed / other)");
        Check(RecordStatusRule.Toast(6, "", 0f) == "Deleted", "Toast DELETED");
        Check(RecordStatusRule.Toast(7, "disk full", 0f) == "Error: disk full" && RecordStatusRule.Toast(7, "", 0f) == "Error", "Toast ERROR (detail / none)");
        Check(RecordStatusRule.Toast(8, "pick cup", 0f) == "Task: pick cup", "Toast TASK_SET");
        Check(RecordStatusRule.Toast(9, "no task set", 0f) == "no task set" && RecordStatusRule.Toast(9, "", 0f) == "Rejected", "Toast REJECTED (detail / none)");
        bool none = true;
        foreach (byte e in new byte[] { 0, 1, 2, 3 }) none &= RecordStatusRule.Toast(e, "x", 1f) == null;
        Check(none, "Toast NONE / STARTED / PAUSED / RESUMED -> null");
        Check(Mathf.Abs(RecordStatusRule.PulseAlpha(0f) - 0.775f) < 1e-3f && RecordStatusRule.PulseAlpha(0.25f) > 0.99f && RecordStatusRule.PulseAlpha(0.75f) < 0.56f, "PulseAlpha swings 0.55 .. 1 at 1 Hz");
    }

    static IEnumerator Main()
    {
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.2");   // isolated: never reach a live ros_tcp_endpoint
        PlayerPrefs.Save();
        HudTheme.Reload();

        StaticChecks();

        var profile = AssetDatabase.LoadAssetAtPath<RobotProfile>("Assets/Robots/SOBIT_HOME.asset");
        Check(profile != null && profile.recordStatusSuffix == RosNames.RecordStatus, $"SOBIT_HOME recordStatusSuffix '{profile?.recordStatusSuffix}'");
        EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
        RobotProfile.Selected = profile;
        Object.FindFirstObjectByType<QuestControllerPublisher>().controlRobot = false;
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Wait(0.1);
        Object.FindFirstObjectByType<QuestControllerPublisher>().controlRobot = false;
        yield return Wait(3.0);

        // (a) nothing received yet
        _rs = RecordStatus.Latest;
        Check(_rs != null && _rs.name == "Record Status", "RecordStatus.Latest created by the HUD");
        if (_rs == null) { EditorApplication.ExitPlaymode(); yield break; }
        Check(!_rs.HasMessage && _rs.Look == RecordStatusRule.Look.Hidden && !_rs.Available && _rs.AgeS < 0f && !_rs.ToastShown, "no message: HasMessage false, Look Hidden, AgeS -1, no toast");
        int changed = 0; _rs.Changed += () => changed++;

        // (b) recording
        Inject(1, 1, 5f);
        Check(changed == 1 && _rs.HasMessage && _rs.Available && _rs.Look == RecordStatusRule.Look.Recording, "RECORDING/STARTED: Changed raised, Look Recording");
        Check(RecordStatusRule.Label(_rs.Look, _rs.ElapsedS) == "REC 00:00:05" && _rs.TaskSet && _rs.TaskName == "pick cup" && _rs.EpisodeName == "episode_test", $"label '{RecordStatusRule.Label(_rs.Look, _rs.ElapsedS)}', task '{_rs.TaskName}'");
        Check(!_rs.ToastShown, "STARTED shows no toast");

        // (c) elapsed keeps counting between heartbeats
        yield return Wait(1.2);
        Check(_rs.ElapsedS >= 6.1f && RecordStatusRule.Label(_rs.Look, _rs.ElapsedS) == "REC 00:00:06", $"after 1.2 s: ElapsedS {_rs.ElapsedS:F2}, '{RecordStatusRule.Label(_rs.Look, _rs.ElapsedS)}'");

        // (d) paused: frozen
        Inject(2, 2, 6f);
        yield return Wait(1.5);
        Inject(2, 0, 6f);   // heartbeat keeps it alive
        yield return Wait(0.2);
        Check(_rs.ElapsedS == 6f && _rs.Look == RecordStatusRule.Look.Paused, $"PAUSED: ElapsedS frozen at {_rs.ElapsedS:F2}, Look {_rs.Look}");

        // (e) toasts
        Inject(0, 4, 6f);
        Check(_rs.ToastShown && _rs.ToastText == "Saved · 00:00:06" && _rs.Look == RecordStatusRule.Look.Idle, $"SAVED: toast '{_rs.ToastText}', Look {_rs.Look}");
        uint seqSaved = _rs.EventSeq;
        Inject(0, 0, 6f);   // heartbeat with the same seq must not re-toast later
        yield return Wait(4.5);
        Inject(0, 0, 6f);
        Check(!_rs.ToastShown && _rs.EventSeq == seqSaved, "toast gone after 4 s, heartbeat does not bring it back");
        Inject(0, 5, 2f, "too_short");
        Check(_rs.ToastShown && _rs.ToastText == "Discarded: too short", $"DISCARDED: '{_rs.ToastText}'");
        Inject(4, 7, 2f, "demo error");
        Check(_rs.Look == RecordStatusRule.Look.Error && _rs.ToastText == "Error: demo error", $"ERROR: Look {_rs.Look}, toast '{_rs.ToastText}'");

        // (f) feed stops: badge vanishes
        yield return Wait(3.5);
        Check(_rs.Look == RecordStatusRule.Look.Hidden && !_rs.Available && _rs.HasMessage, $"stale: Look {_rs.Look}, Available {_rs.Available}");

        // (g) one connection
        Check(Object.FindObjectsByType<ROSConnection>(FindObjectsSortMode.None).Length == 1, "exactly one ROSConnection in the scene");

        // UI checks (strip group, bar pill, toast widget) are added by the HUD step.

        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Wait(0.1);
        PlayerPrefs.DeleteAll();
        PlayerPrefs.Save();
    }
}
