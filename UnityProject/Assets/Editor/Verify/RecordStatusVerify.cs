// Verify harness: recorder status feed (RecordStatusRule + RecordStatus data layer) and its HUD (status strip REC group,
// bar header REC pill, event toast), with pictures in $VERIFY_SHOTS/record. No sim needed (the ROS IP is isolated).
// Run: tools/verify.sh --suite RecordStatusVerify
using System;
using System.Collections;
using System.IO;
using System.Linq;
using System.Reflection;
using RosMessageTypes.SobitsInterfaces;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.UI;
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
        Check(RecordStatusRule.ToastColour(4) == HudTheme.Good && RecordStatusRule.ToastColour(5) == HudTheme.Bad && RecordStatusRule.ToastColour(7) == HudTheme.Bad
              && RecordStatusRule.ToastColour(6) == HudTheme.Warn && RecordStatusRule.ToastColour(9) == HudTheme.Warn && RecordStatusRule.ToastColour(8) == HudTheme.Accent,
              "ToastColour: saved Good, discarded / error Bad, deleted / rejected Warn, task Accent");
        Check(Mathf.Abs(RecordStatusRule.PulseAlpha(0f) - 0.775f) < 1e-3f && RecordStatusRule.PulseAlpha(0.25f) > 0.99f && RecordStatusRule.PulseAlpha(0.75f) < 0.56f, "PulseAlpha swings 0.55 .. 1 at 1 Hz");
    }

    static float _baseWidth;
    static string ShotDir => Directory.CreateDirectory(Path.Combine(Environment.GetEnvironmentVariable("VERIFY_SHOTS")
                                                                     ?? Path.GetFullPath(Path.Combine(Application.dataPath, "../../shots")), "record")).FullName;
    static Transform Find(Transform root, string name) => root.GetComponentsInChildren<Transform>(true).FirstOrDefault(t => t.name == name);
    static int Dividers(StatusStrip s) => s.GetComponentsInChildren<Transform>(true).Count(t => t.name == "Group Divider");
    static float Width(StatusStrip s) => ((RectTransform)s.transform).sizeDelta.x;
    static TextMeshProUGUI Text(Transform root, string name) => Find(root, name)?.GetComponent<TextMeshProUGUI>();
    static bool Near(Color a, Color b, float tol = 0.02f) => Mathf.Abs(a.r - b.r) < tol && Mathf.Abs(a.g - b.g) < tol && Mathf.Abs(a.b - b.b) < tol;
    static Transform BarT(TeleopHud hud) => (Transform)typeof(TeleopHud).GetField("_bar", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(hud);
    static Transform BarPill(TeleopHud hud) => Find(BarT(hud), "Record Pill");
    static bool BarPillShown(TeleopHud hud) { var p = BarPill(hud); return p != null && p.gameObject.activeInHierarchy; }
    static object Toggle(TeleopHud hud) { typeof(TeleopHud).GetMethod("ToggleBar", BindingFlags.Instance | BindingFlags.NonPublic).Invoke(hud, null); return Wait(0.2); }

    // (i)..(p): the REC group, the toast and the bar pill, fed by injected messages.
    static IEnumerable UiChecks(TeleopHud hud)
    {
        var strip = hud.Strip;
        var toast = hud.RecordToast;
        var head = FirstPersonView.Head;

        // (i) recording
        Inject(1, 1, 65f);
        yield return Wait(0.3);
        var group = Find(strip.transform, "Record Group");
        var label = group != null ? Text(group, "Record Label") : null;
        var task = group != null ? Text(group, "Record Task") : null;
        Check(group != null && group.gameObject.activeSelf && Find(group, "Record Divider") != null, "(i) RECORDING: Record Group active, with its Record Divider");
        if (group == null) yield break;
        Check(label.text == RecordStatusRule.Label(RecordStatusRule.Look.Recording, _rs.ElapsedS) && label.text == "REC 00:01:05" && Near(label.color, HudTheme.Record),
              $"(i) label '{label.text}' in Record colour {label.color}");
        Check(task.text == "task: pick cup" && Near(task.color, HudTheme.Muted), $"(i) task line '{task.text}' muted");
        float recW = strip.RecordWidthMm, width = Width(strip);
        Check(Dividers(strip) == 2 && Mathf.Abs(width - (_baseWidth + recW)) < 0.5f && width <= 1500f + recW,
              $"(i) strip {width:F0} mm = {_baseWidth:F0} + REC {recW:F0} (<= 1500 + REC); Group Dividers still {Dividers(strip)}");
        var body = Find(strip.transform, "Body Group") as RectTransform;
        Check(body != null && Mathf.Abs(body.anchoredPosition.x - (((RectTransform)group).anchoredPosition.x + recW)) < 0.5f, $"(i) BODY shifted right by the REC group (x {body?.anchoredPosition.x:F0})");
        var dot = Find(group, "Record Dot").GetComponent<Image>();
        float aMin = 1f, aMax = 0f;
        for (int i = 0; i < 4; i++) { aMin = Mathf.Min(aMin, dot.color.a); aMax = Mathf.Max(aMax, dot.color.a); yield return Wait(0.25); }
        Check(Near(dot.color, HudTheme.Record) && aMax - aMin > 0.1f && aMin >= 0.54f, $"(i) dot pulses (alpha {aMin:F2} .. {aMax:F2} over 1 s)");
        Inject(1, 0, 66f, taskSet: false, task: "");
        yield return Wait(0.3);
        Check(task.text == "no task" && Near(task.color, HudTheme.Warn), $"(i) task not set: '{task.text}' in Warn");
        Inject(1, 8, 66f, "pick cup");   // task set while recording: Accent toast
        yield return Wait(0.3);
        Check(toast.Shown && toast.Label.text == "Task: pick cup" && Near(toast.Colour, HudTheme.Accent), $"(i) TASK_SET toast '{toast.Label.text}' in Accent");
        var tr = toast.Rect; var sr = (RectTransform)strip.transform;
        float toastBottom = tr.localPosition.y - tr.sizeDelta.y * tr.localScale.y / 2f, stripTop = sr.localPosition.y + sr.sizeDelta.y * sr.localScale.y / 2f;
        Check(tr.parent == head && toastBottom > stripTop && toastBottom - stripTop < 0.03f,
              $"(i) toast head-locked just above the strip (gap {(toastBottom - stripTop) * 1000f:F0} mm; toast centre {-Mathf.Atan2(tr.localPosition.y, tr.localPosition.z) * Mathf.Rad2Deg:F1} deg below the view centre, strip {-Mathf.Atan2(sr.localPosition.y, sr.localPosition.z) * Mathf.Rad2Deg:F1} deg; toast {tr.sizeDelta.x:F0} x {tr.sizeDelta.y:F0} mm)");
        Shot("rec_fp_strip_toast", head.position, Quaternion.LookRotation(Vector3.Lerp(sr.position, tr.position, 0.5f) - head.position, Vector3.up), 40f);
        Shot("rec_fp_view", head.position, head.rotation, 90f);

        // (j) paused: Warn, frozen
        Inject(2, 2, 70f);
        yield return Wait(0.3);
        string paused = label.text;
        Check(paused == "PAUSED 00:01:10" && Near(label.color, HudTheme.Warn) && Near(Find(group, "Record Dot").GetComponent<Image>().color, HudTheme.Warn), $"(j) PAUSED: '{paused}' in Warn");
        yield return Wait(1.2);
        Inject(2, 0, 70f);
        yield return Wait(0.3);
        Check(label.text == paused, $"(j) still '{label.text}' 1.5 s later (frozen)");

        // (k) saved: Good toast for 4 s
        Inject(0, 4, 70f);
        yield return Wait(0.3);
        Check(toast.Shown && toast.Label.text == _rs.ToastText && toast.Label.text == "Saved · 00:01:10" && Near(toast.Colour, HudTheme.Good),
              $"(k) SAVED toast '{toast.Label.text}' in Good");
        Check(label.text == "IDLE" && Near(label.color, HudTheme.Muted), $"(k) strip back to '{label.text}' (muted)");
        Shot("rec_fp_saved_toast", head.position, Quaternion.LookRotation(Vector3.Lerp(sr.position, tr.position, 0.5f) - head.position, Vector3.up), 40f);
        yield return Wait(4.5);
        Check(!toast.Shown && !toast.Rect.gameObject.activeSelf, "(k) toast inactive 4.5 s later");

        // (l) feed stopped (stale since 4.8 s): the group goes, the strip shrinks back
        Check(!group.gameObject.activeSelf && Mathf.Abs(Width(strip) - _baseWidth) < 0.5f && Dividers(strip) == 2,
              $"(l) stale: Record Group inactive, strip {Width(strip):F0} mm (was {_baseWidth:F0}), Group Dividers {Dividers(strip)}");

        // (m) bar open: header pill with the same label
        yield return Toggle(hud);
        Inject(1, 1, 80f);
        yield return Wait(0.3);
        var pill = BarPill(hud);
        var pillText = pill != null ? pill.GetComponentInChildren<TextMeshProUGUI>(true) : null;
        var ip = Text(BarT(hud), "IP");
        Check(BarT(hud).gameObject.activeSelf && !strip.gameObject.activeSelf && BarPillShown(hud) && pillText.text == RecordStatusRule.Label(RecordStatusRule.Look.Recording, _rs.ElapsedS)
              && pillText.text.StartsWith("REC 00:01:2") && Near(pillText.color, HudTheme.Record),
              $"(m) bar shown: Record Pill '{pillText?.text}' in Record colour");
        var pr = (RectTransform)pill; var stat = (RectTransform)Find(BarT(hud), "Status");
        Check(ip.rectTransform.anchoredPosition.x + ip.rectTransform.sizeDelta.x < pr.anchoredPosition.x && pr.anchoredPosition.x + pr.sizeDelta.x < stat.anchoredPosition.x,
              $"(m) IP | pill | Status in order (IP {ip.rectTransform.sizeDelta.x:F0} mm wide, text '{ip.text}' truncated {ip.isTextTruncated}; pill {pr.sizeDelta.x:F0} mm)");
        ip.text = "192.168.11.20"; ip.ForceMeshUpdate();   // a typical robot IP (the bar puts the real one back next frame)
        Check(!ip.isTextTruncated && ip.fontSize >= ip.fontSizeMax * 0.6f - 0.01f, $"(m) '192.168.11.20' still fits beside the pill (font {ip.fontSize:F0} of {ip.fontSizeMax:F0} mm)");
        Check(toast.Rect.parent == head, "(m) toast stays head-locked above the strip in first person while the bar is open");
        Shot("rec_bar_pill", head.position, Quaternion.LookRotation(pill.position - head.position, Vector3.up), 30f);
        yield return Toggle(hud);
        yield return Wait(3.5);
        yield return Toggle(hud);
        Check(!BarPillShown(hud) && Mathf.Abs(ip.rectTransform.sizeDelta.x - (((RectTransform)Find(BarT(hud), "Status")).anchoredPosition.x - 40f - ip.rectTransform.anchoredPosition.x)) < 0.5f,
              $"(m) bar reopened without a feed: no pill, IP full width {ip.rectTransform.sizeDelta.x:F0} mm");
        yield return Toggle(hud);

        // (n) blocks layout
        hud.SetCameraLayout(FirstPersonView.LayoutBlocks, save: false);
        yield return Wait(0.5);
        if (BarT(hud).gameObject.activeSelf) yield return Toggle(hud);
        strip = hud.Strip;
        Inject(1, 1, 90f);
        yield return Wait(0.3);
        group = Find(strip.transform, "Record Group");
        Check(!hud.FirstPerson && strip != null && !strip.HasModel && strip.gameObject.activeSelf && Dividers(strip) == 1 && group != null && group.gameObject.activeSelf
              && Text(group, "Record Label").text == "REC 00:01:30",
              $"(n) blocks: strip active, {Dividers(strip)} Group Divider, Record Group active '{Text(group, "Record Label")?.text}', {Width(strip):F0} mm x {strip.transform.localScale.x * 1000f:F2}");
        Inject(1, 4, 90f);   // a toast to see where it goes
        yield return Wait(0.3);
        tr = toast.Rect; sr = (RectTransform)strip.transform;
        Check(toast.Shown && tr.parent == sr.parent && tr.localScale == sr.localScale && tr.localPosition.y > sr.localPosition.y, $"(n) blocks, bar hidden: toast above the strip (y {tr.localPosition.y:F2} vs {sr.localPosition.y:F2} m)");
        var cam = Camera.main.transform;
        Shot("rec_blocks_strip", cam.position, Quaternion.LookRotation(Vector3.Lerp(sr.position, tr.position, 0.5f) - cam.position, Vector3.up), 45f);
        yield return Toggle(hud);
        var bar = (RectTransform)BarT(hud);
        Check(tr.localPosition.y > bar.localPosition.y + bar.sizeDelta.y * bar.localScale.y / 2f, $"(n) blocks, bar shown: toast above the bar (y {tr.localPosition.y:F2} m)");
        Shot("rec_blocks_bar", cam.position, Quaternion.LookRotation(bar.position + Vector3.up * 0.4f - cam.position, Vector3.up), 60f);
        yield return Toggle(hud);

        // (o) lazy follow carries the strip and the toast
        hud.SetLazyFollow(true);
        yield return Wait(0.2);
        var follower = (typeof(TeleopHud).GetField("_follower", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(hud) as Component)?.transform;
        Check(follower != null && strip.transform.parent == follower && toast.Rect.parent == follower, $"(o) lazy follow on: strip and toast under '{toast.Rect.parent?.name}'");
        hud.SetLazyFollow(false);
        yield return Wait(0.2);
        Check(strip.transform.parent == hud.hudParent && toast.Rect.parent == hud.hudParent, $"(o) lazy follow off: back under '{toast.Rect.parent?.name}'");

        // (p)
        Check(Object.FindObjectsByType<ROSConnection>(FindObjectsSortMode.None).Length == 1, "(p) still exactly one ROSConnection");
    }

    static void Shot(string name, Vector3 pos, Quaternion rot, float fov)
    {
        var go = new GameObject("VerifyCam");
        var cam = go.AddComponent<Camera>();
        go.transform.SetPositionAndRotation(pos, rot);
        var main = Camera.main;
        cam.fieldOfView = fov; cam.aspect = 16f / 9f; cam.nearClipPlane = 0.05f; cam.farClipPlane = 200f;
        cam.stereoTargetEye = StereoTargetEyeMask.None;
        if (main != null) { cam.clearFlags = main.clearFlags; cam.backgroundColor = main.backgroundColor; }
        var rt = new RenderTexture(1600, 900, 24);
        cam.targetTexture = rt;
        cam.Render();
        RenderTexture.active = rt;
        var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;
        string path = Path.Combine(ShotDir, name + ".png");
        File.WriteAllBytes(path, tex.EncodeToPNG());
        cam.targetTexture = null; Object.DestroyImmediate(rt); Object.DestroyImmediate(go); Object.DestroyImmediate(tex);
        Debug.Log($"[Verify]     screenshot {path} (fov {fov:F0})");
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
        var hud = Object.FindFirstObjectByType<TeleopHud>();

        // (h) first person, no recorder yet: the HUD looks as it did before the feature
        hud.SetRobotModel(true, save: false);
        hud.SetCameraLayout(FirstPersonView.LayoutFirstPerson, save: false);
        yield return Wait(1.0);
        var strip = hud.Strip;
        Check(hud.FirstPerson && strip != null && strip.HasModel && strip.gameObject.activeSelf, $"(h) first person: strip with the model, active {strip?.gameObject.activeSelf}");
        if (strip == null) { EditorApplication.ExitPlaymode(); yield break; }
        _baseWidth = Width(strip);
        Check(Find(strip.transform, "Record Group") == null && Dividers(strip) == 2 && _baseWidth <= 1500f,
              $"(h) no message: no Record Group, {Dividers(strip)} Group Dividers, strip {_baseWidth:F0} mm (<= 1500)");
        Check(!BarPillShown(hud) && !hud.RecordToast.Shown, "(h) no message: bar Record Pill inactive, toast inactive");

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

        foreach (var w in UiChecks(hud)) yield return w;

        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Wait(0.1);
        PlayerPrefs.DeleteAll();
        PlayerPrefs.Save();
    }
}
