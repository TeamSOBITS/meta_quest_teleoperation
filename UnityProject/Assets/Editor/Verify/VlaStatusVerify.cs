// Verify harness: VLA stage status feed (VlaStatusRule + VlaStatus data layer; recorder and deploy) and its HUD (status
// strip VLA group, bar header pill, event toast), with pictures in $VERIFY_SHOTS/vla. No sim needed (the ROS IP is isolated).
// Run: tools/verify.sh --suite VlaStatusVerify
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

public static class VlaStatusVerify
{
    static IEnumerator _run; static double _until; static int _fail, _pass;
    static VlaStatus _rs;
    static uint _seq, _seqDeploy;
    const string ShortPolicy = "team-sobits/smolvla_fft";

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
        var m = new VlaStatusMsg
        {
            state = state, @event = evt, event_seq = _seq, task_set = taskSet, task_name = task, episode_name = "episode_test",
            elapsed_sec = elapsed, detail = detail, message = "test",
        };
        Feed(m);
    }

    static void Feed(VlaStatusMsg m) => typeof(VlaStatus).GetMethod("OnMessage", BindingFlags.Instance | BindingFlags.NonPublic).Invoke(_rs, new object[] { m });

    // A deploy-stage message (sobits_vla_deploy/status); its own event sequence, like the real node.
    static void InjectDeploy(byte state, byte evt, float elapsed, bool engaged = true, bool enabled = true, uint steps = 240, float hz = 8.1f,
                             string policy = ShortPolicy, string outcome = "", bool taskSet = true, string task = "pick cup")
    {
        if (evt != 0) _seqDeploy++;
        Feed(new VlaStatusMsg
        {
            stage = VlaStatusMsg.STAGE_DEPLOY, state = state, @event = evt, event_seq = _seqDeploy, task_set = taskSet, task_name = task,
            episode_name = "episode_1", elapsed_sec = elapsed, detail = outcome, message = "test", policy = policy,
            deadman_enabled = enabled, deadman_engaged = engaged, steps = steps, inference_hz = hz, outcome = outcome,
        });
    }

    static void DeployStaticChecks()
    {
        Check(VlaStatusRule.Evaluate(true, 0.5f, 3) == VlaStatusRule.Look.Playing && VlaStatusRule.Evaluate(true, 0.5f, 5) == VlaStatusRule.Look.Resetting,
              "Evaluate: PLAYING -> Playing, RESETTING -> Resetting");
        Check(VlaStatusRule.Label(VlaStatusRule.Look.Playing, 12.7f) == "PLAY 00:00:12" && VlaStatusRule.Label(VlaStatusRule.Look.Resetting, 30f) == "RESETTING",
              "Label Playing 'PLAY 00:00:12', Resetting 'RESETTING'");
        Check(VlaStatusRule.Colour(VlaStatusRule.Look.Playing) == HudTheme.Accent && VlaStatusRule.Colour(VlaStatusRule.Look.Resetting) == HudTheme.Warn,
              "Colour: Playing Accent, Resetting Warn");
        Check(VlaStatusRule.Pulses(VlaStatusRule.Look.Recording, false) && VlaStatusRule.Pulses(VlaStatusRule.Look.Playing, true)
              && !VlaStatusRule.Pulses(VlaStatusRule.Look.Playing, false) && !VlaStatusRule.Pulses(VlaStatusRule.Look.Paused, true)
              && !VlaStatusRule.Pulses(VlaStatusRule.Look.Resetting, true), "Pulses: Recording, Playing while driving; not released / paused / resetting");
        string p24 = "abcdefghijklmnopqrstuvwx", longP = VlaStatusDemo.DemoPolicy, shortLong = VlaStatusRule.PolicyShort(longP);
        Check(VlaStatusRule.PolicyShort(ShortPolicy) == "smolvla_fft" && VlaStatusRule.PolicyShort("smolvla_fft") == "smolvla_fft" && VlaStatusRule.PolicyShort("") == ""
              && VlaStatusRule.PolicyShort("org/" + p24) == p24 && shortLong.Length == 24 && shortLong[0] == '\u2026' && longP.EndsWith(shortLong.Substring(1)),
              $"PolicyShort: last segment, 24 chars kept, longer -> '{shortLong}'");
        Check(VlaStatusRule.TaskLine(0, true, "pick cup", "x") == "task: pick cup" && VlaStatusRule.TaskLine(0, false, "", "") == "no task"
              && VlaStatusRule.TaskLine(1, true, "pick cup", ShortPolicy) == "pick cup \u00B7 smolvla_fft"
              && VlaStatusRule.TaskLine(1, false, "", ShortPolicy) == "no task \u00B7 smolvla_fft" && VlaStatusRule.TaskLine(1, true, "pick cup", "") == "pick cup",
              "TaskLine: collection 'task: pick cup' / 'no task'; deploy 'pick cup · smolvla_fft' / 'no task · …' / no policy");
        var off = VlaStatusRule.Deadman(false, true, 3); var rel = VlaStatusRule.Deadman(true, false, 3); var eng = VlaStatusRule.Deadman(true, true, 3);
        Check(!off.HasValue && !VlaStatusRule.Deadman(true, true, 0).HasValue && !VlaStatusRule.Deadman(true, true, 5).HasValue
              && rel.HasValue && rel.Value.text == "GRIP released" && rel.Value.colour == HudTheme.Warn && !rel.Value.pulse
              && eng.HasValue && eng.Value.text == "GRIP driving" && eng.Value.colour == HudTheme.Good && eng.Value.pulse,
              "Deadman: off / not playing -> none; released Warn steady; engaged Good pulsing");
        Check(VlaStatusRule.Stats(240, 8.1f) == "240 steps \u00B7 8.1 Hz" && VlaStatusRule.Stats(0, 0f) == "0 steps \u00B7 0.0 Hz", $"Stats '{VlaStatusRule.Stats(240, 8.1f)}'");
        Check(VlaStatusRule.Supersedes(1, 0, 0, 1, 0.5f) && !VlaStatusRule.Supersedes(0, 0, 0, 1, 0.5f) && VlaStatusRule.Supersedes(0, 0, 0, 1, 3.5f),
              "Supersedes: same stage always; idle heartbeat of the other stage not over a live message, over a stale one yes");
        Check(VlaStatusRule.Supersedes(0, 0, 8, 1, 0.5f) && VlaStatusRule.Supersedes(0, 1, 0, 1, 0.5f) && VlaStatusRule.Supersedes(1, 0, 0, 0, -1f),
              "Supersedes: other stage's event / non-idle state takes over; anything over no message");
        bool none = true;
        foreach (byte e in new byte[] { 10, 11, 12, 13, 14 }) none &= VlaStatusRule.Toast(e, "success_lift", 1f) == null;
        Check(none, "Toast STOPPED / EPISODE_DONE / ENGAGED / RELEASED / RESET_DONE -> null (no outcome toast)");
    }

    static void StaticChecks()
    {
        Check(VlaStatusRule.StaleS == 3f && VlaStatusRule.ToastS == 4f && VlaStatusRule.PulseHz == 1f, "constants StaleS 3, ToastS 4, PulseHz 1");
        var L = VlaStatusRule.Look.Hidden;
        Check(VlaStatusRule.Evaluate(false, 0f, 1) == VlaStatusRule.Look.Hidden, "Evaluate: no message -> Hidden");
        Check(VlaStatusRule.Evaluate(true, 3f, 1) == VlaStatusRule.Look.Hidden && VlaStatusRule.Evaluate(true, 3.1f, 1) == VlaStatusRule.Look.Hidden, "Evaluate: age >= 3 -> Hidden (stale)");
        Check(VlaStatusRule.Evaluate(true, -1f, 1) == VlaStatusRule.Look.Hidden, "Evaluate: negative age -> Hidden");
        Check(VlaStatusRule.Evaluate(true, 0.5f, 0) == VlaStatusRule.Look.Idle, "Evaluate: STOPPED -> Idle");
        Check(VlaStatusRule.Evaluate(true, 0.5f, 1) == VlaStatusRule.Look.Recording, "Evaluate: RECORDING -> Recording");
        Check(VlaStatusRule.Evaluate(true, 0.5f, 2) == VlaStatusRule.Look.Paused, "Evaluate: PAUSED -> Paused");
        Check(VlaStatusRule.Evaluate(true, 0.5f, 4) == VlaStatusRule.Look.Error, "Evaluate: ERROR -> Error");
        Check(VlaStatusRule.Evaluate(true, 2.9f, 1) == VlaStatusRule.Look.Recording, "Evaluate: age 2.9 still alive");
        Check(VlaStatusRule.IsAlive(2.9f) && !VlaStatusRule.IsAlive(3.1f) && !VlaStatusRule.IsAlive(-1f), "IsAlive: 2.9 true, 3.1 false, -1 false");
        Check(VlaStatusRule.FormatElapsed(3661f) == "01:01:01" && VlaStatusRule.FormatElapsed(-5f) == "00:00:00" && VlaStatusRule.FormatElapsed(65.9f) == "00:01:05", "FormatElapsed 3661 -> 01:01:01, negative -> 00:00:00, 65.9 -> 00:01:05");
        Check(VlaStatusRule.Label(VlaStatusRule.Look.Recording, 65f) == "REC 00:01:05", "Label Recording");
        Check(VlaStatusRule.Label(VlaStatusRule.Look.Paused, 65f) == "PAUSED 00:01:05", "Label Paused");
        Check(VlaStatusRule.Label(VlaStatusRule.Look.Idle, 0f) == "IDLE" && VlaStatusRule.Label(VlaStatusRule.Look.Error, 0f) == "ERROR" && VlaStatusRule.Label(L, 0f) == "", "Label Idle / Error / Hidden");
        Check(VlaStatusRule.Colour(VlaStatusRule.Look.Recording) == HudTheme.Record && VlaStatusRule.Colour(VlaStatusRule.Look.Paused) == HudTheme.Warn
              && VlaStatusRule.Colour(VlaStatusRule.Look.Error) == HudTheme.Bad && VlaStatusRule.Colour(L).a == 0f, "Colour: Record / Warn / Bad / clear");
        Check(VlaStatusRule.Toast(4, "", 65f) == "Saved · 00:01:05", "Toast SAVED");
        Check(VlaStatusRule.Toast(5, "too_short", 0f) == "Discarded: too short" && VlaStatusRule.Toast(5, "integrity_failed", 0f) == "Discarded: integrity failed"
              && VlaStatusRule.Toast(5, "other", 0f) == "Discarded: other", "Toast DISCARDED (too_short / integrity_failed / other)");
        Check(VlaStatusRule.Toast(6, "", 0f) == "Deleted", "Toast DELETED");
        Check(VlaStatusRule.Toast(7, "disk full", 0f) == "Error: disk full" && VlaStatusRule.Toast(7, "", 0f) == "Error", "Toast ERROR (detail / none)");
        Check(VlaStatusRule.Toast(8, "pick cup", 0f) == "Task: pick cup", "Toast TASK_SET");
        Check(VlaStatusRule.Toast(9, "no task set", 0f) == "no task set" && VlaStatusRule.Toast(9, "", 0f) == "Rejected", "Toast REJECTED (detail / none)");
        bool none = true;
        foreach (byte e in new byte[] { 0, 1, 2, 3 }) none &= VlaStatusRule.Toast(e, "x", 1f) == null;
        Check(none, "Toast NONE / STARTED / PAUSED / RESUMED -> null");
        Check(VlaStatusRule.ToastColour(4) == HudTheme.Good && VlaStatusRule.ToastColour(5) == HudTheme.Bad && VlaStatusRule.ToastColour(7) == HudTheme.Bad
              && VlaStatusRule.ToastColour(6) == HudTheme.Warn && VlaStatusRule.ToastColour(9) == HudTheme.Warn && VlaStatusRule.ToastColour(8) == HudTheme.Accent,
              "ToastColour: saved Good, discarded / error Bad, deleted / rejected Warn, task Accent");
        Check(Mathf.Abs(VlaStatusRule.PulseAlpha(0f) - 0.775f) < 1e-3f && VlaStatusRule.PulseAlpha(0.25f) > 0.99f && VlaStatusRule.PulseAlpha(0.75f) < 0.56f, "PulseAlpha swings 0.55 .. 1 at 1 Hz");
    }

    static float _baseWidth, _collectionVlaW;
    static string ShotDir => Directory.CreateDirectory(Path.Combine(Environment.GetEnvironmentVariable("VERIFY_SHOTS")
                                                                     ?? Path.GetFullPath(Path.Combine(Application.dataPath, "../../shots")), "vla")).FullName;
    static Transform Find(Transform root, string name) => root.GetComponentsInChildren<Transform>(true).FirstOrDefault(t => t.name == name);
    static int Dividers(StatusStrip s) => s.GetComponentsInChildren<Transform>(true).Count(t => t.name == "Group Divider");
    static float Width(StatusStrip s) => ((RectTransform)s.transform).sizeDelta.x;
    static TextMeshProUGUI Text(Transform root, string name) => Find(root, name)?.GetComponent<TextMeshProUGUI>();
    static bool Near(Color a, Color b, float tol = 0.02f) => Mathf.Abs(a.r - b.r) < tol && Mathf.Abs(a.g - b.g) < tol && Mathf.Abs(a.b - b.b) < tol;
    static Transform BarT(TeleopHud hud) => (Transform)typeof(TeleopHud).GetField("_bar", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(hud);
    static Transform BarPill(TeleopHud hud) => Find(BarT(hud), "Vla Pill");
    static bool BarPillShown(TeleopHud hud) { var p = BarPill(hud); return p != null && p.gameObject.activeInHierarchy; }
    static object Toggle(TeleopHud hud) { typeof(TeleopHud).GetMethod("ToggleBar", BindingFlags.Instance | BindingFlags.NonPublic).Invoke(hud, null); return Wait(0.2); }

    // (i)..(p): the REC group, the toast and the bar pill, fed by injected messages.
    static IEnumerable UiChecks(TeleopHud hud)
    {
        var strip = hud.Strip;
        var toast = hud.VlaToast;
        var head = FirstPersonView.Head;

        // (i) recording
        Inject(1, 1, 65f);
        yield return Wait(0.3);
        var group = Find(strip.transform, "Vla Group");
        var label = group != null ? Text(group, "Vla Label") : null;
        var task = group != null ? Text(group, "Vla Task") : null;
        Check(group != null && group.gameObject.activeSelf && Find(group, "Vla Divider") != null, "(i) RECORDING: Vla Group active, with its Vla Divider");
        if (group == null) yield break;
        Check(label.text == VlaStatusRule.Label(VlaStatusRule.Look.Recording, _rs.ElapsedS) && label.text == "REC 00:01:05" && Near(label.color, HudTheme.Record),
              $"(i) label '{label.text}' in Record colour {label.color}");
        Check(task.text == "task: pick cup" && Near(task.color, HudTheme.Muted), $"(i) task line '{task.text}' muted");
        float recW = strip.VlaWidthMm, width = Width(strip);
        _collectionVlaW = recW;
        Check(Dividers(strip) == 2 && Mathf.Abs(width - (_baseWidth + recW)) < 0.5f && width <= 1500f + recW,
              $"(i) strip {width:F0} mm = {_baseWidth:F0} + REC {recW:F0} (<= 1500 + REC); Group Dividers still {Dividers(strip)}");
        var body = Find(strip.transform, "Body Group") as RectTransform;
        Check(body != null && Mathf.Abs(body.anchoredPosition.x - (((RectTransform)group).anchoredPosition.x + recW)) < 0.5f, $"(i) BODY shifted right by the REC group (x {body?.anchoredPosition.x:F0})");
        var dot = Find(group, "Vla Dot").GetComponent<Image>();
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
        Check(paused == "PAUSED 00:01:10" && Near(label.color, HudTheme.Warn) && Near(Find(group, "Vla Dot").GetComponent<Image>().color, HudTheme.Warn), $"(j) PAUSED: '{paused}' in Warn");
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
              $"(l) stale: Vla Group inactive, strip {Width(strip):F0} mm (was {_baseWidth:F0}), Group Dividers {Dividers(strip)}");

        // (m) bar open: header pill with the same label
        yield return Toggle(hud);
        Inject(1, 1, 80f);
        yield return Wait(0.3);
        var pill = BarPill(hud);
        var pillText = pill != null ? pill.GetComponentInChildren<TextMeshProUGUI>(true) : null;
        var ip = Text(BarT(hud), "IP");
        Check(BarT(hud).gameObject.activeSelf && !strip.gameObject.activeSelf && BarPillShown(hud) && pillText.text == VlaStatusRule.Label(VlaStatusRule.Look.Recording, _rs.ElapsedS)
              && pillText.text.StartsWith("REC 00:01:2") && Near(pillText.color, HudTheme.Record),
              $"(m) bar shown: Vla Pill '{pillText?.text}' in Record colour");
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
        group = Find(strip.transform, "Vla Group");
        Check(!hud.FirstPerson && strip != null && !strip.HasModel && strip.gameObject.activeSelf && Dividers(strip) == 1 && group != null && group.gameObject.activeSelf
              && Text(group, "Vla Label").text == "REC 00:01:30",
              $"(n) blocks: strip active, {Dividers(strip)} Group Divider, Vla Group active '{Text(group, "Vla Label")?.text}', {Width(strip):F0} mm x {strip.transform.localScale.x * 1000f:F2}");
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

    static Vector3 Centre(Transform t) { var r = (RectTransform)t; return r.TransformPoint(r.rect.center); }
    static float Alpha(Transform t) => t.GetComponent<Image>().color.a;

    // (q)..(x): the deploy stage (sobits_vla_deploy/status): PLAY, GRIP chip, steps · Hz, RESETTING, bar pill, both stages.
    static IEnumerable DeployChecks(TeleopHud hud)
    {
        var topics = (System.Collections.Generic.HashSet<string>)typeof(VlaStatus).GetField("_subscribedTopics", BindingFlags.Static | BindingFlags.NonPublic).GetValue(null);
        var profile = RobotProfile.Selected;
        Check(topics.Count == 2 && topics.Contains(profile.FullTopic(RosNames.CollectionStatus)) && topics.Contains(profile.FullTopic(RosNames.DeployStatus)),
              $"(q) subscribed to both stage topics: {string.Join(", ", topics)}");
        hud.SetCameraLayout(FirstPersonView.LayoutFirstPerson, save: false);
        yield return Wait(3.5);   // the collection feed goes stale
        if (BarT(hud).gameObject.activeSelf) yield return Toggle(hud);
        var strip = hud.Strip;
        var head = FirstPersonView.Head;
        Check(hud.FirstPerson && strip != null && !strip.VlaShown && Mathf.Abs(Width(strip) - _baseWidth) < 0.5f, $"(q) first person again, no feed: strip {Width(strip):F0} mm");

        // (x) started with the grip released: the episode clock waits for the first engagement
        InjectDeploy(3, 1, 0f, engaged: false, steps: 0, hz: 0f);
        yield return Wait(1.2);
        Check(_rs.Stage == 1 && _rs.IsDeploy && _rs.ElapsedS == 0f && _rs.Look == VlaStatusRule.Look.Playing && !_rs.ToastShown,
              $"(x) PLAYING released at 0 s: clock waits (ElapsedS {_rs.ElapsedS:F2}), no toast");
        InjectDeploy(3, 12, 0f, steps: 0, hz: 0f);
        yield return Wait(1.2);
        Check(_rs.ElapsedS >= 1.1f && !_rs.ToastShown, $"(x) ENGAGED: clock runs (ElapsedS {_rs.ElapsedS:F2}), no toast");

        // (q) playing, grip held
        InjectDeploy(3, 0, 12f);
        yield return Wait(0.3);
        var group = Find(strip.transform, "Vla Group");
        var label = Text(group, "Vla Label"); var task = Text(group, "Vla Task"); var stats = Text(group, "Vla Stats");
        var chip = Find(group, "Vla Chip"); var chipText = chip.GetComponentInChildren<TextMeshProUGUI>(true);
        float wq = Width(strip), vq = strip.VlaWidthMm;
        Check(strip.VlaShown && strip.VlaDeployShown && label.text == "PLAY 00:00:12" && Near(label.color, HudTheme.Accent),
              $"(q) PLAYING engaged: label '{label.text}' in Accent, deploy column shown");
        Check(task.text == "pick cup \u00B7 smolvla_fft" && Near(task.color, HudTheme.Muted) && !task.isTextTruncated, $"(q) task line '{task.text}' muted");
        Check(chip.gameObject.activeSelf && chipText.text == "GRIP driving" && Near(chipText.color, HudTheme.Good), $"(q) chip '{chipText.text}' in Good");
        Check(stats.text == "240 steps \u00B7 8.1 Hz" && Near(stats.color, HudTheme.Muted) && !stats.isTextTruncated, $"(q) stats '{stats.text}' muted");
        var sr = (RectTransform)strip.transform;
        float stripTop = sr.localPosition.y + sr.sizeDelta.y * sr.localScale.y / 2f;
        Check(Dividers(strip) == 2 && Mathf.Abs(wq - (_baseWidth + vq)) < 0.5f && stripTop < 0f,
              $"(q) strip {wq:F0} mm = {_baseWidth:F0} + VLA deploy {vq:F0} (collection was {_collectionVlaW:F0}); top edge {-Mathf.Atan2(stripTop, sr.localPosition.z) * Mathf.Rad2Deg:F1} deg below the view centre, " +
              $"{2f * Mathf.Atan2(wq * sr.localScale.x / 2f, sr.localPosition.z) * Mathf.Rad2Deg:F0} deg wide");
        var dot = Find(group, "Vla Dot");
        float aMin = 1f, aMax = 0f, cMin = 1f, cMax = 0f;
        for (int i = 0; i < 4; i++)
        {
            aMin = Mathf.Min(aMin, Alpha(dot)); aMax = Mathf.Max(aMax, Alpha(dot));
            cMin = Mathf.Min(cMin, Alpha(chip)); cMax = Mathf.Max(cMax, Alpha(chip));
            yield return Wait(0.25);
        }
        Check(Near(dot.GetComponent<Image>().color, HudTheme.Accent) && aMax - aMin > 0.1f && cMax - cMin > 0.05f,
              $"(q) Accent dot pulses ({aMin:F2} .. {aMax:F2}), chip background pulses ({cMin:F2} .. {cMax:F2})");
        InjectDeploy(3, 0, 13f, policy: VlaStatusDemo.DemoPolicy);
        yield return Wait(0.3);
        Check(task.text.StartsWith("pick cup \u00B7 \u2026") && task.text.EndsWith("smolvla_fft") && !task.isTextTruncated && Mathf.Abs(Width(strip) - wq) < 0.5f,
              $"(q) long policy: '{task.text}' (head cut, tail kept), strip width unchanged");
        Shot("vla_fp_deploy_engaged", head.position, Quaternion.LookRotation(Centre(group) - head.position, Vector3.up), 22f);
        Shot("vla_fp_deploy_view", head.position, head.rotation, 90f);

        // (r) grip released: Warn chip, steady dot
        InjectDeploy(3, 13, 14f, engaged: false, hz: 0f);
        yield return Wait(0.3);
        aMin = 1f; aMax = 0f;
        for (int i = 0; i < 3; i++) { aMin = Mathf.Min(aMin, Alpha(dot)); aMax = Mathf.Max(aMax, Alpha(dot)); yield return Wait(0.2); }
        Check(chip.gameObject.activeSelf && chipText.text == "GRIP released" && Near(chipText.color, HudTheme.Warn) && aMin > 0.99f && Mathf.Abs(Width(strip) - wq) < 0.5f && !_rs.ToastShown,
              $"(r) RELEASED: chip '{chipText.text}' in Warn, dot steady ({aMin:F2}), width unchanged, no toast");
        Shot("vla_fp_deploy_released", head.position, Quaternion.LookRotation(Centre(group) - head.position, Vector3.up), 22f);

        // (s) no deadman: no chip, the stats line alone; the column keeps its width (the stats line is the widest)
        InjectDeploy(3, 0, 15f, engaged: false, enabled: false);
        yield return Wait(0.3);
        Check(!chip.gameObject.activeSelf && stats.gameObject.activeInHierarchy && Width(strip) <= wq && _rs.ElapsedS > 15f,
              $"(s) deadman off: chip inactive, stats '{stats.text}', strip {Width(strip):F0} mm (<= {wq:F0}), clock runs");

        // (t) stopped -> resetting (no outcome toast), then idle
        InjectDeploy(5, 10, 16f, engaged: false, hz: 0f, outcome: "success_lift");
        yield return Wait(0.3);
        Check(label.text == "RESETTING" && Near(label.color, HudTheme.Warn) && !chip.gameObject.activeSelf && !_rs.ToastShown && _rs.Outcome == "success_lift",
              $"(t) RESETTING: '{label.text}' in Warn, no chip, no toast (outcome '{_rs.Outcome}')");
        Shot("vla_fp_deploy_resetting", head.position, Quaternion.LookRotation(Centre(group) - head.position, Vector3.up), 22f);
        InjectDeploy(0, 11, 16f, engaged: false, hz: 0f, outcome: "success_lift");
        yield return Wait(0.3);
        Check(label.text == "IDLE" && !_rs.ToastShown && strip.VlaDeployShown && Mathf.Abs(Width(strip) - wq) < 0.5f, $"(t) EPISODE_DONE: '{label.text}', no toast, deploy column kept");

        // (u) bar shown: pill label from the state, dot in the GRIP colour while the policy plays with a deadman
        yield return Toggle(hud);
        InjectDeploy(3, 1, 20f);
        yield return Wait(0.3);
        var pill = BarPill(hud);
        var pillText = pill.GetComponentInChildren<TextMeshProUGUI>(true);
        var pillDot = Find(pill, "Dot").GetComponent<Image>();
        Check(BarPillShown(hud) && pillText.text.StartsWith("PLAY 00:00:2") && Near(pillText.color, HudTheme.Accent) && Near(pillDot.color, HudTheme.Good),
              $"(u) bar pill '{pillText.text}' in Accent, dot Good while engaged");
        Shot("vla_bar_pill_deploy", head.position, Quaternion.LookRotation(pill.position - head.position, Vector3.up), 30f);
        InjectDeploy(3, 13, 21f, engaged: false);
        yield return Wait(0.3);
        Check(Near(pillDot.color, HudTheme.Warn) && pillDot.color.a > 0.99f, "(u) released: pill dot Warn, steady");
        InjectDeploy(5, 10, 21f, engaged: false, outcome: "timeout");
        yield return Wait(0.3);
        Check(pillText.text == "RESETTING" && Near(pillDot.color, HudTheme.Warn) && Near(pillText.color, HudTheme.Warn), $"(u) resetting: pill '{pillText.text}' Warn");
        yield return Toggle(hud);

        // (v) both stages up: an idle collection heartbeat does not displace a live deploy episode
        InjectDeploy(3, 0, 25f);
        Inject(0, 0, 0f);
        yield return Wait(0.3);
        Check(_rs.Stage == 1 && _rs.State == 3 && label.text.StartsWith("PLAY") && strip.VlaDeployShown,
              $"(v) collection idle heartbeat ignored: stage {_rs.Stage}, state {_rs.State}, '{label.text}'");
        Inject(0, 8, 0f, "pick cup");   // a collection event takes over
        yield return Wait(0.3);
        Check(_rs.Stage == 0 && !strip.VlaDeployShown && Mathf.Abs(strip.VlaWidthMm - _collectionVlaW) < 0.5f,
              $"(v) collection event takes over: stage {_rs.Stage}, deploy column hidden, VLA group {strip.VlaWidthMm:F0} mm");

        // (w) blocks layout: the same group
        hud.SetCameraLayout(FirstPersonView.LayoutBlocks, save: false);
        yield return Wait(0.5);
        if (BarT(hud).gameObject.activeSelf) yield return Toggle(hud);
        strip = hud.Strip;
        InjectDeploy(3, 12, 30f);
        yield return Wait(0.3);
        group = Find(strip.transform, "Vla Group");
        Check(strip.VlaDeployShown && Text(group, "Vla Label").text == "PLAY 00:00:30" && Text(group, "Vla Stats").text == "240 steps \u00B7 8.1 Hz"
              && Find(group, "Vla Chip").gameObject.activeSelf && Dividers(strip) == 1,
              $"(w) blocks: deploy group '{Text(group, "Vla Label").text}', chip, stats; strip {Width(strip):F0} mm");
        var cam = Camera.main.transform;
        Shot("vla_blocks_deploy", cam.position, Quaternion.LookRotation(strip.transform.position - cam.position, Vector3.up), 45f);
        Check(Object.FindObjectsByType<ROSConnection>(FindObjectsSortMode.None).Length == 1, "(w) still exactly one ROSConnection");
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
        DeployStaticChecks();

        var profile = AssetDatabase.LoadAssetAtPath<RobotProfile>("Assets/Robots/SOBIT_HOME.asset");
        Check(profile != null && profile.vlaStatusSuffixes != null && profile.vlaStatusSuffixes.SequenceEqual(RosNames.VlaStatusDefaults),
              $"SOBIT_HOME vlaStatusSuffixes '{string.Join(", ", profile?.vlaStatusSuffixes ?? new string[0])}'");
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
        Check(Find(strip.transform, "Vla Group") == null && Dividers(strip) == 2 && _baseWidth <= 1500f,
              $"(h) no message: no Record Group, {Dividers(strip)} Group Dividers, strip {_baseWidth:F0} mm (<= 1500)");
        Check(!BarPillShown(hud) && !hud.VlaToast.Shown, "(h) no message: bar Vla Pill inactive, toast inactive");

        // (a) nothing received yet
        _rs = VlaStatus.Latest;
        Check(_rs != null && _rs.name == "Vla Status", "VlaStatus.Latest created by the HUD");
        if (_rs == null) { EditorApplication.ExitPlaymode(); yield break; }
        Check(!_rs.HasMessage && _rs.Look == VlaStatusRule.Look.Hidden && !_rs.Available && _rs.AgeS < 0f && !_rs.ToastShown, "no message: HasMessage false, Look Hidden, AgeS -1, no toast");
        int changed = 0; _rs.Changed += () => changed++;

        // (b) recording
        Inject(1, 1, 5f);
        Check(changed == 1 && _rs.HasMessage && _rs.Available && _rs.Look == VlaStatusRule.Look.Recording, "RECORDING/STARTED: Changed raised, Look Recording");
        Check(VlaStatusRule.Label(_rs.Look, _rs.ElapsedS) == "REC 00:00:05" && _rs.TaskSet && _rs.TaskName == "pick cup" && _rs.EpisodeName == "episode_test", $"label '{VlaStatusRule.Label(_rs.Look, _rs.ElapsedS)}', task '{_rs.TaskName}'");
        Check(!_rs.ToastShown, "STARTED shows no toast");

        // (c) elapsed keeps counting between heartbeats
        yield return Wait(1.2);
        Check(_rs.ElapsedS >= 6.1f && VlaStatusRule.Label(_rs.Look, _rs.ElapsedS) == "REC 00:00:06", $"after 1.2 s: ElapsedS {_rs.ElapsedS:F2}, '{VlaStatusRule.Label(_rs.Look, _rs.ElapsedS)}'");

        // (d) paused: frozen
        Inject(2, 2, 6f);
        yield return Wait(1.5);
        Inject(2, 0, 6f);   // heartbeat keeps it alive
        yield return Wait(0.2);
        Check(_rs.ElapsedS == 6f && _rs.Look == VlaStatusRule.Look.Paused, $"PAUSED: ElapsedS frozen at {_rs.ElapsedS:F2}, Look {_rs.Look}");

        // (e) toasts
        Inject(0, 4, 6f);
        Check(_rs.ToastShown && _rs.ToastText == "Saved · 00:00:06" && _rs.Look == VlaStatusRule.Look.Idle, $"SAVED: toast '{_rs.ToastText}', Look {_rs.Look}");
        uint seqSaved = _rs.EventSeq;
        Inject(0, 0, 6f);   // heartbeat with the same seq must not re-toast later
        yield return Wait(4.5);
        Inject(0, 0, 6f);
        Check(!_rs.ToastShown && _rs.EventSeq == seqSaved, "toast gone after 4 s, heartbeat does not bring it back");
        Inject(0, 5, 2f, "too_short");
        Check(_rs.ToastShown && _rs.ToastText == "Discarded: too short", $"DISCARDED: '{_rs.ToastText}'");
        Inject(4, 7, 2f, "demo error");
        Check(_rs.Look == VlaStatusRule.Look.Error && _rs.ToastText == "Error: demo error", $"ERROR: Look {_rs.Look}, toast '{_rs.ToastText}'");

        // (f) feed stops: badge vanishes
        yield return Wait(3.5);
        Check(_rs.Look == VlaStatusRule.Look.Hidden && !_rs.Available && _rs.HasMessage, $"stale: Look {_rs.Look}, Available {_rs.Available}");

        // (g) one connection
        Check(Object.FindObjectsByType<ROSConnection>(FindObjectsSortMode.None).Length == 1, "exactly one ROSConnection in the scene");

        foreach (var w in UiChecks(hud)) yield return w;
        foreach (var w in DeployChecks(hud)) yield return w;

        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Wait(0.1);
        PlayerPrefs.DeleteAll();
        PlayerPrefs.Save();
    }
}
