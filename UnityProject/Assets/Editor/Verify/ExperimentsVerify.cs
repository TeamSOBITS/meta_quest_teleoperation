// Verify harness: Experiments batch 1: latest-frame decode, RTT, status strip, Experiments panel, key light.
// Needs the live sim (HOME) on 127.0.0.1:10000 and no other ROS client (stop the app on the headset).
// Run: tools/verify.sh --suite ExperimentsVerify   (or Unity -batchmode -projectPath <copy> -executeMethod ExperimentsVerify.Run)
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reflection;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Tf2;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.UI;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;

// Copy-only harness: teleoperation experiments batch 1 (latest-frame decode, RTT, status strip,
// Experiments panel, key light). Live against the sim; never edits project code.
public static class ExperimentsVerify
{
    static IEnumerator _run;
    static int _failures, _passes;
    static string ShotDir
    {
        get
        {
            var d = Environment.GetEnvironmentVariable("EXP_SHOTS");
            if (string.IsNullOrEmpty(d)) d = Path.GetFullPath(Path.Combine(Application.dataPath, "../../../exp_shots"));
            Directory.CreateDirectory(d);
            return d;
        }
    }
    static string PubScript => VerifyPaths.Tool("pub.sh");

    public static void Run()
    {
        EditorSettings.enterPlayModeOptionsEnabled = true;
        EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.DisableDomainReload;
        _run = Main();
        EditorApplication.update += Tick;
        EditorApplication.update += InjectTick;
    }

    static double _waitUntil; static int _waitFrame;
    static void Tick()
    {
        if (EditorApplication.timeSinceStartup < _waitUntil) return;
        if (Application.isPlaying && Time.frameCount < _waitFrame) return;
        try
        {
            if (!_run.MoveNext())
            {
                EditorApplication.update -= Tick;
                Log(_failures == 0 ? $"ALL CHECKS PASSED ({_passes})" : $"{_failures} CHECK(S) FAILED, {_passes} passed");
                EditorApplication.Exit(_failures == 0 ? 0 : 1);
            }
        }
        catch (Exception e)
        {
            Debug.LogException(e);
            Log("FAIL exception " + e.Message);
            EditorApplication.Exit(3);
        }
    }
    static object Frames(int n) { _waitFrame = Time.frameCount + n; _waitUntil = EditorApplication.timeSinceStartup + 0.05; return null; }
    static object Seconds(double s) { _waitUntil = EditorApplication.timeSinceStartup + s; return null; }
    static void Log(string m) => Debug.Log("[Verify] " + m);
    static bool Check(bool ok, string what) { if (ok) _passes++; else _failures++; Log((ok ? "PASS " : "FAIL ") + what); return ok; }
    static T Field<T>(object o, string name) => (T)o.GetType().GetField(name, BindingFlags.NonPublic | BindingFlags.Instance).GetValue(o);

    // ---------------- ROS helpers ----------------
    static readonly Dictionary<string, double> _q = new Dictionary<string, double>();
    static int _jsCount;
    static void OnJointStates(JointStateMsg m)
    {
        if (m?.name == null || m.position == null) return;
        _jsCount++;
        for (int i = 0; i < m.name.Length && i < m.position.Length; i++) _q[m.name[i]] = m.position[i];
    }
    static double Q(string j) => _q.TryGetValue(j, out var v) ? v : double.NaN;

    static int _hmdEchoes;
    static void OnTf(TFMessageMsg m)
    {
        if (m?.transforms == null) return;
        foreach (var t in m.transforms) if (t != null && t.child_frame_id == RobotProfile.Selected?.controllerFrames.hmd) _hmdEchoes++;
    }

    // Latest head-camera message seen by the harness (same callback order as ImageSubscriber, which subscribed first).
    static CompressedImageMsg _latest, _prevLatest;
    static int _harnessMsgs;
    static readonly List<CompressedImageMsg> _ring = new List<CompressedImageMsg>();
    static void OnHeadImage(CompressedImageMsg m) { if (m == null) return; _prevLatest = _latest; _latest = m; _harnessMsgs++; _ring.Insert(0, m); if (_ring.Count > 12) _ring.RemoveAt(12); }

    // hmd_odom injection (the Editor has no XR head device, so the publisher sends nothing).
    static bool _inject; static double _nextInject; static int _injected;
    static void InjectTick()
    {
        if (!_inject || !Application.isPlaying) return;
        if (EditorApplication.timeSinceStartup < _nextInject) return;
        _nextInject = EditorApplication.timeSinceStartup + 1.0 / 30.0;
        long ns = (DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).Ticks * 100;
        var t = new TransformStampedMsg
        {
            header = new HeaderMsg { frame_id = RobotProfile.Selected?.baseFrame, stamp = new TimeMsg { sec = (int)(ns / 1_000_000_000), nanosec = (uint)(ns % 1_000_000_000) } },
            child_frame_id = RobotProfile.Selected?.controllerFrames.hmd,
            transform = new TransformMsg { translation = new Vector3Msg(0, 0, 1.2), rotation = new QuaternionMsg(0, 0, 0, 1) }
        };
        ROSConnection.GetOrCreateInstance().Publish(RosNames.Tf, new TFMessageMsg(new[] { t }));
        _injected++;
    }

    static Process Pub(string args)
    {
        Log($"    pub {args}");
        return Process.Start(new ProcessStartInfo("/bin/bash", $"\"{PubScript}\" {args}") { UseShellExecute = false, CreateNoWindow = true });
    }

    static IEnumerator Settle(Process p, Dictionary<string, double> targets, double timeout = 25)
    {
        double end = EditorApplication.timeSinceStartup + timeout;
        while (EditorApplication.timeSinceStartup < end)
        {
            bool done = (p == null || p.HasExited) && targets.All(t => Math.Abs(Q(t.Key) - t.Value) < (t.Key.Contains("lift") ? 0.003 : 0.01));
            if (done) break;
            yield return Seconds(0.2);
        }
        Log($"    settle: {string.Join(", ", targets.Select(t => $"{t.Key} q={Q(t.Key):F4} target={t.Value}"))}");
    }

    static string StripText(StatusStrip s) => s == null ? "" : string.Join(" | ", s.GetComponentsInChildren<TextMeshProUGUI>(true).Select(t => t.text));

    // ---------------- Main ----------------
    static IEnumerator Main()
    {
        Log("===== experiments batch 1");
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.1");
        PlayerPrefs.SetInt("RobotModel/SOBIT_HOME", 1); PlayerPrefs.SetString("CameraLayout/SOBIT_HOME", "firstperson");
        PlayerPrefs.Save();

        var pre = new[] { Pub("head 0.0 0.0"), Pub("lift 0.2"), Pub("armhome") };
        while (pre.Any(p => !p.HasExited)) yield return Seconds(0.2);
        yield return Seconds(2.0);

        var profile = AssetDatabase.LoadAssetAtPath<RobotProfile>("Assets/Robots/SOBIT_HOME.asset");
        EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
        RobotProfile.Selected = profile;
        Object.FindFirstObjectByType<QuestControllerPublisher>().controlRobot = false;
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Seconds(0.1);
        var publisher = Object.FindFirstObjectByType<QuestControllerPublisher>();
        publisher.controlRobot = false;
        yield return Frames(2);

        var ros = ROSConnection.GetOrCreateInstance();
        Check(Object.FindObjectsByType<ROSConnection>(FindObjectsSortMode.None).Length == 1, "exactly one ROSConnection");
        ros.Subscribe<JointStateMsg>("/sobit_home/joint_states", OnJointStates);
        ros.Subscribe<TFMessageMsg>(RosNames.Tf, OnTf);
        var hud = Object.FindFirstObjectByType<TeleopHud>();
        var images = Object.FindFirstObjectByType<ImageSubscriber>();

        FirstPersonView fpv = null; RobotModel model = null;
        double end = EditorApplication.timeSinceStartup + 40;
        while (EditorApplication.timeSinceStartup < end)
        {
            fpv = Object.FindFirstObjectByType<FirstPersonView>();
            model = fpv != null ? fpv.Model : null;
            if (model != null && model.AcceptedTransforms > 0 && fpv.FramesReceived > 0 && _jsCount > 0) break;
            yield return Seconds(0.25);
        }
        if (!Check(hud.FirstPerson && model != null && model.AcceptedTransforms > 0 && fpv.FramesReceived > 0 && _jsCount > 0,
                   $"first person live: TF {model?.AcceptedTransforms}, frames {fpv?.FramesReceived}, js {_jsCount}, connErr {ros.HasConnectionError}"))
            yield break;
        yield return Seconds(1.0);

        // ---- 5. key light (teleop with model)
        var key = GameObject.Find("Model Key Light");
        Check(key != null && key.GetComponent<Light>() != null && key.GetComponent<Light>().type == LightType.Directional,
              $"5: 'Model Key Light' directional light exists in TeleopScene (intensity {key?.GetComponent<Light>()?.intensity})");
        Check(Object.FindObjectsByType<Light>(FindObjectsSortMode.None).Count(l => l.name == "Model Key Light") == 1, "5: exactly one key light");

        // ---- 4a. panel follows the hidden bar in first person
        var barT = FindInactive("HUD Bar");
        Check(barT != null && !barT.gameObject.activeInHierarchy, "4: HUD bar hidden in first person");
        // round 3: Experiments panel removed -> its checks dropped

        // ---- 1. latest-frame decode, head camera
        int idx = fpv.CameraIndex;
        Check(idx >= 0, $"1: head camera index {idx}");
        string headTopic = images.Panels[idx].Topic;
        ros.Subscribe<CompressedImageMsg>(headTopic, OnHeadImage);
        var shownTimes = new List<float>();
        var lagDiffs = new List<double>();
        int newestOk = 0, newestChecked = 0, discriminating = 0;
        bool checkNewest = false;
        var tmp = new Texture2D(2, 2, TextureFormat.RGB24, false);
        var matchPos = new List<int>();
        Action<int, Texture2D> onFrame = (i, tex) =>
        {
            if (i != idx) return;
            shownTimes.Add(Time.unscaledTime);
            lagDiffs.Add(Time.unscaledTime - images.LastFrameTime(idx));
            if (!checkNewest || _latest == null) return;
            newestChecked++;
            var shown = tex.GetPixels32();
            int pos = -1;
            for (int k = 0; k < _ring.Count && pos < 0; k++)
            {
                tmp.LoadImage(_ring[k].data);
                if (tmp.width == tex.width && tmp.height == tex.height && tmp.GetPixels32().SequenceEqual(shown)) pos = k;
            }
            matchPos.Add(pos);
            if (pos == 0) newestOk++;
            if (_prevLatest != null && !_prevLatest.data.SequenceEqual(_latest.data)) discriminating++;
        };
        images.FrameReady += onFrame;

        int d0 = images.DroppedFrames(idx); int h0 = _harnessMsgs; shownTimes.Clear();
        yield return Seconds(5.0);
        int d1 = images.DroppedFrames(idx); int shown15 = shownTimes.Count; int recv15 = _harnessMsgs - h0;
        Check(shown15 > 0, $"1: maxFps 15: {shown15} frames shown in 5 s ({recv15} received by harness), dropped {d0}->{d1}, fps {images.Fps(idx):F1}, decode {images.DecodeMs(idx):F2} ms");

        var cfg = images.Panels[idx].Config;
        float oldFps = cfg.maxFps;
        cfg.maxFps = 2f;
        yield return Seconds(1.0);
        shownTimes.Clear(); lagDiffs.Clear(); checkNewest = true; h0 = _harnessMsgs;
        int d2 = images.DroppedFrames(idx);
        yield return Seconds(6.0);
        checkNewest = false;
        int d3 = images.DroppedFrames(idx); int recv2 = _harnessMsgs - h0;
        var intervals = new List<float>();
        for (int i = 1; i < shownTimes.Count; i++) intervals.Add(shownTimes[i] - shownTimes[i - 1]);
        float meanIv = intervals.Count > 0 ? intervals.Average() : -1f;
        Check(d3 > d2, $"1: maxFps 2: dropped grows {d2}->{d3} (+{d3 - d2}) with {recv2} received, {shownTimes.Count} shown in 6 s");
        Check(meanIv > 0.45f && meanIv < 0.65f, $"1: FrameReady interval mean {meanIv:F3} s (min {(intervals.Count > 0 ? intervals.Min() : -1):F3}, max {(intervals.Count > 0 ? intervals.Max() : -1):F3}) ~ 1/maxFps = 0.5");
        Check(lagDiffs.Count > 0 && lagDiffs.Max() < 0.05, $"1: unscaledTime - LastFrameTime at FrameReady max {(lagDiffs.Count > 0 ? lagDiffs.Max() : -1):F4} s < 0.05");
        Check(newestChecked > 0 && newestOk == newestChecked, $"1: shown frame == newest received message {newestOk}/{newestChecked} (consecutive msgs differ in {discriminating}/{newestChecked}); match position in received ring (0 = newest, -1 none): [{string.Join(",", matchPos)}]");
        Log($"    decode ms head: {images.DecodeMs(idx):F2}; Fps(head) at maxFps 2: {images.Fps(idx):F2}");
        cfg.maxFps = oldFps;
        images.FrameReady -= onFrame;

        // ---- 3. status strip
        var strip = Object.FindFirstObjectByType<StatusStrip>();
        Check(strip != null && strip.transform.parent == FirstPersonView.Head, $"3: status strip exists under the head ({strip?.transform.parent?.name})");
        var p = Pub("head 0.5 0.3");
        var e = Settle(p, new Dictionary<string, double> { ["head_pan_joint"] = 0.5, ["head_tilt_joint"] = 0.3 }); while (e.MoveNext()) yield return e.Current;
        yield return Seconds(3.0);
        Check(Mathf.Abs(strip.PanRad - 0.5f) < 0.03f, $"3: PanRad {strip.PanRad:F4} ~ 0.5 (q pan {Q("head_pan_joint"):F4})");
        Check(Mathf.Abs(strip.TiltRad - 0.3f) < 0.03f, $"3: TiltRad {strip.TiltRad:F4} ~ 0.3 (q tilt {Q("head_tilt_joint"):F4})");
        var dot = Field<RectTransform>(strip, "_headDot");
        Check(dot.anchoredPosition.x < -5f && dot.anchoredPosition.y > 5f, $"3: HEAD dot at ({dot.anchoredPosition.x:F1}, {dot.anchoredPosition.y:F1}) mm: left for pan +0.5, up for tilt +0.3");
        yield return Seconds(0.5);
        string ht = Field<TextMeshProUGUI>(strip, "_headText").text.Replace("\n", " ");
        int panDeg = Mathf.RoundToInt(strip.PanRad * Mathf.Rad2Deg), tiltDeg = Mathf.RoundToInt(strip.TiltRad * Mathf.Rad2Deg);
        Check(ht.Contains($"pan {panDeg}\u00B0") && ht.Contains($"tilt {tiltDeg}\u00B0") && Math.Abs(panDeg - 29) <= 1 && Math.Abs(tiltDeg - 17) <= 1, $"3: head text '{ht}' (pan {panDeg} ~ 29, tilt {tiltDeg} ~ 17)");
        for (int attempt = 0; attempt < 3 && Math.Abs(Q("body_lift_joint") - 0.4) > 0.005; attempt++)
        {
            p = Pub("lift 0.4");
            e = Settle(p, new Dictionary<string, double> { ["body_lift_joint"] = 0.4 }, 12); while (e.MoveNext()) yield return e.Current;
        }
        yield return Seconds(1.0);
        Check(Field<TextMeshProUGUI>(strip, "_liftText").text == "0.40 m", $"3: lift text '{Field<TextMeshProUGUI>(strip, "_liftText").text}' == '0.40 m'");
        {
            var labels = strip.GetComponentsInChildren<TextMeshProUGUI>(true).Where(t => t.gameObject.activeInHierarchy).Select(t => t.text).ToList();
            Check(labels.Contains("HEAD") && labels.Contains("LIFT"), $"3: strip has 'HEAD' and 'LIFT' labels");
            var srt = (RectTransform)strip.transform;
            Log($"    strip size {srt.sizeDelta.x:F0} x {srt.sizeDelta.y:F0} mm");
            Check(srt.sizeDelta.x > 1000f && srt.sizeDelta.x <= 1500f && Mathf.Abs(srt.sizeDelta.y - 130f) < 1f, $"3: strip 1000..1500 x 130 mm, three groups ({srt.sizeDelta.x:F0} x {srt.sizeDelta.y:F0})");
            // nothing in the strip spills out of it (label clipping)
            var sc = new Vector3[4]; srt.GetWorldCorners(sc);
            var spill = new List<string>();
            foreach (var t in strip.GetComponentsInChildren<TextMeshProUGUI>(false))
            {
                if (string.IsNullOrEmpty(t.text)) continue;
                var b = t.textBounds; var mn = srt.InverseTransformPoint(t.transform.TransformPoint(b.min)); var mx = srt.InverseTransformPoint(t.transform.TransformPoint(b.max));
                var r = srt.rect;
                if (mn.x < r.xMin - 1f || mx.x > r.xMax + 1f || mn.y < r.yMin - 1f || mx.y > r.yMax + 1f) spill.Add($"{t.name}:'{t.text}'");
            }
            Check(spill.Count == 0, $"3: all strip texts inside the strip {string.Join(", ", spill)}");
        }
        Check(Mathf.Abs(strip.LiftM - 0.4f) < 0.01f, $"3: LiftM {strip.LiftM:F4} ~ 0.4 (q lift {Q("body_lift_joint"):F4}); fill {Field<RectTransform>(strip, "_liftFill").sizeDelta.y:F1} mm");
        yield return Seconds(0.5);
        string txtOff = StripText(strip);
        Log("    strip text (control off): " + txtOff);
        Check(txtOff.Contains("LAYOUT") && !txtOff.Contains("CONTROL ON"), "3: 'LAYOUT' with control off");
        Check(txtOff.Contains("fps") && txtOff.Contains("Hz"), "3: 'fps' and 'Hz' present");
        var head = FirstPersonView.Head;
        {
            var sc = new Vector3[4]; ((RectTransform)strip.transform).GetWorldCorners(sc);
            Vector3 sCentre = (sc[0] + sc[2]) / 2f;
            Shot("03_status_strip", head.position, Quaternion.LookRotation(sCentre - head.position, Vector3.up), 30f);
        }

        // round 3: the strip is always on (no "status" experiment toggle) -> off/on checks dropped

        // ---- 5. key light shot: arms
        p = Pub("armleft");
        e = Settle(p, new Dictionary<string, double> { ["arm_left_elbow_joint"] = 2.5 }); while (e.MoveNext()) yield return e.Current;
        yield return Seconds(1.0);
        // 10_key_light is shot by ExperimentsVerify2 (hand cams off, aimed at the grippers)

        // ---- 2. RTT
        var rtt = Object.FindFirstObjectByType<RoundTrip>();
        Check(rtt != null, "2: RoundTrip exists (rtt on by default)");
        publisher.controlRobot = true;
        int echo0 = _hmdEchoes;
        yield return Seconds(2.0);
        int ownEchoes = _hmdEchoes - echo0;
        Log($"    with control on, hmd_odom echoes from the Editor publisher in 2 s: {ownEchoes}, RTT samples {rtt.Samples}");
        if (rtt.Samples <= 10)
        {
            Log("    Editor has no XR head device -> injecting hmd_odom TF (wall-clock stamp) at 30 Hz through the same ROSConnection");
            _inject = true;
        }
        int s0 = rtt.Samples;
        end = EditorApplication.timeSinceStartup + 5;
        while (EditorApplication.timeSinceStartup < end && rtt.Samples - s0 <= 10) yield return Seconds(0.1);
        yield return Seconds(1.0);
        Check(rtt.Samples - s0 > 10, $"2: RTT samples {rtt.Samples - s0} > 10 within 5 s (injected {_injected})");
        Check(rtt.RttMs > 0f && rtt.RttMs < 300f, $"2: RttMs {rtt.RttMs:F2} in (0, 300); RttMaxMs {rtt.RttMaxMs:F2}; HasRecent {rtt.HasRecent}");
        yield return Seconds(0.5);
        string txtOn = StripText(strip);
        Log("    strip text (control on): " + txtOn);
        Check(txtOn.Contains("CONTROL ON"), "3: 'CONTROL ON' with control on");
        Check(txtOn.Contains("ms") && txtOn.Contains("fps") && txtOn.Contains("Hz"), "3: 'ms', 'fps', 'Hz' present with control on");
        Shot("02_rtt_strip", head.position, head.rotation, 70f);
        publisher.controlRobot = false;
        _inject = false;
        double offAt = EditorApplication.timeSinceStartup;
        while (rtt.HasRecent && EditorApplication.timeSinceStartup - offAt < 3.5) yield return Seconds(0.05);
        double took = EditorApplication.timeSinceStartup - offAt;
        Check(!rtt.HasRecent && took <= 3.0, $"2: control off -> HasRecent false after {took:F2} s (<= 3)");

        // ---- blocks mode
        var dropsBefore = images.Panels.Select((_, i) => images.DroppedFrames(i)).ToArray();
        hud.SetRobotModel(false);
        yield return Frames(3);
        var counts = new int[images.Panels.Count];
        Action<int, Texture2D> count = (i, _) => { if (i < counts.Length) counts[i]++; };
        images.FrameReady += count;
        int hb = _harnessMsgs;
        yield return Seconds(5.0);
        float srcHz = (_harnessMsgs - hb) / 5f;
        float expect = Mathf.Min(srcHz, images.Panels[idx].Config.maxFps);
        Log($"    head source rate {srcHz:F1} Hz -> expected badge ~{expect:F0} fps");
        images.FrameReady -= count;
        for (int i = 0; i < images.Panels.Count; i++)
        {
            var badge = Field<CameraBadge>(images.Panels[i], "_badge").Text;   // round 2: badge moved into CameraBadge
            int n = 0; int.TryParse(new string(badge.TakeWhile(char.IsDigit).ToArray()), out n);
            Check(counts[i] > 0 && images.Panels[i].Visible, $"1: blocks: cam {i} '{images.Panels[i].Label}' shows frames ({counts[i]} in 5 s), dropped {dropsBefore[i]}->{images.DroppedFrames(i)}, decode {images.DecodeMs(i):F2} ms");
            Check(badge.EndsWith("fps") && Mathf.Abs(n - expect) <= 3f, $"1: blocks: cam {i} badge '{badge}' ~{expect:F0} fps (Fps() {images.Fps(i):F1})");
        }

        // ---- Rename button (layout mode = control off): above each card's top-right corner, clear of the name
        {
            var renames = new List<RectTransform>();
            foreach (var pp in images.Panels.Where(pp => pp.Visible))
            {
                var rn = Field<UnityEngine.UI.Button>(pp, "_rename"); var nm = Field<TextMeshProUGUI>(pp, "_name");
                var prt = (RectTransform)pp.transform; var rrt = (RectTransform)rn.transform;
                Rect WR(RectTransform r) { var c4 = new Vector3[4]; r.GetWorldCorners(c4); var a = c4.Select(w => prt.InverseTransformPoint(w)).ToArray(); return Rect.MinMaxRect(a.Min(v => v.x), a.Min(v => v.y), a.Max(v => v.x), a.Max(v => v.y)); }
                Rect rb = WR(rrt), nb = WR(nm.rectTransform), cb = prt.rect;
                renames.Add(rrt);
                Check(rn.gameObject.activeInHierarchy, $"rename: '{pp.Label}' Rename shown with control off");
                Check(!rb.Overlaps(nb), $"rename: '{pp.Label}' Rename {rb} does not intersect the name label {nb} (mm, panel space)");
                Check(rb.yMin >= cb.yMax - 0.01f && Mathf.Abs(rb.xMax - cb.xMax) < 1f, $"rename: '{pp.Label}' Rename bottom {rb.yMin:F1} >= card top {cb.yMax:F1}, right {rb.xMax:F1} == card right {cb.xMax:F1}");
            }
            Shot("14_rename_button", Fit(head, images.Panels.Where(pp => pp.Visible).Select(x => (RectTransform)x.transform).Concat(renames), 0f, out var fovR), fovR);
        }

        // ---- 4. panel in blocks mode
        barT = FindInactive("HUD Bar");
        Check(barT.gameObject.activeInHierarchy, "4: bar active in blocks mode");
        // round 3: Experiments panel / registry / headlock / status toggles removed -> those checks dropped
        {
            var hs = Resources.FindObjectsOfTypeAll<StatusStrip>().FirstOrDefault(x => x.gameObject.scene.IsValid());
            Check(hs != null && !hs.gameObject.activeSelf, "3: blocks mode: strip exists, inactive while the bar is shown");
        }

        Shot("01_latest_frame", Fit(head, images.Panels.Select(x => (RectTransform)x.transform), 0f, out var fov1), fov1);
        // 0.7 bar in blocks mode: scale, no camera block overlaps it (LayoutVerifier's projected separating-axis test)
        Check(Vector3.Distance(barT.localScale, Vector3.one * (HudBar.CompactScale / HudUi.MmPerMetre)) < 1e-6f, $"bar: blocks mode scale {barT.localScale.x * 1000f:F3}/1000 == 0.7/1000; BarLowered {hud.BarLowered}");
        Check(!hud.BarLowered, "bar: not lowered in blocks mode");
        {
            var eye = images.panelParent;
            var quads = images.Panels.Where(pp => pp.Visible).Select(pp => (pp.Label, Project((RectTransform)pp.transform, eye))).ToList();
            var barQ = Project((RectTransform)barT, eye);
            var bc = new Vector3[4]; ((RectTransform)barT).GetWorldCorners(bc);
            float barTop = eye.InverseTransformPoint(bc[1]).y;
            float lowest = images.Panels.Where(pp => pp.Visible).Min(pp => { var c4 = new Vector3[4]; ((RectTransform)pp.transform).GetWorldCorners(c4); return c4.Min(w => eye.InverseTransformPoint(w).y); });
            var hits = quads.Where(q => Separation(q.Item2, barQ) <= 0f).Select(q => q.Label).ToList();
            float minGap = quads.Min(q => Separation(q.Item2, barQ));
            Check(hits.Count == 0, $"bar: no camera block overlaps the 0.7 bar (min projected gap {minGap:F4}; lowest block bottom y {lowest:F3} m, bar top y {barTop:F3} m, minBottom {images.minBottom}) {string.Join(",", hits)}");
        }
        Shot("10_experiments_panel", Fit(head, images.Panels.Where(pp => pp.Visible).Select(x => (RectTransform)x.transform).Concat(new[] { (RectTransform)barT }), 0f, out var fov2), fov2);


        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Seconds(0.1);

        // ---- 5. selection scene: no key light
        EditorSceneManager.OpenScene("Assets/Scenes/RobotSelectionScene.unity");
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Seconds(0.1);
        yield return Seconds(3.0);
        Check(GameObject.Find("Model Key Light") == null, $"5: no key light in RobotSelectionScene (lights: {string.Join(", ", Object.FindObjectsByType<Light>(FindObjectsSortMode.None).Select(l => l.name))})");
        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Seconds(0.1);
    }

    static Vector2[] Project(RectTransform rt, Transform head)
    {
        var c = new Vector3[4]; rt.GetWorldCorners(c);
        return c.Select(w => { var v = head.InverseTransformPoint(w); return new Vector2(v.x / v.z, v.y / v.z); }).ToArray();
    }

    static float Separation(Vector2[] a, Vector2[] b)
    {
        float best = float.NegativeInfinity;
        foreach (var poly in new[] { a, b })
            for (int i = 0; i < poly.Length; i++)
            {
                var e = poly[(i + 1) % poly.Length] - poly[i];
                var n = new Vector2(-e.y, e.x).normalized;
                float aMin = a.Min(p => Vector2.Dot(p, n)), aMax = a.Max(p => Vector2.Dot(p, n));
                float bMin = b.Min(p => Vector2.Dot(p, n)), bMax = b.Max(p => Vector2.Dot(p, n));
                best = Mathf.Max(best, Mathf.Max(bMin - aMax, aMin - bMax));
            }
        return best;
    }

    static Transform FindInactive(string name)
        => Resources.FindObjectsOfTypeAll<Transform>().FirstOrDefault(t => t.name == name && t.gameObject.scene.IsValid());

    // Camera pose looking from the head (moved `back` metres) at the union of the rects, with a FOV that fits them.
    static Pose Fit(Transform head, IEnumerable<RectTransform> rects, float back, out float fov)
    {
        var pts = new List<Vector3>(); var c = new Vector3[4];
        foreach (var r in rects) { r.GetWorldCorners(c); pts.AddRange(c); }
        Vector3 centre = pts.Aggregate(Vector3.zero, (a, b) => a + b) / pts.Count;
        Vector3 pos = head.position - (centre - head.position).normalized * back;
        var rot = Quaternion.LookRotation(centre - pos, Vector3.up);
        float maxV = 0f, maxH = 0f;
        foreach (var pt in pts)
        {
            var l = Quaternion.Inverse(rot) * (pt - pos);
            maxV = Mathf.Max(maxV, Mathf.Abs(Mathf.Atan2(l.y, l.z)) * Mathf.Rad2Deg);
            maxH = Mathf.Max(maxH, Mathf.Abs(Mathf.Atan2(l.x, l.z)) * Mathf.Rad2Deg);
        }
        float hHalf = Mathf.Atan(Mathf.Tan(maxH * Mathf.Deg2Rad) * 9f / 16f) * Mathf.Rad2Deg;
        fov = Mathf.Clamp(2f * Mathf.Max(maxV, hHalf) * 1.12f, 30f, 110f);
        return new Pose(pos, rot);
    }

    static void Shot(string name, Pose pose, float fov) => Shot(name, pose.position, pose.rotation, fov);

    static void Shot(string name, Vector3 pos, Quaternion rot, float fov)
    {
        var go = new GameObject("VerifyCam");
        var cam = go.AddComponent<Camera>();
        go.transform.SetPositionAndRotation(pos, rot);
        var main = Camera.main;
        cam.fieldOfView = fov; cam.aspect = 16f / 9f;
        cam.nearClipPlane = 0.05f; cam.farClipPlane = 200f;
        cam.stereoTargetEye = StereoTargetEyeMask.None;
        if (main != null) { cam.clearFlags = main.clearFlags; cam.backgroundColor = main.backgroundColor; }
        var rt = new RenderTexture(1280, 720, 24);
        cam.targetTexture = rt;
        cam.Render();
        RenderTexture.active = rt;
        var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;
        File.WriteAllBytes(Path.Combine(ShotDir, name + ".png"), tex.EncodeToPNG());
        cam.targetTexture = null; Object.DestroyImmediate(rt); Object.DestroyImmediate(go); Object.DestroyImmediate(tex);
        Log($"    screenshot {name}.png (fov {fov:F1})");
    }
}
