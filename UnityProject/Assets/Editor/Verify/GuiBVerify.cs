// Verify harness: GUI improvements batch B: arc layout of the camera blocks, text size + high contrast (Display row of the
// selection screen), startup control countdown. No sim needed (the ROS IP is isolated).
// Run: tools/verify.sh --suite GuiBVerify   (or Unity -batchmode -projectPath <copy> -executeMethod GuiBVerify.Run)
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using TMPro;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.UI;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;

public static class GuiBVerify
{
    static IEnumerator _run;
    static int _failures, _passes;
    static readonly List<string> _controlLogs = new List<string>();
    static readonly List<(string text, float time, bool controlRobot)> _controlEvents = new List<(string, float, bool)>();
    static bool _cancelled;   // the callback turned control off (the "cancelled" line it provokes is written inside the callback and not delivered to it)
    static Action<string> _onControlLog;   // runs inside the log callback, on the main thread, in the frame the line is written
    static string ShotDir
    {
        get
        {
            var d = Environment.GetEnvironmentVariable("EXP_SHOTS");
            return Directory.CreateDirectory(string.IsNullOrEmpty(d) ? Path.GetFullPath(Path.Combine(Application.dataPath, "../../../exp_shots_guib")) : d).FullName;
        }
    }

    public static void Run()
    {
        EditorSettings.enterPlayModeOptionsEnabled = true;
        EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.DisableDomainReload;
        _run = Main();
        EditorApplication.update += Tick;
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
    static double Now => EditorApplication.timeSinceStartup;
    static void Log(string m) => Debug.Log("[Verify] " + m);
    static bool Check(bool ok, string what) { if (ok) _passes++; else _failures++; Log((ok ? "PASS " : "FAIL ") + what); return ok; }
    static IEnumerable Until(Func<bool> cond, double seconds) { double end = Now + seconds; while (Now < end && !cond()) yield return Seconds(0.05); }

    static IEnumerable EnterPlay()
    {
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Seconds(0.1);
    }
    static IEnumerable ExitPlay()
    {
        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Seconds(0.1);
        yield return Seconds(0.5);
    }
    static RobotProfile Profile(string robot) => AssetDatabase.LoadAssetAtPath<RobotProfile>($"Assets/Robots/{robot}.asset");

    static IEnumerator Main()
    {
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.2");   // isolated: never reach a live ros_tcp_endpoint
        PlayerPrefs.Save();
        HudTheme.Reload();
        Application.logMessageReceived += (c, s, t) =>
        {
            if (!c.StartsWith("[Control]")) return;
            _controlLogs.Add(c);
            var pub = Object.FindFirstObjectByType<QuestControllerPublisher>();
            _controlEvents.Add((c, Time.unscaledTime, pub != null && pub.controlRobot));
            _onControlLog?.Invoke(c);
        };

        StaticChecks();
        foreach (var robot in new[] { "SOBIT_HOME", "SOBIT_LIGHT" })
            foreach (var w in ArcAndCountdown(robot)) yield return w;
        foreach (var w in DisplaySettings()) yield return w;
        PlayerPrefs.DeleteAll();
        PlayerPrefs.Save();
        HudTheme.Reload();
    }

    // ---------------- Settings and theme without a scene ----------------
    static void StaticChecks()
    {
        Check(Settings.TextScale == 1f && !Settings.HighContrast && HudTheme.FontScale == 1f && !HudTheme.HighContrast, "defaults: TextScale 1, HighContrast off");
        Check(Mathf.Approximately(HudTheme.BodyFont, HudTheme.BodyFontBase) && Mathf.Approximately(HudTheme.CardAlpha, 0.92f) && Mathf.Approximately(HudTheme.Panel.a, 0.92f), "default theme: fonts = design sizes, panel alpha 0.92");
        float mutedA = HudTheme.Muted.a, dividerA = HudTheme.Divider.a;
        HudTheme.SetTextScale(5f); Check(Mathf.Approximately(Settings.TextScale, 1.3f), $"TextScale clamps to 1.3 ({Settings.TextScale})");
        HudTheme.SetTextScale(0.1f); Check(Mathf.Approximately(Settings.TextScale, 0.8f), $"TextScale clamps to 0.8 ({Settings.TextScale})");
        int changed = 0; Action onChanged = () => changed++;
        HudTheme.Changed += onChanged;
        HudTheme.SetTextScale(1.3f); HudTheme.SetHighContrast(true);
        HudTheme.Changed -= onChanged;
        Check(changed == 2 && Mathf.Approximately(HudTheme.FontScale, 1.3f) && Mathf.Approximately(HudTheme.TitleFont, HudTheme.TitleFontBase * 1.3f) && Mathf.Approximately(HudTheme.BodyFont, HudTheme.BodyFontBase * 1.3f),
              $"TextScale 1.3: fonts x1.3, HudTheme.Changed raised twice ({changed})");
        Check(Mathf.Approximately(HudTheme.CardAlpha, 1f) && Mathf.Approximately(HudTheme.Panel.a, 1f) && HudTheme.Muted.a > mutedA && HudTheme.Divider.a > dividerA,
              $"HighContrast: opaque panel, stronger muted text ({mutedA:F2} -> {HudTheme.Muted.a:F2}) and divider ({dividerA:F2} -> {HudTheme.Divider.a:F2})");
        var canvas = HudUi.CreateCanvas("probe", null, Vector3.forward, new Vector2(100, 100), false);
        var label = HudUi.Label(canvas, "L", "x", HudTheme.BodyFont * HudUi.MmPerMetre);
        Check(Mathf.Approximately(label.fontSize, 90f * 1.3f), $"a label built from the theme font has fontSize {label.fontSize:F1} (= 1.3 x 90)");
        Object.DestroyImmediate(canvas.gameObject);
        HudTheme.SetTextScale(1f); HudTheme.SetHighContrast(false);
        Check(HudTheme.FontScale == 1f && !HudTheme.HighContrast && Mathf.Approximately(HudTheme.Panel.a, 0.92f), "back to the defaults");
    }

    // ---------------- Robot screen: startup countdown + arc layout ----------------
    static IEnumerable ArcAndCountdown(string robot)
    {
        bool cancelTest = robot == "SOBIT_LIGHT";
        _controlLogs.Clear(); _controlEvents.Clear();
        // The headless Editor stalls for seconds while the scene starts, so the countdown is read from the log lines (timestamped in
        // the frame they are written) instead of polled. For the cancel test the callback turns control off in the frame the countdown starts.
        _onControlLog = null; _cancelled = false;
        if (cancelTest)
            _onControlLog = line => { if (line.Contains("countdown started")) { _onControlLog = null; _cancelled = true; Object.FindFirstObjectByType<TeleopHud>().RequestControl(false); } };
        EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
        RobotProfile.Selected = Profile(robot);
        foreach (var w in EnterPlay()) yield return w;
        foreach (var w in Until(() => _controlEvents.Count > 0, 30)) yield return w;
        var publisher = Object.FindFirstObjectByType<QuestControllerPublisher>();
        var hud = Object.FindFirstObjectByType<TeleopHud>();
        var images = Object.FindFirstObjectByType<ImageSubscriber>();
        var bar = GameObject.Find("HUD Bar");
        Check(_controlEvents.Count > 0 && _controlEvents[0].text.Contains("countdown started") && !_controlEvents[0].controlRobot,
              $"{robot}: 3: the robot screen starts a countdown with control NOT published ({(_controlEvents.Count > 0 ? _controlEvents[0].text + ", controlRobot " + _controlEvents[0].controlRobot : "no [Control] line")})");
        if (!cancelTest)
        {
            foreach (var w in Until(() => _controlEvents.Any(e => e.text.Contains("armed")), 6)) yield return w;
            var started = _controlEvents.FirstOrDefault(e => e.text.Contains("started")); var armed = _controlEvents.FirstOrDefault(e => e.text.Contains("armed"));
            float dt = armed.time - started.time;
            Check(armed.text != null && armed.controlRobot && publisher.controlRobot && dt > 1.95f && dt < 2.6f, $"{robot}: 3: controlRobot false for {dt:F2} s (2 s countdown), then true");
            yield return Seconds(1.0);
            Check(_controlLogs.Count(l => l.Contains("countdown armed")) == 1 && _controlLogs.Count(l => l.Contains("countdown started")) == 1,
                  $"{robot}: 3: [Control] countdown started once, armed once ({string.Join(" | ", _controlLogs)})");
        }
        else
        {
            yield return Seconds(3.0);
            var countdown = bar.GetComponent<HudBar>().Countdown;
            var toggle = bar.transform.Find("Toggle Control robot").GetComponent<Toggle>();
            Check(_cancelled && !publisher.controlRobot && !countdown.Counting && !toggle.isOn && countdown.LabelText == "Control robot" && !_controlLogs.Any(l => l.Contains("armed")),
                  $"{robot}: 3: turning control off during the startup countdown cancels it, nothing armed ({string.Join(" | ", _controlLogs)})");
            publisher.controlRobot = true;   // the layout checks below run as before
        }

        // ---- 1. arc ----
        var panels = images.Panels.ToList();
        float R = HudTheme.ReferenceDistance;
        var fake = new Texture2D(64, 48);
        var px = new Color[64 * 48];
        for (int y = 0; y < 48; y++) for (int x = 0; x < 64; x++) px[y * 64 + x] = Color.Lerp(new Color(0.55f, 0.6f, 0.66f), new Color(0.35f, 0.3f, 0.25f), y / 47f) * (0.8f + 0.2f * Mathf.Sin(x * 0.3f));
        fake.SetPixels(px); fake.Apply();
        foreach (var p in panels) p.SetTexture(fake);
        yield return Frames(3);
        Check(panels.All(p => Mathf.Abs(p.transform.localPosition.magnitude - R) < 1e-3f), $"{robot}: 1: every block sits on the sphere of radius {R} m (max deviation {panels.Max(p => Mathf.Abs(p.transform.localPosition.magnitude - R)):F5})");
        Check(panels.All(p => Vector3.Dot(p.transform.localRotation * Vector3.forward, p.transform.localPosition.normalized) > 0.99999f), $"{robot}: 1: every block faces the eye");
        Check(panels.Any(p => Mathf.Abs(p.transform.localPosition.z - R) > 0.05f), $"{robot}: 1: the blocks are not on a plane (z differs from {R} for some)");
        // Neighbours in a row are a gap apart in angle: centre step = (w1 + w2) / 2R + columnGap / R.
        var rows = panels.GroupBy(p => Mathf.Round(Mathf.Asin(p.transform.localPosition.y / R) * 1000f)).ToList();
        bool gapsOk = true; string worst = "";
        foreach (var row in rows)
        {
            var sorted = row.OrderBy(p => Mathf.Atan2(p.transform.localPosition.x, p.transform.localPosition.z)).ToList();
            for (int i = 0; i + 1 < sorted.Count; i++)
            {
                float a = Mathf.Atan2(sorted[i].transform.localPosition.x, sorted[i].transform.localPosition.z), b = Mathf.Atan2(sorted[i + 1].transform.localPosition.x, sorted[i + 1].transform.localPosition.z);
                float want = ((sorted[i].Width + sorted[i + 1].Width) / 2f + images.columnGap) / R;
                if (Mathf.Abs((b - a) - want) > 2e-3f) { gapsOk = false; worst = $"{sorted[i].Label}-{sorted[i + 1].Label}: {(b - a) * Mathf.Rad2Deg:F2} deg vs {want * Mathf.Rad2Deg:F2}"; }
            }
        }
        Check(gapsOk, $"{robot}: 1: neighbours in a row are one gap apart in angle (grid x / R) {worst}");
        Check(LayoutVerifier.Inspect(robot, "arc"), $"{robot}: 1: no overlap and inside +/-45 deg on the arc");
        // Hiding cameras re-arranges on the arc too.
        for (int i = panels.Count - 1; i >= 1; i--)
        {
            images.SetCameraVisible(panels[i], false);
            yield return Frames(3);
            Check(panels.Where(p => p.Visible).All(p => Mathf.Abs(p.transform.localPosition.magnitude - R) < 1e-3f) && LayoutVerifier.Inspect(robot, $"arc, {i} hidden"), $"{robot}: 1: {i} camera(s) hidden: still on the sphere, no overlap");
        }
        foreach (var p in panels) images.SetCameraVisible(p, true);
        images.ResetLayout();
        yield return Frames(3);
        // The dragger keeps the radius: Follow() places the block at dir * the radius captured on grab (here: its own).
        var moved = panels[0];
        var dir = Quaternion.Euler(-6f, -9f, 0f) * moved.transform.localPosition.normalized;
        moved.transform.localPosition = dir * moved.transform.localPosition.magnitude;
        images.SavePosition(moved);
        Check(Mathf.Abs(moved.transform.localPosition.magnitude - R) < 1e-3f && Mathf.Abs(Settings.For(images.Profile).Camera(moved.Config).Position.Value.magnitude - R) < 1e-3f, $"{robot}: 1: a dragged block (rotated about the head) stays at R, saved position has |p| = {Settings.For(images.Profile).Camera(moved.Config).Position.Value.magnitude:F3}");
        images.ResetLayout();
        yield return Frames(3);
        Shot(robot == "SOBIT_HOME" ? "gui_b_arc_home" : "gui_b_arc_light", 100f);
        foreach (var w in ExitPlay()) yield return w;
    }

    // ---------------- Selection screen Display row, persistence, robot screens at 1.3 ----------------
    static Button Find(string nameContains) => Object.FindObjectsByType<Button>(FindObjectsSortMode.None).FirstOrDefault(b => b.name.Contains(nameContains));
    static Image Backdrop() => GameObject.Find("Robot Selection Screen").transform.Find("Backdrop").GetComponent<Image>();
    static float HeadingFont() => GameObject.Find("Robot Selection Screen").transform.Find("Heading").GetComponent<TextMeshProUGUI>().fontSize;
    static string SizeText() => GameObject.Find("Robot Selection Screen").transform.Find("Text Size").GetComponent<TextMeshProUGUI>().text;

    static IEnumerable DisplaySettings()
    {
        var robotsDir = Path.Combine(Application.persistentDataPath, "robots");
        if (Directory.Exists(robotsDir)) Directory.Delete(robotsDir, true);   // only built-in robots
        EditorSceneManager.OpenScene("Assets/Scenes/RobotSelectionScene.unity");
        foreach (var w in EnterPlay()) yield return w;
        yield return Seconds(2.0);

        float font0 = HeadingFont(), alpha0 = Backdrop().color.a;
        var minus = Find("A−"); var plus = Find("A+");
        var contrastToggle = Object.FindObjectsByType<Toggle>(FindObjectsSortMode.None).FirstOrDefault(t => t.name.Contains("High contrast"));
        Check(minus != null && plus != null && contrastToggle != null && SizeText() == "Text 100 %", $"2: Display row: A-, A+, 'Text 100 %', High contrast toggle ({SizeText()})");
        Check(Mathf.Abs(alpha0 - 0.92f) < 0.01f, $"2: panel alpha 0.92 by default ({alpha0:F2})");
        Shot("gui_b_display_100", 80f);

        // A- twice -> 80 %, A- again does nothing.
        for (int i = 0; i < 2; i++) { Find("A−").onClick.Invoke(); yield return Frames(3); yield return Seconds(0.2); }
        Check(Mathf.Approximately(Settings.TextScale, 0.8f) && SizeText() == "Text 80 %" && Mathf.Abs(HeadingFont() / font0 - 0.8f) < 0.01f && !Find("A−").interactable,
              $"2: A- twice: Text 80 %, heading font x{HeadingFont() / font0:F2}, A- greyed");
        for (int i = 0; i < 5; i++) { Find("A+").onClick.Invoke(); yield return Frames(3); yield return Seconds(0.2); }
        Check(Mathf.Approximately(Settings.TextScale, 1.3f) && SizeText() == "Text 130 %" && Mathf.Abs(HeadingFont() / font0 - 1.3f) < 0.01f && !Find("A+").interactable,
              $"2: A+ 5x: Text 130 %, heading font x{HeadingFont() / font0:F2} (= 1.3 x {font0:F0}), A+ greyed");
        var screenRt = (RectTransform)GameObject.Find("Robot Selection Screen").transform;
        var head = Camera.main.transform;
        var corners = new Vector3[4]; screenRt.GetWorldCorners(corners);
        var angles = corners.Select(c => head.InverseTransformPoint(c)).Select(v => Mathf.Atan2(v.y, v.z) * Mathf.Rad2Deg).ToList();
        Check(angles.Max() < 45f && angles.Min() > -45f, $"2: selection screen at 130 % still inside +/-45 deg vertically ({angles.Min():F1}..{angles.Max():F1})");
        Object.FindObjectsByType<Toggle>(FindObjectsSortMode.None).First(t => t.name.Contains("High contrast")).isOn = true;
        yield return Frames(3); yield return Seconds(0.2);
        Check(Settings.HighContrast && Mathf.Approximately(Backdrop().color.a, 1f), $"2: High contrast on: backdrop alpha {Backdrop().color.a:F2}, saved {Settings.HighContrast}");
        Shot("gui_b_display_130_contrast", 80f);

        // Persists across the scene reload: open SOBIT HOME from its card.
        Find("Card SOBIT HOME").onClick.Invoke();
        GameObject bar = null;
        foreach (var w in Until(() => (bar = GameObject.Find("HUD Bar")) != null, 20)) yield return w;
        yield return Seconds(3.0);
        var images = Object.FindFirstObjectByType<ImageSubscriber>();
        float built = bar.transform.localScale.x * 1000f;
        Check(Mathf.Abs(built - HudBar.CompactScale * 1.3f) < 1e-3f && Mathf.Approximately(bar.transform.Find("Background").GetComponent<Image>().color.a, 1f),
              $"2: after the scene reload the bar is built at 0.7 x 1.3 (scale {built:F3}/1000) with an opaque background");
        var nameLabel = images.Panels[0].transform.Find("Name").GetComponent<TextMeshProUGUI>();
        Check(Mathf.Abs(nameLabel.fontSize - HudTheme.TitleFontBase * 1000f * 1.3f) < 0.5f, $"2: camera block name font {nameLabel.fontSize:F0} = 1.3 x {HudTheme.TitleFontBase * 1000f:F0}");
        Check(Mathf.Approximately(images.Panels[0].transform.Find("Card").GetComponent<Image>().color.a, 1f), "2: camera card opaque with high contrast");
        // Bar labels fit: every toggle / button label on one line inside its rect (like LayoutVerifier 7).
        Canvas.ForceUpdateCanvases();
        var bad = new List<string>();
        foreach (var l in bar.GetComponentsInChildren<TextMeshProUGUI>(true).Where(l => l.transform.parent.GetComponent<Toggle>() != null && !string.IsNullOrEmpty(l.text)))
        {
            l.ForceMeshUpdate();
            if (l.isTextOverflowing || l.textInfo.lineCount > 1) bad.Add($"'{l.text}' lines {l.textInfo.lineCount}");
        }
        Check(bad.Count == 0, $"2: bar toggle labels on one line at 130 % {string.Join("; ", bad)}");
        Check(LayoutVerifier.Inspect("SOBIT_HOME", "text 130 %"), "2: SOBIT_HOME at 130 %: no overlap (the bar grew; blocks, bar and image clear), inside +/-45 deg");
        foreach (var p in images.Panels) p.SetTexture(MakeFake());
        yield return Frames(3);
        Shot("gui_b_arc_home_130", 100f);
        foreach (var w in ExitPlay()) yield return w;

        // SOBIT LIGHT directly (4 cameras) at 130 %.
        EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
        RobotProfile.Selected = Profile("SOBIT_LIGHT");
        foreach (var w in EnterPlay()) yield return w;
        yield return Seconds(3.0);
        Check(LayoutVerifier.Inspect("SOBIT_LIGHT", "text 130 %"), "2: SOBIT_LIGHT at 130 %: no overlap, inside +/-45 deg");
        foreach (var p in Object.FindFirstObjectByType<ImageSubscriber>().Panels) p.SetTexture(MakeFake());
        yield return Frames(3);
        Shot("gui_b_arc_light_130", 100f);
        foreach (var w in ExitPlay()) yield return w;

        // 80 % on the robot screens too (blocks smaller).
        HudTheme.SetTextScale(0.8f); HudTheme.SetHighContrast(false);
        EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
        RobotProfile.Selected = Profile("SOBIT_LIGHT");
        foreach (var w in EnterPlay()) yield return w;
        yield return Seconds(3.0);
        Check(LayoutVerifier.Inspect("SOBIT_LIGHT", "text 80 %"), "2: SOBIT_LIGHT at 80 %: no overlap, inside +/-45 deg");
        foreach (var w in ExitPlay()) yield return w;
    }

    static Texture2D _fake;
    static Texture2D MakeFake()
    {
        if (_fake != null) return _fake;
        _fake = new Texture2D(64, 48);
        var px = new Color[64 * 48];
        for (int y = 0; y < 48; y++) for (int x = 0; x < 64; x++) px[y * 64 + x] = Color.Lerp(new Color(0.55f, 0.6f, 0.66f), new Color(0.35f, 0.3f, 0.25f), y / 47f) * (0.8f + 0.2f * Mathf.Sin(x * 0.3f));
        _fake.SetPixels(px); _fake.Apply();
        return _fake;
    }

    // Render from the head (the headset camera) with the main camera's background.
    static void Shot(string name, float fov)
    {
        var head = Camera.main.transform;
        var go = new GameObject("VerifyCam");
        var cam = go.AddComponent<Camera>();
        go.transform.SetPositionAndRotation(head.position, head.rotation);
        cam.fieldOfView = fov; cam.aspect = 1.6f; cam.nearClipPlane = 0.05f; cam.farClipPlane = 200f;
        cam.stereoTargetEye = StereoTargetEyeMask.None;
        cam.clearFlags = Camera.main.clearFlags; cam.backgroundColor = Camera.main.backgroundColor;
        var rt = new RenderTexture(1600, 1000, 24);
        cam.targetTexture = rt;
        cam.Render();
        RenderTexture.active = rt;
        var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;
        File.WriteAllBytes(Path.Combine(ShotDir, name + ".png"), tex.EncodeToPNG());
        cam.targetTexture = null; Object.DestroyImmediate(rt); Object.DestroyImmediate(go); Object.DestroyImmediate(tex);
        Log($"    screenshot {name}.png (fov {fov:F0})");
    }
}
