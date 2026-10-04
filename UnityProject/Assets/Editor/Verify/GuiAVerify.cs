// Verify harness: GUI improvements batch A: control countdown, link health, dim surround + horizon, head-lag outline,
// lens undistortion shader, grouped status strip.
// Needs the live sim (HOME) on 127.0.0.1:10000 and no other ROS client (stop the app on the headset).
// Run: tools/verify.sh --suite GuiAVerify   (or Unity -batchmode -projectPath <copy> -executeMethod GuiAVerify.Run)
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using RosMessageTypes.Sensor;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.UI;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;

public static class GuiAVerify
{
    static IEnumerator _run;
    static int _failures, _passes;
    static readonly List<string> _controlLogs = new List<string>();
    static string ShotDir
    {
        get
        {
            var d = Environment.GetEnvironmentVariable("EXP_SHOTS");
            return Directory.CreateDirectory(string.IsNullOrEmpty(d) ? Path.GetFullPath(Path.Combine(Application.dataPath, "../../../exp_shots_guia")) : d).FullName;
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
    static T Field<T>(object o, string name) => (T)o.GetType().GetField(name, BindingFlags.NonPublic | BindingFlags.Instance).GetValue(o);
    static IEnumerable Until(Func<bool> cond, double seconds) { double end = Now + seconds; while (Now < end && !cond()) yield return Seconds(0.05); }
    static bool Near(Color a, Color b, float tol = 0.02f) => Mathf.Abs(a.r - b.r) < tol && Mathf.Abs(a.g - b.g) < tol && Mathf.Abs(a.b - b.b) < tol;
    static IEnumerable PubWait(string args)
    {
        Log($"    pub {args}");
        var p = Process.Start(new ProcessStartInfo("/bin/bash", $"\"{VerifyPaths.Tool("pub.sh")}\" {args}") { UseShellExecute = false, CreateNoWindow = true });
        while (!p.HasExited) yield return Seconds(0.2);
    }

    // ---------------- Static checks: shader assets and the undistortion model ----------------
    static void StaticChecks()
    {
        var assets = AssetDatabase.LoadAssetAtPath<HudAssets>("Assets/Resources/HudAssets.asset");
        Check(assets != null && assets.surround != null && assets.undistort != null && Resources.Load<HudAssets>("HudAssets") != null, "HudAssets (Resources) references the surround and undistort materials");
        if (assets == null) return;
        Check(assets.surround.shader.name == "Hud/Surround" && assets.surround.shader.isSupported && assets.surround.renderQueue == 1000,
              $"surround material: shader {assets.surround.shader.name}, supported {assets.surround.shader.isSupported}, queue {assets.surround.renderQueue} (Background)");
        Check(assets.undistort.shader.name == "Hud/UndistortImage" && assets.undistort.shader.isSupported, $"undistort material: shader {assets.undistort.shader.name}, supported {assets.undistort.shader.isSupported}");

        // LinkHealth rule.
        Check(new[] { LinkHealth.Evaluate(0.1f, 50f, false), LinkHealth.Evaluate(0.1f, -1f, false), LinkHealth.Evaluate(-1f, -1f, false) }.All(l => l == LinkHealth.Level.Good), "LinkHealth: fresh frame, small or no RTT -> Good");
        Check(LinkHealth.Evaluate(0.8f, 50f, false) == LinkHealth.Level.Degraded && LinkHealth.Evaluate(0.1f, 300f, false) == LinkHealth.Level.Degraded && LinkHealth.Evaluate(0.1f, 900f, false) == LinkHealth.Level.Degraded,
              "LinkHealth: frame 0.5..2 s old or RTT >= 150 ms -> Degraded");
        Check(LinkHealth.Evaluate(2f, 10f, false) == LinkHealth.Level.Lost && LinkHealth.Evaluate(0.1f, 10f, true) == LinkHealth.Level.Lost, "LinkHealth: frame >= 2 s old or connection error -> Lost");

        // Fake CameraInfo: material created with K / D, or none.
        var info = new CameraInfoMsg { width = 640, height = 480, distortion_model = "plumb_bob", K = new double[] { 500, 0, 320, 0, 500, 240, 0, 0, 1 }, D = new double[] { -0.2, 0.05, 0.001, -0.001, 0 } };
        var mat = Undistort.Create(info);
        Check(mat != null && mat.GetVector(Undistort.KId) == new Vector4(500, 500, 320, 240) && Mathf.Approximately(mat.GetVector(Undistort.DistId).x, -0.2f)
              && mat.GetVector(Undistort.Dist2Id).y == 640f, "fake CameraInfo (k1 -0.2): material created, K / D / size set");
        info.D = new double[] { 0, 0, 0, 0, 0 };
        Check(Undistort.Create(info) == null, "D = 0 (the sim): no undistortion material");
        info.D = new double[] { -0.2, 0, 0, 0, 0 }; info.distortion_model = "equidistant";
        Check(Undistort.Create(info) == null, "a model other than plumb_bob: not corrected (documented limit)");

        // GPU: a distorted straight line comes out straight.
        if (SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null) { Log("    no GPU: skipped the render check"); return; }
        double[] d = { -0.2, 0.05, 0.001, -0.001, 0 };
        const int W = 640, H = 480; const double y0 = 100;
        var src = new Texture2D(W, H, TextureFormat.RGBA32, false) { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp };
        var px = new Color32[W * H];
        for (double x = 0; x < W; x += 0.25)
        {
            var s = Undistort.SourceUv(new Vector2((float)(x / W), (float)(1.0 - y0 / H)), 500, 500, 320, 240, W, H, d);
            int cx = (int)(s.x * W), cy = (int)(s.y * H);
            for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++)
                if (cx + dx >= 0 && cx + dx < W && cy + dy >= 0 && cy + dy < H) px[(cy + dy) * W + cx + dx] = new Color32(255, 255, 255, 255);
        }
        src.SetPixels32(px); src.Apply();
        var fixedMat = new Material(assets.undistort); Undistort.SetParams(fixedMat, 500, 500, 320, 240, W, H, d);
        var plainMat = new Material(assets.undistort); Undistort.SetParams(plainMat, 500, 500, 320, 240, W, H, new double[5]);
        float devFixed = LineDeviation(src, fixedMat, W, H, y0, out int foundFixed), devPlain = LineDeviation(src, plainMat, W, H, y0, out _);
        Check(foundFixed >= 25 && devFixed < 2.5f, $"GPU: the distorted line is straight after correction (max deviation {devFixed:F2} px over {foundFixed} columns, y = {y0} px)");
        Check(devPlain > 5f, $"GPU: without correction the same line is curved (max deviation {devPlain:F1} px) -> the test is meaningful");
    }

    // Blit the synthetic image through `mat` and measure how far the white line is from row y0 (px, from the top).
    static float LineDeviation(Texture2D src, Material mat, int W, int H, double y0, out int found)
    {
        var rt = RenderTexture.GetTemporary(W, H, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear);
        Graphics.Blit(src, rt, mat);
        var prev = RenderTexture.active; RenderTexture.active = rt;
        var outTex = new Texture2D(W, H, TextureFormat.RGBA32, false, true);
        outTex.ReadPixels(new Rect(0, 0, W, H), 0, 0); outTex.Apply();
        RenderTexture.active = prev; RenderTexture.ReleaseTemporary(rt);
        var pix = outTex.GetPixels32();
        float worst = 0f; found = 0;
        for (int x = 40; x < W - 40; x += 20)
        {
            double sum = 0; int n = 0;
            for (int row = 0; row < H; row++) if (pix[row * W + x].r > 128 && pix[row * W + x].a > 128) { sum += row; n++; }
            if (n == 0) continue;
            found++;
            worst = Mathf.Max(worst, Mathf.Abs((float)(H - (sum / n + 0.5) - y0)));
        }
        Object.DestroyImmediate(outTex);
        return found == 0 ? 999f : worst;
    }

    // ---------------- Live ----------------
    static IEnumerator Main()
    {
        StaticChecks();
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.1");
        PlayerPrefs.SetInt($"RobotModel/{RobotSpec.Current.Asset}", 1); PlayerPrefs.SetString($"CameraLayout/{RobotSpec.Current.Asset}", "firstperson");
        PlayerPrefs.Save();
        Application.logMessageReceived += (c, s, t) => { if (c.StartsWith("[Control]")) _controlLogs.Add(c); };

        foreach (var w in PubWait("head 0 0")) yield return w;
        var profile = AssetDatabase.LoadAssetAtPath<RobotProfile>(RobotSpec.Current.ProfilePath);
        EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
        RobotProfile.Selected = profile;
        Object.FindFirstObjectByType<QuestControllerPublisher>().controlRobot = false;
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Seconds(0.1);
        var publisher = Object.FindFirstObjectByType<QuestControllerPublisher>();
        publisher.controlRobot = false;
        yield return Frames(2);
        var hud = Object.FindFirstObjectByType<TeleopHud>();
        var images = Object.FindFirstObjectByType<ImageSubscriber>();

        FirstPersonView fpv = null;
        foreach (var w in Until(() =>
        {
            fpv = Object.FindFirstObjectByType<FirstPersonView>();
            return fpv != null && fpv.Model != null && fpv.Model.AcceptedTransforms > 0 && fpv.FramesReceived > 10;
        }, 40)) yield return w;
        if (!Check(hud.FirstPerson && fpv != null && fpv.FramesReceived > 0, $"first person live: frames {fpv?.FramesReceived}")) yield break;
        yield return Seconds(2.0);
        fpv.Recenter();
        yield return Seconds(1.0);
        var head = FirstPersonView.Head;
        var barT = Field<Transform>(hud, "_bar");
        int headIdx = fpv.CameraIndex;
        // ================= 6. status strip groups (all good) =================
        var strip = hud.Strip;
        {
            string txt = string.Join(" | ", strip.GetComponentsInChildren<TextMeshProUGUI>(true).Select(t => t.text).Where(t => t.Length > 0));
            Check(txt.Contains("fps") && txt.Contains("Hz") && txt.Contains("HEAD") && txt.Contains("LIFT") && txt.Contains("link ok") && txt.Contains("LAYOUT"), "6: strip texts contain link ok, fps, Hz, LAYOUT, HEAD, LIFT");
            int dividers = strip.GetComponentsInChildren<Transform>(true).Count(t => t.name == "Group Divider");
            float width = ((RectTransform)strip.transform).sizeDelta.x;
            Check(dividers == 2 && width > 1000f && width <= 1500f, $"6: three groups -> {dividers} dividers, strip {width:F0} mm wide (<= 1500)");
            bool calm = strip.GetComponentsInChildren<TextMeshProUGUI>(true).All(t => !Near(t.color, HudTheme.Warn) && !Near(t.color, HudTheme.Bad));
            Check(calm, "6: nothing coloured Warn / Bad while the link is good");
            Shot("gui_a_strip", head.position, Quaternion.LookRotation(strip.transform.position - head.position, Vector3.up), 50f);
        }
        // ================= 1. control countdown =================
        {
            var toggle = barT.GetComponentsInChildren<Toggle>(true).First(t => t.name == "Toggle Control robot");
            var ring = toggle.transform.Find("Countdown Ring").GetComponent<Image>();
            var label = toggle.GetComponentInChildren<TextMeshProUGUI>(true);
            toggle.SetIsOnWithoutNotify(false); publisher.controlRobot = false;
            Check(ring != null && ring.type == Image.Type.Filled && !ring.gameObject.activeSelf && label.text == "Control robot", "1: countdown ring exists (hidden), label 'Control robot'");
            toggle.isOn = true;
            Check(!publisher.controlRobot && ring.gameObject.activeSelf && label.text == "Control in 2…" && Near(ring.color, HudTheme.Accent), $"1: toggled on -> not armed, ring shown (Accent), label '{label.text}'");
            yield return Seconds(1.0);
            Check(!publisher.controlRobot && ring.fillAmount > 0.35f && ring.fillAmount < 0.7f && label.text == "Control in 1…", $"1: at 1 s not armed, ring fill {ring.fillAmount:F2}, label '{label.text}'");
            barT.gameObject.SetActive(true);
            Shot("gui_a_countdown", head.position, Quaternion.LookRotation(toggle.transform.position + head.right * 0.35f - head.position, Vector3.up), 14f);
            barT.gameObject.SetActive(false);
            yield return Seconds(1.2);
            Check(publisher.controlRobot && !ring.gameObject.activeSelf && label.text == "Control robot" && toggle.isOn, $"1: at 2.2 s armed (controlRobot {publisher.controlRobot}), ring hidden, label '{label.text}'");
            toggle.isOn = false;
            Check(!publisher.controlRobot, "1: off is immediate");
            toggle.isOn = true; yield return Seconds(0.8); toggle.isOn = false;
            Check(!publisher.controlRobot && !ring.gameObject.activeSelf && label.text == "Control robot" && !toggle.isOn, "1: second press during the countdown cancels (toggle back off, ring hidden)");
            yield return Seconds(2.0);
            Check(!publisher.controlRobot, "1: still not armed 2.8 s after the cancelled start");
            hud.RequestControl(true);
            Check(!publisher.controlRobot && ring.gameObject.activeSelf && toggle.isOn, "1: TeleopHud.RequestControl(true) goes through the countdown too");
            yield return Seconds(2.3);
            bool armed = publisher.controlRobot;
            hud.RequestControl(false);
            Check(armed && !publisher.controlRobot, "1: armed after RequestControl(true), RequestControl(false) is immediate");
            Check(_controlLogs.Any(l => l.Contains("countdown started")) && _controlLogs.Any(l => l.Contains("countdown cancelled")) && _controlLogs.Any(l => l.Contains("countdown armed")),
                  $"1: log lines [Control] countdown started / cancelled / armed ({_controlLogs.Count})");
        }
        // ================= 3. surround + horizon =================
        {
            var s = fpv.Surround;
            Check(s != null && s.name == "Surround" && Mathf.Abs(s.transform.lossyScale.x / 2f - 20f) < 0.01f, $"3: Surround exists, radius {s?.transform.lossyScale.x / 2f} m");
            var r = s.GetComponent<MeshRenderer>();
            Check((s.transform.position - head.position).magnitude < 0.01f && s.transform.rotation == Quaternion.identity && r.sharedMaterial == HudTheme.Assets.surround
                  && r.sharedMaterial.renderQueue == 1000 && s.GetComponent<Collider>() == null, "3: centred on the head, never rotated, HudAssets material (queue Background), no collider");
            var floor = GameObject.FindGameObjectsWithTag("Floor").Where(g => g != s.gameObject).ToList();
            Check(floor.Count > 0 && floor.All(g => g.activeInHierarchy), $"3: the floor grid is still there ({floor.Count})");
            Color horizon = Sample(head.position, Quaternion.Euler(0f, head.eulerAngles.y + 150f, 0f)), sky = Sample(head.position, Quaternion.Euler(-60f, head.eulerAngles.y + 150f, 0f));
            Check(horizon.grayscale > sky.grayscale + 0.01f && sky.grayscale > 0.001f, $"3: lighter band at eye level ({horizon.grayscale:F3}) than the sky above ({sky.grayscale:F3}), not pure black");
            Shot("gui_a_surround", head.position, Quaternion.Euler(0f, head.eulerAngles.y + 40f, 0f), 100f);
        }
        // ================= 4. head-lag outline =================
        {
            var lag = fpv.Lag;
            Check(lag != null && lag.AngleDeg < 3f && !lag.Visible && lag.Alpha == 0f, $"4: head and camera aligned (after Recenter) -> outline hidden (angle {lag?.AngleDeg:F2})");
            foreach (var w in PubWait("head 0.5 0")) yield return w;
            foreach (var w in Until(() => lag.AngleDeg > 28f && lag.Alpha > 0.99f, 8)) yield return w;
            float offRad = Vector3.Angle(lag.Plane.forward, fpv.Card.Rect.forward) * Mathf.Deg2Rad;
            var ring = lag.GetComponentInChildren<Image>(true);
            Check(lag.Visible && Mathf.Abs(offRad - 0.5f) < 0.1f, $"4: robot head at pan 0.5 -> outline visible, {offRad:F2} rad from the card (angle {lag.AngleDeg:F1} deg)");
            Check(Mathf.Abs(ring.color.a - HeadLagOutline.MaxAlpha) < 0.01f && Near(ring.color, HudTheme.Accent) && Vector2.Distance(ring.rectTransform.sizeDelta, fpv.Card.CardRect.sizeDelta) < 0.5f,
                  $"4: outline Accent at alpha {ring.color.a:F2}, same size as the card ({ring.rectTransform.sizeDelta} vs {fpv.Card.CardRect.sizeDelta})");
            Shot("gui_a_headlag", head.position, Quaternion.LookRotation(Vector3.Slerp(head.forward, fpv.Card.Rect.forward, 0.5f), Vector3.up), 75f);
            foreach (var w in PubWait("head 0 0")) yield return w;
            foreach (var w in Until(() => !lag.Visible, 8)) yield return w;
            Check(!lag.Visible && lag.AngleDeg < 3f, $"4: head back to 0 -> outline hidden (angle {lag.AngleDeg:F2})");
        }
        // ================= 2. connection loss =================
        {
            var cfg = images.Panels[headIdx].Config;
            float fps0 = cfg.maxFps;
            var card = fpv.Card;
            Image Outline() => card.transform.Find("Outline").GetComponent<Image>();
            TextMeshProUGUI Conn() => strip.GetComponentsInChildren<TextMeshProUGUI>(true).First(t => t.name == "Connection");
            Image Dot() => strip.GetComponentsInChildren<Image>(true).First(i => i.name == "Connection Dot");
            Check(card.Link == LinkHealth.Level.Good && !card.LostLabelShown && Outline().color.a == 0f, "2: good link: no ring tint, no label");
            cfg.maxFps = 0.2f;
            double t0 = Now; bool degradedSeen = false; Color degradedRing = default; string degradedText = null;
            while (Now - t0 < 1.0) { if (!degradedSeen && card.Link == LinkHealth.Level.Degraded) { degradedSeen = true; degradedRing = Outline().color; degradedText = Conn().text; } yield return Seconds(0.05); }
            yield return Seconds(0.3);   // the strip refreshes at 5 Hz
            Check(degradedSeen && Near(degradedRing, HudTheme.Warn), $"2: maxFps 0.2 -> Degraded within 1 s, ring Warn ({degradedRing})");
            yield return Seconds(Mathf.Max(0.05f, 2.6f - (float)(Now - t0)));
            Check(card.Link == LinkHealth.Level.Lost && Near(Outline().color, HudTheme.Bad) && card.LostLabelShown && Regex.IsMatch(card.LostLabelText, @"^LAST FRAME \d+\.\d s$"),
                  $"2: after 2.6 s Lost, ring Bad, label '{card.LostLabelText}'");
            Check(Conn().text == "lost" && Near(Conn().color, HudTheme.Bad) && Near(Dot().color, HudTheme.Bad), $"2: status strip shows '{Conn().text}' in Bad");
            Shot("gui_a_link_lost", head.position, Quaternion.LookRotation(card.Rect.position - head.position, Vector3.up), 60f);
            cfg.maxFps = fps0;
            foreach (var w in Until(() => card.Link == LinkHealth.Level.Good, 6)) yield return w;
            yield return Seconds(0.4);
            Check(card.Link == LinkHealth.Level.Good && !card.LostLabelShown && Outline().color.a == 0f && Conn().text == "link ok", $"2: maxFps restored -> Good again, label hidden, strip '{Conn().text}'");

            // Blocks layout: each block's ring from its own age; the first-person extras are gone.
            hud.SetCameraLayout(FirstPersonView.LayoutBlocks, save: false);
            yield return Seconds(1.5);
            var panel = images.Panels[headIdx];
            Image PanelRing() => panel.transform.Find("Outline").GetComponent<Image>();
            Check(panel.Visible && PanelRing().color.a == 0f, "2: blocks layout: head block ring clear while frames flow");
            cfg.maxFps = 0.2f;
            yield return Seconds(2.6);
            Check(Near(PanelRing().color, HudTheme.Bad), $"2: blocks layout: head block ring Bad after 2.6 s without frames ({PanelRing().color})");
            cfg.maxFps = fps0;
            yield return Seconds(1.5);
            Check(PanelRing().color.a == 0f, "2: blocks layout: ring clear again");
            // ================= 3/4 blocks layout: no surround, no outline =================
            Check(fpv.Surround == null && GameObject.Find("Surround") == null && GameObject.Find("Head Lag Outline") == null, "3/4: blocks layout: no Surround, no head-lag outline");
            Check(hud.Strip != null && !hud.Strip.HasModel && hud.Strip.GetComponentsInChildren<Transform>(true).Count(t => t.name == "Group Divider") == 1, "6: blocks-mode strip has two groups (one divider)");
            hud.SetCameraLayout(FirstPersonView.LayoutFirstPerson, save: false);
            yield return Seconds(1.0);
            fpv = Object.FindFirstObjectByType<FirstPersonView>();
            Check(fpv.Surround != null && fpv.Lag != null, "3/4: back to first person: Surround and outline are back");
        }
        // ================= 5. undistortion in the live card =================
        Check(fpv.UndistortMaterial == null && fpv.Card.View.material.shader.name != "Hud/UndistortImage", "5: the sim publishes D = 0 -> the card does not use the undistortion shader");
        Check(Object.FindObjectsByType<ROSConnection>(FindObjectsSortMode.None).Length == 1, "exactly one ROSConnection at the end");
        publisher.controlRobot = false;
        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Seconds(0.1);
    }

    // Render from `pos` looking along `rot` with the main camera's background; the caller destroys the texture.
    static Texture2D Render(Vector3 pos, Quaternion rot, float fov, int w, int h)
    {
        var go = new GameObject("VerifyCam");
        var cam = go.AddComponent<Camera>();
        go.transform.SetPositionAndRotation(pos, rot);
        var main = Camera.main;
        cam.fieldOfView = fov; cam.aspect = (float)w / h;
        cam.nearClipPlane = 0.05f; cam.farClipPlane = 200f;
        cam.stereoTargetEye = StereoTargetEyeMask.None;
        if (main != null) { cam.clearFlags = main.clearFlags; cam.backgroundColor = main.backgroundColor; }
        var rt = new RenderTexture(w, h, 24);
        cam.targetTexture = rt;
        cam.Render();
        RenderTexture.active = rt;
        var tex = new Texture2D(w, h, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, w, h), 0, 0);
        tex.Apply();
        RenderTexture.active = null;
        cam.targetTexture = null; Object.DestroyImmediate(rt); Object.DestroyImmediate(go);
        return tex;
    }

    // Centre pixel of a narrow render.
    static Color Sample(Vector3 pos, Quaternion rot)
    {
        var tex = Render(pos, rot, 6f, 64, 64);
        var c = tex.GetPixel(32, 32);
        Object.DestroyImmediate(tex);
        return c;
    }

    static void Shot(string name, Vector3 pos, Quaternion rot, float fov)
    {
        var tex = Render(pos, rot, fov, 1280, 720);
        File.WriteAllBytes(Path.Combine(ShotDir, name + ".png"), tex.EncodeToPNG());
        Object.DestroyImmediate(tex);
        Log($"    screenshot {name}.png (fov {fov:F1})");
    }
}
