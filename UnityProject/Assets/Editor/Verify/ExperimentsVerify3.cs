// Verify harness: Feedback round 2: fps badges, FP camera toggles, topic line, status strip, hidden controllers.
// Needs the live sim (HOME) on 127.0.0.1:10000 and no other ROS client (stop the app on the headset).
// Run: tools/verify.sh --suite ExperimentsVerify3   (or Unity -batchmode -projectPath <copy> -executeMethod ExperimentsVerify3.Run)
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit.Inputs;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;

// Copy-only harness: feedback round 2 (fps badges in first person, camera toggles in first person,
// topic line only in setup mode, status strip in blocks mode, controller visuals hidden in first person)
// live against the sim; never edits project code.
public static class ExperimentsVerify3
{
    static IEnumerator _run;
    static int _failures, _passes;
    static string ShotDir
    {
        get
        {
            var d = Environment.GetEnvironmentVariable("EXP_SHOTS");
            if (string.IsNullOrEmpty(d)) d = Path.GetFullPath(Path.Combine(Application.dataPath, "../../../exp_shots_v5"));
            Directory.CreateDirectory(d);
            return d;
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
    static void Call(object o, string name) => o.GetType().GetMethod(name, BindingFlags.NonPublic | BindingFlags.Instance).Invoke(o, null);
    static string V(Vector3 v) => $"({v.x:F3},{v.y:F3},{v.z:F3})";
    static Quaternion Down(Transform head, float deg) => Quaternion.Euler(deg, head.eulerAngles.y, 0f);

    // LayoutVerifier's exact overlap test: project a flat quad through the eye onto z = 1, separating axis.
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
    static Rect Bounds(Vector2[] q) => Rect.MinMaxRect(q.Min(p => p.x), q.Min(p => p.y), q.Max(p => p.x), q.Max(p => p.y));

    static string StripText(StatusStrip s) => s == null ? "" : string.Join(" | ", s.GetComponentsInChildren<TextMeshProUGUI>(true).Select(t => t.text).Where(t => t.Length > 0));

    static IEnumerable<GameObject> VisualRoots()
    {
        var m = Object.FindFirstObjectByType<XRInputModalityManager>(FindObjectsInactive.Include);
        if (m == null) yield break;
        foreach (var g in new[] { m.leftController, m.rightController, m.leftHand, m.rightHand })
            if (g != null) yield return g;
    }

    // ---------------- Main ----------------
    static IEnumerator Main()
    {
        Log("===== feedback round 2");
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.1");
        PlayerPrefs.SetInt("RobotModel/SOBIT_HOME", 1); PlayerPrefs.SetString("CameraLayout/SOBIT_HOME", "firstperson");
        PlayerPrefs.Save();

        var profile = AssetDatabase.LoadAssetAtPath<RobotProfile>("Assets/Robots/SOBIT_HOME.asset");
        EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
        RobotProfile.Selected = profile;
        Object.FindFirstObjectByType<QuestControllerPublisher>().controlRobot = false;

        // Visual state before play (renderers / objects / interactors under the modality manager's roots).
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Seconds(0.1);
        var publisher = Object.FindFirstObjectByType<QuestControllerPublisher>();
        publisher.controlRobot = false;
        yield return Frames(2);
        Check(Object.FindObjectsByType<ROSConnection>(FindObjectsSortMode.None).Length == 1, "exactly one ROSConnection");
        var hud = Object.FindFirstObjectByType<TeleopHud>();
        var images = Object.FindFirstObjectByType<ImageSubscriber>();

        FirstPersonView fpv = null; HandCamPip pip = null;
        double end = Now + 40;
        while (Now < end)
        {
            fpv = Object.FindFirstObjectByType<FirstPersonView>();
            pip = Object.FindFirstObjectByType<HandCamPip>();
            if (fpv != null && fpv.Model != null && fpv.Model.AcceptedTransforms > 0 && fpv.FramesReceived > 10 && pip != null) break;
            yield return Seconds(0.25);
        }
        if (!Check(hud.FirstPerson && fpv != null && fpv.FramesReceived > 0 && pip != null,
                   $"first person live: frames {fpv?.FramesReceived}, TF {fpv?.Model?.AcceptedTransforms}, PiP {pip != null}"))
            yield break;
        yield return Seconds(2.0);
        fpv.Recenter();
        yield return Seconds(1.0);
        var head = FirstPersonView.Head;
        var barT = Field<Transform>(hud, "_bar");
        bool AllBlocksInactive() => images.Panels.All(p => !p.gameObject.activeSelf);

        // ================= 5. controller visuals (first person entered with the bar hidden) =================
        var roots = VisualRoots().ToList();
        {
            var rends = roots.SelectMany(r => r.GetComponentsInChildren<Renderer>(true)).ToList();
            Log($"    modality roots: {string.Join(", ", roots.Select(r => $"{r.name}(active {r.activeSelf})"))}; renderers {rends.Count}");
            Check(!barT.gameObject.activeSelf && hud.ControllerVisualsHidden && hud.HiddenVisualCount > 0,
                  $"5: first person, bar hidden -> ControllerVisualsHidden {hud.ControllerVisualsHidden}, HiddenVisualCount {hud.HiddenVisualCount}");
            Check(rends.Count > 0 && rends.All(r => !r.enabled), $"5: every Renderer under the controller/hand objects disabled ({rends.Count(r => !r.enabled)}/{rends.Count})");
        }

        // ================= 1. badges =================
        int headIdx = fpv.CameraIndex;
        {
            end = Now + 3;
            while (Now < end && !(fpv.Badge != null && fpv.Badge.Visible && Regex.IsMatch(fpv.Badge.Text, @"^\d+ fps$"))) yield return Seconds(0.1);
            Check(fpv.Badge != null && fpv.Badge.Visible && Regex.IsMatch(fpv.Badge.Text, @"^\d+ fps$"), $"1: head card badge visible, text '{fpv.Badge?.Text}' matches \\d+ fps");
            var handBadges = pip.GetComponentsInChildren<Transform>(true).Where(t => t.name == "Badge" && t.parent != null && t.parent.name == "View").ToList();
            var htexts = handBadges.Select(b => b.GetComponentInChildren<TextMeshProUGUI>(true)?.text ?? "").ToList();
            Check(handBadges.Count == 2 && handBadges.All(b => b.gameObject.activeSelf) && htexts.All(t => Regex.IsMatch(t, @"^\d+ fps$")),
                  $"1: hand cards have a 'Badge' under 'View' ({handBadges.Count}), texts [{string.Join(", ", htexts)}]");
            Shot("15_fp_badges", head.position, Down(head, 30f), 90f);
            Shot("15_fp_badges_b", head.position, Down(head, 22f), 100f);
            Shot("15_fp_badges_c", head.position, Down(head, 30f), 110f);

            var cfg = images.Panels[headIdx].Config;
            float fps0 = cfg.maxFps;
            cfg.maxFps = 0.2f;
            yield return Seconds(2.5);
            Check(fpv.Badge.Visible && fpv.Badge.Text.StartsWith("stale") && fpv.Badge.Stale, $"1: head camera thinned to 0.2 fps -> badge '{fpv.Badge.Text}' starts with stale");
            cfg.maxFps = fps0;
            end = Now + 3;
            while (Now < end && !Regex.IsMatch(fpv.Badge.Text, @"^[1-9]\d* fps$")) yield return Seconds(0.1);
            Check(Regex.IsMatch(fpv.Badge.Text, @"^[1-9]\d* fps$"), $"1: maxFps restored to {fps0} -> badge '{fpv.Badge.Text}'");
        }

        // ================= 2. camera toggles in first person =================
        {
            var headPanel = images.Panels[headIdx];
            int li = images.IndexOf("hand_left_camera/color/image_raw/compressed");
            var leftPanel = images.Panels[li];
            Check(AllBlocksInactive() && fpv.CardVisible && fpv.CameraOn && pip.ActiveCardCount == 2, $"2: start: blocks inactive, card visible, CameraOn, ActiveCardCount {pip.ActiveCardCount}");
            images.SetCameraVisible(headPanel, false);
            yield return Frames(2);
            int f0 = fpv.FramesReceived;
            Check(!fpv.CardVisible && !fpv.CameraOn && AllBlocksInactive(), $"2: head off -> CardVisible {fpv.CardVisible}, CameraOn {fpv.CameraOn}, blocks inactive {AllBlocksInactive()}");
            yield return Seconds(2.0);
            Check(fpv.FramesReceived == f0 && AllBlocksInactive(), $"2: head off -> FramesReceived constant over 2 s ({f0} -> {fpv.FramesReceived})");
            images.SetCameraVisible(headPanel, true);
            yield return Seconds(1.5);
            Check(fpv.CardVisible && fpv.CameraOn && fpv.FramesReceived > f0 && AllBlocksInactive(), $"2: head on -> card visible, frames {f0} -> {fpv.FramesReceived}, blocks inactive");
            images.SetCameraVisible(leftPanel, false);
            yield return Frames(2);
            Check(pip.ActiveCardCount == 1 && AllBlocksInactive(), $"2: left hand cam off -> ActiveCardCount {pip.ActiveCardCount} (1), blocks inactive");
            images.SetCameraVisible(headPanel, false);
            yield return Frames(2);
            images.ResetLayout();
            yield return Frames(3);
            Check(pip.ActiveCardCount == 2 && fpv.CardVisible && fpv.CameraOn && images.Panels.All(images.IsOn) && AllBlocksInactive(),
                  $"2: ResetLayout -> all on (cards {pip.ActiveCardCount}, head card {fpv.CardVisible}, IsOn {images.Panels.Count(images.IsOn)}/{images.Panels.Count}), blocks still inactive");
            yield return Seconds(1.0);
        }

        // ================= 4 (first person) + 5: strip and visuals with the bar toggle =================
        {
            var strip = hud.Strip;
            Check(strip != null && strip.HasModel && strip.transform.parent == head && Vector3.Distance(strip.transform.localPosition, new Vector3(0f, -0.45f, 1.2f)) < 1e-3f,
                  $"4: first person strip under the head at {V(strip != null ? strip.transform.localPosition : Vector3.zero)} == (0,-0.45,1.2), HasModel {strip?.HasModel}");
            Check(strip.gameObject.activeSelf == !barT.gameObject.activeSelf, $"4: FP strip active {strip.gameObject.activeSelf} == !bar {barT.gameObject.activeSelf}");
            var rends = roots.SelectMany(r => r.GetComponentsInChildren<Renderer>(true)).ToList();
            var objActive = roots.SelectMany(r => r.GetComponentsInChildren<Transform>(true)).ToDictionary(t => t, t => t.gameObject.activeSelf);
            var interactors = roots.SelectMany(r => r.GetComponentsInChildren<UnityEngine.XR.Interaction.Toolkit.Interactors.XRBaseInteractor>(true)).ToList();
            Check(interactors.Count > 0 && interactors.All(i => i.enabled), $"5: hidden: interactor components enabled ({interactors.Count(i => i.enabled)}/{interactors.Count})");
            Call(hud, "ToggleBar");
            yield return Frames(3);
            Check(barT.gameObject.activeSelf && !strip.gameObject.activeSelf, $"4: bar shown -> FP strip inactive ({strip.gameObject.activeSelf})");
            Check(!hud.ControllerVisualsHidden && hud.HiddenVisualCount == 0 && rends.Count(r => r.enabled) > 0, $"5: bar shown -> visuals shown, {rends.Count(r => r.enabled)}/{rends.Count} renderers enabled");
            Check(objActive.All(kv => kv.Key == null || kv.Key.gameObject.activeSelf == kv.Value) && interactors.All(i => i.enabled), "5: GameObjects' activeSelf and interactors unchanged by hide/show");
            Call(hud, "ToggleBar");
            yield return Frames(3);
            Check(!barT.gameObject.activeSelf && strip.gameObject.activeSelf && hud.ControllerVisualsHidden && rends.All(r => !r.enabled),
                  $"4/5: bar hidden again -> strip active, visuals hidden ({hud.HiddenVisualCount})");
            Call(hud, "RebuildBar");
            yield return Frames(3);
            barT = Field<Transform>(hud, "_bar");
            strip = hud.Strip;
            Check(strip != null && strip.gameObject.activeSelf == !barT.gameObject.activeSelf && strip.transform.parent == head,
                  $"4: FP RebuildBar -> strip {(strip != null)}, active {strip?.gameObject.activeSelf} == !bar {barT.gameObject.activeSelf}");
        }

        // leave first person -> visuals re-enabled
        {
            var rends = roots.SelectMany(r => r.GetComponentsInChildren<Renderer>(true)).ToList();
            hud.SetRobotModel(false);
            yield return Frames(3);
            barT = Field<Transform>(hud, "_bar");
            Check(!hud.ControllerVisualsHidden && hud.HiddenVisualCount == 0 && rends.Count(r => r.enabled) > 0,
                  $"5: left first person -> visuals shown, {rends.Count(r => r.enabled)}/{rends.Count} renderers enabled");
        }
        yield return Seconds(2.0);

        // ================= 3. topic line (normal robot screen) =================
        var cam = images.panelParent;
        {
            Check(!images.InSetup && images.Panels.All(p => p.transform.Find("Topic") == null), $"3: normal screen: no panel has a 'Topic' child ({images.Panels.Count(p => p.transform.Find("Topic") != null)} have)");
            Check(images.Panels.All(p => Mathf.Abs(p.Measure(p.Size).BelowViewCentre - p.BelowViewCentre) < 1e-5f),
                  "3: Measure(size).BelowViewCentre == panel.BelowViewCentre for every panel: " + string.Join(", ", images.Panels.Select(p => $"{p.Measure(p.Size).BelowViewCentre:F4}/{p.BelowViewCentre:F4}")));
            var quads = images.Panels.Where(p => p.Visible).Select(p => (p.Config.displayName, Project((RectTransform)p.transform, cam))).ToList();
            quads.Add(("HUD Bar", Project((RectTransform)barT, cam)));
            var bad = new List<string>();
            for (int i = 0; i < quads.Count; i++)
                for (int j = i + 1; j < quads.Count; j++)
                    if (Separation(quads[i].Item2, quads[j].Item2) <= 0f) bad.Add($"{quads[i].Item1}<->{quads[j].Item1}");
            Check(bad.Count == 0, $"3: no two visible blocks (or the bar) overlap ({quads.Count} quads) {string.Join(", ", bad)}");
            publisher.controlRobot = false;
            yield return Seconds(1.0);
            Check(images.Panels.All(p => p.transform.Find("Button Rename") != null && p.transform.Find("Button Rename").gameObject.activeSelf), "3: control off -> Rename shown on every block");
            Shot("17_blocks_no_topic", cam.position, Quaternion.Euler(8f, cam.eulerAngles.y, 0f), 80f);
        }

        // ================= 4. strip in blocks mode =================
        {
            var strip = hud.Strip;
            Check(strip != null && !strip.HasModel && barT.gameObject.activeSelf && !strip.gameObject.activeSelf,
                  $"4: blocks mode: Strip {(strip != null)}, HasModel {strip?.HasModel}, bar shown, strip inactive {strip != null && !strip.gameObject.activeSelf}");
            Call(hud, "ToggleBar");
            yield return Seconds(1.0);
            Check(!barT.gameObject.activeSelf && strip.gameObject.activeSelf, $"4: bar hidden -> strip active {strip.gameObject.activeSelf} == !bar");
            string txt = StripText(strip);
            Check(txt.Contains("TF —"), $"4: strip text contains 'TF —': {txt}");
            var sq = Project((RectTransform)strip.transform, cam);
            var bq = Project((RectTransform)barT, cam);
            var overl = images.Panels.Where(p => p.Visible && Separation(Project((RectTransform)p.transform, cam), sq) <= 0f).Select(p => p.Config.displayName).ToList();
            float minGap = images.Panels.Where(p => p.Visible).Min(p => Separation(Project((RectTransform)p.transform, cam), sq));
            Check(overl.Count == 0, $"4: strip overlaps no camera block (min gap {minGap:F4} on the z=1 plane) {string.Join(", ", overl)}");
            Rect sr = Bounds(sq), br = Bounds(bq);
            bool inside = sr.xMin >= br.xMin - 1e-4f && sr.xMax <= br.xMax + 1e-4f && sr.center.y >= br.yMin && sr.center.y <= br.yMax;
            Check(inside, $"4: strip {sr} within the hidden bar's width and its centre inside the bar's region {br} (bottom overshoot {Mathf.Max(0f, br.yMin - sr.yMin):F4} on z=1 = {Mathf.Atan(br.yMin) * Mathf.Rad2Deg - Mathf.Atan(sr.yMin) * Mathf.Rad2Deg:F2} deg)");
            float topBlocksBottom = images.Panels.Where(p => p.Visible).Min(p => Bounds(Project((RectTransform)p.transform, cam)).yMin);
            Check(sr.yMax < topBlocksBottom, $"4: strip top {sr.yMax:F3} below the lowest block bottom {topBlocksBottom:F3}");
            Shot("16_strip_blocks_mode", cam.position, Quaternion.Euler(14f, cam.eulerAngles.y, 0f), 80f);
            Call(hud, "ToggleBar");
            yield return Frames(3);
            Check(barT.gameObject.activeSelf && !strip.gameObject.activeSelf, "4: bar shown again -> strip inactive");
            Call(hud, "RebuildBar");
            yield return Frames(3);
            barT = Field<Transform>(hud, "_bar");
            strip = hud.Strip;
            Check(strip != null && strip.gameObject.activeSelf == !barT.gameObject.activeSelf && !strip.HasModel,
                  $"4: blocks RebuildBar -> strip {(strip != null)}, active {strip?.gameObject.activeSelf} == !bar {barT.gameObject.activeSelf}");
            Call(hud, "ToggleBar");
            yield return Frames(3);
            Check(strip.gameObject.activeSelf == !barT.gameObject.activeSelf && strip.gameObject.activeSelf, "4: after RebuildBar, bar hidden -> strip active");
            Call(hud, "ToggleBar");
            yield return Frames(3);
        }

        // ================= 5 again: enter first person from blocks mode =================
        {
            hud.SetRobotModel(true);   // the remembered layout (firstperson) comes back
            yield return Seconds(1.0);
            barT = Field<Transform>(hud, "_bar");
            var rends = roots.SelectMany(r => r.GetComponentsInChildren<Renderer>(true)).ToList();
            Check(!barT.gameObject.activeSelf && hud.ControllerVisualsHidden && hud.HiddenVisualCount > 0 && rends.All(r => !r.enabled),
                  $"5: blocks -> first person: bar hidden, visuals hidden ({hud.HiddenVisualCount}), renderers off {rends.Count(r => !r.enabled)}/{rends.Count}");
            Check(hud.Strip != null && hud.Strip.HasModel && hud.Strip.gameObject.activeSelf, "4: blocks -> first person: strip rebuilt with the model, active");
            hud.SetRobotModel(false);
            yield return Frames(3);
            Check(!hud.ControllerVisualsHidden && rends.Count(r => r.enabled) > 0 && hud.Strip != null && !hud.Strip.HasModel, $"5: and back: visuals shown, blocks-mode strip again");
        }

        Check(Object.FindObjectsByType<ROSConnection>(FindObjectsSortMode.None).Length == 1, "exactly one ROSConnection at the end");
        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Seconds(0.1);
    }

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
