// Verify harness: Round 3: robot model / camera layout / compressed images controls, prefs migration, hand card sides.
// Needs the live sim (HOME) on 127.0.0.1:10000 and no other ROS client (stop the app on the headset).
// Run: tools/verify.sh --suite ExperimentsVerify4   (or Unity -batchmode -projectPath <copy> -executeMethod ExperimentsVerify4.Run)
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reflection;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;

// Copy-only harness: round 3 "Robot model + Camera layout + Compressed images" live against the sim
// (controls, prefs + migration, Rename in the first-person layout, hand card sides, badge sizes,
// raw images + reopen, status strip / RTT without a registry). Never edits project code.
public static class ExperimentsVerify4
{
    static IEnumerator _run;
    static int _failures, _passes;
    static string ShotDir
    {
        get
        {
            var d = Environment.GetEnvironmentVariable("EXP_SHOTS");
            if (string.IsNullOrEmpty(d)) d = Path.GetFullPath(Path.Combine(Application.dataPath, "../../exp_shots_v6"));
            Directory.CreateDirectory(d);
            return d;
        }
    }
    static string Script(string name) => VerifyPaths.Tool(name);

    public static void Run()
    {
        EditorSettings.enterPlayModeOptionsEnabled = true;
        EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.DisableDomainReload;
        Application.logMessageReceived += (m, st, t) =>
        {
            lock (_logs) { _logs.Add(m); if (t == LogType.Exception) _exceptions.Add(m); }
        };
        _run = Main();
        EditorApplication.update += Tick;
    }

    static readonly List<string> _logs = new List<string>();
    static readonly List<string> _exceptions = new List<string>();
    static int LogCount(string s) { lock (_logs) return _logs.Count(l => l.Contains(s)); }
    static int ExceptionCount { get { lock (_logs) return _exceptions.Count; } }

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
    static T Field<T>(object o, string name) => (T)o.GetType().GetField(name, BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Public).GetValue(o);
    static void Call(object o, string name) => o.GetType().GetMethod(name, BindingFlags.NonPublic | BindingFlags.Instance).Invoke(o, null);
    static string V(Vector3 v) => $"({v.x:F3},{v.y:F3},{v.z:F3})";
    static Quaternion Down(Transform head, float deg) => Quaternion.Euler(deg, head.eulerAngles.y, 0f);

    static Process Sh(string script, string args)
    {
        Log($"    {script} {args}");
        return Process.Start(new ProcessStartInfo("/bin/bash", $"\"{Script(script)}\" {args}") { UseShellExecute = false, CreateNoWindow = true });
    }
    static IEnumerator WaitExit(Process p, double timeout = 30)
    {
        double end = Now + timeout;
        while (!p.HasExited && Now < end) yield return Seconds(0.1);
    }

    static RobotSpec Spec => RobotSpec.Current;   // env VERIFY_ROBOT, default SOBIT_HOME
    static string Robot => Spec.Asset;
    static RobotProfile Prof => AssetDatabase.LoadAssetAtPath<RobotProfile>($"Assets/Robots/{Robot}.asset");
    static int ModelPref() { var r = Settings.For(Prof); return r.HasModelOn ? (r.ModelOn ? 1 : 0) : -1; }   // -1: nothing saved
    static string LayoutPref() => Settings.For(Prof).Layout;
    static string LeftSuffix => Spec.Sides[0].cameraSuffix;            // the (only / left) hand camera
    static string RightSuffix => Spec.Sides[Spec.Sides.Length - 1].cameraSuffix;
    static int Cards2 => Spec.Sides.Length;                              // hand cards in first person

    static TeleopHud _hud; static ImageSubscriber _images; static QuestControllerPublisher _publisher;
    static Transform Bar => _hud != null ? Field<Transform>(_hud, "_bar") : null;
    static Toggle BarToggle(string label) => Bar.GetComponentsInChildren<Toggle>(true).FirstOrDefault(t => t.name == "Toggle " + label);
    static Button BarButton(string label) => Bar.GetComponentsInChildren<Button>(true).FirstOrDefault(b => b.name == "Button " + label);
    static bool AllBlocksVisible() => _images.Panels.All(p => p.gameObject.activeSelf);
    static bool AllBlocksInactive() => _images.Panels.All(p => !p.gameObject.activeSelf);
    static T Find<T>() where T : Object => Object.FindFirstObjectByType<T>();
    static GameObject FindGo(string name) => Resources.FindObjectsOfTypeAll<Transform>().FirstOrDefault(t => t.name == name && t.gameObject.scene.IsValid())?.gameObject;

    // Hand cards (HandCamPip.Card is private; its fields are public).
    struct HandCard { public bool left; public int index; public RectTransform rt; public CameraBadge badge; public RawImage view; public Button rename; public TextMeshProUGUI name; }
    static List<HandCard> Cards(HandCamPip pip)
    {
        var list = new List<HandCard>();
        if (pip == null) return list;
        var cards = (IList)typeof(HandCamPip).GetField("_cards", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(pip);
        foreach (var c in cards)
        {
            var t = c.GetType();
            list.Add(new HandCard
            {
                left = (bool)t.GetField("left").GetValue(c), index = (int)t.GetField("index").GetValue(c),
                rt = (RectTransform)t.GetField("rt").GetValue(c), badge = (CameraBadge)t.GetField("badge").GetValue(c),
                view = (RawImage)t.GetField("view").GetValue(c), rename = (Button)t.GetField("rename").GetValue(c),
                name = (TextMeshProUGUI)t.GetField("name").GetValue(c),
            });
        }
        return list;
    }

    static IEnumerator EnterTeleop(RobotProfile profile)
    {
        EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
        RobotProfile.Selected = profile;
        Object.FindFirstObjectByType<QuestControllerPublisher>().controlRobot = false;
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Seconds(0.1);
        _publisher = Find<QuestControllerPublisher>();
        _publisher.controlRobot = false;
        yield return Frames(2);
        _hud = Find<TeleopHud>();
        _images = Find<ImageSubscriber>();
        double end = Now + 30;
        while (Now < end && Bar == null) yield return Seconds(0.1);
    }

    static IEnumerator ExitPlay()
    {
        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Seconds(0.1);
        yield return Seconds(0.5);
    }

    // ---------------- Main ----------------
    static IEnumerator Main()
    {
        var profile = AssetDatabase.LoadAssetAtPath<RobotProfile>($"Assets/Robots/{Robot}.asset");
        { var h = Sh("home.sh", ""); var w = WaitExit(h, 90); while (w.MoveNext()) yield return w.Current; }
        { var a = Sh("pub.sh", "armhome"); var b = Sh("pub.sh", "head 0.0 0.0"); var w = WaitExit(a); while (w.MoveNext()) yield return w.Current; w = WaitExit(b); while (w.MoveNext()) yield return w.Current; }

        // =============== Session 1: clean prefs (model off, blocks layout, compressed) ===============
        Log("===== round 3, session 1: controls, rename, sides, badges, strip");
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.1");
        PlayerPrefs.Save();
        var e = EnterTeleop(profile); while (e.MoveNext()) yield return e.Current;
        Check(Object.FindObjectsByType<ROSConnection>(FindObjectsSortMode.None).Length == 1, "exactly one ROSConnection");
        {   // frames first
            double end = Now + 20;
            while (Now < end && _images.Panels.Any(p => p.State != CameraPanel.FeedState.Live)) yield return Seconds(0.2);
        }
        var head = FirstPersonView.Head;

        // ---- 1. controls
        var modelToggle = BarToggle("Robot model");
        var blocksBtn = BarButton("Blocks"); var fpBtn = BarButton("First person");
        var compToggle = BarToggle("Compressed");
        Check(modelToggle != null && blocksBtn != null && fpBtn != null && compToggle != null && BarToggle("First person") == null,
              $"1: bar has 'Toggle Robot model' {modelToggle != null}, 'Button Blocks' {blocksBtn != null}, 'Button First person' {fpBtn != null}, 'Toggle Compressed images' {compToggle != null}, no 'Toggle First person'");
        Check(FindGo("Experiments") == null && !Bar.GetComponentsInChildren<Transform>(true).Any(t => t.name.Contains("Experiment")), "1: no Experiments panel in the bar");
        Check(compToggle.isOn && ImageSubscriber.Compressed, $"1: Compressed images on by default (toggle {compToggle.isOn})");
        Check(_images.Panels.All(p => p.Topic.EndsWith("/compressed")), "1: compressed: every block subscribes '/compressed' (" + string.Join(", ", _images.Panels.Select(p => p.Topic)) + ")");
        Shot("19_bar_v3", Fit(head, new[] { (RectTransform)Bar }, 0f, out var fov19, 1.06f), fov19, 1920, 1080);
        Shot("19_bar_v3_wide", Fit(head, _images.Panels.Select(p => (RectTransform)p.transform).Concat(new[] { (RectTransform)Bar }), 0f, out var fov19w), fov19w);

        int exc0 = ExceptionCount;
        for (int cycle = 1; cycle <= 3; cycle++)
        {
            bool ui = cycle >= 2;   // cycles 2-3 through the bar's own controls
            string tag = $"1 [cycle {cycle}{(ui ? ", bar controls" : "")}]";
            // model off
            Check(!_hud.RobotModelOn && AllBlocksVisible() && Object.FindObjectsByType<RobotModel>(FindObjectsSortMode.None).Length == 0,
                  $"{tag}: model off -> {_images.Panels.Count(p => p.gameObject.activeSelf)}/{_images.Panels.Count} blocks visible, no RobotModel");
            Check(!blocksBtn.interactable && !fpBtn.interactable && !modelToggle.isOn && modelToggle.interactable,
                  $"{tag}: layout buttons not interactable ({blocksBtn.interactable}/{fpBtn.interactable}), model toggle off + interactable");
            // model on (blocks layout)
            if (ui) modelToggle.isOn = true; else _hud.SetRobotModel(true);
            yield return Frames(3);
            var fpv = Find<FirstPersonView>();
            var model = fpv != null ? fpv.Model : null;
            { double end = Now + 10; while (Now < end && (model == null || model.AcceptedTransforms == 0)) yield return Seconds(0.1); }
            Check(_hud.RobotModelOn && model != null && Field<bool>(fpv, "_anchored") && model.AcceptedTransforms > 0,
                  $"{tag}: model on -> RobotModel exists, anchored {(fpv != null && Field<bool>(fpv, "_anchored"))}, TF {model?.AcceptedTransforms}");
            Check(Find<ArmTargets>() != null && Find<BaseVelocity>() != null, $"{tag}: ArmTargets {Find<ArmTargets>() != null} + BaseVelocity {Find<BaseVelocity>() != null} exist");
            Check(AllBlocksVisible() && !fpv.ImageShown && FindGo("FPV Image") == null && Find<HandCamPip>() == null,
                  $"{tag}: blocks layout: blocks still visible, no FPV image card ({fpv.ImageShown}), no hand cards ({Find<HandCamPip>() != null})");
            Check(!_hud.ControllerVisualsHidden && Bar.gameObject.activeSelf && !_hud.BarLowered, $"{tag}: controllers visible, bar still shown, not lowered");
            Check(modelToggle.isOn && blocksBtn.interactable && fpBtn.interactable && Selected(blocksBtn) && !Selected(fpBtn),
                  $"{tag}: toggle on, buttons interactable, Blocks selected (a {blocksBtn.image.color.a:F2} / {fpBtn.image.color.a:F2})");
            Check(ModelPref() == 1, $"{tag}: pref ModelOn = {ModelPref()}");
            if (cycle == 1)
            {
                yield return Seconds(1.0);
                fpv.Recenter();
                yield return Seconds(1.0);
                Shot("20_model_blocks_layout", head.position, Down(head, 30f), 90f);
                Shot("20_model_blocks_layout_b", head.position, Down(head, 22f), 100f);
                {
                    Vector3 fwd = Vector3.ProjectOnPlane(head.forward, Vector3.up).normalized;
                    Shot("20_model_blocks_layout_c", head.position - fwd * 0.6f + Vector3.up * 0.15f, Down(head, 30f), 90f);
                    Shot("20_model_blocks_layout_d", head.position - fwd * 0.9f + Vector3.up * 0.25f, Down(head, 30f), 85f);
                }
            }
            // first-person layout
            if (ui) fpBtn.onClick.Invoke(); else _hud.SetCameraLayout("firstperson");
            yield return Frames(3);
            var pip = Find<HandCamPip>();
            Check(_hud.FirstPerson && AllBlocksInactive() && fpv.ImageShown && FindGo("FPV Image") != null && pip != null && pip.CardCount == Cards2,
                  $"{tag}: first person -> blocks hidden, image card {fpv.ImageShown}, hand cards {pip?.CardCount}");
            Check(!Bar.gameObject.activeSelf && _hud.BarLowered && _hud.ControllerVisualsHidden, $"{tag}: bar hidden {!Bar.gameObject.activeSelf} + lowered {_hud.BarLowered}, controllers hidden {_hud.ControllerVisualsHidden}");
            Check(Selected(fpBtn) && !Selected(blocksBtn) && LayoutPref() == "firstperson",
                  $"{tag}: First person selected, pref CameraLayout = '{LayoutPref()}'");
            // back to blocks
            if (ui) blocksBtn.onClick.Invoke(); else _hud.SetCameraLayout("blocks");
            yield return Frames(3);
            Check(!_hud.FirstPerson && _hud.RobotModelOn && AllBlocksVisible() && !fpv.ImageShown && FindGo("FPV Image") == null && Find<HandCamPip>() == null,
                  $"{tag}: blocks again -> blocks visible, no image card, no hand cards, model kept");
            Check(Bar.gameObject.activeSelf && !_hud.BarLowered && !_hud.ControllerVisualsHidden && Find<ArmTargets>() != null && Find<BaseVelocity>() != null,
                  $"{tag}: bar shown + at its pose, controllers visible, overlays kept");
            Check(Selected(blocksBtn) && LayoutPref() == "blocks", $"{tag}: Blocks selected, pref '{LayoutPref()}'");
            // model off
            if (ui) modelToggle.isOn = false; else _hud.SetRobotModel(false);
            yield return Frames(3);
            Check(!_hud.RobotModelOn && Object.FindObjectsByType<RobotModel>(FindObjectsSortMode.None).Length == 0 && Find<FirstPersonView>() == null
                  && Find<ArmTargets>() == null && Find<BaseVelocity>() == null && Find<HandCamPip>() == null && AllBlocksVisible(),
                  $"{tag}: model off -> model + overlays gone, blocks visible");
            Check(ModelPref() == 0 && !modelToggle.isOn && !blocksBtn.interactable && Selected(blocksBtn),
                  $"{tag}: pref RobotModel = 0, toggle off, buttons greyed with Blocks shown selected");
        }
        Check(ExceptionCount == exc0, $"1: three model/layout cycles without exceptions ({ExceptionCount - exc0})");
        {   // layout chosen while the model is off is remembered and applied when it comes on
            _hud.SetCameraLayout("firstperson");
            yield return Frames(2);
            Check(!_hud.FirstPerson && AllBlocksVisible() && LayoutPref() == "firstperson" && !fpBtn.interactable,
                  "1: layout set with the model off -> remembered (pref firstperson), nothing changes yet");
            _hud.SetRobotModel(true);
            yield return Frames(3);
            Check(_hud.FirstPerson && AllBlocksInactive() && Find<HandCamPip>() != null, "1: model on -> straight into the first-person layout");
        }

        // ---- 6. strip / RTT (first-person layout now, bar hidden)
        {
            var strip = _hud.Strip;
            Check(Find<RoundTrip>() != null, "6: RoundTrip exists without an experiment registry");
            Check(strip != null && strip.HasModel && strip.gameObject.activeSelf == !Bar.gameObject.activeSelf, $"6: FP: strip {(strip != null)}, active {strip?.gameObject.activeSelf} == !bar {Bar.gameObject.activeSelf}");
            Call(_hud, "ToggleBar"); yield return Frames(3);
            Check(strip.gameObject.activeSelf == !Bar.gameObject.activeSelf && !strip.gameObject.activeSelf, $"6: FP bar shown -> strip inactive");
            Call(_hud, "ToggleBar"); yield return Frames(3);
        }

        // ---- 2. Rename in the first-person layout
        var fpv1 = Find<FirstPersonView>();
        var pip1 = Find<HandCamPip>();
        int headIdx = fpv1.CameraIndex;
        {
            yield return Seconds(1.0);
            _publisher.controlRobot = false;
            yield return Frames(3);
            var headRename = Field<Button>(fpv1, "_rename");
            var cards = Cards(pip1);
            Check(headRename != null && headRename.name == "Button Rename" && headRename.gameObject.activeInHierarchy, $"2: control off -> head card 'Button Rename' active ({headRename?.gameObject.activeInHierarchy})");
            Check(cards.Count == Cards2 && cards.All(c => c.rename.name == "Button Rename" && c.rename.gameObject.activeInHierarchy), $"2: both hand cards' 'Button Rename' active ({cards.Count(c => c.rename.gameObject.activeInHierarchy)}/{cards.Count})");
            {   // Rename above the head card's top edge, right-aligned
                var cardRt = Field<RectTransform>(fpv1, "_cardRt"); var rrt = (RectTransform)headRename.transform;
                var canvas = Field<RectTransform>(fpv1, "_canvasRt");
                Rect R(RectTransform r) { var c4 = new Vector3[4]; r.GetWorldCorners(c4); var a = c4.Select(w => canvas.InverseTransformPoint(w)).ToArray(); return Rect.MinMaxRect(a.Min(v => v.x), a.Min(v => v.y), a.Max(v => v.x), a.Max(v => v.y)); }
                Rect rb = R(rrt), cb = R(cardRt);
                Check(rb.yMin >= cb.yMax - 0.5f && Mathf.Abs(rb.xMax - cb.xMax) < 1f, $"2: head Rename {rb} above the card's top-right corner {cb}");
            }
            Shot("21_fp_rename", head.position, Down(head, 22f), 100f);
            Shot("21_fp_rename_b", head.position, Down(head, 28f), 95f);
            {
                var rects = new List<RectTransform> { Field<RectTransform>(fpv1, "_cardRt"), (RectTransform)headRename.transform };
                foreach (var c in cards) { rects.Add(c.rt); rects.Add((RectTransform)c.rename.transform); }
                Shot("21_fp_rename_f", head.position, Down(head, 12f), 100f, 1600, 1000);
                Shot("21_fp_rename_g", head.position, Down(head, 15f), 104f, 1600, 1000);
                Shot("21_fp_rename_h", head.position, Down(head, 10f), 96f, 1920, 1080);
            }

            var headPanel = _images.Panels[headIdx];
            var headName = Field<TextMeshProUGUI>(fpv1, "_name");
            string original = headPanel.Config.displayName;
            _images.RenameCamera(headPanel, "Front Eye");
            yield return Frames(2);
            Check(headName.text == "Front Eye", $"2: RenameCamera -> head card label '{headName.text}'");
            int open0 = LogCount("[Rename] open: Front Eye");
            headRename.onClick.Invoke();   // Editor: no system keyboard -> TextKeyboard returns the fallback (original name)
            yield return Frames(3);
            Check(LogCount("[Rename] open: Front Eye") == open0 + 1 && headName.text == original && headPanel.Label == original,
                  $"2: head card Rename -> BeginRename on '{headPanel.Config.displayName}' (log), keyboard fallback -> label '{headName.text}' == '{original}'");
            var lc = cards[0];
            var lp = _images.Panels[lc.index];
            _images.RenameCamera(lp, "L Cam");
            yield return Frames(2);
            Check(lc.name.text == "L Cam", $"2: hand card label follows RenameCamera ('{lc.name.text}')");
            lc.rename.onClick.Invoke();
            yield return Frames(3);
            Check(lc.name.text == lp.Config.displayName && lp.Label == lp.Config.displayName && LogCount("[Rename] open: L Cam") == 1,
                  $"2: left hand card Rename -> BeginRename, label back to '{lc.name.text}'");
            _publisher.controlRobot = true;
            yield return Frames(3);
            Check(!headRename.gameObject.activeSelf && cards.All(c => !c.rename.gameObject.activeSelf), "2: control on -> head + hand Rename buttons inactive");
            _publisher.controlRobot = false;
            yield return Frames(3);
            Check(headRename.gameObject.activeSelf && cards.All(c => c.rename.gameObject.activeSelf), "2: control off again -> buttons back");
        }

        // ---- 3. hand card sides
        {
            int li = _images.IndexOf(LeftSuffix), ri = _images.IndexOf(RightSuffix);
            var cards = Cards(pip1);
            var lc = cards.First(c => c.index == li); var rc = cards.First(c => c.index == ri);
            if (Spec.DualArm) Check(lc.left && !rc.left, $"3: Card.left true for the left camera ({lc.left}), false for the right ({rc.left})");
            else Check(!lc.left, $"3: the single hand camera's card goes to the viewer's right (Card.left {lc.left})");
            Check(profile.cameras.Where(c => c.role == RobotProfile.CameraRole.Hand).All(c => cards.First(k => k.index == _images.IndexOf(c.topicSuffix)).left == (c.side == RobotProfile.Side.Left)),
                  "3: every hand card's side is its camera's profile side (Left -> viewer's left, else right)");
            void Sides(string when)
            {
                float xl = head.InverseTransformPoint(lc.rt.position).x, xr = head.InverseTransformPoint(rc.rt.position).x;
                var model = Find<FirstPersonView>().Model;
                float hl = head.InverseTransformPoint(model.Frame(profile.arms[0].effectorFrame).position).x;
                if (!Spec.DualArm) { Check(xl > hl, $"3 [{when}]: head-space x: the hand card {xl:F3} right of the hand {hl:F3}"); return; }
                float hr = head.InverseTransformPoint(model.Frame(profile.arms[1].effectorFrame).position).x;
                Check(xl < xr, $"3 [{when}]: head-space x left card {xl:F3} < right card {xr:F3} (hands L {hl:F3} R {hr:F3})");
            }
            Sides("arms home");
            var p = Sh("pub.sh", "arm"); var w = WaitExit(p); while (w.MoveNext()) yield return w.Current;
            yield return Seconds(3.0);
            Sides("arm");
            Shot("21_fp_arm", head.position, Down(head, 28f), 95f);
            p = Sh("pub.sh", "armhome"); w = WaitExit(p); while (w.MoveNext()) yield return w.Current;
            yield return Seconds(3.0);
            Sides("arms home again");
        }

        // ---- 4. badges
        {
            static float Expect(float viewH) => Mathf.Clamp(CameraBadge.FontRatio * viewH, CameraBadge.MinFontMm, CameraBadge.MaxFontMm);
            var hb = fpv1.Badge; float hvh = Field<RectTransform>(fpv1, "_viewRt").sizeDelta.y;
            Check(hb != null && Mathf.Abs(hb.FontMm - Expect(hvh)) < 0.5f, $"4: head card badge FontMm {hb?.FontMm:F1} ~ clamp(0.07 x {hvh:F0}) = {Expect(hvh):F1}");
            foreach (var c in Cards(pip1))
            {
                float vh = c.view.rectTransform.sizeDelta.y;
                Check(Mathf.Abs(c.badge.FontMm - Expect(vh)) < 0.5f && c.badge.FontMm < hb.FontMm,
                      $"4: {(c.left ? "left" : "right")} hand badge FontMm {c.badge.FontMm:F1} ~ clamp(0.07 x {vh:F0}) = {Expect(vh):F1}, < head {hb.FontMm:F1}; rect {c.badge.Rect.sizeDelta.x:F0}x{c.badge.Rect.sizeDelta.y:F0}");
            }
            Check(Mathf.Abs(hb.Rect.anchoredPosition.x + CameraBadge.MarginRatio * Field<RectTransform>(fpv1, "_viewRt").sizeDelta.x) < 0.5f, $"4: head badge margin {-hb.Rect.anchoredPosition.x:F1} = 3 % of the view width");
            // blocks: back to the blocks layout, a block's view is 1200 mm x size tall
            _hud.SetCameraLayout("blocks");
            yield return Frames(3);
            foreach (var panel in _images.Panels)
            {
                var b = Field<CameraBadge>(panel, "_badge");
                float vh = Field<RawImage>(panel, "_view").rectTransform.sizeDelta.y;
                float want = Expect(1200f * panel.Config.scale * panel.Size);   // a size-1, scale-1 block: 0.07 x 1200 = 84
                Check(Mathf.Abs(b.FontMm - Expect(vh)) < 0.5f && Mathf.Abs(b.FontMm - want) < 0.5f,
                      $"4: block '{panel.Label}' badge FontMm {b.FontMm:F1} ~ clamp(0.07 x {vh:F0}) = clamp(0.07 x 1200 x scale {panel.Config.scale} x size {panel.Size:F2}) = {want:F1}{(Mathf.Approximately(panel.Config.scale * panel.Size, 1f) ? " (size-1 block: 84)" : "")}");
            }
        }

        // ---- 6 (blocks layout, model on): strip
        {
            var strip = _hud.Strip;
            Check(strip != null && !strip.HasModel && strip.gameObject.activeSelf == !Bar.gameObject.activeSelf && !strip.gameObject.activeSelf, $"6: blocks layout: strip exists, inactive while the bar is shown");
            Call(_hud, "ToggleBar"); yield return Frames(3);
            Check(strip.gameObject.activeSelf && !Bar.gameObject.activeSelf, "6: blocks: bar hidden -> strip active");
            Call(_hud, "ToggleBar"); yield return Frames(3);
            _hud.SetRobotModel(false);
            yield return Frames(3);
            strip = _hud.Strip;
            Check(strip != null && !strip.gameObject.activeSelf && Find<RoundTrip>() != null, "6: model off: strip inactive with the bar shown, RoundTrip still there");
        }
        Check(Object.FindObjectsByType<ROSConnection>(FindObjectsSortMode.None).Length == 1, "session 1: exactly one ROSConnection at the end");
        e = ExitPlay(); while (e.MoveNext()) yield return e.Current;

        // =============== Session 2: migration of the old ViewMode pref ===============
        Log("===== session 2: ViewMode migration");
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.1");
        PlayerPrefs.SetString($"ViewMode/{Robot}", "firstperson");
        PlayerPrefs.Save();
        e = EnterTeleop(profile); while (e.MoveNext()) yield return e.Current;
        yield return Frames(3);
        Check(_hud.RobotModelOn && _hud.CameraLayout == "firstperson" && _hud.FirstPerson, $"migration: model {_hud.RobotModelOn}, layout '{_hud.CameraLayout}'");
        Check(ModelPref() == 1 && LayoutPref() == "firstperson" && PlayerPrefs.HasKey($"ViewMode/{Robot}"),
              $"migration: new keys ModelOn {ModelPref()}, Layout '{LayoutPref()}', old ViewMode key kept ({PlayerPrefs.HasKey($"ViewMode/{Robot}")})");
        e = ExitPlay(); while (e.MoveNext()) yield return e.Current;
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.1");
        PlayerPrefs.SetString($"ViewMode/{Robot}", "blocks");
        PlayerPrefs.Save();
        e = EnterTeleop(profile); while (e.MoveNext()) yield return e.Current;
        yield return Frames(3);
        Check(!_hud.RobotModelOn && ModelPref() == -1, "migration: old 'blocks' -> model off, nothing saved");
        e = ExitPlay(); while (e.MoveNext()) yield return e.Current;

        // =============== Session 3: raw images, then the Compressed toggle reopens the robot ===============
        Log("===== session 3: Images/Compressed = 0");
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.1");
        PlayerPrefs.SetInt("Images/Compressed", 0);
        PlayerPrefs.Save();
        e = EnterTeleop(profile); while (e.MoveNext()) yield return e.Current;
        head = FirstPersonView.Head;
        var ros = ROSConnection.GetOrCreateInstance();
        Check(!ImageSubscriber.Compressed && !BarToggle("Compressed").isOn, "5: pref 0 -> Compressed false, bar toggle off");
        Check(_images.Panels.All(p => p.Topic.EndsWith("image_raw") && !p.Topic.EndsWith("/compressed")),
              "5: every block subscribed to the raw twin: " + string.Join(", ", _images.Panels.Select(p => p.Topic)));
        Check(_images.Panels.All(p => ros.HasSubscriber(p.Topic) && !ros.HasSubscriber(p.Topic + "/compressed")), "5: ROSConnection has the raw subscriptions, none on '/compressed'");
        Check(_images.IndexOf(LeftSuffix) >= 0 && _images.IndexOf(LeftSuffix.Replace("/compressed", "")) == _images.IndexOf(LeftSuffix) && _images.IndexOf(profile.FirstPersonCamera.topicSuffix) >= 0,
              $"5: IndexOf matches either twin (left {_images.IndexOf(LeftSuffix)}, head {_images.IndexOf(profile.FirstPersonCamera.topicSuffix)})");
        {
            var counts = new int[_images.Panels.Count];
            Action<int, Texture2D> count = (i, _) => { if (i < counts.Length) counts[i]++; };
            double end = Now + 15;
            while (Now < end && _images.Panels.Any(p => p.State != CameraPanel.FeedState.Live)) yield return Seconds(0.2);
            _images.FrameReady += count;
            yield return Seconds(4.0);
            _images.FrameReady -= count;
            for (int i = 0; i < _images.Panels.Count; i++)
            {
                var tex = Field<RawImage>(_images.Panels[i], "_view").texture;
                Check(counts[i] > 10 && _images.Panels[i].State == CameraPanel.FeedState.Live && tex != null,
                      $"5: raw: '{_images.Panels[i].Label}' {counts[i]} frames in 4 s ({_images.Fps(i):F1} fps), decode {_images.DecodeMs(i):F2} ms, texture {tex?.width}x{tex?.height} {(tex as Texture2D)?.format}");
            }
            Shot("22_raw_images", Fit(head, _images.Panels.Select(p => (RectTransform)p.transform), 0f, out var fov22), fov22);
        }
        {   // head card in the first-person layout on the raw topic
            _hud.SetRobotModel(true);
            _hud.SetCameraLayout("firstperson");
            yield return Frames(3);
            var fpv = Find<FirstPersonView>();
            int f0 = fpv.FramesReceived;
            yield return Seconds(3.0);
            var pip = Find<HandCamPip>();
            var cards = Cards(pip);
            Check(fpv.ImageShown && fpv.FramesReceived > f0 + 10 && fpv.CameraIndex >= 0 && _images.Panels[fpv.CameraIndex].Topic.EndsWith("image_raw"),
                  $"5: raw: FP head card frames {f0} -> {fpv.FramesReceived} in 3 s on '{(fpv.CameraIndex >= 0 ? _images.Panels[fpv.CameraIndex].Topic : "-")}'");
            Check(cards.Count == Cards2 && cards.All(c => c.view.texture != null), $"5: raw: all hand cards show a texture ({cards.Count(c => c.view.texture != null)}/{cards.Count})");
            Shot("22_raw_fp", head.position, Down(head, 22f), 100f);
            _hud.SetCameraLayout("blocks");
            _hud.SetRobotModel(false);
            yield return Frames(3);
        }
        {   // the toggle: back through the selection screen, the robot opens again by itself
            var oldRos = ros;
            int readerExc0 = LogCount("Reader "), reopen0 = LogCount("[RobotSelection] reopening");
            var t = BarToggle("Compressed");
            double t0 = Now;
            t.isOn = true;   // the user's click: saves the pref, AutoOpen, BackToRobotSelection
            Check(ImageSubscriber.Compressed && Settings.Compressed, "5: toggle on -> pref saved = 1");
            double end = Now + 10; bool sawSelection = false; double selAt = -1;
            while (Now < end)
            {
                if (SceneManager.GetActiveScene().name == "RobotSelectionScene" && !sawSelection) { sawSelection = true; selAt = Now - t0; }
                if (sawSelection && SceneManager.GetActiveScene().name == "TeleopScene") break;
                yield return Seconds(0.05);
            }
            double teleAt = Now - t0;
            Check(sawSelection && SceneManager.GetActiveScene().name == "TeleopScene" && teleAt < 3.5 && LogCount("[RobotSelection] reopening") == reopen0 + 1 && RobotSelectionHud.AutoOpen == null,
                  $"5: selection scene after {selAt:F2} s, TeleopScene again after {teleAt:F2} s (< 3.5), 'reopening' logged, AutoOpen consumed");
            yield return Frames(3);
            _publisher = Find<QuestControllerPublisher>(); _publisher.controlRobot = false;
            _hud = Find<TeleopHud>(); _images = Find<ImageSubscriber>();
            end = Now + 20;
            while (Now < end && (_images == null || !_images.IsReady || _images.Panels.Any(p => p.State != CameraPanel.FeedState.Live))) yield return Seconds(0.2);
            var conns = Object.FindObjectsByType<ROSConnection>(FindObjectsSortMode.None);
            Check(RobotProfile.Selected == profile && _images.Profile == profile && _images.Panels.All(p => p.Topic.EndsWith("/compressed")),
                  "5: reopened SOBIT HOME with compressed topics: " + string.Join(", ", _images.Panels.Select(p => p.Topic)));
            Check(conns.Length == 1 && conns[0] != oldRos && conns[0].HasConnectionThread && !conns[0].HasConnectionError,
                  $"5: exactly one ROSConnection ({conns.Length}), new instance {conns.Length == 1 && conns[0] != oldRos}, HasConnectionThread {conns.FirstOrDefault()?.HasConnectionThread}, error {conns.FirstOrDefault()?.HasConnectionError}");
            var counts = new int[_images.Panels.Count];
            Action<int, Texture2D> count = (i, _) => { if (i < counts.Length) counts[i]++; };
            _images.FrameReady += count;
            yield return Seconds(4.0);
            _images.FrameReady -= count;
            Check(counts.All(c => c > 10), $"5: reopened: frames in 4 s per camera [{string.Join(", ", counts)}]");
            Check(LogCount("Reader ") == readerExc0, $"5: no 'Reader … exception' in the log ({LogCount("Reader ") - readerExc0})");
            Check(BarToggle("Compressed").isOn, "5: reopened bar: Compressed images on");
        }
        e = ExitPlay(); while (e.MoveNext()) yield return e.Current;
        PlayerPrefs.DeleteAll(); PlayerPrefs.Save();
    }

    static bool Selected(Button b) => b != null && Mathf.Abs(b.image.color.a - 0.25f) < 0.01f;

    // Camera pose looking from the head at the union of the rects, with a FOV that fits them.
    static Pose Fit(Transform head, IEnumerable<RectTransform> rects, float back, out float fov, float margin = 1.12f)
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
        fov = Mathf.Clamp(2f * Mathf.Max(maxV, hHalf) * margin, 10f, 110f);
        return new Pose(pos, rot);
    }

    static void Shot(string name, Pose pose, float fov, int w = 1280, int h = 720) => Shot(name, pose.position, pose.rotation, fov, w, h);

    static void Shot(string name, Vector3 pos, Quaternion rot, float fov, int w = 1280, int h = 720)
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
        var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;
        File.WriteAllBytes(Path.Combine(ShotDir, name + ".png"), tex.EncodeToPNG());
        cam.targetTexture = null; Object.DestroyImmediate(rt); Object.DestroyImmediate(go); Object.DestroyImmediate(tex);
        Log($"    screenshot {name}.png (fov {fov:F1})");
    }
}
