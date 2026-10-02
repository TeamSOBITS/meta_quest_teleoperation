// Verify harness: Teleop HUD layout in Play mode (isolated RosIP, no live ROS).
// Run: tools/verify.sh --suite LayoutVerifier   (or Unity -batchmode -projectPath <copy> -executeMethod LayoutVerifier.Run)
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

// Headless Play-mode check of the teleop HUD. Copy-only test harness, not part of the project.
public static class LayoutVerifier
{
    static IEnumerator _run;
    static int _failures;
    static string ShotDir => System.Environment.GetEnvironmentVariable("VERIFY_SHOTS");

    public static void Run()
    {
        EditorSettings.enterPlayModeOptionsEnabled = true;
        EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.DisableDomainReload;
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.2");   // isolated: never reach a live ros_tcp_endpoint
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
                Log(_failures == 0 ? "ALL CHECKS PASSED" : $"{_failures} CHECK(S) FAILED");
                EditorApplication.Exit(_failures == 0 ? 0 : 1);
            }
        }
        catch (System.Exception e)
        {
            Debug.LogException(e);
            EditorApplication.Exit(3);
        }
    }
    static object Frames(int n) { _waitFrame = Time.frameCount + n; _waitUntil = EditorApplication.timeSinceStartup + 0.05; return null; }
    static object Seconds(double s) { _waitUntil = EditorApplication.timeSinceStartup + s; return null; }

    static void Log(string m) => Debug.Log("[Verify] " + m);
    static void Check(bool ok, string what) { if (!ok) _failures++; Log((ok ? "PASS " : "FAIL ") + what); }

    static IEnumerator Main()
    {
        foreach (var robot in new[] { "SOBIT_HOME", "SOBIT_LIGHT" })
        {
            var profile = AssetDatabase.LoadAssetAtPath<RobotProfile>($"Assets/Robots/{robot}.asset");
            EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
            RobotProfile.Selected = profile;
            EditorApplication.EnterPlaymode();
            while (!Application.isPlaying) yield return Seconds(0.1);
            yield return Frames(3);
            bool hintsShown = GameObject.Find("Controller Hints") != null;   // they fade after 10 s
            yield return Frames(30);
            yield return Seconds(1.0);

            var images = Object.FindFirstObjectByType<ImageSubscriber>();
            var publisher = Object.FindFirstObjectByType<QuestControllerPublisher>();
            var panels = images.Panels.ToList();
            Log($"===== {robot}: {panels.Count} cameras");
            Check(panels.Count == profile.cameras.Length, $"{robot}: one block per camera");
            Check(hintsShown, $"{robot}: controller hints shown at start");
            Check(GameObject.Find("HUD Bar") != null, $"{robot}: HUD bar exists");
            var toggles = GameObject.Find("HUD Bar")?.GetComponentsInChildren<UnityEngine.UI.Toggle>(true).ToArray();
            // round 3: + Compressed images, "First person" toggle -> "Robot model" (layout buttons are Buttons)
            Check(toggles != null && toggles.Length == panels.Count + 5, $"{robot}: bar has Control robot, Lazy follow, Passthrough, Compressed images, Robot model + {panels.Count} camera toggles ({toggles?.Length})");
            {   // 7: 5000 mm bar: every toggle / button label fits on one line inside its rect (no clipping, no wrap)
                var barGo = GameObject.Find("HUD Bar");
                var barRt = (RectTransform)barGo.transform;
                Check(Mathf.Abs(barRt.sizeDelta.x - 5000f) < 1f && Mathf.Abs(barRt.localScale.x * 1000f - 0.7f) < 1e-4f, $"{robot}: bar 5000 mm wide at 0.7 scale (= {barRt.sizeDelta.x * barRt.localScale.x:F2} m) ({barRt.sizeDelta.x:F0} mm, scale {barRt.localScale.x * 1000f:F3}/1000)");
                Canvas.ForceUpdateCanvases();
                var bad = new List<string>(); var info = new List<string>();
                var labels = barGo.GetComponentsInChildren<UnityEngine.UI.Toggle>(true).Select(t => t.transform.Find("Label")?.GetComponent<TMPro.TextMeshProUGUI>())
                    .Concat(barGo.GetComponentsInChildren<UnityEngine.UI.Button>(true).Select(bt => bt.GetComponentInChildren<TMPro.TextMeshProUGUI>(true)))
                    .Where(l => l != null && !string.IsNullOrEmpty(l.text)).ToList();
                foreach (var l in labels)
                {
                    l.ForceMeshUpdate();
                    float pw = l.GetPreferredValues(l.text, Mathf.Infinity, Mathf.Infinity).x, rw = l.rectTransform.rect.width;
                    bool ok = !l.isTextOverflowing && pw <= rw + 0.5f && l.textInfo.lineCount <= 1;
                    bool camera = panels.Any(p => p.Label == l.text);
                    if (!ok && camera) bad.Add($"'{l.text}' pref {pw:F0} > rect {rw:F0} (overflow {l.isTextOverflowing}, lines {l.textInfo.lineCount})");
                    else if (!ok) info.Add($"(wraps, not clipped: '{l.text}' {pw:F0}/{rw:F0} mm, {l.textInfo.lineCount} lines, overflow {l.isTextOverflowing})");
                    if (l.text.Contains("Hand Camera") || l.text.Contains("Compressed") || l.text.Contains("Robot model") || l.text.Contains("First person")) info.Add($"'{l.text}' {pw:F0}/{rw:F0} mm");
                }
                Check(labels.Count > 0 && bad.Count == 0, $"{robot}: 7: camera toggle labels fit their rects on one line ({labels.Count} labels checked) [{string.Join("; ", info)}] {string.Join("; ", bad)}");
            }

            var initial = panels.ToDictionary(p => p, p => p.transform.localPosition);
            Inspect(robot, "all cameras");
            Shot($"{robot}_1_all");

            // Hide cameras one at a time (auto layout).
            var hideOrder = panels.AsEnumerable().Reverse().Take(panels.Count - 1).ToList();
            int step = 2;
            foreach (var p in hideOrder)
            {
                float before = panels.Where(q => q.Visible && q != p).Select(q => q.Size).DefaultIfEmpty(1).Min();
                images.SetCameraVisible(p, false);
                yield return Frames(3);
                var vis = panels.Where(q => q.Visible).ToList();
                Inspect(robot, $"hidden: {string.Join(", ", panels.Where(q => !q.Visible).Select(q => q.Config.displayName))}");
                Check(vis.All(q => q.Size >= before - 1e-3f), $"{robot}: remaining views did not shrink (min size {vis.Min(q => q.Size):F2} >= {before:F2})");
                Check(vis.All(q => q.Size <= images.maxGrow + 1e-3f), $"{robot}: growth capped at {images.maxGrow}");
                Shot($"{robot}_{step++}_{vis.Count}visible");
            }

            // Reset with all visible -> default layout restored.
            foreach (var p in panels) images.SetCameraVisible(p, true);
            images.ResetLayout();
            yield return Frames(3);
            Check(panels.All(p => (p.transform.localPosition - initial[p]).magnitude < 1e-3f && Mathf.Abs(p.Size - 1f) < 1e-3f),
                  $"{robot}: showing all + Reset restores the default layout");

            // Joy toggle gates dragging.
            Check(panels.All(p => !p.Interactable.enabled), $"{robot}: blocks not grabbable while Control robot is on");
            publisher.controlRobot = false;
            yield return Frames(3);
            Check(panels.All(p => p.Interactable.enabled), $"{robot}: blocks grabbable while Control robot is off");
            publisher.controlRobot = true;
            yield return Frames(3);
            Check(panels.All(p => !p.Interactable.enabled), $"{robot}: blocks not grabbable again after Control robot back on");

            // Custom layout: a dragged (saved) block freezes the layout.
            var moved = panels[0];
            var newPos = Quaternion.Euler(0, -8, 0) * moved.transform.localPosition;
            moved.transform.localPosition = newPos;
            images.SavePosition(moved);
            Check(images.HasCustomLayout, $"{robot}: dragged position saved -> custom layout");
            var frozen = panels.ToDictionary(p => p, p => p.transform.localPosition);
            images.SetCameraVisible(panels[panels.Count - 1], false);
            yield return Frames(3);
            Check(panels.Where(p => p.Visible).All(p => (p.transform.localPosition - frozen[p]).magnitude < 1e-4f),
                  $"{robot}: hiding a camera in custom layout leaves the others in place");
            images.SetCameraVisible(panels[panels.Count - 1], true);
            images.ResetLayout();
            yield return Frames(3);
            Check(!images.HasCustomLayout && panels.All(p => (p.transform.localPosition - initial[p]).magnitude < 1e-3f),
                  $"{robot}: Reset clears the custom layout");

            // Step 1: header, back button, camera feed states.
            var bar = GameObject.Find("HUD Bar");
            var joyLabel = bar.transform.Find("Joy State/Label").GetComponent<TMPro.TextMeshProUGUI>();
            Check(joyLabel.text == "CONTROL ON", $"{robot}: bar shows JOY ON while Joy is published");
            publisher.controlRobot = false; yield return Frames(3);
            Check(joyLabel.text == "LAYOUT MODE", $"{robot}: bar shows LAYOUT MODE when Joy is off");
            publisher.controlRobot = true; yield return Frames(3);
            Check(bar.GetComponentsInChildren<UnityEngine.UI.Button>().Any(b => b.name.Contains("Robots")), $"{robot}: bar has a Back to robots button");
            Check(bar.transform.Find("Robot").GetComponent<TMPro.TextMeshProUGUI>().text == profile.displayName, $"{robot}: bar shows the robot name");
            Check(panels.All(p => p.State == CameraPanel.FeedState.Waiting), $"{robot}: cameras start in Waiting");

            var fake = new Texture2D(64, 48);
            var px = new Color[64 * 48];
            for (int y = 0; y < 48; y++) for (int x = 0; x < 64; x++) px[y * 64 + x] = Color.Lerp(new Color(0.55f, 0.6f, 0.66f), new Color(0.35f, 0.3f, 0.25f), y / 47f) * (0.8f + 0.2f * Mathf.Sin(x * 0.3f));
            fake.SetPixels(px); fake.Apply();
            var live = panels.Where((p, i) => i % 2 == 0).ToList();          // keep feeding
            var stale = panels.Count > 1 ? panels[1] : null;                  // one frame, then nothing
            var waiting = panels.Count > 3 ? panels[3] : null;               // never fed
            stale?.SetTexture(fake);
            double end = EditorApplication.timeSinceStartup + 2.0;
            while (EditorApplication.timeSinceStartup < end)
            {
                foreach (var p in live) p.SetTexture(fake);
                yield return Seconds(1.0 / 15);
            }
            Check(live.All(p => p.State == CameraPanel.FeedState.Live), $"{robot}: fed cameras show Live");
            if (stale != null) Check(stale.State == CameraPanel.FeedState.Stale, $"{robot}: camera without new frames for >1 s shows Stale");
            if (waiting != null) Check(waiting.State == CameraPanel.FeedState.Waiting, $"{robot}: never-fed camera stays Waiting");
            foreach (var p in live) p.SetTexture(fake);
            yield return Frames(2);
            Inspect(robot, "feed states");
            panels[panels.Count - 1].SetHighlight(CameraPanel.Highlight.Drag);
            Shot($"{robot}_9_states");
            panels[panels.Count - 1].SetHighlight(CameraPanel.Highlight.None);

            // Step 4: controller hints, lazy follow toggle and behaviour.
            Check(bar.GetComponentsInChildren<UnityEngine.UI.Toggle>().Any(t => t.name.Contains("Lazy follow")), $"{robot}: bar has a Lazy follow toggle");
            var hud = Object.FindFirstObjectByType<TeleopHud>();
            var cam = Camera.main.transform;
            var beforeLazy = panels.ToDictionary(p => p, p => p.transform.localPosition);
            hud.SetLazyFollow(true);
            yield return Frames(3);
            var anchor = panels[0].transform.parent;
            Check(anchor != cam && panels.All(p => p.transform.parent == anchor) && bar.transform.parent == anchor, $"{robot}: lazy follow moves HUD under the follow anchor");
            Check(panels.All(p => (p.transform.localPosition - beforeLazy[p]).magnitude < 1e-4f), $"{robot}: switching lazy follow keeps panel positions");
            var offset = cam.parent;  // Camera Offset; turning it turns the head
            var startRot = offset.localRotation;
            offset.localRotation = startRot * Quaternion.Euler(0, 5, 0);
            yield return Seconds(0.5);
            float small = Quaternion.Angle(anchor.rotation, cam.rotation);
            Check(small > 3f, $"{robot}: 5 deg head turn stays inside the dead zone (HUD lags {small:F1} deg)");
            offset.localRotation = startRot * Quaternion.Euler(0, 30, 0);
            yield return Seconds(2.0);
            float caught = Quaternion.Angle(anchor.rotation, cam.rotation);
            Check(caught < 2f, $"{robot}: after a 30 deg head turn the HUD catches up (off by {caught:F1} deg)");
            offset.localRotation = startRot;
            hud.SetLazyFollow(false);
            yield return Frames(3);
            Check(panels.All(p => p.transform.parent == cam) && bar.transform.parent == cam, $"{robot}: lazy follow off re-attaches HUD to the head");

            // Step 5: passthrough toggle.
            var pt = bar.GetComponentsInChildren<UnityEngine.UI.Toggle>().FirstOrDefault(t => t.name.Contains("Passthrough"));
            Check(pt != null, $"{robot}: bar has a Passthrough toggle");
            if (pt != null)
            {
                var floor = Resources.FindObjectsOfTypeAll<Transform>().FirstOrDefault(t => !(t is RectTransform) && t.gameObject.scene.IsValid() && t.CompareTag(PassthroughMode.FloorTag));
                Check(floor != null && floor.gameObject.activeSelf, $"{robot}: floor visible before passthrough");
                Check(Camera.main.backgroundColor.a > 0.99f, $"{robot}: camera background opaque before passthrough");
                pt.isOn = true;
                yield return Frames(3);
                Check(Camera.main.backgroundColor.a == 0f && Camera.main.clearFlags == CameraClearFlags.SolidColor, $"{robot}: passthrough on -> camera background alpha 0");
                Check(!floor.gameObject.activeSelf, $"{robot}: passthrough on -> floor hidden");
                Check(PassthroughMode.Enabled, $"{robot}: passthrough choice saved");
                var mgr = Camera.main.GetComponent<UnityEngine.XR.ARFoundation.ARCameraManager>();
                Check(Object.FindFirstObjectByType<UnityEngine.XR.ARFoundation.ARSession>() != null && mgr != null
                      && mgr.enabled == (UnityEngine.XR.ARFoundation.ARSession.state >= UnityEngine.XR.ARFoundation.ARSessionState.Ready),
                      $"{robot}: ARSession + ARCameraManager, started once the session is ready (state {UnityEngine.XR.ARFoundation.ARSession.state})");
                Shot($"{robot}_10_passthrough");
                pt.isOn = false;
                yield return Frames(3);
                Check(Camera.main.backgroundColor.a > 0.99f && floor.gameObject.activeSelf, $"{robot}: passthrough off restores background and floor");
                Check(!Camera.main.GetComponent<UnityEngine.XR.ARFoundation.ARCameraManager>().enabled, $"{robot}: ARCameraManager disabled when off");
                Check(!PassthroughMode.Enabled, $"{robot}: passthrough off saved");
            }

            // Control off: nothing is published (TF + Joy gated); hands' UI interactors are on.
            publisher.controlRobot = false; yield return Frames(3);
            Check(panels.All(p => p.transform.Find("Button Rename") != null && p.transform.Find("Button Rename").gameObject.activeSelf),
                  $"{robot}: layout mode shows Rename on every camera");
            publisher.controlRobot = true; yield return Frames(3);
            Check(panels.All(p => !p.transform.Find("Button Rename").gameObject.activeSelf), $"{robot}: Rename hidden while controlling");
            var handUi = Object.FindObjectsByType<UnityEngine.XR.Interaction.Toolkit.Interactors.NearFarInteractor>(FindObjectsInactive.Include, FindObjectsSortMode.None)
                .Where(n => n.transform.parent != null && n.transform.parent.name.Contains("Hand")).ToList();
            Check(handUi.Count > 0 && handUi.All(n => n.gameObject.activeSelf), $"{robot}: hand Near-Far interactors switched on ({handUi.Count})");

            // Rename a camera and restore it.
            var first = panels[0]; string original = first.Config.displayName;
            images.RenameCamera(first, "Renamed Cam"); yield return Frames(2);
            var toggleText = bar.GetComponentsInChildren<UnityEngine.UI.Toggle>(true).SelectMany(t => t.GetComponentsInChildren<TMPro.TextMeshProUGUI>()).Select(t => t.text).ToList();
            Check(first.Label == "Renamed Cam" && toggleText.Contains("Renamed Cam"), $"{robot}: rename updates the card and the bar toggle");
            images.RenameCamera(first, ""); yield return Frames(2);
            Check(first.Label == original, $"{robot}: empty name restores the original");

            EditorApplication.ExitPlaymode();
            while (Application.isPlaying) yield return Seconds(0.1);
            yield return Seconds(0.5);
        }
    }

    // Angular extents (deg) of a RectTransform as seen from the head.
    struct Box { public string name; public float l, r, b, t; }
    static Box Angles(RectTransform rt, Transform head, string name)
    {
        var c = new Vector3[4]; rt.GetWorldCorners(c);
        var bx = new Box { name = name, l = 999, r = -999, b = 999, t = -999 };
        foreach (var w in c)
        {
            var v = head.InverseTransformPoint(w);
            float h = Mathf.Atan2(v.x, v.z) * Mathf.Rad2Deg, e = Mathf.Atan2(v.y, new Vector2(v.x, v.z).magnitude) * Mathf.Rad2Deg;
            bx.l = Mathf.Min(bx.l, h); bx.r = Mathf.Max(bx.r, h); bx.b = Mathf.Min(bx.b, e); bx.t = Mathf.Max(bx.t, e);
        }
        return bx;
    }

    static void Inspect(string robot, string state)
    {
        var images = Object.FindFirstObjectByType<ImageSubscriber>();
        var head = images.panelParent;
        var boxes = images.Panels.Where(p => p.Visible)
            .Select(p => Angles((RectTransform)p.transform, head, p.Config.displayName)).ToList();
        Log($"--- {robot} [{state}]");
        foreach (var p in images.Panels.Where(p => p.Visible))
        {
            var b = boxes.First(x => x.name == p.Config.displayName);
            Log($"    {p.Config.displayName,-18} size {p.Size:F2}  view {p.Width:F2} m wide  " +
                $"horiz {b.l,6:F1}..{b.r,6:F1} deg  vert {b.b,6:F1}..{b.t,6:F1} deg");
        }
        var rows = images.Panels.Where(p => p.Visible).GroupBy(p => Mathf.Round(p.transform.localPosition.y * 20f))
            .OrderByDescending(g => g.Key).Select(g => g.Count().ToString());
        Log($"    grid rows: {string.Join(" + ", rows)}");
        var bar = Angles((RectTransform)GameObject.Find("HUD Bar").transform, head, "HUD Bar");
        Log($"    {"HUD Bar",-18} horiz {bar.l,6:F1}..{bar.r,6:F1} deg  vert {bar.b,6:F1}..{bar.t,6:F1} deg");

        // Exact test: project each flat quad through the eye onto the plane z = 1 (straight edges
        // stay straight), then separating-axis test on the resulting convex quads.
        var quads = images.Panels.Where(p => p.Visible)
            .Select(p => (p.Config.displayName, Project((RectTransform)p.transform, head))).ToList();
        quads.Add(("HUD Bar", Project((RectTransform)GameObject.Find("HUD Bar").transform, head)));
        bool overlap = false;
        for (int i = 0; i < quads.Count; i++)
            for (int j = i + 1; j < quads.Count; j++)
            {
                float gap = Separation(quads[i].Item2, quads[j].Item2);
                if (gap <= 0f) { overlap = true; Log($"    overlap: {quads[i].Item1} <-> {quads[j].Item1}"); }
            }
        var all = boxes.Concat(new[] { bar }).ToList();
        Check(!overlap, $"{robot} [{state}]: no blocks overlap each other or the HUD bar");
        Check(all.All(x => x.l > -45 && x.r < 45 && x.b > -45 && x.t < 45), $"{robot} [{state}]: everything within +/-45 deg");
    }

    static Vector2[] Project(RectTransform rt, Transform head)
    {
        var c = new Vector3[4]; rt.GetWorldCorners(c);
        return c.Select(w => { var v = head.InverseTransformPoint(w); return new Vector2(v.x / v.z, v.y / v.z); }).ToArray();
    }

    // Largest separating gap between two convex polygons (> 0: apart, <= 0: overlapping).
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

    static void Shot(string name)
    {
        if (string.IsNullOrEmpty(ShotDir)) return;
        var head = Object.FindFirstObjectByType<ImageSubscriber>().panelParent;
        var go = new GameObject("VerifyCam");
        var cam = go.AddComponent<Camera>();
        go.transform.SetPositionAndRotation(head.position, head.rotation);
        cam.fieldOfView = 80f; cam.aspect = 1.6f; cam.nearClipPlane = 0.05f; cam.farClipPlane = 200f;
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
        cam.targetTexture = null; Object.DestroyImmediate(rt); Object.DestroyImmediate(go);
        Log("    screenshot " + name + ".png");
    }
}
