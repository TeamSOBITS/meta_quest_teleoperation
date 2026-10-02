// Verify harness: baseline pictures + scene statistics. Renders fixed poses to VERIFY_SHOTS/baseline/*.png
// (selection.png, teleop_blocks.png, teleop_model_blocks.png, teleop_firstperson.png) against the live sim and writes
// baseline/scene_stats.json (skybox, ambient, fog, lighting data, lights, missing scripts / references per scene + rig prefabs).
// Needs the live sim (HOME) on 127.0.0.1:10000 and no other ROS client (stop the app on the headset).
// Run: tools/verify.sh --shots   (or Unity -batchmode -projectPath <copy> -executeMethod SceneShots.Run)
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reflection;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;

public static class SceneShots
{
    [Serializable] class SceneStat
    {
        public string scene, skybox, ambientMode, lightingData;
        public bool fog;
        public int lights, missingScripts, missingReferences;
    }
    [Serializable] class PrefabStat { public string prefab; public int missingScripts, missingReferences; }
    [Serializable] class Stats { public List<SceneStat> scenes = new List<SceneStat>(); public List<PrefabStat> prefabs = new List<PrefabStat>(); }

    static IEnumerator _run; static double _waitUntil; static int _waitFrame;
    static string OutDir
    {
        get
        {
            var d = Environment.GetEnvironmentVariable("VERIFY_SHOTS");
            if (string.IsNullOrEmpty(d)) d = Path.GetFullPath(Path.Combine(Application.dataPath, "../../shots"));
            d = Path.Combine(d, "baseline"); Directory.CreateDirectory(d); return d;
        }
    }
    static void Log(string m) => Debug.Log("[Shots] " + m);
    static double Now => EditorApplication.timeSinceStartup;
    static object Seconds(double s) { _waitUntil = Now + s; return null; }
    static object Frames(int n) { _waitFrame = Time.frameCount + n; _waitUntil = Now + 0.05; return null; }
    static T Find<T>() where T : Object => Object.FindFirstObjectByType<T>();

    public static void Run()
    {
        EditorSettings.enterPlayModeOptionsEnabled = true;
        EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.DisableDomainReload;
        _run = Main(); EditorApplication.update += Tick;
    }

    static void Tick()
    {
        if (Now < _waitUntil) return;
        if (Application.isPlaying && Time.frameCount < _waitFrame) return;
        try
        {
            if (!_run.MoveNext()) { EditorApplication.update -= Tick; Log("done"); EditorApplication.Exit(0); }
        }
        catch (Exception e) { Debug.LogException(e); EditorApplication.Exit(3); }
    }

    // ---------------- statistics ----------------
    static (int scripts, int refs) CountMissing(IEnumerable<GameObject> roots)
    {
        int scripts = 0, refs = 0;
        foreach (var root in roots)
            foreach (var go in root.GetComponentsInChildren<Transform>(true).Select(t => t.gameObject))
            {
                scripts += GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(go);
                foreach (var c in go.GetComponents<Component>())
                {
                    if (c == null) continue;
                    var so = new SerializedObject(c);
                    var p = so.GetIterator();
                    while (p.NextVisible(true))
                        if (p.propertyType == SerializedPropertyType.ObjectReference && p.objectReferenceValue == null
                            && p.objectReferenceInstanceIDValue != 0) refs++;
                }
            }
        return (scripts, refs);
    }

    static SceneStat SceneInfo(string path, Stats stats)
    {
        var scene = EditorSceneManager.OpenScene(path);
        var m = CountMissing(scene.GetRootGameObjects());
        var s = new SceneStat
        {
            scene = Path.GetFileNameWithoutExtension(path),
            skybox = RenderSettings.skybox != null ? RenderSettings.skybox.name : null,
            ambientMode = RenderSettings.ambientMode.ToString(),
            fog = RenderSettings.fog,
            lightingData = Lightmapping.lightingDataAsset != null ? Lightmapping.lightingDataAsset.name : null,
            lights = Object.FindObjectsByType<Light>(FindObjectsInactive.Include, FindObjectsSortMode.None).Length,
            missingScripts = m.scripts, missingReferences = m.refs,
        };
        // Prefabs instantiated in the scene (the rig).
        foreach (var go in scene.GetRootGameObjects().SelectMany(r => r.GetComponentsInChildren<Transform>(true)).Select(t => t.gameObject))
        {
            if (!PrefabUtility.IsOutermostPrefabInstanceRoot(go)) continue;
            string pp = AssetDatabase.GetAssetPath(PrefabUtility.GetCorrespondingObjectFromSource(go));
            if (string.IsNullOrEmpty(pp) || stats.prefabs.Any(x => x.prefab == pp)) continue;
            var contents = PrefabUtility.LoadPrefabContents(pp);
            var pm = CountMissing(new[] { contents });
            PrefabUtility.UnloadPrefabContents(contents);
            stats.prefabs.Add(new PrefabStat { prefab = pp, missingScripts = pm.scripts, missingReferences = pm.refs });
        }
        Log($"scene {s.scene}: skybox={s.skybox ?? "none"} ambient={s.ambientMode} fog={s.fog} lighting={s.lightingData ?? "none"} lights={s.lights} missingScripts={s.missingScripts} missingRefs={s.missingReferences}");
        return s;
    }

    // ---------------- pictures ----------------
    static Quaternion Down(Transform head, float deg) => Quaternion.Euler(deg, head.eulerAngles.y, 0f);

    static void Shot(string name, Vector3 pos, Quaternion rot, float fov, int w = 1600, int h = 900)
    {
        var go = new GameObject("ShotCam"); var cam = go.AddComponent<Camera>();
        go.transform.SetPositionAndRotation(pos, rot);
        var main = Camera.main;
        cam.fieldOfView = fov; cam.aspect = (float)w / h; cam.nearClipPlane = 0.05f; cam.farClipPlane = 200f;
        cam.stereoTargetEye = StereoTargetEyeMask.None;
        if (main != null) { cam.clearFlags = main.clearFlags; cam.backgroundColor = main.backgroundColor; }
        var rt = new RenderTexture(w, h, 24); cam.targetTexture = rt; cam.Render(); RenderTexture.active = rt;
        var tex = new Texture2D(w, h, TextureFormat.RGB24, false); tex.ReadPixels(new Rect(0, 0, w, h), 0, 0); tex.Apply();
        RenderTexture.active = null;
        File.WriteAllBytes(Path.Combine(OutDir, name + ".png"), tex.EncodeToPNG());
        cam.targetTexture = null; Object.DestroyImmediate(rt); Object.DestroyImmediate(go); Object.DestroyImmediate(tex);
        Log("wrote " + name + ".png");
    }

    static Process Sh(string script, string args)
        => Process.Start(new ProcessStartInfo("/bin/bash", $"\"{VerifyPaths.Tool(script)}\" {args}") { UseShellExecute = false, CreateNoWindow = true });
    static IEnumerator WaitExit(Process p, double timeout = 90) { double end = Now + timeout; while (!p.HasExited && Now < end) yield return Seconds(0.1); }

    static IEnumerator ExitPlay() { EditorApplication.ExitPlaymode(); while (Application.isPlaying) yield return Seconds(0.1); yield return Seconds(0.5); }

    static IEnumerator Main()
    {
        // 1. statistics (edit mode)
        var stats = new Stats();
        stats.scenes.Add(SceneInfo("Assets/Scenes/RobotSelectionScene.unity", stats));
        stats.scenes.Add(SceneInfo("Assets/Scenes/TeleopScene.unity", stats));
        foreach (var p in stats.prefabs) Log($"prefab {p.prefab}: missingScripts={p.missingScripts} missingRefs={p.missingReferences}");
        File.WriteAllText(Path.Combine(OutDir, "scene_stats.json"), JsonUtility.ToJson(stats, true));

        // 2. selection screen
        PlayerPrefs.DeleteAll(); PlayerPrefs.SetString("RosIPAddress", "127.0.0.1"); PlayerPrefs.Save();
        var robotsDir = Path.Combine(Application.persistentDataPath, "robots");
        if (Directory.Exists(robotsDir)) Directory.Delete(robotsDir, true);
        EditorSceneManager.OpenScene("Assets/Scenes/RobotSelectionScene.unity");
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Seconds(0.1);
        yield return Seconds(3.0);
        { var head = Camera.main.transform; Shot("selection", head.position, head.rotation, 80f); }
        { var e = ExitPlay(); while (e.MoveNext()) yield return e.Current; }

        // 3. teleop scene against the live sim
        { var h = Sh("home.sh", ""); var w = WaitExit(h); while (w.MoveNext()) yield return w.Current; }
        { var a = Sh("pub.sh", "armhome"); var b = Sh("pub.sh", "head 0.0 0.0"); var w = WaitExit(a); while (w.MoveNext()) yield return w.Current; w = WaitExit(b); while (w.MoveNext()) yield return w.Current; }
        PlayerPrefs.DeleteAll(); PlayerPrefs.SetString("RosIPAddress", "127.0.0.1"); PlayerPrefs.Save();
        var profile = AssetDatabase.LoadAssetAtPath<RobotProfile>("Assets/Robots/SOBIT_HOME.asset");
        EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
        RobotProfile.Selected = profile;
        Find<QuestControllerPublisher>().controlRobot = false;
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Seconds(0.1);
        Find<QuestControllerPublisher>().controlRobot = false;
        yield return Frames(2);
        var hud = Find<TeleopHud>(); var images = Find<ImageSubscriber>();
        { double end = Now + 10; while (Now < end && (images.Panels.Count == 0 || images.Panels.Any(p => p.State != CameraPanel.FeedState.Live))) yield return Seconds(0.2); }
        yield return Seconds(1.0);
        var headT = FirstPersonView.Head != null ? FirstPersonView.Head : Camera.main.transform;
        Log($"live panels {images.Panels.Count(p => p.State == CameraPanel.FeedState.Live)}/{images.Panels.Count}");
        Shot("teleop_blocks", headT.position, headT.rotation, 90f);

        hud.SetRobotModel(true);
        yield return Frames(3);
        FirstPersonView fpv = null;
        { double end = Now + 10; while (Now < end) { fpv = Find<FirstPersonView>(); if (fpv != null && fpv.Model != null && fpv.Model.AcceptedTransforms > 0) break; yield return Seconds(0.2); } }
        if (fpv != null) fpv.Recenter();
        yield return Seconds(1.0);
        headT = FirstPersonView.Head != null ? FirstPersonView.Head : Camera.main.transform;
        Shot("teleop_model_blocks", headT.position, Down(headT, 25f), 90f);

        hud.SetCameraLayout("firstperson");
        yield return Frames(3);
        { double end = Now + 10; while (Now < end && (fpv == null || fpv.FramesReceived == 0)) yield return Seconds(0.2); }
        yield return Seconds(1.0);
        Shot("teleop_firstperson", headT.position, Down(headT, 22f), 100f);

        { var e = ExitPlay(); while (e.MoveNext()) yield return e.Current; }
    }
}
