// Verify harness: Renders model pictures to SHOTS_DIR (default /tmp/shots); diagnostic, not a pass/fail suite.
// Env VERIFY_ROBOT (default SOBIT_HOME) picks the prefab.
// Run: tools/verify.sh --suite ModelShots   (or Unity -batchmode -projectPath <copy> -executeMethod ModelShots.Run)
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class ModelShots
{
    public static void Run()
    {
        try { Go(); EditorApplication.Exit(0); }
        catch (System.Exception e) { Debug.LogError("SHOTS: " + e); EditorApplication.Exit(1); }
    }

    static void Go()
    {
        string outDir = System.Environment.GetEnvironmentVariable("SHOTS_DIR");
        if (string.IsNullOrEmpty(outDir)) outDir = "/tmp/shots";
        Directory.CreateDirectory(outDir);
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Robots/Models/" + (System.Environment.GetEnvironmentVariable("VERIFY_ROBOT") ?? "SOBIT_HOME") + ".prefab");
        var robot = (GameObject)PrefabUtility.InstantiatePrefab(prefab);

        var lg = new GameObject("Sun"); var l = lg.AddComponent<Light>();
        l.type = LightType.Directional; l.intensity = 2.0f; l.shadows = LightShadows.None;
        lg.transform.rotation = Quaternion.Euler(40, 30, 0);
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
        RenderSettings.ambientLight = new Color(0.9f, 0.9f, 0.95f);

        Bounds b = default; bool first = true;
        foreach (var r in robot.GetComponentsInChildren<Renderer>()) { if (first) { b = r.bounds; first = false; } else b.Encapsulate(r.bounds); }
        Debug.Log("SHOTS: world bounds " + b);

        var cg = new GameObject("Cam"); var cam = cg.AddComponent<Camera>();
        cam.orthographic = true; cam.clearFlags = CameraClearFlags.SolidColor; cam.backgroundColor = new Color(0.82f, 0.86f, 0.9f);
        cam.nearClipPlane = 0.01f; cam.farClipPlane = 20f;
        var rt = new RenderTexture(1024, 1024, 24, RenderTextureFormat.ARGB32);
        cam.targetTexture = rt;

        // name, camera direction (looking along), up, ortho half size
        Shot(cam, rt, b, "front", new Vector3(0, 0, -1), Vector3.up, outDir);   // camera in front of robot (+z) looking back
        Shot(cam, rt, b, "side", new Vector3(-1, 0, 0), Vector3.up, outDir);    // camera at robot's right (+x)
        Shot(cam, rt, b, "top", new Vector3(0, -1, 0), new Vector3(0, 0, 1), outDir); // from above, robot front is up in the image
    }

    static void Shot(Camera cam, RenderTexture rt, Bounds b, string name, Vector3 dir, Vector3 up, string outDir)
    {
        cam.transform.position = b.center - dir.normalized * 6f;
        cam.transform.rotation = Quaternion.LookRotation(dir, up);
        float half = Mathf.Max(b.size.x, b.size.y, b.size.z) * 0.55f;
        cam.orthographicSize = half;
        cam.Render();
        RenderTexture.active = rt;
        var tex = new Texture2D(1024, 1024, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, 1024, 1024), 0, 0); tex.Apply();
        RenderTexture.active = null;
        string p = Path.Combine(outDir, "model_" + name + ".png");
        File.WriteAllBytes(p, tex.EncodeToPNG());
        Debug.Log("SHOTS: wrote " + p);
    }
}
