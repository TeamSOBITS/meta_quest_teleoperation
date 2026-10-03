// Verify harness: Robot selection screen: cards, IP row, angular size, card click (isolated RosIP).
// Run: tools/verify.sh --suite SelectionShot   (or Unity -batchmode -projectPath <copy> -executeMethod SelectionShot.Run)
using System.Collections;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

// Copy-only: check and screenshot the robot selection screen, then click a card.
public static class SelectionShot
{
    static IEnumerator _run; static double _until; static int _fail;
    public static void Run()
    {
        EditorSettings.enterPlayModeOptionsEnabled = true;
        EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.DisableDomainReload;
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.2");   // isolated: never reach a live ros_tcp_endpoint
        var robotsDir = Path.Combine(Application.persistentDataPath, "robots");
        if (Directory.Exists(robotsDir)) Directory.Delete(robotsDir, true);   // only built-in robots
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
                Debug.Log(_fail == 0 ? "[Verify] ALL SELECTION CHECKS PASSED" : $"[Verify] {_fail} SELECTION CHECK(S) FAILED");
                EditorApplication.Exit(_fail == 0 ? 0 : 1);
            }
        }
        catch (System.Exception e) { Debug.LogException(e); EditorApplication.Exit(3); }
    }
    static object Wait(double s) { _until = EditorApplication.timeSinceStartup + s; return null; }
    static void Check(bool ok, string what) { if (!ok) _fail++; Debug.Log("[Verify] " + (ok ? "PASS " : "FAIL ") + what); }

    static IEnumerator Main()
    {
        EditorSceneManager.OpenScene("Assets/Scenes/RobotSelectionScene.unity");
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Wait(0.1);
        yield return Wait(3.0);

        var root = GameObject.Find("Robot Selection Screen");
        Check(root != null, "selection screen exists");
        var cards = root.GetComponentsInChildren<UnityEngine.UI.Button>().Where(b => b.name.StartsWith("Card ") && b.name != "Card Add robot").ToList();
        Check(cards.Count == 2, $"two robot cards plus Add robot ({string.Join(", ", cards.Select(c => c.name))})");
        var texts = root.GetComponentsInChildren<TMPro.TextMeshProUGUI>().Select(t => t.text).ToList();
        Check(texts.Contains("/sobit_home  ·  3 cameras") && texts.Contains("/sobit_light  ·  4 cameras"), "cards show namespace and camera count");
        Check(texts.Contains("127.0.0.2"), "IP row shows the saved IP");
        Check(!texts.Any(t => t.Contains("reachable")), "no ping pill on the IP row (UnityEngine.Ping does not work on Quest)");

        // Angular size of the screen as seen from the head.
        var head = Camera.main.transform;
        var c = new Vector3[4]; ((RectTransform)root.GetComponentsInChildren<Canvas>()[0].transform).GetWorldCorners(c);
        var a = c.Select(w => head.InverseTransformPoint(w)).Select(v => Mathf.Atan2(v.x, v.z) * Mathf.Rad2Deg).ToList();
        Debug.Log($"[Verify]     selection spans {a.Min():F1}..{a.Max():F1} deg horizontally");
        Check(a.Max() - a.Min() > 30f && a.Max() - a.Min() < 70f, "selection screen is 30-70 deg wide");

        var go = new GameObject("ShotCam"); var cam = go.AddComponent<Camera>();
        go.transform.SetPositionAndRotation(head.position, head.rotation);
        cam.fieldOfView = 80f; cam.aspect = 1.6f; cam.nearClipPlane = 0.05f; cam.farClipPlane = 200f;
        cam.stereoTargetEye = StereoTargetEyeMask.None;
        cam.clearFlags = Camera.main.clearFlags; cam.backgroundColor = Camera.main.backgroundColor;
        var rt = new RenderTexture(1600, 1000, 24); cam.targetTexture = rt; cam.Render(); RenderTexture.active = rt;
        var tex = new Texture2D(1600, 1000, TextureFormat.RGB24, false); tex.ReadPixels(new Rect(0, 0, 1600, 1000), 0, 0); tex.Apply();
        RenderTexture.active = null;
        File.WriteAllBytes(Path.Combine(System.Environment.GetEnvironmentVariable("VERIFY_SHOTS"), "Selection_new.png"), tex.EncodeToPNG());
        Object.DestroyImmediate(go);

        // Click SOBIT LIGHT -> teleop scene with that profile.
        cards.First(b => b.name.Contains("LIGHT")).onClick.Invoke();
        yield return Wait(3.0);
        var images = Object.FindFirstObjectByType<ImageSubscriber>();
        Check(images != null && images.Profile != null && images.Profile.displayName == "SOBIT LIGHT", "clicking SOBIT LIGHT opens its robot screen");

        // Back to selection: SOBIT LIGHT carries the "Last used" tag, hint visible.
        Object.FindFirstObjectByType<QuestControllerPublisher>().BackToRobotSelection();
        yield return Wait(3.0);
        var screen = GameObject.Find("Robot Selection Screen");
        var light = screen.GetComponentsInChildren<UnityEngine.UI.Button>().First(b => b.name.Contains("LIGHT"));
        var home = screen.GetComponentsInChildren<UnityEngine.UI.Button>().First(b => b.name.Contains("HOME"));
        Check(light.GetComponentsInChildren<Transform>().Any(t => t.name == "Last used"), "last used robot (SOBIT LIGHT) has the Last used tag");
        Check(!home.GetComponentsInChildren<Transform>().Any(t => t.name == "Last used"), "other robot has no Last used tag");
        Check(screen.GetComponentsInChildren<TMPro.TextMeshProUGUI>().Any(t => t.text == "Point at a robot and pull the trigger"), "hint line shown");
        var head2 = Camera.main.transform;
        var go2 = new GameObject("ShotCam"); var cam2 = go2.AddComponent<Camera>();
        go2.transform.SetPositionAndRotation(head2.position, head2.rotation);
        cam2.fieldOfView = 80f; cam2.aspect = 1.6f; cam2.nearClipPlane = 0.05f; cam2.farClipPlane = 200f;
        cam2.stereoTargetEye = StereoTargetEyeMask.None;
        cam2.clearFlags = Camera.main.clearFlags; cam2.backgroundColor = Camera.main.backgroundColor;
        var rt2 = new RenderTexture(1600, 1000, 24); cam2.targetTexture = rt2; cam2.Render(); RenderTexture.active = rt2;
        var tex2 = new Texture2D(1600, 1000, TextureFormat.RGB24, false); tex2.ReadPixels(new Rect(0, 0, 1600, 1000), 0, 0); tex2.Apply();
        RenderTexture.active = null;
        File.WriteAllBytes(Path.Combine(System.Environment.GetEnvironmentVariable("VERIFY_SHOTS"), "Selection_lastused.png"), tex2.EncodeToPNG());
        Object.DestroyImmediate(go2);

        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Wait(0.1);
    }
}
