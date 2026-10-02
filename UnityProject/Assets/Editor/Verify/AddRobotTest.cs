// Verify harness: Add-robot flow: discovery rules, raw decoder, Add robot card (topics injected, no live ROS).
// Run: tools/verify.sh --suite AddRobotTest   (or Unity -batchmode -projectPath <copy> -executeMethod AddRobotTest.Run)
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using RosMessageTypes.Sensor;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

// Copy-only: discovery rules, raw decoder, and the Add robot flow end to end (topics injected).
public static class AddRobotTest
{
    static IEnumerator _run; static double _until; static int _fail;
    public static void Run()
    {
        EditorSettings.enterPlayModeOptionsEnabled = true;
        EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.DisableDomainReload;
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.2");   // isolated: never reach a live ros_tcp_endpoint
        ImageSubscriber.RequireLiveTopics = false;   // no ROS here: test topics never deliver frames
        var dir = Path.Combine(Application.persistentDataPath, "robots");
        if (Directory.Exists(dir)) Directory.Delete(dir, true);
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
                Debug.Log(_fail == 0 ? "[Verify] ALL ADD-ROBOT CHECKS PASSED" : $"[Verify] {_fail} ADD-ROBOT CHECK(S) FAILED");
                EditorApplication.Exit(_fail == 0 ? 0 : 1);
            }
        }
        catch (System.Exception e) { Debug.LogException(e); EditorApplication.Exit(3); }
    }
    static object Wait(double s) { _until = EditorApplication.timeSinceStartup + s; return null; }
    static void Check(bool ok, string what) { if (!ok) _fail++; Debug.Log("[Verify] " + (ok ? "PASS " : "FAIL ") + what); }

    static readonly (string, string)[] LightTopics =
    {
        ("/sobit_light/head_camera/color/image_raw", "sensor_msgs/msg/Image"),
        ("/sobit_light/head_camera/color/image_raw/compressed", "sensor_msgs/msg/CompressedImage"),
        ("/sobit_light/head_camera/depth/image_rect_raw", "sensor_msgs/msg/Image"),
        ("/sobit_light/head_camera/depth/image_rect_raw/compressedDepth", "sensor_msgs/msg/CompressedImage"),
        ("/sobit_light/hand_camera/color/image_raw", "sensor_msgs/msg/Image"),
        ("/sobit_light/hand_camera/color/image_raw/compressed", "sensor_msgs/msg/CompressedImage"),
        ("/sobit_light/front_camera/image_raw", "sensor_msgs/msg/Image"),
        ("/sobit_light/front_camera/image_raw/compressed", "sensor_msgs/msg/CompressedImage"),
        ("/sobit_light/back_camera/image_raw", "sensor_msgs/msg/Image"),
        ("/sobit_light/joint_states", "sensor_msgs/msg/JointState"),
        ("/tf", "tf2_msgs/msg/TFMessage"),
    };

    static IEnumerator Main()
    {
        // --- Discovery rules ---
        var found = CameraDiscovery.Select(LightTopics);
        Debug.Log("[Verify]     discovered: " + string.Join(" | ", found.Select(f => $"{f.displayName} {(f.raw ? "[raw]" : "")} {f.topic}")));
        Check(found.Count == 4, "SOBIT LIGHT topics -> 4 cameras (depth, TF and joint states skipped)");
        Check(found.Single(f => f.topic.Contains("back_camera")).raw, "back camera has no compressed topic -> raw fallback");
        Check(found.Where(f => !f.topic.Contains("back_camera")).All(f => !f.raw && f.topic.EndsWith("/compressed")), "others use their compressed topic");
        Check(new[] { "Back Camera", "Front Camera", "Hand Camera", "Head Camera" }.SequenceEqual(found.Select(f => f.displayName).OrderBy(n => n)), "camera names from topics");
        Check(CameraDiscovery.CommonNamespace(found.Select(f => f.topic)) == "sobit_light", "namespace sobit_light detected");
        var mixed = CameraDiscovery.Select(LightTopics.Concat(new[] { ("/sobit_home/head_camera/color/image_raw/compressed", "sensor_msgs/CompressedImage") }));
        Check(CameraDiscovery.CommonNamespace(mixed.Select(f => f.topic)) == "" && mixed.Select(f => f.displayName).Distinct().Count() == mixed.Count,
              "two robots -> no common namespace, names still unique");

        // --- Raw decoder ---
        Texture2D tex = null; byte[] buf = null;
        var rgb = new ImageMsg { width = 2, height = 2, step = 6, encoding = "rgb8",
            data = new byte[] { 255,0,0, 0,255,0,   0,0,255, 255,255,255 } };   // top row: red, green; bottom: blue, white
        Check(RawImageDecoder.TryDecode(rgb, ref tex, ref buf) && tex.GetPixel(0, 0) == Color.blue && tex.GetPixel(1, 1) == Color.green,
              "rgb8 decodes with the top row at the top");
        var bgr = new ImageMsg { width = 1, height = 1, step = 3, encoding = "bgr8", data = new byte[] { 255, 0, 0 } };
        Check(RawImageDecoder.TryDecode(bgr, ref tex, ref buf) && tex.GetPixel(0, 0) == Color.blue, "bgr8 swaps red and blue");
        var mono = new ImageMsg { width = 1, height = 1, step = 1, encoding = "mono8", data = new byte[] { 128 } };
        Check(RawImageDecoder.TryDecode(mono, ref tex, ref buf) && Mathf.Abs(tex.GetPixel(0, 0).g - 128 / 255f) < 0.01f, "mono8 becomes grey");
        var depth = new ImageMsg { width = 1, height = 1, step = 2, encoding = "16UC1", data = new byte[] { 0, 1 } };
        Check(!RawImageDecoder.TryDecode(depth, ref tex, ref buf), "16-bit depth is rejected");

        // --- Add robot flow ---
        EditorSceneManager.OpenScene("Assets/Scenes/RobotSelectionScene.unity");
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Wait(0.1);
        yield return Wait(2.0);
        var screen = GameObject.Find("Robot Selection Screen");
        var add = screen.GetComponentsInChildren<UnityEngine.UI.Button>().FirstOrDefault(b => b.name == "Card Add robot");
        Check(add != null, "selection has an Add robot card");
        add.onClick.Invoke();          // Editor: no system keyboard -> name falls back to "Robot 3"
        yield return Wait(3.0);

        var images = Object.FindFirstObjectByType<ImageSubscriber>();
        Check(images != null && images.InSetup && !images.IsReady, "Add robot opens the robot screen in setup mode, waiting for cameras");
        Check(GameObject.Find("Setup Status") != null, "setup shows the 'Setting up' card while discovering");
        images.ConfigureDiscovered(LightTopics);
        yield return Wait(1.0);
        Check(images.IsReady && images.Panels.Count == 4, "injected topics -> 4 camera blocks");
        Check(images.Panels.All(p => p.transform.Find("Topic") != null), "setup mode: every block has its 'Topic' line");
        Check(GameObject.Find("Setup Status") == null && GameObject.Find("HUD Bar") != null, "setup card replaced by the HUD bar");
        var bar = GameObject.Find("HUD Bar");
        var chip = bar.transform.Find("Joy State/Label").GetComponent<TMPro.TextMeshProUGUI>().text;
        Check(chip.Contains("SETUP") && chip.Contains("/sobit_light"), $"bar chip shows setup and namespace ({chip})");
        var publisher = Object.FindFirstObjectByType<QuestControllerPublisher>();
        Check(!publisher.controlRobot, "setup keeps Joy off (layout mode)");
        var buttons = bar.GetComponentsInChildren<UnityEngine.UI.Button>().Select(b => b.name).ToList();
        Check(buttons.Any(n => n.Contains("Save robot")) && buttons.Any(n => n.Contains("Cancel")), "bar has Save robot and Cancel");
        var back = images.Panels.First(p => p.Config.displayName == "Back Camera");
        images.SetCameraVisible(back, false);

        var shot = System.Environment.GetEnvironmentVariable("VERIFY_SHOTS");
        Shot(Path.Combine(shot, "AddRobot_setup.png"));

        Object.FindFirstObjectByType<TeleopHud>().SaveSetup();
        yield return Wait(3.0);
        var file = Path.Combine(Application.persistentDataPath, "robots", "custom_robot_3.json");
        Check(File.Exists(file), "Save robot writes robots/custom_robot_3.json");
        Debug.Log("[Verify]     saved: " + (File.Exists(file) ? File.ReadAllText(file).Replace("\n", " ") : "-"));
        {   // v2 schema: the new robot carries the sobits_teleop frame names; an old file without them loads with empty values.
            var saved = RobotLibrary.LoadAll().FirstOrDefault(r => r.name == "custom_robot_3");
            Check(saved != null && saved.baseFrame == TeleopConventions.BaseFrame && saved.controllerFrames.hmd == TeleopConventions.Hmd
                  && saved.controllerFrames.left == TeleopConventions.LeftController && saved.controllerFrames.right == TeleopConventions.RightController,
                  $"saved JSON carries baseFrame '{saved?.baseFrame}' and controllerFrames '{saved?.controllerFrames.hmd}'");
            string oldFile = Path.Combine(Application.persistentDataPath, "robots", "custom_old_json.json");
            File.WriteAllText(oldFile, "{\"displayName\":\"Old\",\"robotNamespace\":\"old\",\"cameras\":[{\"displayName\":\"Cam\",\"topic\":\"/old/cam/image_raw\",\"raw\":true,\"resolution\":{\"x\":320,\"y\":240},\"maxFps\":10,\"scale\":1}]}");
            var old = RobotLibrary.LoadAll().FirstOrDefault(r => r.name == "custom_old_json");
            File.Delete(oldFile);
            Check(old != null && old.cameras.Length == 1 && string.IsNullOrEmpty(old.baseFrame) && string.IsNullOrEmpty(old.controllerFrames.hmd) && string.IsNullOrEmpty(old.head.panFrame)
                  && string.IsNullOrEmpty(old.lift.frame) && old.arms.Length == 0 && old.cameras[0].role == RobotProfile.CameraRole.Other && !old.cameras[0].firstPerson,
                  "old JSON (no v2 keys) loads with empty model fields");
        }
        screen = GameObject.Find("Robot Selection Screen");
        var card = screen.GetComponentsInChildren<UnityEngine.UI.Button>().FirstOrDefault(b => b.name == "Card Robot 3");
        Check(card != null, "selection shows the new robot's card");
        var texts = card.GetComponentsInChildren<TMPro.TextMeshProUGUI>().Select(t => t.text).ToList();
        Check(texts.Contains("R3") && texts.Contains("/sobit_light  ·  4 cameras"), $"new card shows initials and details ({string.Join(" | ", texts)})");
        Check(card.GetComponentsInChildren<Transform>().Any(t => t.name == "Last used"), "new robot is tagged Last used");
        Check(RobotLibrary.LoadAll().FirstOrDefault(r => r.name == "custom_robot_3") is RobotProfile r3 && !Settings.For(r3).Camera(r3.cameras.First(c => c.displayName == "Back Camera")).Visible, "hidden camera choice kept for the new robot");
        Shot(Path.Combine(shot, "AddRobot_selection.png"));

        // Open it again: 4 blocks, back camera still hidden, not in setup mode.
        card.onClick.Invoke();
        yield return Wait(3.0);
        images = Object.FindFirstObjectByType<ImageSubscriber>();
        Check(images.IsReady && !images.InSetup && images.Panels.Count == 4 && !images.Panels.First(p => p.Config.displayName == "Back Camera").Visible,
              "reopening the robot: 4 blocks, Back Camera still hidden");
        Check(images.Panels.All(p => p.transform.Find("Topic") == null), "reopened (not setup): no block has a 'Topic' line");
        Object.FindFirstObjectByType<QuestControllerPublisher>().BackToRobotSelection();
        yield return Wait(3.0);

        // Remove needs two presses.
        screen = GameObject.Find("Robot Selection Screen");
        card = screen.GetComponentsInChildren<UnityEngine.UI.Button>().First(b => b.name == "Card Robot 3");
        var remove = card.GetComponentsInChildren<UnityEngine.UI.Button>().First(b => b.name.Contains("Remove"));
        remove.onClick.Invoke();
        yield return Wait(0.5);
        Check(File.Exists(file), "first Remove press only asks for confirmation");
        remove.onClick.Invoke();
        yield return Wait(1.0);
        screen = GameObject.Find("Robot Selection Screen");
        Check(!File.Exists(file) && !screen.GetComponentsInChildren<UnityEngine.UI.Button>().Any(b => b.name == "Card Robot 3"),
              "second Remove press deletes the robot and its card");

        // --- Typing display and Joy-only namespace rule ---
        Check(QuestControllerPublisher.TypingDisplay("", "Type\u2026") == "Type\u2026" && QuestControllerPublisher.TypingDisplay("10.0", "x") == "10.0|",
              "while typing: prompt when empty, typed text with a caret otherwise");
        Check(CameraDiscovery.RobotNamespace(new[] { "/tf", "/rosout", "/my_bot/joint_states", "/my_bot/odom", "/joy" }) == "my_bot",
              "Joy-only robot: namespace from its other topics (my_bot)");
        Check(CameraDiscovery.RobotNamespace(new[] { "/tf", "/a/odom", "/b/odom" }) == "", "topics of two robots -> no namespace");

        // --- Robot without cameras, then Find cameras ---
        screen = GameObject.Find("Robot Selection Screen");
        screen.GetComponentsInChildren<UnityEngine.UI.Button>().First(b => b.name == "Card Add robot").onClick.Invoke();
        yield return Wait(3.0);
        images = Object.FindFirstObjectByType<ImageSubscriber>();
        var waiting = GameObject.Find("Setup Status");
        var waitButtons = waiting.GetComponentsInChildren<UnityEngine.UI.Button>().Select(b => b.name).ToList();
        Check(waitButtons.Any(n => n.Contains("Search again")) && waitButtons.Any(n => n.Contains("Continue without cameras")) && waitButtons.Any(n => n.Contains("Cancel")),
              $"setup card offers Search again / Continue without cameras / Cancel ({string.Join(", ", waitButtons)})");
        waiting.GetComponentsInChildren<UnityEngine.UI.Button>().First(b => b.name.Contains("Continue without cameras")).onClick.Invoke();
        yield return Wait(1.0);
        Check(images.IsReady && images.Panels.Count == 0 && GameObject.Find("HUD Bar") != null, "continue without cameras -> robot screen with the bar and no camera blocks");
        bar = GameObject.Find("HUD Bar");
        var find = bar.GetComponentsInChildren<UnityEngine.UI.Button>().FirstOrDefault(b => b.name.Contains("Find cameras"));
        Check(find != null, "added robot's bar has a Find cameras button");
        Shot(Path.Combine(shot, "AddRobot_nocams.png"));

        // A camera topic appears on the ROS side (simulated by registering it locally).
        Object.FindFirstObjectByType<Unity.Robotics.ROSTCPConnector.ROSConnection>()
              .RegisterPublisher<CompressedImageMsg>("/joy_bot/arm_camera/image_raw/compressed");
        find.onClick.Invoke();
        yield return Wait(4.0);
        bar = GameObject.Find("HUD Bar");
        Check(images.Panels.Count == 1 && images.Panels[0].Config.displayName == "Arm Camera", $"Find cameras adds the new camera ({images.Panels.Count} block(s))");
        Check(bar.GetComponentsInChildren<UnityEngine.UI.Toggle>().Any(t => t.name.Contains("Arm Camera")), "bar rebuilt with the new camera's toggle");
        Shot(Path.Combine(shot, "AddRobot_found.png"));
        find = bar.GetComponentsInChildren<UnityEngine.UI.Button>().First(b => b.name.Contains("Find cameras"));
        find.onClick.Invoke();
        yield return Wait(4.0);
        Check(images.Panels.Count == 1 && find.GetComponentInChildren<TMPro.TextMeshProUGUI>().text == "No new cameras", "searching again with nothing new says so");
        var nsButton = bar.GetComponentsInChildren<UnityEngine.UI.Button>().FirstOrDefault(b => b.name.Contains("Joy:"));
        var pub = Object.FindFirstObjectByType<QuestControllerPublisher>();
        Check(nsButton != null && pub.JoyTopic == "/joy_bot/joy", $"Joy namespace button present; Joy on {pub.JoyTopic}");
        pub.SetNamespace(""); 
        Check(pub.JoyTopic == "/joy", "empty namespace -> Joy on /joy");
        pub.SetNamespace("joy_bot");

        Object.FindFirstObjectByType<TeleopHud>().SaveSetup();
        yield return Wait(3.0);
        screen = GameObject.Find("Robot Selection Screen");
        var cards = screen.GetComponentsInChildren<UnityEngine.UI.Button>().Where(b => b.name.StartsWith("Card Robot")).ToList();
        var newest = cards.LastOrDefault();
        var details = newest?.GetComponentsInChildren<TMPro.TextMeshProUGUI>().Select(t => t.text).ToList();
        Check(newest != null && details.Any(t => t.Contains("/joy_bot") && t.Contains("1 camera")), $"saved robot card shows namespace and camera ({(details == null ? "-" : string.Join(" | ", details))})");

        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Wait(0.1);
    }

    static void Shot(string path)
    {
        var head = Camera.main.transform;
        var go = new GameObject("ShotCam"); var cam = go.AddComponent<Camera>();
        go.transform.SetPositionAndRotation(head.position, head.rotation);
        cam.fieldOfView = 80f; cam.aspect = 1.6f; cam.nearClipPlane = 0.05f; cam.farClipPlane = 200f;
        cam.stereoTargetEye = StereoTargetEyeMask.None;
        cam.clearFlags = Camera.main.clearFlags; cam.backgroundColor = Camera.main.backgroundColor;
        var rt = new RenderTexture(1600, 1000, 24); cam.targetTexture = rt; cam.Render(); RenderTexture.active = rt;
        var t = new Texture2D(1600, 1000, TextureFormat.RGB24, false); t.ReadPixels(new Rect(0, 0, 1600, 1000), 0, 0); t.Apply();
        RenderTexture.active = null;
        File.WriteAllBytes(path, t.EncodeToPNG());
        Object.DestroyImmediate(go);
    }
}
