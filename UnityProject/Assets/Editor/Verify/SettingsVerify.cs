// Verify harness: Settings (typed prefs, v2 keys) and the one-way migration from the old keys.
// Run: tools/verify.sh --suite SettingsVerify   (no sim needed; the ROS IP is isolated)
using System;
using System.Collections;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Object = UnityEngine.Object;

public static class SettingsVerify
{
    static IEnumerator _run; static double _until; static int _fail, _pass;
    static RobotProfile _profile;
    static RobotProfile.CameraConfig _head;

    public static void Run()
    {
        EditorSettings.enterPlayModeOptionsEnabled = true;
        EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.DisableDomainReload;
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
                Debug.Log(_fail == 0 ? $"[Verify] ALL SETTINGS CHECKS PASSED ({_pass})" : $"[Verify] {_fail} SETTINGS CHECK(S) FAILED");
                EditorApplication.Exit(_fail == 0 ? 0 : 1);
            }
        }
        catch (Exception e) { Debug.LogException(e); EditorApplication.Exit(3); }
    }
    static object Wait(double s) { _until = EditorApplication.timeSinceStartup + s; return null; }
    static void Check(bool ok, string what) { if (ok) _pass++; else _fail++; Debug.Log("[Verify] " + (ok ? "PASS " : "FAIL ") + what); }

    static IEnumerator EnterTeleop()
    {
        EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
        RobotProfile.Selected = _profile;
        Object.FindFirstObjectByType<QuestControllerPublisher>().controlRobot = false;
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Wait(0.1);
        Object.FindFirstObjectByType<QuestControllerPublisher>().controlRobot = false;
        yield return Wait(3.0);
    }
    static IEnumerator ExitPlay()
    {
        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Wait(0.1);
        yield return Wait(0.5);
    }

    static IEnumerator Main()
    {
        _profile = AssetDatabase.LoadAssetAtPath<RobotProfile>(RobotSpec.Current.ProfilePath);
        _head = _profile.cameras.First(c => c.role == RobotProfile.CameraRole.Head);
        string robot = _profile.name, cam = _head.displayName;
        string sfx = _head.topicSuffix.Replace('/', '_');
        Check(_profile != null && _head != null, $"profile {robot}, head camera '{cam}' ({_head.topicSuffix})");

        // ---- migration from seeded old keys
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "10.9.8.7");
        PlayerPrefs.SetString("LastRobot", robot);
        PlayerPrefs.SetInt("Passthrough", 1);
        PlayerPrefs.SetInt("LazyFollow", 1);
        PlayerPrefs.SetInt("Images/Compressed", 0);
        PlayerPrefs.SetString($"ViewMode/{robot}", "firstperson");
        PlayerPrefs.SetInt($"PanelVisible/{robot}/{cam}", 0);
        PlayerPrefs.SetString($"CameraLabel/{robot}/{cam}", "Front Eye");
        PlayerPrefs.SetString($"PanelPosition/{robot}/{cam}", "0;0.1;4.3;1.2");
        PlayerPrefs.Save();
        Check(Settings.Version == 0 && Settings.RosIp == Settings.DefaultRosIp && Settings.Compressed, "before: Version 0, new keys absent (RosIp default, Compressed default on)");
        Settings.EnsureMigrated(new[] { _profile });
        var rs = Settings.For(_profile); var cs = rs.Camera(_head);
        Check(Settings.RosIp == "10.9.8.7" && Settings.LastRobot == robot && Settings.Passthrough && Settings.LazyFollow && !Settings.Compressed && !Settings.DebugCapture,
              "globals migrated 1:1 (RosIp, LastRobot, Passthrough, LazyFollow, Compressed=false, DebugCapture off)");
        Check(rs.ModelOn && rs.Layout == "firstperson", $"ViewMode=firstperson -> ModelOn {rs.ModelOn}, Layout '{rs.Layout}'");
        Check(!cs.Visible && cs.Label == "Front Eye", $"camera keyed by topicSuffix: Visible {cs.Visible}, Label '{cs.Label}'");
        Check(cs.Position is Vector3 p && Mathf.Abs(p.y - 0.1f) < 1e-4f && Mathf.Abs(p.z - 4.3f) < 1e-4f && cs.Size is float sz && Mathf.Abs(sz - 1.2f) < 1e-4f, $"position parsed {cs.Position}, size {cs.Size}");
        Check(PlayerPrefs.HasKey($"v2/robot/{robot}/cam/{sfx}/Label") && PlayerPrefs.HasKey("v2/RosIp") && PlayerPrefs.HasKey($"v2/robot/{robot}/ModelOn"), "key scheme v2/..., v2/robot/<name>/..., v2/robot/<name>/cam/<suffix_>/...");
        Check(Settings.Version == 2, "Version == 2");
        Check(PlayerPrefs.HasKey("RosIPAddress") && PlayerPrefs.HasKey($"ViewMode/{robot}") && PlayerPrefs.HasKey($"CameraLabel/{robot}/{cam}"), "old keys kept");

        // running it again changes nothing, even when the old values change
        PlayerPrefs.SetString("RosIPAddress", "1.1.1.1");
        PlayerPrefs.SetString($"CameraLabel/{robot}/{cam}", "Changed");
        Settings.EnsureMigrated(new[] { _profile });
        Check(Settings.RosIp == "10.9.8.7" && cs.Label == "Front Eye" && Settings.Version == 2, "second migration run changes nothing");
        PlayerPrefs.SetString("RosIPAddress", "10.9.8.7");   // isolation: never reach a live ros_tcp_endpoint below
        PlayerPrefs.SetString("v2/RosIp", "127.0.0.2");
        Settings.Passthrough = false; Settings.LazyFollow = false;

        // ---- end to end: TeleopScene shows the migrated state
        { var e = EnterTeleop(); while (e.MoveNext()) yield return e.Current; }
        var hud = Object.FindFirstObjectByType<TeleopHud>(); var images = Object.FindFirstObjectByType<ImageSubscriber>();
        var panel = images.Panels.First(x => x.Config.topicSuffix == _head.topicSuffix);
        Check(hud.RobotModelOn && hud.CameraLayout == "firstperson", $"scene: model on {hud.RobotModelOn}, layout '{hud.CameraLayout}' from the migrated keys");
        Check(panel.Label == "Front Eye" && !panel.gameObject.activeSelf, $"scene: head block label '{panel.Label}', hidden {!panel.gameObject.activeSelf}");
        { var e = ExitPlay(); while (e.MoveNext()) yield return e.Current; }

        // blocks layout: the saved hidden state itself hides the head block
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.2");
        PlayerPrefs.SetInt($"PanelVisible/{robot}/{cam}", 0);
        PlayerPrefs.SetString($"CameraLabel/{robot}/{cam}", "Front Eye");
        PlayerPrefs.Save();
        { var e = EnterTeleop(); while (e.MoveNext()) yield return e.Current; }
        images = Object.FindFirstObjectByType<ImageSubscriber>();
        panel = images.Panels.First(x => x.Config.topicSuffix == _head.topicSuffix);
        Check(!panel.gameObject.activeSelf && panel.Label == "Front Eye" && images.Panels.Where(x => x != panel).All(x => x.gameObject.activeSelf),
              $"scene (blocks): head block hidden {!panel.gameObject.activeSelf}, label '{panel.Label}', the others shown");

        // rename + layout flow persists through Settings
        images.RenameCamera(panel, "Eyes");
        Check(Settings.For(_profile).Camera(_head).Label == "Eyes", "rename persists via Settings.Label");
        images.SetCameraVisible(panel, true);
        images.SavePosition(panel);
        Check(Settings.For(_profile).Camera(_head).Visible && Settings.For(_profile).Camera(_head).Position != null, "show + drag persist (Visible, Position)");
        images.ResetLayout();
        Check(Settings.For(_profile).Camera(_head).Position == null && Settings.For(_profile).Camera(_head).Label == "Eyes", "ResetLayout clears position/visibility, keeps the name");
        Object.FindFirstObjectByType<TeleopHud>().SetRobotModel(true);
        Check(Settings.For(_profile).ModelOn, "model toggle persists via Settings.ModelOn");
        { var e = ExitPlay(); while (e.MoveNext()) yield return e.Current; }

        // ---- Forget
        Settings.For(_profile).Camera(_head).SetPlacement(new Vector3(1, 2, 3), 1f);
        Settings.For(_profile).Forget();
        var f = Settings.For(_profile);
        Check(!f.HasModelOn && !f.HasLayout && f.Camera(_head).Label == "" && f.Camera(_head).Visible && f.Camera(_head).Position == null && !f.HasAnyPosition,
              "Forget clears model, layout and every camera");
        string prefix = $"v2/robot/{robot}/";
        Check(!PlayerPrefs.HasKey(prefix + "ModelOn") && !PlayerPrefs.HasKey(prefix + $"cam/{sfx}/Label") && !PlayerPrefs.HasKey(prefix + $"cam/{sfx}/Position") && !PlayerPrefs.HasKey(prefix + $"cam/{sfx}/Visible"), "Forget removed the keys");
        Settings.EnsureMigrated(new[] { _profile });
        Check(!Settings.For(_profile).HasModelOn, "a forgotten robot is not migrated back from the kept old keys");
        PlayerPrefs.DeleteAll();
    }
}
