using System.Collections;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// On-device demo recorder for a headset lying on a desk: films the robot screen from a fixed,
/// level "Demo Head" (block view, then the first-person layout) into numbered JPEGs under
/// persistentDataPath/demo for `adb pull`. Started by launch extras (record / fps / switchat),
/// session only. The HUD and the first-person anchor follow the Demo Head, not the real headset.
/// </summary>
public class DemoRecorder : MonoBehaviour
{
    public struct Settings
    {
        public float seconds, fps, switchAt;
    }

    // Set by RobotSelectionHud from the launch extras; consumed by TeleopHud.BuildHud.
    public static Settings? Request;

    const int Width = 1280, Height = 720;
    const float VerticalFov = 70f, NearClip = 0.1f, LogSeconds = 5f;
    // First person: the robot's arms hang ~55 deg below the head camera's axis, out of a level 70 deg
    // view. Look down a little, wider, so the image quad and the arms are both in the frame.
    const float FirstPersonFov = 90f, FirstPersonPitch = 18f;

    TeleopHud _hud;
    Camera _cam;
    Settings _settings;
    string _dir;

    public static DemoRecorder Create(TeleopHud hud, ImageSubscriber images, Settings settings)
    {
        var main = Camera.main;
        if (main == null) { Debug.LogWarning("DEMO: no main camera"); return null; }

        // Level head at the main camera's position, fixed for the whole recording.
        var head = new GameObject("Demo Head").transform;
        head.SetParent(main.transform.parent, false);
        Vector3 f = main.transform.forward; f.y = 0f;
        if (f.sqrMagnitude < 1e-6f) { f = main.transform.up; f.y = 0f; }   // pointing straight down: top of the headset
        head.SetPositionAndRotation(main.transform.position, Quaternion.LookRotation(f.normalized, Vector3.up));

        var rec = head.gameObject.AddComponent<DemoRecorder>();
        rec._hud = hud;
        rec._settings = settings;
        rec._dir = Path.Combine(Application.persistentDataPath, "demo");

        // The camera hangs under the head so it can pitch without tilting the HUD / first-person anchor.
        var camGo = new GameObject("Demo Camera");
        camGo.transform.SetParent(head, false);
        var cam = camGo.AddComponent<Camera>();
        cam.clearFlags = main.clearFlags;
        cam.backgroundColor = main.backgroundColor;
        cam.cullingMask = main.cullingMask;
        cam.nearClipPlane = NearClip;
        cam.farClipPlane = main.farClipPlane;
        cam.fieldOfView = VerticalFov;
        cam.stereoTargetEye = StereoTargetEyeMask.None;
        cam.enabled = false;   // rendered by hand
        rec._cam = cam;

        hud.ReparentHud(head);
        FirstPersonView.HeadOverride = head;
        rec.StartCoroutine(rec.Record());
        return rec;
    }

    IEnumerator Record()
    {
        if (Directory.Exists(_dir)) Directory.Delete(_dir, true);
        Directory.CreateDirectory(_dir);
        Debug.Log($"DEMO: recording {_settings.seconds}s at {_settings.fps} fps, switch at {_settings.switchAt}s -> {_dir}");

        var rt = new RenderTexture(Width, Height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
        var tex = new Texture2D(Width, Height, TextureFormat.RGB24, false);
        var timing = new StringBuilder();
        float start = Time.unscaledTime, interval = 1f / _settings.fps;
        float accumulator = interval;   // first frame immediately
        float last = start, nextLog = start + LogSeconds;
        bool switched = false;
        int frames = 0;
        var previousActive = RenderTexture.active;

        while (Time.unscaledTime - start < _settings.seconds)
        {
            yield return new WaitForEndOfFrame();
            float now = Time.unscaledTime;
            accumulator += now - last;
            last = now;

            if (!switched && now - start >= _settings.switchAt)
            {
                switched = true;
                Debug.Log("DEMO: switch to first person");
                _hud.SetCameraLayout(FirstPersonView.LayoutFirstPerson, save: false);
                _hud.SetRobotModel(true, save: false);   // both layouts show the model; first person needs it on
                _cam.fieldOfView = FirstPersonFov;
                _cam.transform.localRotation = Quaternion.Euler(FirstPersonPitch, 0f, 0f);
            }
            if (accumulator < interval) continue;
            accumulator = Mathf.Min(accumulator - interval, interval);   // drop, don't burst, after a stall

            try
            {
                _cam.targetTexture = rt;
                _cam.Render();
                RenderTexture.active = rt;
                tex.ReadPixels(new Rect(0, 0, Width, Height), 0, 0);
                tex.Apply(false);
                frames++;
                File.WriteAllBytes(Path.Combine(_dir, $"frame_{frames:D5}.jpg"), tex.EncodeToJPG(85));
                timing.Append(frames).Append(' ').Append(now.ToString("F3")).Append('\n');
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"DEMO: frame failed: {e.Message}");
            }
            if (now >= nextLog)
            {
                nextLog += LogSeconds;
                Debug.Log($"DEMO: frame {frames} t={now - start:F1}s");
            }
        }

        RenderTexture.active = previousActive;
        _cam.targetTexture = null;
        File.WriteAllText(Path.Combine(_dir, "timing.txt"), timing.ToString());
        rt.Release();
        Destroy(rt);
        Destroy(tex);
        Debug.Log($"DEMO: done {frames} frames in {_dir}");
        // Leave the Demo Head: the HUD hangs under it. Only stop capturing.
        Destroy(_cam.gameObject);
        Destroy(this);
    }
}
