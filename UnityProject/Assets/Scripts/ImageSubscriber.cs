using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// <summary>
/// Builds one <see cref="CameraPanel"/> per camera of the selected robot, lays them
/// out in front of the user (head-locked) and shows each camera's images (compressed, or raw
/// sensor_msgs/Image). For a robot being added (setup mode) the cameras are first discovered
/// from the ROS topic list.
/// </summary>
public class ImageSubscriber : MonoBehaviour
{
    public ROSConnection ros;

    // Used when the scene is opened directly in the Editor (no robot selected).
    public RobotProfile defaultProfile;

    // Panels follow this transform; defaults to the main camera.
    public Transform panelParent;

    [Header("Layout (metres, relative to the head)")]
    public float distance = 4.3f;
    public float columnGap = 0.15f;
    public float rowGap = 0.15f;
    // Area the auto layout may use. minBottom keeps blocks above the HUD bar.
    public float maxWidth = 6.0f;
    public float maxTop = 2.0f;
    public float minBottom = -1.45f;
    // Upper limit for how much views grow when cameras are hidden (1 = default size).
    public float maxGrow = 1.6f;

    readonly List<CameraPanel> _panels = new List<CameraPanel>();
    RobotProfile _profile;
    float _defaultFit;   // best-fit size with every camera visible; sizes are relative to it
    Texture2D[] _textures;
    byte[][] _rawBuffers;
    double[] _lastRenderTime;
    bool[] _warnedFormat;

    const float DiscoveryWaitSeconds = 2.5f;

    public IReadOnlyList<CameraPanel> Panels => _panels;
    public RobotProfile Profile => _profile;

    // Camera blocks exist (immediately for known robots, after discovery in setup mode).
    public bool IsReady { get; private set; }
    public event Action Ready;
    // Raised when Reset layout changes which cameras are shown.
    public event Action VisibilityReset;

    public bool InSetup => RobotProfile.SetupMode && _profile != null && _profile.isCustom;
    public string SetupStatus { get; private set; } = "";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        var profile = _profile = RobotProfile.Selected != null ? RobotProfile.Selected : defaultProfile;
        if (profile == null)
        {
            Debug.LogError("ImageSubscriber: no robot selected and no default profile set.");
            return;
        }

        if (panelParent == null && Camera.main != null)
            panelParent = Camera.main.transform;

        if (InSetup && profile.cameras.Length == 0)
            StartCoroutine(Discover());
        else
            BuildPanels();
    }

    void BuildPanels()
    {
        int n = _profile.cameras.Length;
        _textures = new Texture2D[n];
        _rawBuffers = new byte[n][];
        _lastRenderTime = new double[n];
        _warnedFormat = new bool[n];

        for (int i = 0; i < n; i++)
        {
            var cam = _profile.cameras[i];
            string topic = _profile.FullTopic(cam);
            _panels.Add(CameraPanel.Create(panelParent, cam, topic));
            _textures[i] = new Texture2D(cam.resolution.x, cam.resolution.y, TextureFormat.RGB24, false);

            int index = i;
            if (cam.raw)
                ros.Subscribe<ImageMsg>(topic, msg => RenderRawTexture(msg, index));
            else
                ros.Subscribe<CompressedImageMsg>(topic, msg => RenderCompressedTexture(msg, index));
        }

        ApplySavedVisibility();
        if (_panels.Count > 0) _defaultFit = BestFit(_panels, out _);
        Layout();
        ApplySavedPositions();

        IsReady = true;
        Ready?.Invoke();
    }

    // Setup mode: ask the ROS endpoint for its topic list until camera topics show up.
    IEnumerator Discover()
    {
        var topics = new Dictionary<string, string>();
        ros.ListenForTopics(t => topics[t.Topic] = t.RosMessageName, notifyAllExistingTopics: true);
        SetupStatus = "Looking for camera topics\u2026";
        while (!IsReady)
        {
            ros.RefreshTopicsList();
            yield return new WaitForSeconds(DiscoveryWaitSeconds);
            if (ConfigureDiscovered(topics.Select(kv => (kv.Key, kv.Value)).ToList())) yield break;
            SetupStatus = $"No camera topics found on {ros.RosIPAddress} yet. Retrying\u2026";
        }
    }

    // Use the camera topics found in `topics` (topic, ROS type) for the robot being set up.
    // Returns false if there are none yet.
    public bool ConfigureDiscovered(IEnumerable<(string topic, string type)> topics)
    {
        if (IsReady) return true;
        var found = CameraDiscovery.Select(topics);
        if (found.Count == 0) return false;

        _profile.robotNamespace = CameraDiscovery.CommonNamespace(found.Select(f => f.topic));
        _profile.cameras = found.Select(f => new RobotProfile.CameraConfig
        {
            displayName = f.displayName,
            topicSuffix = f.topic,
            raw = f.raw,
            resolution = new Vector2Int(640, 480),   // corrected from the first frame
        }).ToArray();
        SetupStatus = $"{found.Count} camera{(found.Count == 1 ? "" : "s")} found";
        BuildPanels();
        return true;
    }

    // True once the user has dragged any block: the layout is then theirs and is left alone.
    public bool HasCustomLayout
    {
        get
        {
            foreach (var p in _panels)
                if (PlayerPrefs.HasKey(PositionKey(p))) return true;
            return false;
        }
    }

    // Show/hide a camera. In the automatic layout the remaining cameras are re-arranged
    // and grow into the freed space; a custom (dragged) layout is kept as it is.
    public void SetCameraVisible(CameraPanel panel, bool visible)
    {
        panel.Visible = visible;
        PlayerPrefs.SetInt(VisibleKey(panel), visible ? 1 : 0);
        PlayerPrefs.Save();
        if (!HasCustomLayout) Layout();
    }

    // Back to the default: every camera shown, automatic layout, dragged positions forgotten.
    public void ResetLayout()
    {
        foreach (var p in _panels)
        {
            PlayerPrefs.DeleteKey(PositionKey(p));
            PlayerPrefs.DeleteKey(VisibleKey(p));
            p.Visible = true;
        }
        PlayerPrefs.Save();
        Layout();
        VisibilityReset?.Invoke();
    }

    // Forget a robot's saved positions and shown/hidden cameras (e.g. when it is removed).
    public static void ForgetLayout(RobotProfile robot)
    {
        foreach (var cam in robot.cameras)
        {
            PlayerPrefs.DeleteKey(PositionKey(robot, cam));
            PlayerPrefs.DeleteKey(VisibleKey(robot, cam));
        }
        PlayerPrefs.Save();
    }

    void ApplySavedVisibility()
    {
        foreach (var p in _panels)
            if (PlayerPrefs.GetInt(VisibleKey(p), 1) == 0) p.Visible = false;
    }

    // Remember where the user dragged a block (head-relative), per robot and camera.
    public void SavePosition(CameraPanel panel)
    {
        var v = panel.transform.localPosition;
        PlayerPrefs.SetString(PositionKey(panel),
            string.Format(CultureInfo.InvariantCulture, "{0};{1};{2}", v.x, v.y, v.z));
        PlayerPrefs.Save();
    }

    void ApplySavedPositions()
    {
        foreach (var p in _panels)
        {
            var parts = PlayerPrefs.GetString(PositionKey(p), "").Split(';');
            if (parts.Length == 3 &&
                float.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out float x) &&
                float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float y) &&
                float.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float z))
            {
                var pos = new Vector3(x, y, z);
                p.transform.localPosition = pos;
                p.transform.localRotation = Quaternion.LookRotation(pos);
            }
        }
    }

    string PositionKey(CameraPanel p) => PositionKey(_profile, p.Config);
    string VisibleKey(CameraPanel p) => VisibleKey(_profile, p.Config);
    static string PositionKey(RobotProfile r, RobotProfile.CameraConfig c) => $"PanelPosition/{r.name}/{c.displayName}";
    static string VisibleKey(RobotProfile r, RobotProfile.CameraConfig c) => $"PanelVisible/{r.name}/{c.displayName}";

    // Automatic layout of the visible cameras: pick the column count that allows the largest
    // views within the layout area, size the views relative to the all-cameras layout (so with
    // every camera shown the layout is the default one), then place the grid centred in front
    // of the head, each block facing the eye. Rows are sized from the actual blocks, so blocks
    // never overlap; views in a row share a horizontal centre line.
    void Layout()
    {
        var visible = _panels.FindAll(p => p.Visible);
        if (visible.Count == 0) return;

        float fit = BestFit(visible, out int cols);
        float size = Mathf.Min(maxGrow, fit / _defaultFit);
        foreach (var p in visible) p.SetSize(size);

        Measure(visible, cols, size, out float[] rowAbove, out float[] rowBelow, out float[] rowWidths);
        float totalH = (rowAbove.Length - 1) * rowGap;
        for (int r = 0; r < rowAbove.Length; r++) totalH += rowAbove[r] + rowBelow[r];
        float top = Mathf.Max(totalH / 2f, minBottom + totalH);

        for (int r = 0, i = 0; r < rowAbove.Length; r++)
        {
            float viewCentreY = top - rowAbove[r];
            float x = -rowWidths[r] / 2f;
            for (int c = 0; c < cols && i < visible.Count; c++, i++)
            {
                var p = visible[i];
                float blockCentreY = viewCentreY + p.AboveViewCentre - p.Height / 2f;
                var pos = new Vector3(x + p.Width / 2f, blockCentreY, distance);
                p.transform.localPosition = pos;
                p.transform.localRotation = Quaternion.LookRotation(pos);
                x += p.Width + columnGap;
            }
            top -= rowAbove[r] + rowBelow[r] + rowGap;
        }
    }

    // Largest view size factor at which `panels` fit the layout area, over all column counts.
    // A grid with empty slots (e.g. 3+1) only wins if its views are more than 5% larger than
    // those of a full grid, so near-ties go to the tidier layout (4 -> 2x2, 3 -> one row).
    const float EmptySlotPenalty = 0.95f;

    float BestFit(List<CameraPanel> panels, out int bestCols)
    {
        float best = 0f, bestScore = 0f;
        bestCols = panels.Count;
        for (int cols = panels.Count; cols >= 1; cols--)
        {
            // Binary search: block width and height only grow with size.
            float lo = 0.05f, hi = 4f;
            for (int it = 0; it < 20; it++)
            {
                float mid = (lo + hi) / 2f;
                if (Fits(panels, cols, mid)) lo = mid; else hi = mid;
            }
            float score = panels.Count % cols == 0 ? lo : lo * EmptySlotPenalty;
            if (score > bestScore + 1e-3f) { bestScore = score; best = lo; bestCols = cols; }
        }
        return best;
    }

    bool Fits(List<CameraPanel> panels, int cols, float size)
    {
        Measure(panels, cols, size, out float[] above, out float[] below, out float[] widths);
        float h = (above.Length - 1) * rowGap, w = 0f;
        for (int r = 0; r < above.Length; r++) { h += above[r] + below[r]; w = Mathf.Max(w, widths[r]); }
        return w <= maxWidth && h <= maxTop - minBottom;
    }

    void Measure(List<CameraPanel> panels, int cols, float size,
                 out float[] rowAbove, out float[] rowBelow, out float[] rowWidths)
    {
        int rows = Mathf.CeilToInt(panels.Count / (float)cols);
        rowAbove = new float[rows]; rowBelow = new float[rows]; rowWidths = new float[rows];
        for (int i = 0; i < panels.Count; i++)
        {
            int r = i / cols;
            var m = panels[i].Measure(size);
            rowAbove[r] = Mathf.Max(rowAbove[r], m.AboveViewCentre);
            rowBelow[r] = Mathf.Max(rowBelow[r], m.BelowViewCentre);
            rowWidths[r] += m.Width + (i % cols > 0 ? columnGap : 0f);
        }
    }

    // Hidden cameras skip decoding entirely; others are limited to their maxFps.
    bool ShouldRender(int index)
    {
        if (!_panels[index].Visible) return false;
        float fps = _panels[index].Config.maxFps;
        if (fps <= 0f) return true;
        double now = Time.timeAsDouble;
        if (now - _lastRenderTime[index] < 1.0 / fps) return false;
        _lastRenderTime[index] = now;
        return true;
    }

    void RenderCompressedTexture(CompressedImageMsg msg, int index)
    {
        if (msg == null || msg.data == null || msg.data.Length == 0 || !ShouldRender(index))
            return;

        if (!_textures[index].LoadImage(msg.data))
        {
            Debug.LogWarning($"Failed to decode compressed image on {_panels[index].Topic}. format={msg.format}");
            return;
        }
        ShowFrame(index);
    }

    void RenderRawTexture(ImageMsg msg, int index)
    {
        if (msg == null || !ShouldRender(index))
            return;

        if (!RawImageDecoder.TryDecode(msg, ref _textures[index], ref _rawBuffers[index]))
        {
            if (!_warnedFormat[index])
                Debug.LogWarning($"Unsupported image on {_panels[index].Topic}: encoding={msg.encoding} {msg.width}x{msg.height}");
            _warnedFormat[index] = true;
            return;
        }
        ShowFrame(index);
    }

    void ShowFrame(int index)
    {
        var panel = _panels[index];
        var tex = _textures[index];
        var cam = panel.Config;

        // Added robots learn each camera's real size from its first frame.
        if (_profile.isCustom && (cam.resolution.x != tex.width || cam.resolution.y != tex.height))
        {
            cam.resolution = new Vector2Int(tex.width, tex.height);
            panel.SetSize(panel.Size);
            if (!HasCustomLayout)
            {
                _defaultFit = BestFit(_panels, out _);
                Layout();
            }
            if (!InSetup) RobotLibrary.Save(_profile);
        }
        panel.SetTexture(tex);
    }
}
