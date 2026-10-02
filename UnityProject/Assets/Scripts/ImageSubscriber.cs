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
    // Area the auto layout may use. minBottom keeps blocks above the HUD bar (compact: 0.7x, 0.3 m lower than at full size).
    public float maxWidth = 6.0f;
    public float maxTop = 2.0f;
    public float minBottom = -1.75f;
    // Upper limit for how much views grow when cameras are hidden (1 = default size).
    public float maxGrow = 1.6f;

    readonly List<CameraPanel> _panels = new List<CameraPanel>();
    RobotProfile _profile;
    float _defaultFit;   // best-fit size with every camera visible; sizes are relative to it
    // Per camera, same index as _panels (lists, because cameras can be added later).
    readonly List<Texture2D> _textures = new List<Texture2D>();
    readonly List<byte[]> _rawBuffers = new List<byte[]>();
    readonly List<double> _lastRenderTime = new List<double>();
    readonly List<bool> _warnedFormat = new List<bool>();
    // Latest-frame decode: the subscription callbacks only keep the newest message per camera;
    // LateUpdate decodes it (once per frame at most). An older message that is replaced before it
    // was decoded counts as dropped, so a backlog never makes the app show stale frames.
    readonly List<CompressedImageMsg> _pendingCompressed = new List<CompressedImageMsg>();
    readonly List<ImageMsg> _pendingRaw = new List<ImageMsg>();
    readonly List<int> _dropped = new List<int>();
    readonly List<float> _decodeMs = new List<float>();
    readonly List<double> _lastFrameTime = new List<double>();   // Time.unscaledTime of the last shown frame
    readonly List<float> _fps = new List<float>();
    readonly List<int> _framesInWindow = new List<int>();
    readonly List<float> _fpsWindowStart = new List<float>();
    readonly List<int> _loggedDropped = new List<int>();
    readonly List<int> _received = new List<int>(), _loggedReceived = new List<int>();
    float _lastDropLog;
    readonly System.Diagnostics.Stopwatch _decodeWatch = new System.Diagnostics.Stopwatch();
    const float FpsWindowSeconds = 1f, LogSeconds = 10f;
    float _nextLog;

    // ROS topic list as reported by the endpoint (topic -> type), filled once discovery starts.
    readonly Dictionary<string, string> _topics = new Dictionary<string, string>();
    bool _listening, _searchNow;

    const float DiscoveryWaitSeconds = 2.5f;
    // How long a candidate camera topic may take to deliver a frame before it counts as idle.
    const float LiveCheckSeconds = 3f;

    // Discovery keeps only camera topics that actually deliver frames: the endpoint lists every
    // topic with any publisher OR subscriber, so a topic only subscribed to (e.g. by this app on
    // another robot) would otherwise show up as a camera. Tests without ROS turn this off.
    public static bool RequireLiveTopics = true;

    public IReadOnlyList<CameraPanel> Panels => _panels;
    public RobotProfile Profile => _profile;

    // Camera blocks exist (immediately for known robots, after discovery in setup mode).
    public bool IsReady { get; private set; }
    public event Action Ready;
    // Raised when Reset layout changes which cameras are shown.
    public event Action VisibilityReset;
    // Raised when "Find cameras" added camera blocks.
    public event Action CamerasAdded;
    // Raised when a camera was renamed.
    public event Action LabelsChanged;
    // Raised when discovery decides the robot's namespace (the Joy topic follows it).
    public event Action<string> NamespaceChanged;

    // Raised after a camera's texture was shown; carries the current texture instance
    // (raw decoding can replace it).
    public event Action<int, Texture2D> FrameReady;

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
        foreach (var cam in _profile.cameras)
            AddPanel(cam);

        ApplySavedVisibility();
        if (_panels.Count > 0) _defaultFit = BestFit(_panels, out _);
        Layout();
        ApplySavedPositions();

        IsReady = true;
        Ready?.Invoke();
    }

    void AddPanel(RobotProfile.CameraConfig cam)
    {
        string topic = _profile.FullTopic(cam);
        int index = _panels.Count;
        var panel = CameraPanel.Create(panelParent, cam, topic);
        string label = PlayerPrefs.GetString(LabelKey(_profile, cam), "");
        if (label.Length > 0) panel.SetLabel(label);
        panel.RenameRequested += BeginRename;
        _panels.Add(panel);
        if (_blocksHidden) { _hiddenByMode.Add(panel); panel.Visible = false; }   // new camera found during first person
        _textures.Add(new Texture2D(cam.resolution.x, cam.resolution.y, TextureFormat.RGB24, false));
        _rawBuffers.Add(null);
        _lastRenderTime.Add(0.0);
        _warnedFormat.Add(false);
        _pendingCompressed.Add(null);
        _pendingRaw.Add(null);
        _dropped.Add(0);
        _decodeMs.Add(0f);
        _lastFrameTime.Add(-1.0);
        _fps.Add(0f);
        _framesInWindow.Add(0);
        _fpsWindowStart.Add(Time.unscaledTime);
        _loggedDropped.Add(0);
        _received.Add(0);
        _loggedReceived.Add(0);

        if (cam.raw)
            ros.Subscribe<ImageMsg>(topic, msg => OnRawMessage(msg, index));
        else
            ros.Subscribe<CompressedImageMsg>(topic, msg => OnCompressedMessage(msg, index));
    }

    // Keep the endpoint's topic list in _topics and ask for a fresh copy.
    void RequestTopics()
    {
        if (!_listening)
        {
            ros.ListenForTopics(t => _topics[t.Topic] = t.RosMessageName, notifyAllExistingTopics: true);
            _listening = true;
        }
        ros.RefreshTopicsList();
    }

    List<(string, string)> TopicList() => _topics.Select(kv => (kv.Key, kv.Value)).ToList();

    // Wait for the topic list, or less if SearchAgain() was pressed.
    IEnumerator WaitForTopics()
    {
        _searchNow = false;
        float until = Time.time + DiscoveryWaitSeconds;
        while (Time.time < until && !_searchNow) yield return null;
    }

    // Setup mode: ask the ROS endpoint for its topic list until publishing cameras show up
    // (or the user continues without cameras).
    IEnumerator Discover()
    {
        while (!IsReady)
        {
            SetupStatus = "Looking for camera topics\u2026";
            RequestTopics();
            yield return WaitForTopics();
            if (IsReady) yield break;

            var live = new List<(string, string)>();
            SetupStatus = "Checking which cameras are publishing\u2026";
            yield return LiveTopics(TopicList(), new HashSet<string>(), live);
            if (IsReady || ConfigureDiscovered(live)) yield break;
            SetupStatus = $"No publishing cameras found on {ros.RosIPAddress} yet. Searching again every few seconds.";
        }
    }

    // Of the camera candidates in `topics` (except `skip`), add to `result` those that deliver a
    // frame within LiveCheckSeconds. The check subscriptions are kept: TeamSOBITS ros_tcp_endpoint
    // has no remove_subscriber command, and ROSConnection.Unsubscribe kills its client connection
    // (the connection resets and the stream to the headset breaks). Live ones become blocks anyway.
    IEnumerator LiveTopics(List<(string, string)> topics, HashSet<string> skip, List<(string, string)> result)
    {
        var candidates = topics.Where(t => CameraDiscovery.IsCameraCandidate(t.Item1, t.Item2) && !skip.Contains(t.Item1)).ToList();
        if (!RequireLiveTopics) { result.AddRange(candidates); yield break; }

        var alive = new HashSet<string>();
        foreach (var (topic, type) in candidates)
        {
            string t = topic;
            if (CameraDiscovery.IsCompressed(type)) ros.Subscribe<CompressedImageMsg>(t, _ => alive.Add(t));
            else ros.Subscribe<ImageMsg>(t, _ => alive.Add(t));
        }
        float until = Time.time + LiveCheckSeconds;
        while (Time.time < until && alive.Count < candidates.Count) yield return null;

        result.AddRange(candidates.Where(c => alive.Contains(c.Item1)));
    }

    // Setup mode: search immediately instead of waiting for the next retry.
    public void SearchAgain()
    {
        SetupStatus = "Looking for camera topics\u2026";
        _searchNow = true;
    }

    // Use the camera topics found in `topics` (topic, ROS type) for the robot being set up.
    // Returns false if there are none yet.
    public bool ConfigureDiscovered(IEnumerable<(string topic, string type)> topics)
    {
        if (IsReady) return true;
        var list = topics.ToList();
        var found = CameraDiscovery.Select(list);
        if (found.Count == 0) return false;

        _profile.robotNamespace = CameraDiscovery.CommonNamespace(found.Select(f => f.topic));
        NamespaceChanged?.Invoke(_profile.robotNamespace);
        _profile.cameras = found.Select(ToConfig).ToArray();
        SetupStatus = $"{found.Count} camera{(found.Count == 1 ? "" : "s")} found";
        BuildPanels();
        return true;
    }

    // Setup mode: keep the robot without cameras (e.g. to drive it with the controllers only).
    // The Joy namespace then comes from the robot's other topics, if they share one.
    public void ContinueWithoutCameras()
    {
        if (IsReady) return;
        _profile.robotNamespace = CameraDiscovery.RobotNamespace(_topics.Keys);
        NamespaceChanged?.Invoke(_profile.robotNamespace);
        _profile.cameras = Array.Empty<RobotProfile.CameraConfig>();
        SetupStatus = "No cameras";
        BuildPanels();
    }

    // "Find cameras": search the topic list again and add cameras this robot does not have yet.
    // Existing blocks keep their place; `done` receives how many cameras were added.
    public void FindNewCameras(Action<int> done) => StartCoroutine(FindNewCamerasRoutine(done));

    IEnumerator FindNewCamerasRoutine(Action<int> done)
    {
        RequestTopics();
        yield return WaitForTopics();

        // Known cameras, including the raw/compressed twin of each, so a camera is never added twice.
        var known = new HashSet<string>();
        foreach (var t in _profile.cameras.Select(c => _profile.FullTopic(c)))
        {
            known.Add(t);
            known.Add(t.EndsWith("/compressed") ? t.Substring(0, t.Length - "/compressed".Length) : t + "/compressed");
        }
        var live = new List<(string, string)>();
        yield return LiveTopics(TopicList(), known, live);
        var fresh = CameraDiscovery.Select(live).Where(f => !known.Contains(f.topic)).ToList();
        if (fresh.Count > 0)
        {
            var names = new HashSet<string>(_profile.cameras.Select(c => c.displayName));
            var added = fresh.Select(ToConfig).ToList();
            foreach (var c in added)
                while (names.Contains(c.displayName)) c.displayName += " 2";   // keep layout keys unique
            if (string.IsNullOrEmpty(_profile.robotNamespace) && _profile.cameras.Length == 0)
            {
                _profile.robotNamespace = CameraDiscovery.CommonNamespace(fresh.Select(f => f.topic));
                NamespaceChanged?.Invoke(_profile.robotNamespace);
            }
            _profile.cameras = _profile.cameras.Concat(added).ToArray();
            foreach (var c in added) AddPanel(c);

            _defaultFit = BestFit(_panels, out _);
            if (!HasCustomLayout) Layout();
            else foreach (var p in _panels.Skip(_panels.Count - added.Count)) PlaceNewPanel(p);
            if (!InSetup) RobotLibrary.Save(_profile);
            CamerasAdded?.Invoke();
        }
        done?.Invoke(fresh.Count);
    }

    // Custom (dragged) layout: put a newly found camera just above the others instead of
    // re-arranging blocks the user placed by hand.
    void PlaceNewPanel(CameraPanel p)
    {
        float top = _panels.Where(o => o != p && IsOn(o)).Select(o => o.transform.localPosition.y + o.Height / 2f)
                           .DefaultIfEmpty(0f).Max();
        var pos = new Vector3(0f, top + rowGap + p.Height / 2f, distance);
        p.transform.localPosition = pos;
        p.transform.localRotation = Quaternion.LookRotation(pos);
    }

    static RobotProfile.CameraConfig ToConfig(CameraDiscovery.Found f) => new RobotProfile.CameraConfig
    {
        displayName = f.displayName,
        topicSuffix = f.topic,
        raw = f.raw,
        resolution = new Vector2Int(640, 480),   // corrected from the first frame
    };

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
        SetOn(panel, visible);
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
            SetOn(p, true);
        }
        PlayerPrefs.Save();
        Layout();
        VisibilityReset?.Invoke();
    }

    // Forget a robot's saved positions, shown/hidden cameras and camera names (e.g. when it is removed).
    public static void ForgetLayout(RobotProfile robot)
    {
        foreach (var cam in robot.cameras)
        {
            PlayerPrefs.DeleteKey(PositionKey(robot, cam));
            PlayerPrefs.DeleteKey(VisibleKey(robot, cam));
            PlayerPrefs.DeleteKey(LabelKey(robot, cam));
        }
        PlayerPrefs.Save();
    }

    // --- Renaming (layout mode): the Quest keyboard starts empty; confirming it empty restores
    // the original name. Names are kept per robot; layouts stay keyed to the original name. ---
    readonly TextKeyboard _renameKeyboard = new TextKeyboard();
    CameraPanel _renaming;
    string _labelBeforeRename;

    void BeginRename(CameraPanel panel)
    {
        if (_renameKeyboard.IsOpen) return;
        _renaming = panel;
        _labelBeforeRename = panel.Label;
        _renameKeyboard.Open(panel.Label, panel.Config.displayName);   // empty or cancelled keeps the name
        Debug.Log($"[Rename] open: {panel.Label}");
    }

    public void RenameCamera(CameraPanel panel, string label)
    {
        label = (label ?? "").Trim();
        if (label.Length == 0 || label == panel.Config.displayName)
        {
            PlayerPrefs.DeleteKey(LabelKey(_profile, panel.Config));
            label = panel.Config.displayName;
        }
        else PlayerPrefs.SetString(LabelKey(_profile, panel.Config), label);
        PlayerPrefs.Save();
        panel.SetLabel(label);
        if (!HasCustomLayout) Layout();
        LabelsChanged?.Invoke();
    }

    void Update()
    {
        if (_renaming == null) return;
        string label = _renameKeyboard.Poll();   // closes the keyboard once the user is done
        if (_renameKeyboard.IsOpen)
        {
            _renaming.SetLabel(QuestControllerPublisher.TypingDisplay(_renameKeyboard.Text, "Type a name\u2026"));
            return;
        }
        if (label != null) RenameCamera(_renaming, label);
        else _renaming.SetLabel(_labelBeforeRename);   // cancelled: keep the previous name
        Debug.Log($"[Rename] {(label != null ? "renamed to " + label : "cancelled, kept " + _labelBeforeRename)}");
        _renaming = null;
    }

    void ApplySavedVisibility()
    {
        foreach (var p in _panels)
            if (PlayerPrefs.GetInt(VisibleKey(p), 1) == 0) SetOn(p, false);
    }

    // Remember where the user dragged or resized a block, per robot and camera. Saved as
    // "x;y;z;size": the block's place relative to the head and its view size.
    public void SavePosition(CameraPanel panel)
    {
        var v = panel.transform.localPosition;
        PlayerPrefs.SetString(PositionKey(panel),
            string.Format(CultureInfo.InvariantCulture, "{0};{1};{2};{3}", v.x, v.y, v.z, panel.Size));
        PlayerPrefs.Save();
    }

    void ApplySavedPositions()
    {
        foreach (var p in _panels)
        {
            var parts = PlayerPrefs.GetString(PositionKey(p), "").Split(';');
            if (parts.Length >= 4 &&
                float.TryParse(parts[3], NumberStyles.Float, CultureInfo.InvariantCulture, out float size))
                p.SetSize(size);   // older saves (x;y;z) keep the automatic size
            if (parts.Length >= 3 &&
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
    static string LabelKey(RobotProfile r, RobotProfile.CameraConfig c) => $"CameraLabel/{r.name}/{c.displayName}";

    // Automatic layout of the visible cameras: pick the column count that allows the largest
    // views within the layout area, size the views relative to the all-cameras layout (so with
    // every camera shown the layout is the default one), then place the grid centred in front
    // of the head, each block facing the eye. Rows are sized from the actual blocks, so blocks
    // never overlap; views in a row share a horizontal centre line.
    void Layout()
    {
        var visible = _panels.FindAll(IsOn);
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

    // Index of the camera whose topic is `topicSuffixOrFullTopic` (relative to the namespace, or
    // absolute), -1 if there is none.
    public int IndexOf(string topicSuffixOrFullTopic)
    {
        if (_profile == null || string.IsNullOrEmpty(topicSuffixOrFullTopic)) return -1;
        string wanted = _profile.FullTopic(topicSuffixOrFullTopic);
        for (int i = 0; i < _panels.Count; i++)
            if (_panels[i].Topic == wanted) return i;
        return -1;
    }

    // Keep decoding a camera although its block is hidden (first-person view uses it).
    readonly HashSet<int> _forceDecode = new HashSet<int>();

    public void ForceDecode(int index, bool on)
    {
        if (index < 0 || index >= _panels.Count) return;
        if (on) _forceDecode.Add(index); else _forceDecode.Remove(index);
    }

    // First-person view: hide / show every camera block without touching the saved
    // visibility or layout. Showing restores exactly the blocks that were visible before.
    readonly List<CameraPanel> _hiddenByMode = new List<CameraPanel>();

    bool _blocksHidden;
    public bool BlocksShown => !_blocksHidden;

    // Wanted visibility; while blocks are hidden (first person) it only lives in _hiddenByMode and every panel GameObject stays inactive.
    public bool IsOn(CameraPanel p) => _blocksHidden ? _hiddenByMode.Contains(p) : p.Visible;

    void SetOn(CameraPanel p, bool on)
    {
        if (!_blocksHidden) { p.Visible = on; return; }
        if (on) { if (!_hiddenByMode.Contains(p)) _hiddenByMode.Add(p); }
        else _hiddenByMode.Remove(p);
    }

    public void SetBlocksShown(bool shown)
    {
        if (shown == !_blocksHidden) return;
        SetBlocksShownInternal(shown);
    }

    void SetBlocksShownInternal(bool shown)
    {
        _blocksHidden = !shown;
        if (!shown)
        {
            foreach (var p in _panels)
                if (p.Visible) { _hiddenByMode.Add(p); p.Visible = false; }
            return;
        }
        foreach (var p in _hiddenByMode)
            if (p != null) p.Visible = true;
        _hiddenByMode.Clear();
    }

    // Hidden cameras skip decoding entirely; others are limited to their maxFps.
    bool IsWanted(int index) => _panels[index].Visible || _forceDecode.Contains(index);

    bool ShouldRender(int index)
    {
        float fps = _panels[index].Config.maxFps;
        if (fps <= 0f) return true;
        double now = Time.unscaledTimeAsDouble;
        if (now - _lastRenderTime[index] < 1.0 / fps) return false;
        _lastRenderTime[index] = now;
        return true;
    }

    // Callbacks only store the newest message; decoding happens in LateUpdate.
    void OnCompressedMessage(CompressedImageMsg msg, int index)
    {
        if (this == null || !isActiveAndEnabled) return;
        if (msg == null || msg.data == null || msg.data.Length == 0 || index >= _panels.Count || !IsWanted(index)) return;
        _received[index]++;
        if (_pendingCompressed[index] != null) _dropped[index]++;
        _pendingCompressed[index] = msg;
    }

    void OnRawMessage(ImageMsg msg, int index)
    {
        if (this == null || !isActiveAndEnabled) return;
        if (msg == null || index >= _panels.Count || !IsWanted(index)) return;
        _received[index]++;
        if (_pendingRaw[index] != null) _dropped[index]++;
        _pendingRaw[index] = msg;
    }

    void LateUpdate()
    {
        for (int i = 0; i < _panels.Count; i++)
        {
            var compressed = _pendingCompressed[i];
            var raw = _pendingRaw[i];
            if (compressed == null && raw == null) continue;
            if (!IsWanted(i))
            {
                _pendingCompressed[i] = null;   // hidden meanwhile: nothing to decode
                _pendingRaw[i] = null;
                continue;
            }
            if (!ShouldRender(i)) continue;     // keep the newest for the next allowed frame
            _pendingCompressed[i] = null;
            _pendingRaw[i] = null;

            _decodeWatch.Restart();
            if (compressed != null) RenderCompressedTexture(compressed, i);
            else RenderRawTexture(raw, i);
            _decodeWatch.Stop();
            _decodeMs[i] = (float)_decodeWatch.Elapsed.TotalMilliseconds;
        }

        float now = Time.unscaledTime;
        for (int i = 0; i < _panels.Count; i++)
            if (now - _fpsWindowStart[i] >= FpsWindowSeconds)
            {
                _fps[i] = _framesInWindow[i] / (now - _fpsWindowStart[i]);
                _framesInWindow[i] = 0;
                _fpsWindowStart[i] = now;
            }

        if (now >= _nextLog)
        {
            _nextLog = now + LogSeconds;
            for (int i = 0; i < _panels.Count; i++)
            {
                // Drops beyond the maxFps thinning mean the backlog really grew. Otherwise log at most once a minute.
                int drops = _dropped[i] - _loggedDropped[i];
                int received = _received[i] - _loggedReceived[i];
                float maxFps = _panels[i].Config.maxFps;
                float expected = maxFps > 0f ? Mathf.Max(0f, received - maxFps * LogSeconds) : 0f;
                bool grew = drops > expected + 5f;
                bool minuteDue = drops > 0 && now - _lastDropLog >= 60f;
                _loggedDropped[i] = _dropped[i];
                _loggedReceived[i] = _received[i];
                if (grew || minuteDue)
                {
                    _lastDropLog = now;
                    Debug.Log($"[ImageSubscriber] cam {i}: fps {_fps[i]:F1}, dropped {_dropped[i]}, decode {_decodeMs[i]:F1} ms");
                }
            }
        }
    }

    // Messages replaced before they were decoded (includes maxFps thinning), since start.
    public int DroppedFrames(int index) => index >= 0 && index < _dropped.Count ? _dropped[index] : 0;
    // Duration of the last decode (ms).
    public float DecodeMs(int index) => index >= 0 && index < _decodeMs.Count ? _decodeMs[index] : 0f;
    // Time.unscaledTime of the last shown frame, -1 before the first.
    public double LastFrameTime(int index) => index >= 0 && index < _lastFrameTime.Count ? _lastFrameTime[index] : -1.0;
    // Shown frames per second over the last second.
    public float Fps(int index) => index >= 0 && index < _fps.Count ? _fps[index] : 0f;

    void RenderCompressedTexture(CompressedImageMsg msg, int index)
    {
        if (!_textures[index].LoadImage(msg.data))
        {
            Debug.LogWarning($"Failed to decode compressed image on {_panels[index].Topic}. format={msg.format}");
            return;
        }
        ShowFrame(index);
    }

    void RenderRawTexture(ImageMsg msg, int index)
    {
        var tex = _textures[index];
        var buf = _rawBuffers[index];
        bool ok = RawImageDecoder.TryDecode(msg, ref tex, ref buf);
        _textures[index] = tex;
        _rawBuffers[index] = buf;
        if (!ok)
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
        _lastFrameTime[index] = Time.unscaledTime;
        _framesInWindow[index]++;
        FrameReady?.Invoke(index, tex);
    }
}
