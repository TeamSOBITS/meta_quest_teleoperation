using System;
using System.Collections;
using System.Collections.Generic;
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
    public float distance = HudTheme.ReferenceDistance;
    public float columnGap = HudTheme.BlockGap;
    public float rowGap = HudTheme.BlockGap;
    // Area the auto layout may use. minBottom keeps blocks above the HUD bar (compact: 0.7x, 0.3 m lower than at full size).
    public float maxWidth = 6.0f;
    public float maxTop = 2.0f;
    public float minBottom = HudTheme.LayoutMinBottom;
    // Upper limit for how much views grow when cameras are hidden (1 = default size).
    public float maxGrow = 1.6f;

    readonly List<CameraPanel> _panels = new List<CameraPanel>();
    RobotProfile _profile;
    CameraStreams _streams;           // subscribe / decode / statistics
    CameraLayout _layout;             // grid, saved positions, visibility
    CameraDiscoveryFlow _discovery;   // setup mode: topic discovery

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
    // Raised when a camera is shown or hidden (bar toggle, Reset layout, saved state); (index, on).
    // First person follows it: the head-camera card and the hand cards show or hide with their camera.
    public event Action<int, bool> CameraVisibilityChanged;
    // Raised when a camera was renamed.
    public event Action LabelsChanged;
    // Raised when discovery decides the robot's namespace (the Joy topic follows it).
    public event Action<string> NamespaceChanged;

    // Raised after a camera's texture was shown; carries the current texture instance
    // (raw decoding can replace it).
    public event Action<int, Texture2D> FrameReady;

    public bool InSetup => RobotProfile.SetupMode && _profile != null && _profile.isCustom;
    public string SetupStatus { get; internal set; } = "";

    void Awake()
    {
        _streams = new CameraStreams(this, _panels);
        _layout = new CameraLayout(this, _panels);
        _discovery = new CameraDiscoveryFlow(this);
        _streams.FrameDecoded = AdaptToFrame;
        _streams.FrameReady = (i, tex) => FrameReady?.Invoke(i, tex);
        _layout.VisibilityChanged = (i, on) => CameraVisibilityChanged?.Invoke(i, on);
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        var profile = _profile = RobotProfile.Selected != null ? RobotProfile.Selected : defaultProfile;
        if (profile == null)
        {
            Debug.LogError("ImageSubscriber: no robot selected and no default profile set.");
            return;
        }
        Settings.EnsureMigrated(RobotLibrary.LoadAll().Prepend(profile));   // no-op once done

        if (panelParent == null && Camera.main != null)
            panelParent = Camera.main.transform;

        if (InSetup && profile.cameras.Length == 0)
            StartCoroutine(_discovery.Discover());
        else
            BuildPanels();
    }

    internal void BuildPanels()
    {
        foreach (var cam in _profile.cameras)
            AddPanel(cam);
        _layout.Build();

        IsReady = true;
        Ready?.Invoke();
    }

    // "Compressed images" (HUD bar, default on): off = the compressed topics of the robot are not
    // used, their raw twin is (sensor_msgs/Image) instead. Global; changing it reopens the robot
    // screen (subscriptions cannot be dropped, see HudBar).
    public static bool Compressed => Settings.Compressed;

    void AddPanel(RobotProfile.CameraConfig cam)
    {
        string topic = _streams.EffectiveTopic(cam, out bool raw);
        var panel = CameraPanel.Create(panelParent, cam, topic, InSetup);   // topic line: setup mode only
        string label = Settings.For(_profile).Camera(cam).Label;
        if (label.Length > 0) panel.SetLabel(label);
        panel.RenameRequested += BeginRename;
        _panels.Add(panel);
        _layout.OnPanelAdded(panel);   // new camera found during first person
        _streams.Add(cam, topic, raw);
    }

    // Cameras found by "Find cameras" (already in the profile): add their blocks.
    internal void AddCameras(List<RobotProfile.CameraConfig> added)
    {
        foreach (var c in added) AddPanel(c);
        _layout.OnCamerasAdded(added.Count);
        if (!InSetup) RobotLibrary.Save(_profile);
        CamerasAdded?.Invoke();
    }

    internal void NotifyNamespace(string ns) => NamespaceChanged?.Invoke(ns);

    // --- Setup mode (see CameraDiscoveryFlow) ---
    public void SearchAgain() => _discovery.SearchAgain();
    public bool ConfigureDiscovered(IEnumerable<(string topic, string type)> topics) => _discovery.ConfigureDiscovered(topics);
    public void ContinueWithoutCameras() => _discovery.ContinueWithoutCameras();
    public void FindNewCameras(Action<int> done) => StartCoroutine(_discovery.FindNewCameras(done));

    // --- Layout and visibility (see CameraLayout) ---
    public bool HasCustomLayout => _layout.HasCustomLayout;
    public bool BlocksShown => _layout.BlocksShown;
    public bool IsOn(CameraPanel p) => _layout.IsOn(p);
    public void SetBlocksShown(bool shown) => _layout.SetBlocksShown(shown);
    public void SetCameraVisible(CameraPanel panel, bool visible) => _layout.SetCameraVisible(panel, visible);
    public void SavePosition(CameraPanel panel) => _layout.SavePosition(panel);

    public void ResetLayout()
    {
        _layout.Reset();
        VisibilityReset?.Invoke();
    }

    // --- Streams (see CameraStreams) ---
    // Keep decoding a camera although its block is hidden (first-person view uses it).
    public void ForceDecode(int index, bool on) => _streams.ForceDecode(index, on);
    public int DroppedFrames(int index) => _streams.DroppedFrames(index);
    public float DecodeMs(int index) => _streams.DecodeMs(index);
    public double LastFrameTime(int index) => _streams.LastFrameTime(index);
    public float Fps(int index) => _streams.Fps(index);

    // The view takes each camera's real frame size from its first frame (sim and real cameras can
    // differ from the configured resolution). Added robots keep it in their config and save it;
    // built-in robots only adjust the live aspect.
    void AdaptToFrame(int index, Texture2D tex)
    {
        var panel = _panels[index];
        var cam = panel.Config;
        bool learned = _profile.isCustom && (cam.resolution.x != tex.width || cam.resolution.y != tex.height);
        bool live = !_profile.isCustom && Mathf.Abs(panel.Aspect - (float)tex.width / tex.height) > 1e-3f;
        if (!learned && !live) return;
        if (learned) cam.resolution = new Vector2Int(tex.width, tex.height);
        else panel.SetLiveAspect((float)tex.width / tex.height);
        panel.SetSize(panel.Size);
        _layout.OnFrameSizeChanged();
        if (learned && !InSetup) RobotLibrary.Save(_profile);
    }

    void LateUpdate() => _streams?.Tick();

    // Index of the camera whose topic is `topicSuffixOrFullTopic` (relative to the namespace, or
    // absolute), -1 if there is none.
    public int IndexOf(string topicSuffixOrFullTopic)
    {
        if (_profile == null || string.IsNullOrEmpty(topicSuffixOrFullTopic)) return -1;
        string wanted = _profile.FullTopic(topicSuffixOrFullTopic);
        string raw = wanted.EndsWith(RosNames.CompressedSuffix) ? wanted.Substring(0, wanted.Length - RosNames.CompressedSuffix.Length) : wanted;
        for (int i = 0; i < _panels.Count; i++)
            if (_panels[i].Topic == wanted || _panels[i].Topic == raw) return i;
        return -1;
    }

    // --- Renaming (layout mode): the Quest keyboard starts empty; confirming it empty restores
    // the original name. Names are kept per robot; layouts stay keyed to the original name. ---
    readonly TextKeyboard _renameKeyboard = new TextKeyboard();
    CameraPanel _renaming;
    string _labelBeforeRename;

    public void BeginRename(CameraPanel panel)
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
        var saved = Settings.For(_profile).Camera(panel.Config);
        if (label.Length == 0 || label == panel.Config.displayName)
        {
            saved.Label = "";
            label = panel.Config.displayName;
        }
        else saved.Label = label;
        Settings.Save();
        panel.SetLabel(label);
        _layout.ArrangeIfAutomatic();
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
}
