using System;
using System.Collections;
using System.IO;
using TMPro;
using UnityEngine;
using UnityEngine.XR.Hands;

/// <summary>
/// Sets up the robot screen around the camera blocks: studio surroundings, the HUD bar
/// (ROS IP + controls), the camera block dragger, controller hints and the optional
/// "lazy follow" mode. Runs after ImageSubscriber so it can use the blocks it created.
///
/// For a robot being added (setup mode) it first shows a "Setting up" card while the
/// cameras are discovered (Search again / Continue without cameras / Cancel), keeps Joy off
/// (layout mode) and offers Save robot / Cancel.
/// </summary>
[DefaultExecutionOrder(100)]
public class TeleopHud : MonoBehaviour
{
    public QuestControllerPublisher publisher;
    public ImageSubscriber images;

    // Panels follow this transform; defaults to the main camera.
    public Transform hudParent;

    const string LazyFollowKey = "LazyFollow";

    Transform _bar;
    HeadFollower _follower;
    GameObject _waiting;
    TextMeshProUGUI _waitingStatus;

    public bool LazyFollow { get; private set; }

    // First-person view (3D robot model + head camera image) instead of the camera blocks.
    public bool FirstPerson { get; private set; }
    // Raised when the view mode changes (the bar syncs its toggle and buttons).
    public event Action ViewModeChanged;
    FirstPersonView _fpv;
    // Experiments (see Experiments/): round-trip latency probe and the first-person status strip.
    RoundTrip _roundTrip;
    StatusStrip _strip;
    HandCamPip _handCams;
    ArmTargets _targets;
    BaseVelocity _baseVel;

    const string CaptureKey = "DebugCapture";
    const float CaptureDelaySeconds = 10f;

    void Start()
    {
        StudioEnvironment.Apply(Camera.main);

        if (hudParent == null && Camera.main != null)
            hudParent = Camera.main.transform;

        images.NamespaceChanged += publisher.SetNamespace;   // Joy topic follows discovery
        if (images.InSetup)
            publisher.controlRobot = false;  // layout mode: the trigger arranges blocks, nothing reaches a robot

        if (images.IsReady)
            BuildHud();
        else
        {
            ShowWaiting();
            images.Ready += BuildHud;
        }
    }

    void BuildHud()
    {
        images.Ready -= BuildHud;
        if (_waiting != null) Destroy(_waiting);

        ExperimentSettings.RegisterAll();
        ExperimentSettings.Changed += OnExperimentChanged;
        _bar = HudBar.Create(hudParent, publisher, images, this).transform;
        ExperimentsPanel.Create(_bar);
        UpdateRoundTrip();
        SyncDeadman();
        images.CamerasAdded += RebuildBar;

        var dragger = gameObject.AddComponent<PanelDragger>();
        dragger.publisher = publisher;
        dragger.images = images;

        ControllerHints.Create(hudParent);

        if (PlayerPrefs.GetInt(LazyFollowKey, 0) == 1)
            SetLazyFollow(true);

        var profile = images.Profile;
        if (profile != null && profile.HasModel) StudioEnvironment.EnsureModelKeyLight();
        string modeOverride = FirstPersonView.ViewModeOverride;
        FirstPersonView.ViewModeOverride = null;   // one robot screen only
        if (profile != null && profile.HasModel)
        {
            string mode = modeOverride ?? PlayerPrefs.GetString(FirstPersonView.ViewModeKey(profile), FirstPersonView.ModeBlocks);
            if (mode == FirstPersonView.ModeFirstPerson)
                SetFirstPerson(true, save: modeOverride == null);
        }

        if (PlayerPrefs.GetInt(CaptureKey, 0) == 1)
            StartCoroutine(CaptureLater());

        if (DemoRecorder.Request != null)
        {
            var request = DemoRecorder.Request.Value;
            DemoRecorder.Request = null;   // this launch only
            DemoRecorder.Create(this, images, request);
        }
    }

    // Move the HUD (panels, bar, future panels) under another transform, keeping local poses.
    internal void ReparentHud(Transform parent)
    {
        foreach (var panel in images.Panels)
            panel.transform.SetParent(parent, false);
        _bar.SetParent(parent, false);
        images.panelParent = parent;
        hudParent = parent;
    }

    // Blocks (default) or first person. The choice is kept per robot. Robots without a model
    // stay in blocks mode.
    public void SetFirstPerson(bool on) => SetFirstPerson(on, true);

    internal void SetFirstPerson(bool on, bool save)
    {
        var profile = images != null ? images.Profile : null;
        if (on && (profile == null || !profile.HasModel))
        {
            Debug.Log("FPV: this robot has no model, staying in blocks mode");
            ViewModeChanged?.Invoke();   // puts the toggle back
            return;
        }
        if (on == FirstPerson) return;
        FirstPerson = on;
        if (profile != null && save)
        {
            PlayerPrefs.SetString(FirstPersonView.ViewModeKey(profile),
                on ? FirstPersonView.ModeFirstPerson : FirstPersonView.ModeBlocks);
            PlayerPrefs.Save();
        }

        if (on)
        {
            _fpv = FirstPersonView.Create(images, profile);
            UpdateStatusStrip();
            UpdateFpvExperiments();
            images.SetBlocksShown(false);
            if (_bar != null) _bar.gameObject.SetActive(false);   // menu / hand gesture brings it back
            PlaceBar();
        }
        else
        {
            if (_strip != null) Destroy(_strip.gameObject);
            _strip = null;
            UpdateFpvExperiments();   // destroys them
            if (_fpv != null) Destroy(_fpv.gameObject);
            _fpv = null;
            images.SetBlocksShown(true);
            PlaceBar();
            if (_bar != null) _bar.gameObject.SetActive(true);
        }
        Debug.Log($"FPV: view mode -> {(on ? "firstperson" : "blocks")}");
        ViewModeChanged?.Invoke();
    }

    // First person: put the model back under the headset.
    public void Recenter()
    {
        if (_fpv != null) _fpv.Recenter();
    }

    // Autonomous tests (PlayerPrefs DebugCapture=1, set from an intent extra): once the screen has
    // been up a while, save what the main camera sees as a PNG for `adb pull`, then clear the flag.
    IEnumerator CaptureLater()
    {
        yield return new WaitForSecondsRealtime(CaptureDelaySeconds);
        yield return null;
        PlayerPrefs.DeleteKey(CaptureKey);
        PlayerPrefs.Save();

        var cam = Camera.main;
        if (cam == null) { Debug.LogWarning("FPV: capture failed, no main camera"); yield break; }
        RenderTexture rt = null;
        Texture2D png = null;
        var previousTarget = cam.targetTexture;
        var previousEye = cam.stereoTargetEye;
        var previousActive = RenderTexture.active;
        try
        {
            rt = new RenderTexture(1280, 720, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
            cam.stereoTargetEye = StereoTargetEyeMask.None;
            cam.targetTexture = rt;
            cam.Render();
            RenderTexture.active = rt;
            png = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
            png.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
            png.Apply(false);
            string path = Path.Combine(Application.persistentDataPath, "fpv_capture.png");
            File.WriteAllBytes(path, png.EncodeToPNG());
            Debug.Log($"FPV: capture {path}");
        }
        catch (Exception e)
        {
            Debug.LogWarning($"FPV: capture failed: {e.Message}");
        }
        finally
        {
            cam.targetTexture = previousTarget;
            cam.stereoTargetEye = previousEye;
            RenderTexture.active = previousActive;
            if (rt != null) { rt.Release(); Destroy(rt); }
            if (png != null) Destroy(png);
        }
    }

    // "Find cameras" added blocks: rebuild the bar so it lists their toggles too.
    void RebuildBar()
    {
        var parent = _bar.parent;
        Destroy(_bar.gameObject);
        _bar = HudBar.Create(hudParent, publisher, images, this).transform;
        _bar.SetParent(parent, false);
        ExperimentsPanel.Create(_bar);
        BarCompact = false;   // the new bar is at its normal pose
        PlaceBar();
    }

    void OnExperimentChanged(string key, bool on)
    {
        if (this == null) return;
        if (key == ExperimentSettings.Deadman) SyncDeadman();
        else if (key == ExperimentSettings.Rtt) UpdateRoundTrip();
        else if (key == ExperimentSettings.Status) UpdateStatusStrip();
        else if (key == ExperimentSettings.HandCams || key == ExperimentSettings.Targets
                 || key == ExperimentSettings.BaseVel) UpdateFpvExperiments();
    }

    void SyncDeadman()
    {
        if (publisher != null) publisher.deadmanEnabled = ExperimentSettings.IsOn(ExperimentSettings.Deadman);
    }

    // The round-trip probe exists while the "rtt" experiment is on (and setup mode is not).
    void UpdateRoundTrip()
    {
        bool want = ExperimentSettings.IsOn(ExperimentSettings.Rtt) && publisher != null;
        if (want && _roundTrip == null) _roundTrip = RoundTrip.Create(publisher);
        else if (!want && _roundTrip != null) { Destroy(_roundTrip.gameObject); _roundTrip = null; }
    }

    // The status strip exists in first person while the "status" experiment is on.
    void UpdateStatusStrip()
    {
        bool want = FirstPerson && _fpv != null && _fpv.Model != null && ExperimentSettings.IsOn(ExperimentSettings.Status);
        if (want && _strip == null)
            _strip = StatusStrip.Create(FirstPersonView.Head, publisher, images, _fpv.Model, images.Profile,
                                        _fpv.CameraIndex, () => _roundTrip);
        else if (!want && _strip != null) { Destroy(_strip.gameObject); _strip = null; }
    }

    // Hand cams, arm targets and base velocity exist in first person while their experiment is on.
    void UpdateFpvExperiments()
    {
        var model = FirstPerson && _fpv != null ? _fpv.Model : null;
        var profile = images != null ? images.Profile : null;
        Sync(ref _handCams, model != null && ExperimentSettings.IsOn(ExperimentSettings.HandCams),
             () => HandCamPip.Create(images, model, profile));
        Sync(ref _targets, model != null && ExperimentSettings.IsOn(ExperimentSettings.Targets),
             () => ArmTargets.Create(model));
        Sync(ref _baseVel, model != null && ExperimentSettings.IsOn(ExperimentSettings.BaseVel),
             () => BaseVelocity.Create(model, profile));
    }

    static void Sync<T>(ref T field, bool want, System.Func<T> create) where T : Component
    {
        if (want && field == null) field = create();
        else if (!want && field != null) { Destroy(field.gameObject); field = null; }
    }

    void OnDestroy()
    {
        ExperimentSettings.Changed -= OnExperimentChanged;
        if (images != null)
        {
            images.CamerasAdded -= RebuildBar;
            images.NamespaceChanged -= publisher.SetNamespace;
        }
    }

    // Shown in setup mode until camera topics have been found or the user continues without.
    void ShowWaiting()
    {
        float title = HudUi.TitleFontSize * HudUi.MmPerMetre;
        float body = HudUi.BodyFontSize * HudUi.MmPerMetre;
        const float w = 3300f, h = 760f, pad = 80f, buttonW = 980f, buttonH = 150f, gap = 50f;

        var root = HudUi.CreateCanvas("Setup Status", hudParent, new Vector3(0f, 0.2f, HudUi.ReferenceDistance),
            new Vector2(w, h), interactive: true);
        _waiting = root.gameObject;
        HudUi.Stretch(HudUi.Round(HudUi.Box(root, "Background", HudUi.PanelColor), 60f).rectTransform);

        var heading = HudUi.Label(root, "Heading", $"Setting up {images.Profile.displayName}", title);
        heading.fontStyle = FontStyles.Bold;
        HudUi.Place(heading.rectTransform, pad, pad, w - 2 * pad, title * 1.5f);

        _waitingStatus = HudUi.Label(root, "Status", "", body);
        _waitingStatus.color = HudUi.MutedText;
        HudUi.Place(_waitingStatus.rectTransform, pad, pad + title * 1.6f, w - 2 * pad, body * 3f);

        // Search again | Continue without cameras (e.g. a robot driven by Joy only) | Cancel
        float left = (w - 3f * buttonW - 2f * gap) / 2f, top = h - pad - buttonH;
        var again = HudUi.Button(root, "Search again", body, images.SearchAgain);
        HudUi.Place((RectTransform)again.transform, left, top, buttonW, buttonH);
        var without = HudUi.Button(root, "Continue without cameras", body, images.ContinueWithoutCameras);
        HudUi.Place((RectTransform)without.transform, left + buttonW + gap, top, buttonW, buttonH);
        var cancel = HudUi.Button(root, "Cancel", body, CancelSetup);
        HudUi.Place((RectTransform)cancel.transform, left + 2f * (buttonW + gap), top, buttonW, buttonH);
    }

    bool _menuWasPressed, _menuDeferred, _menuLongDone, _menuSuppressed, _simMenu;
    // Experiment "menurecenter": hold the menu input this long in first person to Recenter.
    const float LongPressSeconds = 1.5f;

    // How long the menu input has been held in the current press (0 when not pressed).
    public float MenuHeldSeconds { get; private set; }
    // Test hook: once SimulateMenu has been called, the menu input comes only from it.
    public bool SimulateMenuInput;

    public void SimulateMenu(bool pressed)
    {
        SimulateMenuInput = true;
        _simMenu = pressed;
    }

    void Update()
    {
        if (_waitingStatus != null) _waitingStatus.text = images.SetupStatus;

        // Menu shows/hides the HUD bar: the left controller's menu button, or with hand tracking
        // the hand menu gesture (left palm facing you + pinch). Back to robots is on the bar.
        // In first person with "menurecenter" on, a short press toggles on release and a hold of
        // LongPressSeconds recenters instead; otherwise the bar toggles on press.
        bool pressed = SimulateMenuInput ? _simMenu : ControllerMenuPressed() || HandMenuGesture();
        if (pressed)
        {
            if (!_menuWasPressed)
            {
                MenuHeldSeconds = 0f;
                _menuLongDone = _menuSuppressed = false;
                _menuDeferred = FirstPerson && ExperimentSettings.IsOn(ExperimentSettings.MenuRecenter);
                if (!_menuDeferred) ToggleBar();
            }
            else
                MenuHeldSeconds += Time.unscaledDeltaTime;

            if (PanelDragger.Dragging) _menuSuppressed = true;
            if (_menuDeferred && !_menuLongDone && !_menuSuppressed && MenuHeldSeconds >= LongPressSeconds)
            {
                _menuLongDone = true;
                Debug.Log("[TeleopHud] long-press menu -> recenter");
                Recenter();
            }
        }
        else if (_menuWasPressed)
        {
            if (_menuDeferred && !_menuLongDone && !_menuSuppressed && !PanelDragger.Dragging
                && MenuHeldSeconds < LongPressSeconds)
                ToggleBar();
            MenuHeldSeconds = 0f;
            _menuDeferred = false;
        }
        _menuWasPressed = pressed;
    }

    void ToggleBar()
    {
        if (_bar == null) return;
        _bar.gameObject.SetActive(!_bar.gameObject.activeSelf);
        PlaceBar();
        Debug.Log($"[TeleopHud] menu -> bar {(_bar.gameObject.activeSelf ? "shown" : "hidden")}");
    }

    // In first person the bar sits lower and smaller so it never covers the image centre; in
    // blocks mode it is at its normal pose. Relative to the bar's normal local pose.
    const float BarCompactScale = 0.7f, BarCompactDrop = 0.35f;
    Vector3 _barNormalPos, _barNormalScale;

    public bool BarCompact { get; private set; }

    void PlaceBar()
    {
        if (_bar == null) return;
        if (FirstPerson && !BarCompact)
        {
            _barNormalPos = _bar.localPosition;
            _barNormalScale = _bar.localScale;
            _bar.localScale = _barNormalScale * BarCompactScale;
            _bar.localPosition = _barNormalPos + Vector3.down * BarCompactDrop;
            BarCompact = true;
        }
        else if (!FirstPerson && BarCompact)
        {
            _bar.localPosition = _barNormalPos;
            _bar.localScale = _barNormalScale;
            BarCompact = false;
        }
    }

    static bool ControllerMenuPressed()
        => UnityEngine.XR.InputDevices.GetDeviceAtXRNode(UnityEngine.XR.XRNode.LeftHand)
               .TryGetFeatureValue(UnityEngine.XR.CommonUsages.menuButton, out bool pressed) && pressed;

    // --- Hand menu gesture: left palm facing the head + thumb and index pinched. ---
    // Read from the hand joints (XR Hands): Meta's aim "menu pressed" flag did not arrive on the
    // Quest 3S, so it is only kept as a second source.
    const float PinchDistance = 0.02f;     // thumb tip to index tip (m)
    const float PalmFacingDot = 0.5f;      // cos of max angle between palm normal and the head
    static XRHandSubsystem _hands;
    static readonly System.Collections.Generic.List<XRHandSubsystem> _found = new();

    public static XRHandSubsystem Hands
    {
        get
        {
            if (_hands != null && _hands.running) return _hands;
            SubsystemManager.GetSubsystems(_found);
            _hands = _found.Find(h => h.running);
            return _hands;
        }
    }

    public static bool HandsTracked => Hands != null && Hands.leftHand.isTracked;

    bool HandMenuGesture()
    {
        if (PanelDragger.Dragging) return false;
        if (MetaAimHand.left != null && ((ulong)MetaAimHand.left.aimFlags.ReadValue() & (ulong)MetaAimFlags.MenuPressed) != 0)
            return true;
        if (!HandsTracked || hudParent == null) return false;

        var hand = Hands.leftHand;
        if (!hand.GetJoint(XRHandJointID.Palm).TryGetPose(out Pose palm) ||
            !hand.GetJoint(XRHandJointID.ThumbTip).TryGetPose(out Pose thumb) ||
            !hand.GetJoint(XRHandJointID.IndexTip).TryGetPose(out Pose index))
            return false;
        if (Vector3.Distance(thumb.position, index.position) > PinchDistance) return false;

        // Joint poses are relative to the XR Origin's tracking space, i.e. the camera's parent.
        Vector3 head = hudParent.localPosition;
        Vector3 palmNormal = palm.rotation * Vector3.down;   // +Y points out of the back of the hand
        return Vector3.Dot(palmNormal, (head - palm.position).normalized) > PalmFacingDot;
    }

    // Setup mode: keep the discovered cameras, shown/hidden choices and layout as a new robot.
    public void SaveSetup()
    {
        RobotLibrary.Save(images.Profile);
        PlayerPrefs.SetString(RobotSelectionHud.LastRobotKey, images.Profile.name);
        PlayerPrefs.Save();
        RobotProfile.SetupMode = false;
        publisher.BackToRobotSelection();
    }

    // Setup mode: discard the robot being added.
    public void CancelSetup()
    {
        ImageSubscriber.ForgetLayout(images.Profile);
        RobotProfile.SetupMode = false;
        publisher.BackToRobotSelection();
    }

    // Off (default): the HUD is rigidly attached to the head. On: it follows the head with a
    // dead zone and a short delay (see HeadFollower). Panel positions relative to the head
    // are kept either way, so switching never moves a panel within the view.
    public void SetLazyFollow(bool on)
    {
        if (on == LazyFollow) return;
        LazyFollow = on;
        PlayerPrefs.SetInt(LazyFollowKey, on ? 1 : 0);
        PlayerPrefs.Save();

        if (on)
        {
            if (_follower == null) _follower = HeadFollower.Create(hudParent);
            _follower.Snap();
        }
        Transform parent = on ? _follower.transform : hudParent;

        foreach (var panel in images.Panels)
            panel.transform.SetParent(parent, false);
        _bar.SetParent(parent, false);
        images.panelParent = parent;
    }
}
