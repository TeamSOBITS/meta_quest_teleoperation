using System;
using TMPro;
using UnityEngine;

/// <summary>
/// Sets up the robot screen around the camera blocks: studio surroundings, the HUD bar
/// (ROS IP + controls), the camera block dragger, controller hints and the optional
/// "lazy follow" mode. Runs after ImageSubscriber so it can use the blocks it created.
/// The view state (robot model, camera layout, first person, overlays) is in
/// <see cref="ViewController"/>, the menu input in <see cref="HudInput"/>; the public members
/// for them forward here.
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

    Transform _bar;
    RecordStatus _recordStatus;   // recorder status feed of this robot screen (display only)
    public RecordStatus RecordStatus => _recordStatus;
    RecordToast _recordToast;     // event notices (saved, discarded, ...) above the strip / bar
    public RecordToast RecordToast => _recordToast;
    HeadFollower _follower;
    GameObject _waiting;
    TextMeshProUGUI _waitingStatus;
    ViewController _view;
    bool _startControl;   // control was on at the start and the countdown has not been requested yet
    readonly HudInput _input = new HudInput();

    internal Transform Bar => _bar;

    // Control robot on (after the countdown) / off (at once), the same as pressing the bar's toggle.
    public void RequestControl(bool on) => _bar?.GetComponent<HudBar>()?.RequestControl(on);

    public bool LazyFollow { get; private set; }

    // --- View state (see ViewController) ---
    public bool RobotModelOn => _view.RobotModelOn;
    public string CameraLayout => _view.CameraLayout;
    public bool FirstPerson => _view.FirstPerson;
    public event Action ViewChanged { add => _view.ViewChanged += value; remove => _view.ViewChanged -= value; }
    public void SetRobotModel(bool on, bool save = true) => _view.SetRobotModel(on, save);
    public void SetCameraLayout(string layout, bool save = true) => _view.SetCameraLayout(layout, save);
    public void Recenter() => _view.Recenter();
    public StatusStrip Strip => _view.Strip;
    public bool ControllerVisualsHidden => _view.ControllerVisualsHidden;
    public int HiddenVisualCount => _view.HiddenVisualCount;
    public bool BarLowered => _view.BarLowered;

    void Awake()
    {
        _view = gameObject.AddComponent<ViewController>();
        _view.Init(this);
    }

    void Start()
    {
        StudioEnvironment.Apply(Camera.main);

        if (hudParent == null && Camera.main != null)
            hudParent = Camera.main.transform;

        images.NamespaceChanged += publisher.SetNamespace;   // Joy topic follows discovery
        // Control on at the start (saved state / default) does not publish at once: the bar runs the same 2 s countdown
        // as its toggle once it is built. Setup mode stays in layout mode: the trigger arranges blocks, nothing reaches a robot.
        _startControl = publisher.controlRobot && !images.InSetup;
        if (_startControl || images.InSetup)
            publisher.controlRobot = false;

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

        _bar = HudBar.Create(hudParent, publisher, images, this).transform;
        _view.CreateRoundTrip();
        _recordStatus = RecordStatus.Create(images.Profile);
        _recordToast = RecordToast.Create(this);
        images.CamerasAdded += RebuildBar;
        if (_startControl) { _startControl = false; RequestControl(true); }

        var dragger = gameObject.AddComponent<PanelDragger>();
        dragger.publisher = publisher;
        dragger.images = images;

        ControllerHints.Create(hudParent);

        if (Settings.LazyFollow)
            SetLazyFollow(true);

        var profile = images.Profile;
        if (profile != null && profile.HasModel) StudioEnvironment.EnsureModelKeyLight();
        string modeOverride = FirstPersonView.ViewModeOverride;
        FirstPersonView.ViewModeOverride = null;   // one robot screen only
        _view.ApplyInitial(modeOverride);

        if (Settings.DebugCapture)
            StartCoroutine(DebugCapture.Run());

        if (DebugLaunchOptions.RecStatusDemo)
        {
            DebugLaunchOptions.RecStatusDemo = false;   // this launch only
            RecordStatusDemo.Start(this, _recordStatus);
        }

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
        if (Strip != null && !FirstPerson) Strip.transform.SetParent(parent, false);
        images.panelParent = parent;
        hudParent = parent;
        if (_recordToast != null) _recordToast.Place();   // follows the strip / bar
    }

    // "Find cameras" added blocks: rebuild the bar so it lists their toggles too.
    void RebuildBar()
    {
        var parent = _bar.parent;
        bool counting = _bar.GetComponent<HudBar>().Countdown?.Counting == true;   // the new bar carries it on
        Destroy(_bar.gameObject);
        _bar = HudBar.Create(hudParent, publisher, images, this).transform;
        _bar.SetParent(parent, false);
        if (counting) RequestControl(true);
        _view.OnBarRebuilt();
    }

    void OnDestroy()
    {
        if (images != null)
        {
            images.CamerasAdded -= RebuildBar;
            images.NamespaceChanged -= publisher.SetNamespace;
        }
    }

    // Shown in setup mode until camera topics have been found or the user continues without.
    void ShowWaiting()
    {
        float title = HudTheme.TitleFont * HudUi.MmPerMetre;
        float body = HudTheme.BodyFont * HudUi.MmPerMetre;
        // The card's own sizes grow with the text size, like the text.
        float k = HudTheme.FontScale;
        float w = 3300f * k, h = 760f * k, pad = 80f * k, buttonW = 980f * k, buttonH = 150f * k, gap = 50f * k;

        var root = HudUi.CreateCanvas("Setup Status", hudParent, new Vector3(0f, 0.2f, HudTheme.ReferenceDistance),
            new Vector2(w, h), interactive: true);
        _waiting = root.gameObject;
        HudUi.Stretch(HudUi.Round(HudUi.Box(root, "Background", HudTheme.Panel), HudTheme.PanelRadius).rectTransform);

        var heading = HudUi.Label(root, "Heading", $"Setting up {images.Profile.displayName}", title);
        heading.fontStyle = FontStyles.Bold;
        HudUi.Place(heading.rectTransform, pad, pad, w - 2 * pad, title * 1.5f);

        _waitingStatus = HudUi.Label(root, "Status", "", body);
        _waitingStatus.color = HudTheme.Muted;
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

    void Update()
    {
        if (_waitingStatus != null) _waitingStatus.text = images.SetupStatus;
        _bar?.GetComponent<HudBar>()?.Countdown?.Tick();   // also while the bar is hidden

        // Menu shows/hides the HUD bar (see HudInput).
        if (_input.BarTogglePressed(hudParent) && _bar != null)
            ToggleBar();
    }

    void ToggleBar()
    {
        if (_bar == null) return;
        _bar.gameObject.SetActive(!_bar.gameObject.activeSelf);
        _view.OnBarToggled();
        DevLog.Log("[TeleopHud]", $"menu -> bar {(_bar.gameObject.activeSelf ? "shown" : "hidden")}");
    }

    // Setup mode: keep the discovered cameras, shown/hidden choices and layout as a new robot.
    public void SaveSetup()
    {
        RobotLibrary.Save(images.Profile);
        Settings.LastRobot = images.Profile.name;
        Settings.Save();
        RobotProfile.SetupMode = false;
        publisher.BackToRobotSelection();
    }

    // Setup mode: discard the robot being added.
    public void CancelSetup()
    {
        Settings.For(images.Profile).Forget();
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
        Settings.LazyFollow = on;
        Settings.Save();

        if (on)
        {
            if (_follower == null) _follower = HeadFollower.Create(hudParent);
            _follower.Snap();
        }
        Transform parent = on ? _follower.transform : hudParent;

        foreach (var panel in images.Panels)
            panel.transform.SetParent(parent, false);
        _bar.SetParent(parent, false);
        if (Strip != null && !FirstPerson) Strip.transform.SetParent(parent, false);
        images.panelParent = parent;
        if (_recordToast != null) _recordToast.Place();   // follows the strip / bar
    }
}
