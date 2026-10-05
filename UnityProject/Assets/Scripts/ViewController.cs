using System;
using UnityEngine;

/// <summary>
/// The robot screen's view state, owned by <see cref="TeleopHud"/> (which forwards its public
/// members): robot model on / off and camera layout ("blocks" or "firstperson"), both kept per robot;
/// the first-person view, the overlays (see Overlays/), the status strip, the lowered bar and the
/// hidden controller visuals that go with them.
///
/// Camera layout: "blocks" (the camera blocks stay as arranged) or "firstperson" (head camera image at
/// its true field of view and hand cards instead of the blocks). Without a robot model the layout
/// control is disabled and the blocks are shown.
/// </summary>
public class ViewController : MonoBehaviour
{
    TeleopHud _hud;
    ImageSubscriber Images => _hud.images;
    QuestControllerPublisher Publisher => _hud.publisher;
    Transform Bar => _hud.Bar;

    public bool RobotModelOn { get; private set; }
    public string CameraLayout { get; private set; } = FirstPersonView.LayoutBlocks;
    // The first-person layout is in effect (model on + layout "firstperson").
    public bool FirstPerson => RobotModelOn && CameraLayout == FirstPersonView.LayoutFirstPerson;
    // Raised when the model or the layout changes (the bar syncs its controls).
    public event Action ViewChanged;

    FirstPersonView _fpv;
    // Overlays: round-trip latency probe, status strip, hand cams, arm targets, base velocity.
    RoundTrip _roundTrip;
    StatusStrip _strip;
    HandCamPip _handCams;
    ArmTargets _targets;
    BaseVelocity _baseVel;
    readonly ControllerVisuals _visuals = new ControllerVisuals();
    bool _fpApplied;   // the first-person layout was in effect after the last ApplyView

    public void Init(TeleopHud hud) => _hud = hud;

    public bool ControllerVisualsHidden => _visuals.IsHidden;
    public int HiddenVisualCount => _visuals.Count;
    public StatusStrip Strip => _strip;

    public void CreateRoundTrip()
    {
        if (Publisher != null) _roundTrip = RoundTrip.Create(Publisher);
    }

    // Start of the robot screen: the saved model / layout of the robot, or the launch override
    // (session only, not saved).
    public void ApplyInitial(string modeOverride)
    {
        var profile = Images.Profile;
        if (profile != null && profile.HasModel)
        {
            Settings.EnsureMigrated(profile);
            var saved = Settings.For(profile);
            bool model = saved.ModelOn;
            string layout = saved.Layout;
            bool save = modeOverride == null;
            if (modeOverride == FirstPersonView.LayoutFirstPerson) { model = true; layout = FirstPersonView.LayoutFirstPerson; }
            else if (modeOverride == "model") { model = true; layout = FirstPersonView.LayoutBlocks; }
            else if (modeOverride == FirstPersonView.LayoutBlocks) { model = false; layout = FirstPersonView.LayoutBlocks; }
            CameraLayout = NormalizeLayout(layout);
            if (model) SetRobotModel(true, save);
        }
        UpdateStatusStrip();   // model off (the model path made its own above)
    }

    static string NormalizeLayout(string layout)
        => layout == FirstPersonView.LayoutFirstPerson ? FirstPersonView.LayoutFirstPerson : FirstPersonView.LayoutBlocks;

    // Robot model on / off (default off, kept per robot). Robots without a model stay off.
    public void SetRobotModel(bool on, bool save = true)
    {
        var profile = Images != null ? Images.Profile : null;
        if (on && (profile == null || !profile.HasModel))
        {
            DevLog.Log("FPV", "this robot has no model, staying in blocks view");
            ViewChanged?.Invoke();   // puts the toggle back
            return;
        }
        if (on == RobotModelOn) return;
        RobotModelOn = on;
        if (profile != null && save)
        {
            Settings.For(profile).ModelOn = on;
            Settings.Save();
        }
        ApplyView();
    }

    // Camera layout, "blocks" (default) or "firstperson" (kept per robot). It only takes effect while
    // the robot model is on; the choice is remembered meanwhile.
    public void SetCameraLayout(string layout, bool save = true)
    {
        layout = NormalizeLayout(layout);
        var profile = Images != null ? Images.Profile : null;
        if (layout == CameraLayout) return;
        CameraLayout = layout;
        if (profile != null && save && profile.HasModel)
        {
            Settings.For(profile).Layout = layout;
            Settings.Save();
        }
        ApplyView();
    }

    // Bring the scene in line with RobotModelOn / CameraLayout. Safe to repeat.
    void ApplyView()
    {
        var profile = Images != null ? Images.Profile : null;
        bool fp = FirstPerson;

        DestroyStrip();   // placement and model readings differ between layouts
        if (RobotModelOn && _fpv == null) _fpv = FirstPersonView.Create(Images, profile);
        else if (!RobotModelOn && _fpv != null)
        {
            Destroy(_fpv.gameObject);
            _fpv = null;
        }
        if (_fpv != null) _fpv.SetFirstPersonLayout(fp);
        SyncOverlays();
        Images.SetBlocksShown(!fp);

        if (fp != _fpApplied)   // only a layout change moves the bar; toggling the model must not hide the menu in use
        {
            _fpApplied = fp;
            if (Bar != null) Bar.gameObject.SetActive(!fp);   // menu / hand gesture brings it back
            PlaceBar();
        }
        UpdateStatusStrip();
        UpdateControllerVisuals();
        PushEditable(true);
        DevLog.Log("FPV", $"view -> model {(RobotModelOn ? "on" : "off")}, layout {(RobotModelOn ? CameraLayout : FirstPersonView.LayoutBlocks)}");
        ViewChanged?.Invoke();
    }

    // First person: put the model back under the headset.
    public void Recenter()
    {
        if (_fpv != null) _fpv.Recenter();
    }

    // --- Status strip: exists in both camera layouts and is active only while the bar (menu) is
    // hidden: it takes the bar's place. In the first-person layout it hangs under the head with the
    // model's head / lift readings; in the blocks layout at the bar's bottom row. ---

    public void UpdateStatusStrip()
    {
        bool want = Bar != null && (!FirstPerson || (_fpv != null && _fpv.Model != null));
        if (want && _strip == null)
        {
            var profile = Images.Profile;
            Func<RoundTrip> rtt = () => _roundTrip;
            Func<VlaStatus> vla = () => _hud.VlaStatus;   // VLA group while a recorder / deploy node publishes
            if (FirstPerson)
                _strip = StatusStrip.CreateInFirstPerson(FirstPersonView.Head, Publisher, Images, _fpv.Model, profile, _fpv.CameraIndex, rtt, vla);
            else
            {
                // Where the bar sits: same parent, distance and height as its bottom row.
                var pos = new Vector3(0f, Bar.localPosition.y - BarHeightM() / 2f + HudBar.BottomRowCentreM, Bar.localPosition.z);
                _strip = StatusStrip.Create(Bar.parent, pos, StatusStrip.BlocksScale, Publisher, Images, null, profile, BlocksStripCamera(), rtt, vla);
            }
        }
        else if (!want) DestroyStrip();
        SyncStripActive();
    }

    float BarHeightM() => ((RectTransform)Bar).sizeDelta.y * Bar.localScale.y;

    // Blocks mode: the head camera's rate if the robot has one, else the first camera that is shown.
    int BlocksStripCamera()
    {
        var profile = Images.Profile;
        var fpCamera = profile != null ? profile.FirstPersonCamera : null;
        int index = fpCamera != null ? Images.IndexOf(fpCamera.topicSuffix) : -1;
        for (int i = 0; index < 0 && i < Images.Panels.Count; i++)
            if (Images.IsOn(Images.Panels[i])) index = i;
        return index;
    }

    void DestroyStrip()
    {
        if (_strip != null) Destroy(_strip.gameObject);
        _strip = null;
    }

    // Active exactly while the bar is hidden.
    public void SyncStripActive()
    {
        if (_strip != null && Bar != null) _strip.gameObject.SetActive(!Bar.gameObject.activeSelf);
    }

    // --- Bar ---
    // The bar is always compact (HudBar.CompactScale, built that way); in first person it also
    // sits lower so it never covers the image centre. Relative to the bar's built local pose.
    // Numbers: head-camera quad at 1.5 m, vfov = 2 atan(240/462) -> 1.45 m tall, bottom edge at
    // -0.72 m = -25.8 deg. Bar top edge = ImageSubscriber.minBottom - 0.17 = -1.92 m at 4.3 m
    // (-24.0 deg, inside the image) minus the drop: 0.35 m -> -27.8 deg (2.0 deg below, the bare
    // minimum, and the quad's camera_info centre shift ate it), 0.50 m -> atan(2.42/4.3) = -29.4 deg
    // (3.6 deg below, the panel top still touched the image bottom), 0.55 m -> atan(2.47/4.3) = -29.9 deg
    // (4.1 deg below the image bottom).
    const float BarLoweredDrop = 0.55f;
    Vector3 _barNormalPos;

    public bool BarLowered { get; private set; }

    void PlaceBar()
    {
        if (Bar == null) return;
        if (FirstPerson && !BarLowered)
        {
            _barNormalPos = Bar.localPosition;
            Bar.localPosition = _barNormalPos + Vector3.down * BarLoweredDrop;
            BarLowered = true;
        }
        else if (!FirstPerson && BarLowered)
        {
            Bar.localPosition = _barNormalPos;
            BarLowered = false;
        }
    }

    // The bar was shown or hidden.
    public void OnBarToggled()
    {
        PlaceBar();
        SyncStripActive();
        UpdateControllerVisuals();
    }

    // "Find cameras" replaced the bar (a new one, at its normal pose).
    public void OnBarRebuilt()
    {
        BarLowered = false;
        PlaceBar();
        DestroyStrip();   // blocks mode: it sits where the (possibly taller) bar's bottom row is
        UpdateStatusStrip();
        UpdateControllerVisuals();
    }

    void UpdateControllerVisuals()
        => _visuals.SetHidden(FirstPerson && Bar != null && !Bar.gameObject.activeSelf);

    // Arm targets and base velocity exist while the robot model is on, hand cams in the first-person layout.
    void SyncOverlays()
    {
        var model = _fpv != null ? _fpv.Model : null;
        var profile = Images != null ? Images.Profile : null;
        Sync(ref _handCams, model != null && FirstPerson, () => HandCamPip.Create(Images, model, profile));
        Sync(ref _targets, model != null, () => ArmTargets.Create(model, profile));
        Sync(ref _baseVel, model != null, () => BaseVelocity.Create(model, profile));
    }

    // Layout mode (controls off): the first-person cards get their Rename buttons, as the blocks do.
    bool? _editable;

    void PushEditable(bool force)
    {
        bool editable = Publisher != null && !Publisher.controlRobot;
        if (!force && _editable == editable) return;
        _editable = editable;
        if (_fpv != null) _fpv.SetEditable(editable);
        if (_handCams != null) _handCams.SetEditable(editable);
    }

    static void Sync<T>(ref T field, bool want, Func<T> create) where T : Component
    {
        if (want && field == null) field = create();
        else if (!want && field != null) { Destroy(field.gameObject); field = null; }
    }

    void Update() => PushEditable(false);

    void LateUpdate() => _visuals.Tick();

    void OnDestroy() => _visuals.SetHidden(false);
}
