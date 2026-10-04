using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Head-locked bar under the camera blocks, the same for every robot:
///
///   +-----------------------------------------------------------------------------+
///   | SOBIT LIGHT (JOY ON)  ROS IP 192.168.11.20 (REC 00:01:05) (connected) [Edit]|
///   |-----------------------------------------------------------------------------|
///   | [x] Control robot [ ] Lazy follow | [x] Head Camera   [x] Hand Camera | [Reset layout]|
///   | [ ] Passthrough [x] Compressed img| [x] Front Camera  [ ] Back Camera | [<- Robots   ]|
///   | [ ] Robot model                   |                                   | [Recenter    ]|
///   | Camera layout [Blocks][First person]| (hint, first-person layout only) |               |
///   +-----------------------------------------------------------------------------+
///
/// Robot model shows the robot's 3D model around you (greyed "(no model)" for robots without one).
/// Camera layout (only with the model on): Blocks keeps the camera blocks, First person shows the
/// head camera at its true field of view and the hand cameras at the hands. Compressed images off
/// subscribes the raw image topics instead; it reopens the robot screen (subscriptions cannot be
/// dropped), so it is applied at once.
///
/// Passthrough swaps the dark studio background for the real room (see PassthroughMode);
/// the choice is saved and also applies to the robot selection screen.
///
/// The REC pill shows only while a recorder publishes its status (HudBar.Record.cs).
///
/// While Joy is published the bar also gets an amber outline, so it is obvious at a
/// glance that the controllers are driving the robot.
/// </summary>
public partial class HudBar : MonoBehaviour
{
    // Sizes in mm on a canvas at HudTheme.ReferenceDistance.
    const float WidthMm = 5000f, PaddingMm = HudTheme.Padding, GapMm = HudTheme.Gap, OutlineMm = 22f;
    const float HeaderHeightMm = 220f, RowHeightMm = 150f, DividerMm = 4f, EditInsetMm = 30f;
    const float JoyColumnMm = 2000f, ButtonColumnMm = 560f, PillWidthMm = 640f, LayoutLabelMm = 640f;
    const float JoyOutlineAlpha = 0.6f, SelectedSegmentAlpha = 0.25f, SaveTint = 0.55f;
    const int CameraColumns = 2, LeftColumnRows = 4;
    // Space between the lowest camera block and the bar (metres). Blocks are turned to face
    // the eye, which brings their lower outer corners slightly down in view; this gap absorbs it.
    const float GapBelowCamerasM = 0.17f;
    // The bar is always built at this scale (3.5 m wide instead of 5 m); it hangs from the
    // same top edge, so the space below the camera blocks that it frees is given to them
    // (ImageSubscriber.minBottom is lowered by the same amount).
    public const float CompactScale = 0.7f;
    // Width of the built bar and the centre of its bottom control row above its bottom edge (metres,
    // after CompactScale): the status strip takes this spot while the bar is hidden in blocks mode.
    public const float CompactWidthMm = WidthMm * CompactScale;
    public static float BottomRowCentreM => (PaddingMm + RowHeightMm / 2f) * BuiltScale / HudUi.MmPerMetre;
    // The bar is built for the design font sizes and scaled as a whole by the text size, so a larger text
    // size grows the bar (and its columns) with it instead of wrapping labels.
    static float BuiltScale => CompactScale * HudTheme.FontScale;

    const string JoyOnText = "CONTROL ON", JoyOffText = "LAYOUT MODE";

    ImageSubscriber _images;
    readonly System.Collections.Generic.List<(CameraPanel panel, Toggle toggle)> _cameraToggles = new();

    QuestControllerPublisher _publisher;
    TeleopHud _hud;
    Toggle _modelToggle;
    Button _blocksButton, _firstPersonButton, _recenter;
    TextMeshProUGUI _fpHint;
    TextMeshProUGUI _ip, _pillText, _joyText;
    Image _pill, _joyChip, _outline;
    bool? _shownConnected, _shownJoy;
    ControlCountdown _countdown;

    // Control on / off from any path: on goes through the countdown, off applies at once.
    public void RequestControl(bool on) => _countdown?.Request(on);
    public ControlCountdown Countdown => _countdown;

    public static HudBar Create(Transform parent, QuestControllerPublisher publisher, ImageSubscriber images, TeleopHud hud)
    {
        // Button column: Reset layout + Back, or in setup mode Reset layout + Save robot + Cancel.
        int buttonRows = images.InSetup ? 3 : 2;
        // The left column holds two toggles per row (Control robot, Lazy follow, Passthrough, Compressed
        // images, Robot model) and the Camera layout row.
        // Added robots get "Find cameras" and "Joy namespace" buttons in the next free slots of
        // the camera toggles.
        int cameraSlots = CameraSlots(images);
        // Models: the first-person hint takes the row below the camera toggles; Recenter takes a
        // button slot (shown while the model is on).
        bool hasModel = images.Profile.HasModel;
        if (hasModel) buttonRows = Mathf.Max(buttonRows, 3);
        int cameraRows = CameraRows(images);
        int controlRows = Mathf.Max(LeftColumnRows, buttonRows, cameraRows + (hasModel ? 1 : 0));
        float controlsH = controlRows * RowHeightMm + (controlRows - 1) * GapMm;
        float heightMm = PaddingMm + HeaderHeightMm + GapMm + DividerMm + GapMm + controlsH + PaddingMm;

        // Top edge just below the lowest point the camera blocks may reach.
        float topY = images.minBottom - GapBelowCamerasM;
        var position = new Vector3(0f, topY - heightMm * BuiltScale / HudUi.MmPerMetre / 2f, HudTheme.ReferenceDistance);

        var root = HudUi.CreateCanvas("HUD Bar", parent, position, new Vector2(WidthMm, heightMm), interactive: true);
        root.localScale *= BuiltScale;
        var bar = root.gameObject.AddComponent<HudBar>();
        bar._publisher = publisher;
        bar._images = images;
        bar._hud = hud;
        hud.ViewChanged += bar.SyncView;
        bar.Build(root, images, hud, heightMm);
        images.VisibilityReset += bar.SyncCameraToggles;
        images.LabelsChanged += bar.SyncCameraToggles;
        return bar;
    }

    // Camera toggles, plus "Find cameras" and "Joy namespace" for added robots, fill the camera columns row by row.
    static int CameraSlots(ImageSubscriber images) => images.Panels.Count + (images.Profile.isCustom ? 2 : 0);
    static int CameraRows(ImageSubscriber images) => Mathf.CeilToInt(CameraSlots(images) / (float)CameraColumns);

    // Columns of the control area, in mm from the bar's top-left corner; `Top` is where its first row starts.
    struct Cursor
    {
        public float Top;
        public float Half, RightX;                 // the left column holds two toggles per row
        public float CameraLeft, CameraWidth, CameraColW;
        public float ButtonsLeft;

        public static Cursor At(float top)
        {
            float inner = WidthMm - 2f * PaddingMm;
            var c = new Cursor { Top = top };
            c.Half = (JoyColumnMm - GapMm) / 2f;
            c.RightX = PaddingMm + c.Half + GapMm;
            c.CameraLeft = PaddingMm + JoyColumnMm + GapMm;
            c.CameraWidth = inner - JoyColumnMm - GapMm - ButtonColumnMm - GapMm;
            c.CameraColW = (c.CameraWidth - (CameraColumns - 1) * GapMm) / CameraColumns;
            c.ButtonsLeft = WidthMm - PaddingMm - ButtonColumnMm;
            return c;
        }

        public float Row(int i) => Top + i * (RowHeightMm + GapMm);
        // Slot i of the camera columns (filled left to right, then down).
        public void CameraSlot(int i, out float left, out float top)
        {
            left = CameraLeft + (i % CameraColumns) * (CameraColW + GapMm);
            top = Row(i / CameraColumns);
        }
    }

    void Build(RectTransform root, ImageSubscriber images, TeleopHud hud, float heightMm)
    {
        HudUi.Stretch(HudUi.Round(HudUi.Box(root, "Background", HudTheme.Panel), HudTheme.PanelRadius).rectTransform);
        // Ring just outside the bar; its thickness is radius x RingThicknessRatio.
        _outline = HudUi.Ring(HudUi.Box(root, "Outline", Color.clear), OutlineMm / HudUi.RingThicknessRatio);
        HudUi.Stretch(_outline.rectTransform, -OutlineMm);

        var cursor = Cursor.At(BuildHeader(root, images));
        BuildControls(root, cursor, images, hud);
        BuildCameraToggles(root, cursor, images);
        BuildActions(root, cursor, images, hud);
        SyncView();
    }

    // Robot name and Joy state chip on the left; ROS IP, connection pill and Edit on the right; then the divider.
    // Returns the top of the control rows.
    float BuildHeader(RectTransform root, ImageSubscriber images)
    {
        float title = HudTheme.TitleFontBase * HudUi.MmPerMetre;
        float body  = HudTheme.BodyFontBase  * HudUi.MmPerMetre;
        float top = PaddingMm;

        string robotName = images.Profile != null ? images.Profile.displayName : "";
        var name = HudUi.Label(root, "Robot", robotName, title, TextAlignmentOptions.Left);
        name.fontStyle = FontStyles.Bold;
        name.textWrappingMode = TextWrappingModes.NoWrap;
        float nameW = name.GetPreferredValues(robotName).x;
        HudUi.Place(name.rectTransform, PaddingMm, top, nameW, HeaderHeightMm);

        float chipH = body * 1.7f;
        (_joyChip, _joyText) = HudUi.Pill(root, "Joy State", "", body, chipH, Color.clear, Color.white, bold: true);
        float chipW = Mathf.Max(_joyText.GetPreferredValues(JoyOnText).x, _joyText.GetPreferredValues(JoyOffText).x,
                                images.InSetup ? _joyText.GetPreferredValues(SetupChipText(images)).x : 0f) + 2f * body;
        float chipLeft = PaddingMm + nameW + GapMm;
        HudUi.Place(_joyChip.rectTransform, chipLeft, top + (HeaderHeightMm - chipH) / 2f, chipW, chipH);

        // ROS IP, connection pill, Edit
        var edit = HudUi.Button(root, "Edit", body, _publisher.OpenIpKeyboard);
        float editLeft = WidthMm - PaddingMm - HudTheme.EditButtonWidth;
        HudUi.Place((RectTransform)edit.transform, editLeft, top + EditInsetMm, HudTheme.EditButtonWidth, HeaderHeightMm - 2f * EditInsetMm);

        float pillLeft = editLeft - GapMm - PillWidthMm;
        (_pill, _pillText) = HudUi.Pill(root, "Status", "", body, chipH, Color.clear, Color.white);
        HudUi.Place(_pill.rectTransform, pillLeft, top + (HeaderHeightMm - chipH) / 2f, PillWidthMm, chipH);

        var caption = HudUi.Label(root, "Caption", "ROS IP", body, TextAlignmentOptions.Left);
        caption.color = HudTheme.Muted;
        caption.textWrappingMode = TextWrappingModes.NoWrap;
        float captionW = caption.GetPreferredValues("ROS IP").x;
        float captionLeft = chipLeft + chipW + 3f * GapMm;
        HudUi.Place(caption.rectTransform, captionLeft, top, captionW, HeaderHeightMm);

        float ipLeft = captionLeft + captionW + GapMm;
        _ip = HudUi.Label(root, "IP", "", title, TextAlignmentOptions.Left);
        _ip.textWrappingMode = TextWrappingModes.NoWrap;
        _ip.overflowMode = TextOverflowModes.Ellipsis;
        HudUi.Place(_ip.rectTransform, ipLeft, top, pillLeft - GapMm - ipLeft, HeaderHeightMm);
        BuildRecordPill(root, top + (HeaderHeightMm - chipH) / 2f, chipH, pillLeft - GapMm, pillLeft - GapMm - ipLeft);
        top += HeaderHeightMm + GapMm;

        var divider = HudUi.Box(root, "Divider", HudTheme.Divider);
        HudUi.Place(divider.rectTransform, PaddingMm, top, WidthMm - 2f * PaddingMm, DividerMm);
        return top + DividerMm + GapMm;
    }

    // Left column: two toggles per row (Control robot, Lazy follow, Passthrough, Compressed, Robot model) and the
    // Camera layout row.
    void BuildControls(RectTransform root, Cursor at, ImageSubscriber images, TeleopHud hud)
    {
        float body = HudTheme.BodyFontBase * HudUi.MmPerMetre;

        // Control robot: TF (head/controller poses) + Joy. Off = the robot receives nothing.
        // Turning it on starts a 2 s countdown (see ControlCountdown); off is immediate.
        var joy = HudUi.Toggle(root, "Control robot", body, _publisher.controlRobot, RequestControl);
        _countdown = new ControlCountdown(_publisher, joy);
        joy.interactable = !images.InSetup;  // setup mode stays in layout mode
        HudUi.Place((RectTransform)joy.transform, PaddingMm, at.Row(0), at.Half, RowHeightMm);
        var lazy = HudUi.Toggle(root, "Lazy follow", body, hud.LazyFollow, hud.SetLazyFollow);
        HudUi.Place((RectTransform)lazy.transform, at.RightX, at.Row(0), at.Half, RowHeightMm);

        var passthrough = HudUi.Toggle(root, "Passthrough", body, PassthroughMode.Enabled, on =>
        {
            PassthroughMode.Enabled = on;
            PassthroughMode.Apply(Camera.main, on);
        });
        HudUi.Place((RectTransform)passthrough.transform, PaddingMm, at.Row(1), at.Half, RowHeightMm);

        // Compressed images (global, default on). Off = raw image topics. Topics cannot be unsubscribed,
        // so a change reopens the robot screen through the selection screen (RobotSelectionHud.AutoOpen).
        var compressed = HudUi.Toggle(root, "Compressed", body, ImageSubscriber.Compressed, on =>
        {
            if (on == ImageSubscriber.Compressed) return;
            Settings.Compressed = on;
            Settings.Save();
            RobotSelectionHud.AutoOpen = images.Profile;
            _publisher.BackToRobotSelection();
        });
        compressed.interactable = !images.InSetup;   // reopening would drop the robot being set up
        HudUi.Place((RectTransform)compressed.transform, at.RightX, at.Row(1), at.Half, RowHeightMm);

        // Robot model: needs the robot's 3D model; without one the toggle is greyed out.
        _modelToggle = HudUi.Toggle(root, "Robot model", body, hud.RobotModelOn, on => hud.SetRobotModel(on));
        _modelToggle.interactable = images.Profile.HasModel && !images.InSetup;
        HudUi.Place((RectTransform)_modelToggle.transform, PaddingMm, at.Row(2), at.Half, RowHeightMm);

        // Camera layout: segmented Blocks | First person, only usable while the model is on.
        float segFont = body * 0.9f, segW = (JoyColumnMm - LayoutLabelMm - 2f * GapMm) / 2f;
        var layoutLabel = HudUi.Label(root, "Camera layout", "Camera layout", body, TextAlignmentOptions.Left);
        layoutLabel.textWrappingMode = TextWrappingModes.NoWrap;
        HudUi.Place(layoutLabel.rectTransform, PaddingMm, at.Row(3), LayoutLabelMm, RowHeightMm);
        _blocksButton = HudUi.Button(root, "Blocks", segFont, () => hud.SetCameraLayout(FirstPersonView.LayoutBlocks));
        HudUi.Place((RectTransform)_blocksButton.transform, PaddingMm + LayoutLabelMm + GapMm, at.Row(3), segW, RowHeightMm);
        _firstPersonButton = HudUi.Button(root, "First person", segFont, () => hud.SetCameraLayout(FirstPersonView.LayoutFirstPerson));
        HudUi.Place((RectTransform)_firstPersonButton.transform, PaddingMm + LayoutLabelMm + 2f * GapMm + segW, at.Row(3), segW, RowHeightMm);
    }

    // The model or the layout changed (or a toggle press was refused): follow it. The layout buttons
    // work with the model on only (off: Blocks shown as selected, greyed); Recenter needs the model;
    // the hint is about the first-person layout.
    void SyncView()
    {
        if (_hud == null) return;
        bool model = _hud.RobotModelOn;
        string layout = model ? _hud.CameraLayout : FirstPersonView.LayoutBlocks;
        if (_modelToggle != null) _modelToggle.SetIsOnWithoutNotify(model);
        Segment(_blocksButton, layout == FirstPersonView.LayoutBlocks, model);
        Segment(_firstPersonButton, layout == FirstPersonView.LayoutFirstPerson, model);
        if (_recenter != null) _recenter.gameObject.SetActive(model);
        if (_fpHint != null) _fpHint.gameObject.SetActive(_hud.FirstPerson);
    }

    // Segmented control look: the selected button is tinted with the accent colour (25 %), text white.
    static void Segment(Button button, bool selected, bool interactable)
    {
        if (button == null) return;
        button.interactable = interactable;
        button.GetComponent<Image>().color = selected ? HudTheme.WithAlpha(HudTheme.Accent, SelectedSegmentAlpha) : HudTheme.Control;
        var text = button.GetComponentInChildren<TextMeshProUGUI>();
        if (text != null) text.color = Color.white;
    }

    // Reset layout shows every camera again, and cameras can be renamed; keep the toggles in step
    // without re-triggering them.
    void SyncCameraToggles()
    {
        foreach (var (panel, toggle) in _cameraToggles)
        {
            toggle.SetIsOnWithoutNotify(_images.IsOn(panel));
            var label = toggle.GetComponentInChildren<TextMeshProUGUI>();
            if (label != null) label.text = panel.Label;
        }
    }

    void OnDestroy()
    {
        if (_hud != null) _hud.ViewChanged -= SyncView;
        if (_images != null)
        {
            _images.VisibilityReset -= SyncCameraToggles;
            _images.LabelsChanged -= SyncCameraToggles;
        }
    }

    void Update()
    {
        _ip.text = _publisher.DisplayedIp;
        UpdateRecordPill();

        if (_namespaceLabel != null)
        {
            if (_namespaceKeyboard.IsOpen)
                _namespaceLabel.text = "Joy: /" + QuestControllerPublisher.TypingDisplay(_namespaceKeyboard.Text, "") + "/" + RosNames.Joy;
            string ns = _namespaceKeyboard.Poll();
            if (ns != null) SetNamespace(ns);
            else if (!_namespaceKeyboard.IsOpen && _namespaceLabel.text.Contains("|"))
                _namespaceLabel.text = NamespaceButtonText(_images.Profile);   // cancelled
        }

        bool connected = !_publisher.HasConnectionError;
        if (_shownConnected != connected)
        {
            _shownConnected = connected;
            var c = connected ? HudTheme.Good : HudTheme.Bad;
            _pill.color = HudTheme.PillBackground(c);
            _pillText.color = c;
            _pillText.text = connected ? "connected" : "not connected";
        }

        bool joy = _publisher.controlRobot;
        if (_images.InSetup)
        {
            if (_shownJoy == null)
            {
                _shownJoy = false;
                _joyChip.color = HudTheme.PillBackground(HudTheme.Accent);
                _joyText.color = HudTheme.Accent;
                _joyText.text = SetupChipText(_images);
            }
        }
        else if (_shownJoy != joy)
        {
            _shownJoy = joy;
            var c = joy ? HudTheme.Warn : HudTheme.Accent;
            _joyChip.color = HudTheme.PillBackground(c);
            _joyText.color = c;
            _joyText.text = joy ? JoyOnText : JoyOffText;
            _outline.color = joy ? HudTheme.WithAlpha(c, JoyOutlineAlpha) : Color.clear;
        }
    }
}
