using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Head-locked bar under the camera blocks, the same for every robot:
///
///   +-----------------------------------------------------------------------------+
///   | SOBIT LIGHT  (JOY ON)                 ROS IP 192.168.11.20 (connected) [Edit]|
///   |-----------------------------------------------------------------------------|
///   | [x] Publish Joy | [x] Head Camera    [x] Hand Camera    | [ Reset layout ]   |
///   | [ ] Lazy follow | [x] Front Camera   [ ] Back Camera    | [ <- Robots    ]   |
///   | [ ] Passthrough |                                       | [ Recenter     ]   |
///   | [ ] First person|  (hint, first person only)            |                    |
///   +-----------------------------------------------------------------------------+
///
/// Passthrough swaps the dark studio background for the real room (see PassthroughMode);
/// the choice is saved and also applies to the robot selection screen.
///
/// While Joy is published the bar also gets an amber outline, so it is obvious at a
/// glance that the controllers are driving the robot.
/// </summary>
public class HudBar : MonoBehaviour
{
    // Sizes in mm on a canvas at HudUi.ReferenceDistance.
    const float WidthMm = 4200f, PaddingMm = 50f, GapMm = 40f, RadiusMm = 60f, OutlineMm = 22f;
    const float HeaderHeightMm = 220f, RowHeightMm = 150f;
    const float JoyColumnMm = 850f, ButtonColumnMm = 560f, EditWidthMm = 420f, PillWidthMm = 640f;
    const int CameraColumns = 2, LeftColumnRows = 4;
    // Space between the lowest camera block and the bar (metres). Blocks are turned to face
    // the eye, which brings their lower outer corners slightly down in view; this gap absorbs it.
    const float GapBelowCamerasM = 0.17f;

    const string JoyOnText = "CONTROL ON", JoyHoldText = "HOLD GRIP", JoyOffText = "LAYOUT MODE";

    ImageSubscriber _images;
    readonly System.Collections.Generic.List<(CameraPanel panel, Toggle toggle)> _cameraToggles = new();

    QuestControllerPublisher _publisher;
    TeleopHud _hud;
    Toggle _firstPersonToggle;
    Button _recenter;
    TextMeshProUGUI _fpHint;
    TextMeshProUGUI _ip, _pillText, _joyText;
    Image _pill, _joyChip, _outline;
    bool? _shownConnected, _shownJoy;
    bool _shownHold;

    public static HudBar Create(Transform parent, QuestControllerPublisher publisher, ImageSubscriber images, TeleopHud hud)
    {
        // Button column: Reset layout + Back, or in setup mode Reset layout + Save robot + Cancel.
        int buttonRows = images.InSetup ? 3 : 2;
        // The left column holds three toggles (Publish Joy, Lazy follow, Passthrough).
        // Added robots get "Find cameras" and "Joy namespace" buttons in the next free slots of
        // the camera toggles.
        int cameraSlots = images.Panels.Count + (images.Profile.isCustom ? 2 : 0);
        // Models: the first-person hint takes the row below the camera toggles; Recenter takes a
        // button slot (shown in first person only).
        bool hasModel = images.Profile.HasModel;
        if (hasModel) buttonRows = Mathf.Max(buttonRows, 3);
        int cameraRows = Mathf.CeilToInt(cameraSlots / (float)CameraColumns);
        int controlRows = Mathf.Max(LeftColumnRows, buttonRows, cameraRows + (hasModel ? 1 : 0));
        float controlsH = controlRows * RowHeightMm + (controlRows - 1) * GapMm;
        float heightMm = PaddingMm + HeaderHeightMm + GapMm + 4f + GapMm + controlsH + PaddingMm;

        // Top edge just below the lowest point the camera blocks may reach.
        float topY = images.minBottom - GapBelowCamerasM;
        var position = new Vector3(0f, topY - heightMm / HudUi.MmPerMetre / 2f, HudUi.ReferenceDistance);

        var root = HudUi.CreateCanvas("HUD Bar", parent, position, new Vector2(WidthMm, heightMm), interactive: true);
        var bar = root.gameObject.AddComponent<HudBar>();
        bar._publisher = publisher;
        bar._images = images;
        bar._hud = hud;
        hud.ViewModeChanged += bar.SyncViewMode;
        bar.Build(root, images, hud, heightMm);
        images.VisibilityReset += bar.SyncCameraToggles;
        images.LabelsChanged += bar.SyncCameraToggles;
        return bar;
    }

    void Build(RectTransform root, ImageSubscriber images, TeleopHud hud, float heightMm)
    {
        float title = HudUi.TitleFontSize * HudUi.MmPerMetre;
        float body  = HudUi.BodyFontSize  * HudUi.MmPerMetre;
        float inner = WidthMm - 2 * PaddingMm;

        HudUi.Stretch(HudUi.Round(HudUi.Box(root, "Background", HudUi.PanelColor), RadiusMm).rectTransform);
        // Ring just outside the bar; its thickness is radius x RingThicknessRatio.
        float ringRadius = OutlineMm / HudUi.RingThicknessRatio;
        _outline = HudUi.Ring(HudUi.Box(root, "Outline", Color.clear), ringRadius);
        HudUi.Stretch(_outline.rectTransform, -OutlineMm);

        // --- Header left: robot name and Joy state chip ---
        float top = PaddingMm;
        string robotName = images.Profile != null ? images.Profile.displayName : "";
        var name = HudUi.Label(root, "Robot", robotName, title, TextAlignmentOptions.Left);
        name.fontStyle = FontStyles.Bold;
        name.textWrappingMode = TextWrappingModes.NoWrap;
        float nameW = name.GetPreferredValues(robotName).x;
        HudUi.Place(name.rectTransform, PaddingMm, top, nameW, HeaderHeightMm);

        float chipH = body * 1.7f;
        _joyChip = HudUi.Round(HudUi.Box(root, "Joy State", Color.clear), chipH / 2f);
        _joyText = HudUi.Label(_joyChip.transform, "Label", "", body);
        _joyText.fontStyle = FontStyles.Bold;
        _joyText.textWrappingMode = TextWrappingModes.NoWrap;
        HudUi.Stretch(_joyText.rectTransform);
        float chipW = Mathf.Max(_joyText.GetPreferredValues(JoyOnText).x, _joyText.GetPreferredValues(JoyOffText).x,
                                images.InSetup ? _joyText.GetPreferredValues(SetupChipText(images)).x : 0f) + 2f * body;
        float chipLeft = PaddingMm + nameW + GapMm;
        HudUi.Place(_joyChip.rectTransform, chipLeft, top + (HeaderHeightMm - chipH) / 2f, chipW, chipH);

        // --- Header right: ROS IP, connection pill, Edit ---
        var edit = HudUi.Button(root, "Edit", body, _publisher.OpenIpKeyboard);
        float editLeft = WidthMm - PaddingMm - EditWidthMm;
        HudUi.Place((RectTransform)edit.transform, editLeft, top + 30f, EditWidthMm, HeaderHeightMm - 60f);

        float pillLeft = editLeft - GapMm - PillWidthMm;
        _pill = HudUi.Round(HudUi.Box(root, "Status", Color.clear), chipH / 2f);
        HudUi.Place(_pill.rectTransform, pillLeft, top + (HeaderHeightMm - chipH) / 2f, PillWidthMm, chipH);
        _pillText = HudUi.Label(_pill.transform, "Label", "", body);
        HudUi.Stretch(_pillText.rectTransform);

        var caption = HudUi.Label(root, "Caption", "ROS IP", body, TextAlignmentOptions.Left);
        caption.color = HudUi.MutedText;
        caption.textWrappingMode = TextWrappingModes.NoWrap;
        float captionW = caption.GetPreferredValues("ROS IP").x;
        float captionLeft = chipLeft + chipW + 3f * GapMm;
        HudUi.Place(caption.rectTransform, captionLeft, top, captionW, HeaderHeightMm);

        float ipLeft = captionLeft + captionW + GapMm;
        _ip = HudUi.Label(root, "IP", "", title, TextAlignmentOptions.Left);
        _ip.textWrappingMode = TextWrappingModes.NoWrap;
        _ip.overflowMode = TextOverflowModes.Ellipsis;
        HudUi.Place(_ip.rectTransform, ipLeft, top, pillLeft - GapMm - ipLeft, HeaderHeightMm);
        top += HeaderHeightMm + GapMm;

        var divider = HudUi.Box(root, "Divider", new Color(1f, 1f, 1f, 0.12f));
        HudUi.Place(divider.rectTransform, PaddingMm, top, inner, 4f);
        top += 4f + GapMm;

        // --- Controls: Publish Joy | camera toggles | Reset layout / Back to robots ---
        // Control robot: TF (head/controller poses) + Joy. Off = the robot receives nothing.
        var joy = HudUi.Toggle(root, "Control robot", body, _publisher.controlRobot, on => _publisher.controlRobot = on);
        joy.interactable = !images.InSetup;  // setup mode stays in layout mode
        HudUi.Place((RectTransform)joy.transform, PaddingMm, top, JoyColumnMm, RowHeightMm);
        var lazy = HudUi.Toggle(root, "Lazy follow", body, hud.LazyFollow, hud.SetLazyFollow);
        HudUi.Place((RectTransform)lazy.transform, PaddingMm, top + RowHeightMm + GapMm, JoyColumnMm, RowHeightMm);

        var passthrough = HudUi.Toggle(root, "Passthrough", body, PassthroughMode.Enabled, on =>
        {
            PassthroughMode.Enabled = on;
            PassthroughMode.Apply(Camera.main, on);
        });
        HudUi.Place((RectTransform)passthrough.transform, PaddingMm, top + 2f * (RowHeightMm + GapMm), JoyColumnMm, RowHeightMm);

        // First person: needs the robot's 3D model; without one the toggle is greyed out.
        bool hasModel = images.Profile.HasModel;
        _firstPersonToggle = HudUi.Toggle(root, hasModel ? "First person" : "First person (no model)", body,
            hud.FirstPerson, hud.SetFirstPerson);
        _firstPersonToggle.interactable = hasModel && !images.InSetup;
        HudUi.Place((RectTransform)_firstPersonToggle.transform, PaddingMm, top + 3f * (RowHeightMm + GapMm), JoyColumnMm, RowHeightMm);

        float camLeft = PaddingMm + JoyColumnMm + GapMm;
        float camWidth = inner - JoyColumnMm - GapMm - ButtonColumnMm - GapMm;
        float colW = (camWidth - (CameraColumns - 1) * GapMm) / CameraColumns;
        for (int i = 0; i < images.Panels.Count; i++)
        {
            var cam = images.Panels[i];
            var t = HudUi.Toggle(root, cam.Label, body, cam.Visible, on => images.SetCameraVisible(cam, on));
            _cameraToggles.Add((cam, t));
            HudUi.Place((RectTransform)t.transform,
                camLeft + (i % CameraColumns) * (colW + GapMm),
                top + (i / CameraColumns) * (RowHeightMm + GapMm),
                colW, RowHeightMm);
        }

        if (images.Profile.isCustom)
        {
            int i = images.Panels.Count;
            var find = HudUi.Button(root, FindCamerasText, body, null);
            var findLabel = find.GetComponentInChildren<TextMeshProUGUI>();
            find.onClick.AddListener(() =>
            {
                if (findLabel.text != FindCamerasText) return;   // a search is already running
                findLabel.text = "Searching\u2026";
                images.FindNewCameras(added =>
                {
                    // When cameras were added the bar is rebuilt (TeleopHud); otherwise say so briefly.
                    if (this == null || added > 0) return;
                    findLabel.text = "No new cameras";
                    StartCoroutine(ResetLabelLater(findLabel));
                });
            });
            HudUi.Place((RectTransform)find.transform,
                camLeft + (i % CameraColumns) * (colW + GapMm),
                top + (i / CameraColumns) * (RowHeightMm + GapMm),
                colW, RowHeightMm);

            // Joy namespace: typed on the Quest keyboard; confirming it empty means no namespace.
            i++;
            var ns = HudUi.Button(root, NamespaceButtonText(images.Profile), body, null);
            _namespaceLabel = ns.GetComponentInChildren<TextMeshProUGUI>();
            ns.onClick.AddListener(() => _namespaceKeyboard.Open(images.Profile.robotNamespace, allowEmpty: true));
            HudUi.Place((RectTransform)ns.transform,
                camLeft + (i % CameraColumns) * (colW + GapMm),
                top + (i / CameraColumns) * (RowHeightMm + GapMm),
                colW, RowHeightMm);
        }

        if (hasModel)
        {
            int cameraRows = Mathf.CeilToInt((images.Panels.Count + (images.Profile.isCustom ? 2 : 0)) / (float)CameraColumns);
            _fpHint = HudUi.Label(root, "First person hint", "You see the robot's arms twice (image + model) \u2014 expected.",
                                  body * 0.85f, TextAlignmentOptions.Left);
            _fpHint.color = HudUi.MutedText;
            HudUi.Place(_fpHint.rectTransform, camLeft, top + cameraRows * (RowHeightMm + GapMm), camWidth, RowHeightMm);
        }

        float buttonsLeft = WidthMm - PaddingMm - ButtonColumnMm;
        var reset = HudUi.Button(root, "Reset layout", body, images.ResetLayout);
        HudUi.Place((RectTransform)reset.transform, buttonsLeft, top, ButtonColumnMm, RowHeightMm);
        if (images.InSetup)
        {
            var save = HudUi.Button(root, "Save robot", body, hud.SaveSetup);
            var a = HudUi.AccentColor;
            save.GetComponent<Image>().color = new Color(a.r * 0.55f, a.g * 0.55f, a.b * 0.55f, 1f);
            HudUi.Place((RectTransform)save.transform, buttonsLeft, top + RowHeightMm + GapMm, ButtonColumnMm, RowHeightMm);
            var cancel = HudUi.Button(root, "Cancel", body, hud.CancelSetup);
            HudUi.Place((RectTransform)cancel.transform, buttonsLeft, top + 2f * (RowHeightMm + GapMm), ButtonColumnMm, RowHeightMm);
        }
        else
        {
            var back = HudUi.Button(root, "← Robots", body, _publisher.BackToRobotSelection);
            HudUi.Place((RectTransform)back.transform, buttonsLeft, top + RowHeightMm + GapMm, ButtonColumnMm, RowHeightMm);
        }
        if (hasModel)
        {
            _recenter = HudUi.Button(root, "Recenter", body, hud.Recenter);
            HudUi.Place((RectTransform)_recenter.transform, buttonsLeft, top + 2f * (RowHeightMm + GapMm), ButtonColumnMm, RowHeightMm);
        }
        SyncViewMode();
    }

    // The view mode changed (or a toggle press was refused): follow it. Recenter and the hint
    // only make sense in first person.
    void SyncViewMode()
    {
        if (_hud == null) return;
        if (_firstPersonToggle != null) _firstPersonToggle.SetIsOnWithoutNotify(_hud.FirstPerson);
        if (_recenter != null) _recenter.gameObject.SetActive(_hud.FirstPerson);
        if (_fpHint != null) _fpHint.gameObject.SetActive(_hud.FirstPerson);
    }

    const string FindCamerasText = "Find cameras";

    readonly TextKeyboard _namespaceKeyboard = new TextKeyboard();
    TextMeshProUGUI _namespaceLabel;

    static string NamespaceButtonText(RobotProfile robot)
        => string.IsNullOrEmpty(robot.robotNamespace) ? "Joy: /joy" : $"Joy: /{robot.robotNamespace}/joy";

    System.Collections.IEnumerator ResetLabelLater(TextMeshProUGUI label)
    {
        yield return new WaitForSeconds(3f);
        if (label != null) label.text = FindCamerasText;
    }

    // Change the added robot's Joy namespace ("" = none) and keep it.
    void SetNamespace(string ns)
    {
        var robot = _images.Profile;
        _publisher.SetNamespace(ns);
        robot.robotNamespace = _publisher.robotNamespace;
        if (!_images.InSetup) RobotLibrary.Save(robot);
        _namespaceLabel.text = NamespaceButtonText(robot);
        if (_images.InSetup) _joyText.text = SetupChipText(_images);
    }

    static string SetupChipText(ImageSubscriber images)
    {
        string ns = images.Profile.robotNamespace;
        return string.IsNullOrEmpty(ns) ? "SETUP · no namespace" : $"SETUP · /{ns}";
    }

    // Reset layout shows every camera again, and cameras can be renamed; keep the toggles in step
    // without re-triggering them.
    void SyncCameraToggles()
    {
        foreach (var (panel, toggle) in _cameraToggles)
        {
            toggle.SetIsOnWithoutNotify(panel.Visible);
            var label = toggle.GetComponentInChildren<TextMeshProUGUI>();
            if (label != null) label.text = panel.Label;
        }
    }

    void OnDestroy()
    {
        if (_hud != null) _hud.ViewModeChanged -= SyncViewMode;
        if (_images != null)
        {
            _images.VisibilityReset -= SyncCameraToggles;
            _images.LabelsChanged -= SyncCameraToggles;
        }
    }

    void Update()
    {
        _ip.text = _publisher.DisplayedIp;

        if (_namespaceLabel != null)
        {
            if (_namespaceKeyboard.IsOpen)
                _namespaceLabel.text = "Joy: /" + QuestControllerPublisher.TypingDisplay(_namespaceKeyboard.Text, "") + "/joy";
            string ns = _namespaceKeyboard.Poll();
            if (ns != null) SetNamespace(ns);
            else if (!_namespaceKeyboard.IsOpen && _namespaceLabel.text.Contains("|"))
                _namespaceLabel.text = NamespaceButtonText(_images.Profile);   // cancelled
        }

        bool connected = !_publisher.HasConnectionError;
        if (_shownConnected != connected)
        {
            _shownConnected = connected;
            var c = connected ? HudUi.GoodColor : HudUi.BadColor;
            _pill.color = new Color(c.r, c.g, c.b, 0.18f);
            _pillText.color = c;
            _pillText.text = connected ? "connected" : "not connected";
        }

        bool joy = _publisher.controlRobot;
        bool hold = joy && _publisher.deadmanEnabled && !_publisher.DeadmanHeld;
        if (_images.InSetup)
        {
            if (_shownJoy == null)
            {
                _shownJoy = false;
                var a = HudUi.AccentColor;
                _joyChip.color = new Color(a.r, a.g, a.b, 0.18f);
                _joyText.color = a;
                _joyText.text = SetupChipText(_images);
            }
        }
        else if (_shownJoy != joy || _shownHold != hold)
        {
            _shownJoy = joy;
            _shownHold = hold;
            var c = joy ? HudUi.WarnColor : HudUi.AccentColor;
            _joyChip.color = new Color(c.r, c.g, c.b, 0.18f);
            _joyText.color = c;
            _joyText.text = joy ? (hold ? JoyHoldText : JoyOnText) : JoyOffText;
            _outline.color = joy ? new Color(c.r, c.g, c.b, 0.6f) : Color.clear;
        }
    }
}
