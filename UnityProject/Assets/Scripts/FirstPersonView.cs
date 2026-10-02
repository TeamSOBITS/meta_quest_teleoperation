using System.Collections.Generic;
using System.Text;
using RosMessageTypes.Sensor;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR;

/// <summary>
/// First-person view: the headset is the robot's head camera. Shows a life-size
/// <see cref="RobotModel"/> around the user and the head camera's image, drawn at the camera's true
/// field of view, on a quad in front of the camera frame. Created by TeleopHud when the mode is on
/// and destroyed when it is off.
///
/// Placement. The model hangs under the XR Origin (not the camera offset): the origin is the
/// transform whose space is the tracking space, so it moves with the playspace and its floor is
/// y = 0. On <see cref="Recenter"/> (enable, first TF, tracking origin reset, the Recenter button)
/// the model is turned about the vertical axis so the camera frame faces where the headset looks,
/// and moved so the camera frame sits at the headset position. Its pan-axis point (panFrame origin;
/// for robots without a pan/tilt head, i.e. panFrame empty or absent from the model, the camera frame
/// itself) is then remembered, and every frame the model is shifted so that point stays where it was:
/// when the lift or head moves, the model moves under the user instead of the image moving off
/// the eyes. Rotation is never touched after Recenter, so a turning robot head turns the image.
/// </summary>
[DefaultExecutionOrder(200)]
public class FirstPersonView : MonoBehaviour
{
    public const string ModeBlocks = "blocks", ModeFirstPerson = "firstperson";
    // Distance of the image quad from the camera frame (metres).
    const float QuadDistance = 1.5f;
    const float NearClip = 0.1f;
    // The camera blocks' card (CameraPanel: padding 40 mm, radius 60 mm, outline margin 30 mm, label gap 0.05 m)
    // shrinks with the distance ratio quad / block distance.
    const float CardPaddingMm = 40f, CardRadiusMm = 60f, CardOutlineMm = 30f;
    const float CardScale = QuadDistance / HudUi.ReferenceDistance;
    const float LogSeconds = 5f;
    static readonly Color WaitingColor = new Color(0.15f, 0.15f, 0.15f, 1f);

    // Set by the launch intent (autonomous tests): overrides the mode for the next robot screen
    // only, is never saved. TeleopHud.BuildHud consumes and clears it.
    public static string ViewModeOverride;
    // Replaces the headset pose for Recenter (the demo recorder films from a fixed, level head).
    public static Transform HeadOverride;
    public static Transform Head => HeadOverride != null ? HeadOverride : Camera.main != null ? Camera.main.transform : null;

    public static string ViewModeKey(RobotProfile r) => ViewModeKey(r.name);
    public static string ViewModeKey(string robotName) => $"ViewMode/{robotName}";

    ImageSubscriber _images;
    RobotProfile _profile;
    RobotModel _model;
    Transform _camFrame, _panFrame;
    Vector3 _anchorPoint;
    bool _anchored;

    float _prevNearClip = -1f;
    readonly List<XRInputSubsystem> _inputs = new List<XRInputSubsystem>();

    // Image quad
    RectTransform _canvasRt, _viewRt;
    RawImage _view;
    TextMeshProUGUI _waiting, _name;
    CameraBadge _badge;
    RectTransform _cardRt, _outlineRt;
    int _cameraIndex = -1;
    int _frames;
    float _nextLog;
    // Intrinsics (pixels); until camera_info arrives the field of view is the profile default.
    bool _hasInfo;
    double _fx, _fy, _cx, _cy, _infoW, _infoH;
    float _textureAspect = 4f / 3f;

    // Experiment "headlock": the image hangs under the headset instead of the robot's camera frame.
    public bool HeadLocked { get; private set; }

    public RobotModel Model => _model;
    public int FramesReceived => _frames;
    // Index of the head camera in ImageSubscriber (-1 if the robot has none).
    public int CameraIndex => _cameraIndex;
    // The bar's head-camera toggle: the card (image, name, "Waiting" label, fps badge) shows only while it is on.
    public bool CameraOn { get; private set; } = true;
    public bool CardVisible => _canvasRt != null && _canvasRt.gameObject.activeSelf;
    public CameraBadge Badge => _badge;

    public static FirstPersonView Create(ImageSubscriber images, RobotProfile profile)
    {
        var origin = FindFirstObjectByType<XROrigin>();
        Transform parent = origin != null ? origin.transform
            : Camera.main != null && Camera.main.transform.parent != null ? Camera.main.transform.parent : null;
        var go = new GameObject("First Person View");
        go.transform.SetParent(parent, false);
        var view = go.AddComponent<FirstPersonView>();
        view.Init(images, profile);
        return view;
    }

    void Init(ImageSubscriber images, RobotProfile profile)
    {
        _images = images;
        _profile = profile;

        _model = RobotModel.Create(profile, transform);
        if (_model == null)
        {
            Debug.LogWarning("FPV: no model, first-person view is empty");
            return;
        }
        _camFrame = _model.Frame(profile.cameraFrame);
        _panFrame = _model.Frame(profile.panFrame);
        if (_camFrame == null) Debug.LogWarning($"FPV: camera frame '{profile.cameraFrame}' not in the model; the image quad hangs on the model root");
        if (_panFrame == null && _camFrame != null)
        {
            // Robots without a pan/tilt head: keep the eyes at the camera instead.
            _panFrame = _camFrame;
            Debug.Log("FPV: no pan frame, anchoring at the camera frame");
        }

        _model.SetLinksVisible(profile.firstPersonHiddenLinkPrefixes, false);

        var cam = Camera.main;
        if (cam != null)
        {
            _prevNearClip = cam.nearClipPlane;
            cam.nearClipPlane = NearClip;
        }

        BuildQuad();
        ExperimentSettings.Changed += OnExperimentChanged;
        ApplyHeadLock();
        UseTexture();
        SubscribeCameraInfo();

        SubsystemManager.GetSubsystems(_inputs);
        foreach (var s in _inputs) s.trackingOriginUpdated += OnTrackingOriginUpdated;
        _model.Updated += OnFirstTf;

        Recenter();
        LogState("enable");
        _nextLog = Time.unscaledTime + LogSeconds;
    }

    void OnDestroy()
    {
        ExperimentSettings.Changed -= OnExperimentChanged;
        foreach (var s in _inputs)
            if (s != null) s.trackingOriginUpdated -= OnTrackingOriginUpdated;
        if (_images != null)
        {
            _images.FrameReady -= OnFrame;
            _images.CameraVisibilityChanged -= OnCameraVisibility;
            if (_cameraIndex >= 0) _images.ForceDecode(_cameraIndex, false);
        }
        if (_canvasRt != null) Destroy(_canvasRt.gameObject);   // hangs under the head while "headlock" is on
        if (_prevNearClip > 0f && Camera.main != null) Camera.main.nearClipPlane = _prevNearClip;
    }

    void OnTrackingOriginUpdated(XRInputSubsystem _) => Recenter();

    // The first transforms move the head away from the zero pose the first Recenter used. They
    // arrive in ROSConnection.Update but RobotModel applies them in its LateUpdate, so recenter
    // after that (see LateUpdate), not here.
    bool _recenterPending;

    void OnFirstTf()
    {
        if (this == null) return;
        _recenterPending = true;
    }

    // --- Anchor ---

    public void Recenter()
    {
        var head = Head;
        if (_model == null || head == null) return;
        var root = _model.Root;
        Transform cam = _camFrame != null ? _camFrame : root;

        // Turn the model about the vertical axis so the camera frame's yaw equals the head's yaw.
        Quaternion camRelRoot = Quaternion.Inverse(root.rotation) * cam.rotation;
        root.rotation = YawOnly(head.rotation) * Quaternion.Inverse(YawOnly(camRelRoot));
        // Then move it so the camera frame is at the head.
        root.position += head.position - cam.position;

        if (_panFrame != null)
        {
            _anchorPoint = _panFrame.position;
            _anchored = true;
        }
        Debug.Log($"FPV: recenter head=({head.position.x:F2},{head.position.y:F2},{head.position.z:F2}) " +
                  $"yaw={head.eulerAngles.y:F0} anchor=({_anchorPoint.x:F2},{_anchorPoint.y:F2},{_anchorPoint.z:F2})");
    }

    // Rotation about the vertical axis only (the direction q looks at, flattened).
    static Quaternion YawOnly(Quaternion q)
    {
        Vector3 f = q * Vector3.forward;
        f.y = 0f;
        if (f.sqrMagnitude < 1e-6f)
        {
            f = q * Vector3.up;   // looking straight up or down: use the top of the head
            f.y = 0f;
            if (f.sqrMagnitude < 1e-6f) return Quaternion.identity;
        }
        return Quaternion.LookRotation(f.normalized, Vector3.up);
    }

    void LateUpdate()
    {
        // After RobotModel's LateUpdate: keep the pan-axis point fixed while the lift/head move.
        if (_recenterPending)
        {
            _recenterPending = false;
            Recenter();
        }
        else if (_model != null && _anchored && _panFrame != null)
            _model.Root.position += _anchorPoint - _panFrame.position;

        if (Time.unscaledTime >= _nextLog)
        {
            _nextLog = Time.unscaledTime + LogSeconds;
            LogState("status");
        }
    }

    // --- Image quad ---

    void BuildQuad()
    {
        Transform parent = _camFrame != null ? _camFrame : _model.Root;

        var go = new GameObject("FPV Image", typeof(RectTransform));
        go.transform.SetParent(parent, false);
        var canvas = go.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.sortingOrder = HudUi.CanvasSortingOrder - 10;   // behind the HUD
        canvas.worldCamera = Camera.main;
        _canvasRt = (RectTransform)go.transform;
        _canvasRt.localScale = Vector3.one / HudUi.MmPerMetre;
        _canvasRt.localPosition = new Vector3(0f, 0f, QuadDistance);
        _canvasRt.localRotation = Quaternion.identity;

        // Same name frame as a camera block (CameraPanel), scaled from the block distance to the
        // quad's: outline ring and rounded card first so they sit behind the label and the image.
        var outline = HudUi.Ring(HudUi.Box(go.transform, "Outline", Color.clear), CardOutlineMm * CardScale / HudUi.RingThicknessRatio);
        _outlineRt = outline.rectTransform;
        var card = HudUi.Round(HudUi.Box(go.transform, "Card", HudUi.PanelColor), CardRadiusMm * CardScale);
        _cardRt = card.rectTransform;
        _name = HudUi.Label(go.transform, "Name", "Head Camera", CameraPanel.NameFontSize * HudUi.MmPerMetre * CardScale);

        _view = new GameObject("View", typeof(RectTransform)).AddComponent<RawImage>();
        _view.transform.SetParent(go.transform, false);
        _view.color = WaitingColor;
        _view.raycastTarget = false;
        _viewRt = _view.rectTransform;
        _viewRt.anchorMin = _viewRt.anchorMax = _viewRt.pivot = new Vector2(0.5f, 0.5f);

        float body = HudUi.TitleFontSize * HudUi.MmPerMetre * QuadDistance / HudUi.ReferenceDistance * 3f;
        _waiting = HudUi.Label(_view.transform, "Waiting", "Waiting for head camera", body);
        _waiting.color = HudUi.MutedText;
        HudUi.Stretch(_waiting.rectTransform);

        // Same fps / stale badge as a camera block, at the card's scale.
        _badge = CameraBadge.Create(_view.transform, CameraPanel.TopicFontSize * HudUi.MmPerMetre * CardScale);

        ApplyQuadSize();
    }

    void OnExperimentChanged(string key, bool on)
    {
        if (this == null) return;
        if (key == ExperimentSettings.HeadLock) ApplyHeadLock();
    }

    // Re-parents the image quad (size and intrinsics untouched): under the head while "headlock" is
    // on, else under the robot's camera frame. Same local offset either way.
    void ApplyHeadLock()
    {
        if (_canvasRt == null) return;
        var head = Head;
        bool want = ExperimentSettings.IsOn(ExperimentSettings.HeadLock) && head != null;
        Transform parent = want ? head : _camFrame != null ? _camFrame : _model.Root;
        HeadLocked = want;
        _canvasRt.SetParent(parent, false);
        _canvasRt.localPosition = new Vector3(0f, 0f, QuadDistance);
        _canvasRt.localRotation = Quaternion.identity;
        _canvasRt.localScale = Vector3.one / HudUi.MmPerMetre;
        Debug.Log($"FPV: image {(want ? "follows the head" : "on the robot camera frame")}");
    }

    // Size and offset from the camera intrinsics, or the default field of view until they arrive.
    void ApplyQuadSize()
    {
        if (_viewRt == null) return;
        float widthM, heightM, shiftXM = 0f, shiftYM = 0f;
        if (_hasInfo)
        {
            widthM = (float)(QuadDistance * _infoW / _fx);
            heightM = (float)(QuadDistance * _infoH / _fy);
            shiftXM = (float)(-(_cx - _infoW / 2.0) / _fx * QuadDistance);
            shiftYM = (float)((_cy - _infoH / 2.0) / _fy * QuadDistance);
        }
        else
        {
            widthM = 2f * QuadDistance * Mathf.Tan(_profile.defaultHfov / 2f);
            heightM = widthM / _textureAspect;
        }
        _viewRt.sizeDelta = new Vector2(widthM, heightM) * HudUi.MmPerMetre;
        _viewRt.anchoredPosition = new Vector2(shiftXM, shiftYM) * HudUi.MmPerMetre;
        _canvasRt.sizeDelta = _viewRt.sizeDelta;
        ApplyCard();
    }

    // Card behind the image with the camera name above it (no topic line), around the view rect.
    void ApplyCard()
    {
        if (_cardRt == null || _name == null) return;
        float pad = CardPaddingMm * CardScale, gap = CameraPanel.LabelGap * HudUi.MmPerMetre * CardScale;
        Vector2 view = _viewRt.sizeDelta;
        float nameH = _name.GetPreferredValues(_name.text, view.x, Mathf.Infinity).y;
        Vector2 centre = _viewRt.anchoredPosition;

        var card = new Vector2(view.x + 2f * pad, pad + nameH + gap + view.y + pad);
        foreach (var rt in new[] { _cardRt, _outlineRt })
        {
            rt.anchorMin = rt.anchorMax = rt.pivot = new Vector2(0.5f, 0.5f);
            rt.sizeDelta = card + (rt == _outlineRt ? Vector2.one * 2f * CardOutlineMm * CardScale : Vector2.zero);
            rt.anchoredPosition = centre + new Vector2(0f, (nameH + gap) / 2f);
        }
        _name.rectTransform.anchorMin = _name.rectTransform.anchorMax = _name.rectTransform.pivot = new Vector2(0.5f, 0.5f);
        _name.rectTransform.sizeDelta = new Vector2(view.x, nameH);
        _name.rectTransform.anchoredPosition = centre + new Vector2(0f, view.y / 2f + gap + nameH / 2f);
    }

    void UseTexture()
    {
        _cameraIndex = _images.IndexOf(_profile.firstPersonCameraTopicSuffix);
        if (_cameraIndex < 0)
        {
            Debug.LogWarning($"FPV: robot has no camera on '{_profile.firstPersonCameraTopicSuffix}'; no image");
            return;
        }
        var config = _images.Panels[_cameraIndex].Config;
        _textureAspect = config.Aspect;
        _name.text = _images.Panels[_cameraIndex].Label;
        ApplyQuadSize();
        _images.FrameReady += OnFrame;
        _images.CameraVisibilityChanged += OnCameraVisibility;
        SetCameraOn(_images.IsOn(_images.Panels[_cameraIndex]));
    }

    void OnCameraVisibility(int index, bool on)
    {
        if (this == null || index != _cameraIndex) return;
        SetCameraOn(on);
    }

    // Camera toggled in the bar: show / hide the card and stop / resume decoding its frames.
    void SetCameraOn(bool on)
    {
        CameraOn = on;
        if (_canvasRt != null) _canvasRt.gameObject.SetActive(on);
        _images.ForceDecode(_cameraIndex, on);
        Debug.Log($"FPV: head camera {(on ? "on" : "off")}");
    }

    void Update()
    {
        if (_badge != null && CameraOn && _cameraIndex >= 0) _badge.Tick(_images, _cameraIndex);
    }

    void OnFrame(int index, Texture2D tex)
    {
        if (this == null || index != _cameraIndex || tex == null) return;
        _frames++;
        if (_frames == 1)
        {
            _waiting.gameObject.SetActive(false);
            _view.color = Color.white;
            if (!_hasInfo)
            {
                _textureAspect = (float)tex.width / tex.height;
                ApplyQuadSize();
            }
        }
        _view.texture = tex;   // raw decoding can replace the instance

        // Same flips as CameraPanel.SetTexture.
        var config = _images.Panels[_cameraIndex].Config;
        _view.uvRect = new Rect(
            config.flipHorizontal ? 1f : 0f, config.flipVertical ? 1f : 0f,
            config.flipHorizontal ? -1f : 1f, config.flipVertical ? -1f : 1f);
    }

    void SubscribeCameraInfo()
    {
        if (string.IsNullOrEmpty(_profile.cameraInfoSuffix)) return;
        string topic = _profile.FullTopic(_profile.cameraInfoSuffix);
        ROSConnection.GetOrCreateInstance().Subscribe<CameraInfoMsg>(topic, OnCameraInfo);
    }

    // The first usable message wins.
    void OnCameraInfo(CameraInfoMsg msg)
    {
        if (this == null || !isActiveAndEnabled || _hasInfo || msg == null || msg.K == null || msg.K.Length < 6) return;
        if (msg.K[0] <= 0.0 || msg.K[4] <= 0.0 || msg.width == 0 || msg.height == 0) return;
        _fx = msg.K[0]; _fy = msg.K[4]; _cx = msg.K[2]; _cy = msg.K[5];
        _infoW = msg.width; _infoH = msg.height;
        _hasInfo = true;
        ApplyQuadSize();
        Debug.Log($"FPV: camera_info {msg.width}x{msg.height} fx={_fx:F1} fy={_fy:F1} cx={_cx:F1} cy={_cy:F1} " +
                  $"quad={_viewRt.sizeDelta.x / 1000f:F3}x{_viewRt.sizeDelta.y / 1000f:F3} m");
    }

    // --- Logging ---

    void LogState(string what)
    {
        if (_model == null) return;
        var sb = new StringBuilder("FPV: ").Append(what);
        if (what == "enable")
        {
            int visuals = 0; long tris = 0;
            foreach (var mf in _model.GetComponentsInChildren<MeshFilter>(true))
            {
                var mesh = mf.sharedMesh;
                if (mesh == null) continue;
                visuals++;
                for (int i = 0; i < mesh.subMeshCount; i++) tris += mesh.GetIndexCount(i) / 3;
            }
            sb.Append($" links={_model.LinkCount} visuals={visuals} triangles={tris}");
            if (_viewRt != null)
                sb.Append($" quad={_viewRt.sizeDelta.x / 1000f:F3}x{_viewRt.sizeDelta.y / 1000f:F3} m@{QuadDistance} m");
            sb.Append($" cameraIndex={_cameraIndex}");
        }
        else
        {
            sb.Append($" tfHz={_model.TfHz:F1} accepted={_model.AcceptedTransforms} frames={_frames}");
            if (_panFrame != null) sb.Append($" head_pan_local_yaw={_panFrame.localEulerAngles.y:F1}");
        }
        Debug.Log(sb.ToString());
    }
}
