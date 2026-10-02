using System.Collections.Generic;
using System.Text;
using RosMessageTypes.Sensor;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// <summary>
/// Robot model around the user (the headset is the robot's head camera): a life-size
/// <see cref="RobotModel"/> with the head and microphone links hidden. In the first-person camera
/// layout (<see cref="SetFirstPersonLayout"/>) it also shows the head camera's image, drawn at the
/// camera's true field of view, on a quad in front of the camera frame; in the blocks layout there is
/// no quad (the camera blocks show the images). Created by TeleopHud while the robot model is on and
/// destroyed when it is off.
///
/// Placement. The model hangs under the XR Origin (not the camera offset): the origin is the
/// transform whose space is the tracking space, so it moves with the playspace and its floor is
/// y = 0. On <see cref="Recenter"/> (enable, first TF, tracking origin reset, the Recenter button)
/// the model is turned about the vertical axis so the camera frame faces where the headset looks,
/// and moved so the camera frame sits at the headset position. Its pan-axis point (head.panFrame origin;
/// for robots without a pan/tilt head, i.e. head.panFrame empty or absent from the model, the camera frame
/// itself) is then remembered, and every frame the model is shifted so that point stays where it was:
/// when the lift or head moves, the model moves under the user instead of the image moving off
/// the eyes. Rotation is never touched after Recenter, so a turning robot head turns the image.
/// </summary>
[DefaultExecutionOrder(200)]
public class FirstPersonView : MonoBehaviour
{
    public const string LayoutBlocks = "blocks", LayoutFirstPerson = "firstperson";
    // Distance of the image quad from the camera frame (metres).
    const float QuadDistance = 1.5f;
    const float NearClip = 0.1f;
    // The camera blocks' card (CameraCard.Style.Block) shrinks with the distance ratio quad / block distance.
    const float CardScale = QuadDistance / HudTheme.ReferenceDistance;
    const float WaitingFontRatio = 3f;   // "Waiting for head camera" in the card's name font size x this
    const float LogSeconds = 5f;

    // Set by the launch intent (autonomous tests): "firstperson" (model on + first-person layout),
    // "model" (model on + blocks layout) or "blocks" (model off) for the next robot screen only,
    // never saved. TeleopHud.BuildHud consumes and clears it.
    public static string ViewModeOverride;
    // Replaces the headset pose for Recenter (the demo recorder films from a fixed, level head).
    public static Transform HeadOverride;
    public static Transform Head => HeadOverride != null ? HeadOverride : Camera.main != null ? Camera.main.transform : null;

    ImageSubscriber _images;
    RobotProfile _profile;
    RobotModel _model;
    Transform _camFrame, _panFrame;
    Vector3 _anchorPoint;
    bool _anchored;

    float _prevNearClip = -1f;
    readonly List<XRInputSubsystem> _inputs = new List<XRInputSubsystem>();

    // Image quad: a CameraCard whose view is the quad (its parts are kept as fields for the harness).
    CameraCard _card;
    RectTransform _canvasRt, _viewRt;
    RawImage _view;
    TextMeshProUGUI _name;
    CameraBadge _badge;
    RectTransform _cardRt;
    int _cameraIndex = -1;
    int _frames;
    float _nextLog;
    // Intrinsics (pixels); until camera_info arrives the field of view is the profile default.
    bool _hasInfo;
    double _fx, _fy, _cx, _cy, _infoW, _infoH;
    float _textureAspect = 4f / 3f;

    bool _quadFramed;   // the current quad has shown a frame
    public bool ImageShown => _canvasRt != null;
    Button _rename;

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
        var fpCamera = profile.FirstPersonCamera;
        _camFrame = _model.Frame(fpCamera?.mountFrame);
        _panFrame = _model.Frame(profile.head.panFrame);
        if (_camFrame == null) Debug.LogWarning($"FPV: camera frame '{fpCamera?.mountFrame}' not in the model; the image quad hangs on the model root");
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

        _cameraIndex = fpCamera != null ? _images.IndexOf(fpCamera.topicSuffix) : -1;
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
        foreach (var s in _inputs)
            if (s != null) s.trackingOriginUpdated -= OnTrackingOriginUpdated;
        if (_images != null)
        {
            _images.FrameReady -= OnFrame;
            _images.CameraVisibilityChanged -= OnCameraVisibility;
            _images.LabelsChanged -= OnLabels;
            if (_cameraIndex >= 0 && _canvasRt != null) _images.ForceDecode(_cameraIndex, false);
        }
        if (_canvasRt != null) Destroy(_canvasRt.gameObject);
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

    // First-person layout: the head camera's image card on the camera frame. Blocks layout: no card,
    // the camera blocks show the images. Safe to call repeatedly.
    public void SetFirstPersonLayout(bool on)
    {
        if (_model == null || on == ImageShown) return;
        if (on)
        {
            BuildQuad();
            UseTexture();
        }
        else DestroyQuad();
        Debug.Log($"FPV: image card {(on ? "shown" : "removed")}");
    }

    void DestroyQuad()
    {
        _images.FrameReady -= OnFrame;
        _images.CameraVisibilityChanged -= OnCameraVisibility;
        _images.LabelsChanged -= OnLabels;
        if (_cameraIndex >= 0) _images.ForceDecode(_cameraIndex, false);
        if (_canvasRt != null) Destroy(_canvasRt.gameObject);
        _canvasRt = _viewRt = _cardRt = null;
        _card = null; _view = null; _name = null; _badge = null; _rename = null;
        _quadFramed = false;
    }

    // Layout mode: the card's Rename button is available.
    public void SetEditable(bool editable)
    {
        if (_rename != null) _rename.gameObject.SetActive(editable);
    }

    void BuildQuad()
    {
        Transform parent = _camFrame != null ? _camFrame : _model.Root;

        var style = CameraCard.Style.Block(CardScale);
        style.SortingOrder = HudTheme.SortingOrder - 10;   // behind the HUD
        style.WaitingText = "Waiting for head camera";
        style.WaitingFontMm = style.NameFontMm * WaitingFontRatio;
        style.WaitingInsetMm = 0f;
        style.PinView = true;
        _card = CameraCard.Create(parent, "FPV Image", style, null, "Head Camera");
        _canvasRt = _card.Rect;
        _canvasRt.localPosition = new Vector3(0f, 0f, QuadDistance);
        _canvasRt.localRotation = Quaternion.identity;
        _card.Renamed += () =>
        {
            if (_cameraIndex >= 0) _images.BeginRename(_images.Panels[_cameraIndex]);
        };
        _viewRt = _card.ViewRect;
        _view = _card.View;
        _name = _card.NameLabel;
        _cardRt = _card.CardRect;
        _badge = _card.Badge;
        _rename = _card.RenameButton;

        ApplyQuadSize();
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
            widthM = 2f * QuadDistance * Mathf.Tan((_profile.defaultHfov > 0f ? _profile.defaultHfov : RobotProfile.FallbackHfov) / 2f);
            heightM = widthM / _textureAspect;
        }
        _card.Config = _cameraIndex >= 0 ? _images.Panels[_cameraIndex].Config : null;
        _card.SetSize(widthM * HudUi.MmPerMetre, heightM * HudUi.MmPerMetre, new Vector2(shiftXM, shiftYM) * HudUi.MmPerMetre);
    }

    void UseTexture()
    {
        if (_cameraIndex < 0)
        {
            Debug.LogWarning($"FPV: robot has no first-person camera in the profile (or not shown); no image");
            return;
        }
        var config = _images.Panels[_cameraIndex].Config;
        _textureAspect = config.Aspect;
        _card.SetLabel(_images.Panels[_cameraIndex].Label);
        ApplyQuadSize();
        _images.FrameReady += OnFrame;
        _images.CameraVisibilityChanged += OnCameraVisibility;
        _images.LabelsChanged += OnLabels;
        SetCameraOn(_images.IsOn(_images.Panels[_cameraIndex]));
    }

    // A camera was renamed: the card shows the head camera's name.
    void OnLabels()
    {
        if (this == null || _card == null || _cameraIndex < 0) return;
        _card.SetLabel(_images.Panels[_cameraIndex].Label);
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
        if (this == null || _view == null || index != _cameraIndex || tex == null) return;
        _frames++;
        if (!_quadFramed)
        {
            _quadFramed = true;
            if (!_hasInfo)
            {
                _textureAspect = (float)tex.width / tex.height;
                ApplyQuadSize();
            }
        }
        _card.Config = _images.Panels[_cameraIndex].Config;
        _card.SetTexture(tex);   // raw decoding can replace the instance
    }

    void SubscribeCameraInfo()
    {
        string info = _profile.FirstPersonCamera?.cameraInfoSuffix;
        if (string.IsNullOrEmpty(info)) return;
        string topic = _profile.FullTopic(info);
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
                  (_viewRt != null ? $"quad={_viewRt.sizeDelta.x / 1000f:F3}x{_viewRt.sizeDelta.y / 1000f:F3} m" : "(no image card in the blocks layout)"));
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
