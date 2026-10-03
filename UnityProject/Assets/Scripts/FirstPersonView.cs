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
public partial class FirstPersonView : MonoBehaviour
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
    // First-person extras: link health ring + "LAST FRAME" label, dim surround, head-lag outline, lens undistortion.
    LinkHealth _health;
    float _nextLink;
    Surround _surround;
    HeadLagOutline _lag;
    Material _undistort;
    public Surround Surround => _surround;
    public HeadLagOutline Lag => _lag;
    public CameraCard Card => _card;
    public Material UndistortMaterial => _undistort;
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
            DevLog.Log("FPV", "no pan frame, anchoring at the camera frame");
        }

        _model.SetLinksVisible(profile.firstPersonHiddenLinkPrefixes, false);

        var cam = Camera.main;
        if (cam != null)
        {
            _prevNearClip = cam.nearClipPlane;
            cam.nearClipPlane = NearClip;
        }

        _cameraIndex = fpCamera != null ? _images.IndexOf(fpCamera.topicSuffix) : -1;
        _health = new LinkHealth(_images, _cameraIndex, () => RoundTrip.Latest);
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
        if (_lag != null) Destroy(_lag.gameObject);   // hangs on the headset, not on this object
        if (_undistort != null) Destroy(_undistort);
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
        DevLog.Log("FPV", $"recenter head=({head.position.x:F2},{head.position.y:F2},{head.position.z:F2}) " +
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

    // --- Logging ---

    void LogState(string what)
    {
        if (_model == null) return;
        var sb = new StringBuilder(what);
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
        DevLog.Log("FPV", sb.ToString());
    }
}
