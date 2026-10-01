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
/// and moved so the camera frame sits at the headset position. Its pan-axis point (panFrame origin)
/// is then remembered, and every frame the model is shifted so that point stays where it was:
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
    const float LogSeconds = 5f;
    static readonly Color WaitingColor = new Color(0.15f, 0.15f, 0.15f, 1f);

    // Set by the launch intent (autonomous tests): overrides the mode for the next robot screen
    // only, is never saved. TeleopHud.BuildHud consumes and clears it.
    public static string ViewModeOverride;
    // Replaces the headset pose for Recenter (the demo recorder films from a fixed, level head).
    public static Transform HeadOverride;
    static Transform Head => HeadOverride != null ? HeadOverride : Camera.main != null ? Camera.main.transform : null;

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
    TextMeshProUGUI _waiting;
    int _cameraIndex = -1;
    int _frames;
    float _nextLog;
    // Intrinsics (pixels); until camera_info arrives the field of view is the profile default.
    bool _hasInfo;
    double _fx, _fy, _cx, _cy, _infoW, _infoH;
    float _textureAspect = 4f / 3f;

    public RobotModel Model => _model;
    public int FramesReceived => _frames;

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
        if (_panFrame == null) Debug.LogWarning($"FPV: pan frame '{profile.panFrame}' not in the model; no anchor correction");

        _model.SetLinksVisible(profile.firstPersonHiddenLinkPrefixes, false);

        var cam = Camera.main;
        if (cam != null)
        {
            _prevNearClip = cam.nearClipPlane;
            cam.nearClipPlane = NearClip;
        }

        BuildQuad();
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
        foreach (var s in _inputs)
            if (s != null) s.trackingOriginUpdated -= OnTrackingOriginUpdated;
        if (_images != null)
        {
            _images.FrameReady -= OnFrame;
            if (_cameraIndex >= 0) _images.ForceDecode(_cameraIndex, false);
        }
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
            widthM = 2f * QuadDistance * Mathf.Tan(_profile.defaultHfov / 2f);
            heightM = widthM / _textureAspect;
        }
        _viewRt.sizeDelta = new Vector2(widthM, heightM) * HudUi.MmPerMetre;
        _viewRt.anchoredPosition = new Vector2(shiftXM, shiftYM) * HudUi.MmPerMetre;
        _canvasRt.sizeDelta = _viewRt.sizeDelta;
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
        ApplyQuadSize();
        _images.ForceDecode(_cameraIndex, true);
        _images.FrameReady += OnFrame;
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
