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

        _bar = HudBar.Create(hudParent, publisher, images, this).transform;
        images.CamerasAdded += RebuildBar;

        var dragger = gameObject.AddComponent<PanelDragger>();
        dragger.publisher = publisher;
        dragger.images = images;

        ControllerHints.Create(hudParent);

        if (PlayerPrefs.GetInt(LazyFollowKey, 0) == 1)
            SetLazyFollow(true);
    }

    // "Find cameras" added blocks: rebuild the bar so it lists their toggles too.
    void RebuildBar()
    {
        var parent = _bar.parent;
        Destroy(_bar.gameObject);
        _bar = HudBar.Create(hudParent, publisher, images, this).transform;
        _bar.SetParent(parent, false);
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

    bool _menuWasPressed;

    void Update()
    {
        if (_waitingStatus != null) _waitingStatus.text = images.SetupStatus;

        // Menu shows/hides the HUD bar: the left controller's menu button, or with hand tracking
        // the hand menu gesture (left palm facing you + pinch). Back to robots is on the bar.
        bool pressed = ControllerMenuPressed() || HandMenuGesture();
        if (pressed && !_menuWasPressed && _bar != null)
        {
            _bar.gameObject.SetActive(!_bar.gameObject.activeSelf);
            Debug.Log($"[TeleopHud] menu -> bar {(_bar.gameObject.activeSelf ? "shown" : "hidden")}");
        }
        _menuWasPressed = pressed;
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

    static XRHandSubsystem Hands
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
