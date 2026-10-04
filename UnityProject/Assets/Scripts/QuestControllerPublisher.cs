using System;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using RosMessageTypes.Sensor;
using UnityEngine.XR;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine.SceneManagement;

public class QuestControllerPublisher : MonoBehaviour
{
    public ROSConnection ros;

    // Robot namespace; controls the joy topic: /<robotNamespace>/joy.
    // All of these come from the selected robot's profile at Awake (see ApplyProfile); the values
    // here only matter when no profile is selected and none is set on the scene's ImageSubscriber.
    public string robotNamespace = "";

    // TF parent frame — publish directly under the robot's base frame so the Quest frames
    // are always expressed relative to the robot, even after the robot drives.
    public string parent_frame_id = "";
    public string headChildFrame = "";
    public string rightChildFrame = "";
    public string leftChildFrame = "";
    public string tfTopicName = RosNames.Tf;

    // TFs are always stamped with wall-clock (UTC) time.
    // sobits_teleop uses a wall-clock TF buffer so sim/real time mixing is not an issue.
    public bool useSimTime = false;  // kept for Inspector compatibility, no longer used
    public float publishFrequency = 1.0f / 60.0f;

    // Drive the robot: publish head/controller poses (TF) and controller buttons/axes (Joy).
    // Off = nothing is sent, so the robot stays still ("layout mode"; camera blocks can be dragged).
    [UnityEngine.Serialization.FormerlySerializedAs("publishJoy")]
    public bool controlRobot = true;

    private float _timeElapsed;
    private string _joyTopicName;
    private string _confirmedIp;  // IP that was last explicitly connected to

    // Registration gate: ros_tcp_endpoint has no registration ACK, so the queued RegisterPublisher
    // syscommands race against Publish calls. After every (re)connect, hold publishing for this long.
    public const float RegistrationDelaySeconds = 2f;
    float _registrationOpenAt;
    bool _gateLogged;

    public bool RegistrationReady => !ros.HasConnectionError && Time.unscaledTime >= _registrationOpenAt;

    void RestartRegistrationGate()
    {
        _registrationOpenAt = Time.unscaledTime + RegistrationDelaySeconds;
        _gateLogged = false;
    }

    // Message instances allocated once at Start() and mutated in place by Update(); the ROS-TCP-Connector
    // serialises synchronously inside Publish(), so reuse is safe and nothing is allocated per frame.
    HeaderMsg _header;
    TimeMsg _stamp;
    TransformStampedMsg _tfHmd, _tfRight, _tfLeft;
    TransformStampedMsg[] _tfAll;
    JoyMsg _joyMsg;
    TransformStampedMsg[][] _tfBySize;
    static readonly DateTime UnixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

    private readonly IpKeyboard _keyboard = new IpKeyboard();

    // Name of the scene to return to when the user wants to pick a different robot.
    public string robotSelectionSceneName = "RobotSelectionScene";

    // Restore the last IP typed on the keyboard. Done in Awake because ROSConnection
    // connects in its own Start, and every Awake runs before any Start — so the first
    // connection already goes to the saved IP, with no Disconnect/Connect cycle.
    public void Awake()
    {
        var images = FindFirstObjectByType<ImageSubscriber>();
        ApplyProfile(RobotProfile.Selected != null ? RobotProfile.Selected : images != null ? images.defaultProfile : null);

        ros.RosIPAddress = RosIpSettings.Load(ros.RosIPAddress);
    }

    // Namespace and TF frame names from the robot's profile. A profile that leaves a frame empty (a robot
    // added on the headset before the v2 schema) gets the sobits_teleop names (TeleopConventions).
    void ApplyProfile(RobotProfile profile)
    {
        if (profile != null) robotNamespace = profile.robotNamespace;
        var f = profile != null ? profile.controllerFrames : null;
        parent_frame_id = Pick(profile != null ? profile.baseFrame : null, parent_frame_id, TeleopConventions.BaseFrame);
        headChildFrame = Pick(f?.hmd, headChildFrame, TeleopConventions.Hmd);
        leftChildFrame = Pick(f?.left, leftChildFrame, TeleopConventions.LeftController);
        rightChildFrame = Pick(f?.right, rightChildFrame, TeleopConventions.RightController);
    }

    static string Pick(string fromProfile, string current, string fallback)
        => !string.IsNullOrEmpty(fromProfile) ? fromProfile : !string.IsNullOrEmpty(current) ? current : fallback;

    public void Start()
    {
        ros.RegisterPublisher<TFMessageMsg>(tfTopicName);
        SetNamespace(robotNamespace);

        _confirmedIp = ros.RosIPAddress;  // record what we are already connected to
        RestartRegistrationGate();
        AllocateMessages();
    }

    // IP to show in the HUD: what is being typed while the keyboard is open, else the connected one.
    public string DisplayedIp => _keyboard.IsOpen ? TypingDisplay(_keyboard.Text, "Type the ROS PC IP\u2026") : _confirmedIp;

    // What to show while the (overlay) keyboard is open: the text so far, or a prompt.
    public static string TypingDisplay(string typed, string prompt)
        => string.IsNullOrEmpty(typed) ? prompt : typed + "|";

    public bool HasConnectionError => ros.HasConnectionError;

    // Leave the robot screen and go back to choosing a robot (HUD button and left menu button).
    public void BackToRobotSelection() => SceneManager.LoadScene(robotSelectionSceneName);

    // Joy goes to /<ns>/joy, or /joy when the robot has no namespace.
    public string JoyTopic => _joyTopicName;

    public void SetNamespace(string ns)
    {
        robotNamespace = (ns ?? "").Trim().Trim('/');
        _joyTopicName = string.IsNullOrEmpty(robotNamespace) ? "/" + RosNames.Joy : "/" + robotNamespace + "/" + RosNames.Joy;
        ros.RegisterPublisher<JoyMsg>(_joyTopicName);
    }

    // ROSConnection only disconnects on application quit. Without this, every robot screen
    // left behind a live connection that kept reconnecting, and ros_tcp_endpoint hands its
    // single outgoing stream to the newest connection, so old ones stole camera images.
    void OnDestroy()
    {
        if (ros != null) ros.Disconnect();
    }

    // Opens the Quest system keyboard; the typed IP is applied when the user confirms.
    public void OpenIpKeyboard() => _keyboard.Open(_confirmedIp);


    public void Update()
    {
        // Only reconnect when the user explicitly submits a new IP via the keyboard.
        // Comparing against _confirmedIp (not ros.RosIPAddress) avoids the startup
        // race where a transient mismatch between the UI text and ros.RosIPAddress
        // triggers an extra Disconnect/Connect cycle and causes the
        // "InvalidHandle: cannot use Destroyable" exception in ros_tcp_endpoint.
        string newIp = _keyboard.Poll();
        if (newIp != null)
        {
            _confirmedIp = newIp;
            ros.Disconnect();
            ros.Connect(_confirmedIp, RosIpSettings.Port);
            RestartRegistrationGate();
            RosIpSettings.Save(_confirmedIp);
        }


        // Stop publishing when disconnected (avoids injecting stale TFs into a freshly-started
        // ROS session) and when robot control is off (layout mode: the robot must not move).
        if (!RegistrationReady || !controlRobot) return;
        if (!_gateLogged) { _gateLogged = true; Debug.Log("[Publisher] registration gate open"); }

        _timeElapsed += Time.deltaTime;
        if (_timeElapsed > publishFrequency)
        {
            PublishTfJoy();
            _timeElapsed = 0;
        }
    }

    void AllocateMessages()
    {
        _stamp = new TimeMsg();
        _header = new HeaderMsg { frame_id = parent_frame_id, stamp = _stamp };
        _tfHmd = NewStamped(headChildFrame);
        _tfRight = NewStamped(rightChildFrame);
        _tfLeft = NewStamped(leftChildFrame);
        _tfAll = new[] { _tfHmd, _tfRight, _tfLeft };
        // TFMessageMsg holds an array whose length is the message length, so keep one prebuilt
        // array (and message) per possible tracked-count/subset combination.
        _tfBySize = new TransformStampedMsg[8][];
        _tfMsgBySubset = new TFMessageMsg[8];
        for (int m = 1; m < 8; m++)
        {
            var list = new System.Collections.Generic.List<TransformStampedMsg>(3);
            for (int b = 0; b < 3; b++) if ((m & (1 << b)) != 0) list.Add(_tfAll[b]);
            _tfBySize[m] = list.ToArray();
            _tfMsgBySubset[m] = new TFMessageMsg(_tfBySize[m]);
        }
        _joyMsg = new JoyMsg
        {
            header = _header,
            axes = new float[8],
            buttons = new int[7]
        };
    }
    TFMessageMsg[] _tfMsgBySubset;

    TransformStampedMsg NewStamped(string child) => new TransformStampedMsg
    {
        header = _header,
        child_frame_id = child,
        transform = new TransformMsg { translation = new Vector3Msg(), rotation = new QuaternionMsg() }
    };

    // Always stamp with wall-clock (UTC). sobits_teleop uses a wall-clock TF buffer so this works
    // correctly in both sim and real-robot scenarios. Mutates the shared stamp in place.
    void UpdateRosTime()
    {
        long totalNanoseconds = (DateTime.UtcNow - UnixEpoch).Ticks * 100;
        _stamp.sec = (int)(totalNanoseconds / 1_000_000_000);
        _stamp.nanosec = (uint)(totalNanoseconds % 1_000_000_000);
    }

    static void Fill(TransformStampedMsg m, Vector3 pos, Quaternion rot)
    {
        var t = m.transform.translation; var r = m.transform.rotation;
        Vector3<FLU> p = pos.To<FLU>(); Quaternion<FLU> q = rot.To<FLU>();
        t.x = p.x; t.y = p.y; t.z = p.z;
        r.x = q.x; r.y = q.y; r.z = q.z; r.w = q.w;
    }

    private void PublishTfJoy()
    {
    // get head and controller poses
    UnityEngine.XR.InputDevice headDevice = InputDevices.GetDeviceAtXRNode(XRNode.Head);
    UnityEngine.XR.InputDevice rightDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
    UnityEngine.XR.InputDevice leftDevice = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);

    Pose tmpHead = new Pose();
    Pose tmpRight = new Pose();
    Pose tmpLeft = new Pose();

    bool headTracked = false;
    if (headDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out Vector3 headPos) &&
        headDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.deviceRotation, out Quaternion headRot) &&
        (headRot.x != 0f || headRot.y != 0f || headRot.z != 0f || headRot.w != 0f))
    {
        tmpHead.position = headPos;
        tmpHead.rotation = headRot;
        headTracked = true;
    }

    bool rightTracked = false;
    if (rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out Vector3 rightPos) &&
        rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.deviceRotation, out Quaternion rightRot) &&
        (rightRot.x != 0f || rightRot.y != 0f || rightRot.z != 0f || rightRot.w != 0f))
    {
        tmpRight.position = rightPos;
        tmpRight.rotation = rightRot;
        rightTracked = true;
    }

    bool leftTracked = false;
    if (leftDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out Vector3 leftPos) &&
        leftDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.deviceRotation, out Quaternion leftRot) &&
        (leftRot.x != 0f || leftRot.y != 0f || leftRot.z != 0f || leftRot.w != 0f))
    {
        tmpLeft.position = leftPos;
        tmpLeft.rotation = leftRot;
        leftTracked = true;
    }

    UpdateRosTime();
    _header.frame_id = parent_frame_id;
    _tfHmd.child_frame_id = headChildFrame;
    _tfRight.child_frame_id = rightChildFrame;
    _tfLeft.child_frame_id = leftChildFrame;
    Fill(_tfHmd, tmpHead.position, tmpHead.rotation);
    Fill(_tfRight, tmpRight.position, tmpRight.rotation);
    Fill(_tfLeft, tmpLeft.position, tmpLeft.rotation);

    // Only publish transforms for devices that are actively tracked.
    // If no device is tracked at all, skip publishing entirely.
    int mask = (headTracked ? 1 : 0) | (rightTracked ? 2 : 0) | (leftTracked ? 4 : 0);
    if (mask == 0) return;

    ros.Publish(tfTopicName, _tfMsgBySubset[mask]);


    // --- Publish controller button states as sensor_msgs/Joy on a single /joy topic ---
    // Use Unity XR InputDevices to query Meta Quest controller buttons and axes

    // Right controller values
    rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primaryButton, out bool rightPrimary);
    rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.secondaryButton, out bool rightSecondary);
    rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.trigger, out float rightTrigger);
    rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.grip, out float rightGrip);
    rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primary2DAxis, out Vector2 rightAxis);
    rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primary2DAxisClick, out bool rightStickClick);
    // Left controller values
    leftDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primaryButton, out bool leftPrimary);
    leftDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.secondaryButton, out bool leftSecondary);
    leftDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.trigger, out float leftTrigger);
    leftDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.grip, out float leftGrip);
    leftDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primary2DAxis, out Vector2 leftAxis);
    leftDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primary2DAxisClick, out bool leftStickClick);
    leftDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.menuButton, out bool leftMenuButton);

    // Build axes and buttons arrays for sensor_msgs/Joy
    // Axes order: left_x, left_y, left_trigger, left_grip, right_x, right_y, right_trigger, right_grip
    var axes = _joyMsg.axes;
    axes[0] = leftAxis.x;  axes[1] = leftAxis.y;  axes[2] = leftTrigger;  axes[3] = leftGrip;
    axes[4] = rightAxis.x; axes[5] = rightAxis.y; axes[6] = rightTrigger; axes[7] = rightGrip;

    // Buttons order: left_primary, left_secondary, left_stick_click, left_menu, right_primary, right_secondary, right_stick_click
    var buttons = _joyMsg.buttons;
    buttons[0] = leftPrimary ? 1 : 0;
    buttons[1] = leftSecondary ? 1 : 0;
    buttons[2] = leftStickClick ? 1 : 0;
    buttons[3] = leftMenuButton ? 1 : 0;
    buttons[4] = rightPrimary ? 1 : 0;
    buttons[5] = rightSecondary ? 1 : 0;
    buttons[6] = rightStickClick ? 1 : 0;

    ros.Publish(_joyTopicName, _joyMsg);
    }
}
