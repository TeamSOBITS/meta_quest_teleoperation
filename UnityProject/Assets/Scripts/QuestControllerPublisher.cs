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

    // Robot namespace (e.g. "sobit_home", "sobit_pro").
    // Controls the joy topic: /<robotNamespace>/joy
    public string robotNamespace = "sobit_home";

    // TF parent frame — publish directly under base_footprint so the Quest frames
    // are always expressed relative to the robot, even after the robot drives.
    public string parent_frame_id = "base_footprint";
    public string headChildFrame = "hmd_odom";
    public string rightChildFrame = "right_controller_odom";
    public string leftChildFrame = "left_controller_odom";
    public string tfTopicName = "/tf";

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

    private readonly IpKeyboard _keyboard = new IpKeyboard();

    // Name of the scene to return to when the user wants to pick a different robot.
    public string robotSelectionSceneName = "RobotSelectionScene";

    // Restore the last IP typed on the keyboard. Done in Awake because ROSConnection
    // connects in its own Start, and every Awake runs before any Start — so the first
    // connection already goes to the saved IP, with no Disconnect/Connect cycle.
    public void Awake()
    {
        if (RobotProfile.Selected != null)
            robotNamespace = RobotProfile.Selected.robotNamespace;

        ros.RosIPAddress = RosIpSettings.Load(ros.RosIPAddress);
    }

    public void Start()
    {
        ros.RegisterPublisher<TFMessageMsg>(tfTopicName);
        SetNamespace(robotNamespace);

        _confirmedIp = ros.RosIPAddress;  // record what we are already connected to
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
        _joyTopicName = string.IsNullOrEmpty(robotNamespace) ? "/joy" : "/" + robotNamespace + "/joy";
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
            RosIpSettings.Save(_confirmedIp);
        }


        // Stop publishing when disconnected (avoids injecting stale TFs into a freshly-started
        // ROS session) and when robot control is off (layout mode: the robot must not move).
        if (ros.HasConnectionError || !controlRobot) return;

        _timeElapsed += Time.deltaTime;
        if (_timeElapsed > publishFrequency)
        {
            PublishTfJoy();
            _timeElapsed = 0;
        }
    }
    
    private TimeMsg GetRosTime()
    {
        // Always stamp with wall-clock (UTC). sobits_teleop uses a wall-clock TF
        // buffer so this works correctly in both sim and real-robot scenarios.
        DateTime unixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        long totalNanoseconds = (DateTime.UtcNow - unixEpoch).Ticks * 100;
        return new TimeMsg
        {
            sec     = (int)(totalNanoseconds / 1_000_000_000),
            nanosec = (uint)(totalNanoseconds % 1_000_000_000)
        };
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

    // Convert to ROS coordinate system (FLU)
    Vector3<FLU> rosHeadPos = tmpHead.position.To<FLU>();
    Quaternion<FLU> rosHeadRot = tmpHead.rotation.To<FLU>();

    Vector3<FLU> rosPositionRight = tmpRight.position.To<FLU>();
    Quaternion<FLU> rosRotationRight = tmpRight.rotation.To<FLU>();

    Vector3<FLU> rosPositionLeft = tmpLeft.position.To<FLU>();
    Quaternion<FLU> rosRotationLeft = tmpLeft.rotation.To<FLU>();

    HeaderMsg header = new HeaderMsg
    {
        frame_id = parent_frame_id,
        stamp = GetRosTime()
    };

    var transformMsgHmd = new TransformMsg
    {
        translation = rosHeadPos,
        rotation = rosHeadRot
    };

    var transformStampedHmd = new TransformStampedMsg
    {
        header = header,
        child_frame_id = headChildFrame,
        transform = transformMsgHmd
    };

    var transformMsgRight = new TransformMsg
    {
        translation = rosPositionRight,
        rotation = rosRotationRight
    };

    var transformMsgLeft = new TransformMsg
    {
        translation = rosPositionLeft,
        rotation = rosRotationLeft
    };

    var transformStampedRight = new TransformStampedMsg
    {
        header = header,
        child_frame_id = rightChildFrame,
        transform = transformMsgRight
    };

    var transformStampedLeft = new TransformStampedMsg
    {
        header = header,
        child_frame_id = leftChildFrame,
        transform = transformMsgLeft
    };

    // Only publish transforms for devices that are actively tracked.
    // If no device is tracked at all, skip publishing entirely.
    var tfList = new System.Collections.Generic.List<TransformStampedMsg>();
    if (headTracked)  tfList.Add(transformStampedHmd);
    if (rightTracked) tfList.Add(transformStampedRight);
    if (leftTracked)  tfList.Add(transformStampedLeft);

    if (tfList.Count == 0) return;

    ros.Publish(tfTopicName, new TFMessageMsg(tfList.ToArray()));


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
    float[] axes = new float[]
    {
        leftAxis.x, leftAxis.y, leftTrigger, leftGrip,
        rightAxis.x, rightAxis.y, rightTrigger, rightGrip
    };

    // Buttons order: left_primary, left_secondary, left_grip_button, right_primary, right_secondary, right_grip_button
    int[] buttons = new int[]
    {
        leftPrimary ? 1 : 0,
        leftSecondary ? 1 : 0,
        leftStickClick ? 1 : 0,
        leftMenuButton ? 1 : 0,
        rightPrimary ? 1 : 0,
        rightSecondary ? 1 : 0,
        rightStickClick ? 1 : 0,
    };

    var joyMsg = new JoyMsg
    {
        header = header,
        axes = axes,
        buttons = buttons
    };

    ros.Publish(_joyTopicName, joyMsg);
    }
}