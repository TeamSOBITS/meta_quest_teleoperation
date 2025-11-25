using System;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using RosMessageTypes.Sensor;
using UnityEngine.XR;
using TMPro;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine.InputSystem;

public class QuestControllerPublisher : MonoBehaviour
{
    public ROSConnection ros;
    public string parent_frame_id = "quest";
    public string headChildFrame = "hmd_odom";
    public string rightChildFrame = "right_controller_odom";
    public string leftChildFrame = "left_controller_odom";
    public string tfTopicName = "/tf";
    public string joyTopicName = "/joy";
    public float publishFrequency = 1.0f / 60.0f;
    public InputActionAsset inputActions;

    private float _timeElapsed;
    private InputAction _clutchAction;
    private InputAction _keyboardAction;

    private TouchScreenKeyboard _keyboard;
    public TextMeshProUGUI textInput;

    public void Start()
    {
        ros.RegisterPublisher<TFMessageMsg>(tfTopicName);
        // Publish controller buttons as sensor_msgs/Joy on a single topic for both controllers
        ros.RegisterPublisher<JoyMsg>(joyTopicName);
        
        _keyboardAction = inputActions.FindAction("OpenKeyboard");
        _keyboardAction.Enable();
        textInput = GameObject.Find("Text").GetComponent<TextMeshProUGUI>();
        textInput.text = ros.RosIPAddress;
    }


    public void Update()
    {
        if (_keyboardAction.WasPressedThisFrame())
        {
            TouchScreenKeyboard.hideInput = false;
            _keyboard = TouchScreenKeyboard.Open("",
                TouchScreenKeyboardType.NumbersAndPunctuation, false, false, false, false);
        }
        
        if (!ros.RosIPAddress.Equals(textInput.text))
        {
            ros.Disconnect();
            ros.Connect(textInput.text, 10000);
            PlayerPrefs.SetString("RosIPAddress", ros.RosIPAddress);
        }

        _timeElapsed += Time.deltaTime;
        if (_timeElapsed > publishFrequency)
        {
            PublishTfJoy();
            _timeElapsed = 0;
        }
    }
    
    void OnGUI()
    {
        if (_keyboard != null)
        {
            textInput.text = _keyboard.text;
        }
    }

    private static TimeMsg GetRosTime()
    {
        DateTime unixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        DateTime now = DateTime.UtcNow;

        TimeSpan timeSinceEpoch = now - unixEpoch;
        long totalTicks = timeSinceEpoch.Ticks;
        long totalNanoseconds = totalTicks * 100;
        return new TimeMsg
        {
            sec = (int)(totalNanoseconds / 1_000_000_000),
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

    if (headDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out Vector3 headPos) &&
        headDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.deviceRotation, out Quaternion headRot))
    {
        tmpHead.position = headPos;
        tmpHead.rotation = headRot;
    }

    if (rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out Vector3 rightPos) &&
        rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.deviceRotation, out Quaternion rightRot))
    {
        tmpRight.position = rightPos;
        tmpRight.rotation = rightRot;
    }
    
    if (leftDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out Vector3 leftPos) &&
        leftDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.deviceRotation, out Quaternion leftRot))
    {
        tmpLeft.position = leftPos;
        tmpLeft.rotation = leftRot;
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

    var tfMessage = new TFMessageMsg(new TransformStampedMsg[]
    {
        transformStampedHmd,
        transformStampedRight,
        transformStampedLeft
    });

    ros.Publish(tfTopicName, tfMessage);

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

    ros.Publish(joyTopicName, joyMsg);
    }
}