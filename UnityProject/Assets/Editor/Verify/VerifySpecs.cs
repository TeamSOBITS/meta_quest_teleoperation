using System;
using System.Collections.Generic;

// Per-robot values of the live suites (env VERIFY_ROBOT = asset name, default SOBIT_HOME). Joint targets match
// tools/sim/pub.sh for the same robot; the sim itself is switched with tools/sim.sh home|light.
public class RobotSpec
{
    public class Side { public string name, targetsArg, marker; public float markerX; public string cameraSuffix; }

    public string Asset, Lower;
    public int Links, MaxTris;
    public float MinHeight, MaxHeight, MinCameraY, MinMinY = -0.05f, MaxMinY = 0.1f;
    public string WheelLinkContains;
    public string PanJoint, TiltJoint, PanLink, TiltLink;
    public string LiftJoint, LiftLink;                       // null: no lift
    public string ArmJointPrefix, HandFrame;                 // joints moved by pub.sh arm; frame that moves with them
    public Dictionary<string, double> ArmMoved, ArmReady;    // joint -> value after pub.sh arm / armhome (settle probes)
    public float MinCardMove = 0.2f;                         // hand card travel (m) between pub.sh armhome and arm
    public Side[] Sides;                                     // arms with a hand camera, in profile order
    public bool DualArm => Sides.Length > 1;
    public string PrefabPath => $"Assets/Robots/Models/{Asset}.prefab";
    public string ProfilePath => $"Assets/Robots/{Asset}.asset";
    public string HeadCamSuffix = "head_camera/color/image_raw/compressed";
    public string JointStates => $"/{Lower}/joint_states";
    public string OdomSuffix = "odom";
    public string Odom => $"/{Lower}/{OdomSuffix}";
    public string Topic(string suffix) => $"/{Lower}/{suffix}";
    public bool HasLift => LiftJoint != null;

    static RobotSpec _current;
    public static RobotSpec Current
    {
        get
        {
            if (_current != null) return _current;
            string name = Environment.GetEnvironmentVariable("VERIFY_ROBOT");
            if (string.IsNullOrEmpty(name)) name = "SOBIT_HOME";
            if (name == "SOBIT_HOME") return _current = Home();
            if (name == "SOBIT_LIGHT") return _current = Light();
            throw new Exception("VERIFY_ROBOT must be SOBIT_HOME or SOBIT_LIGHT, got " + name);
        }
    }

    static RobotSpec Home() => new RobotSpec
    {
        Asset = "SOBIT_HOME", Lower = "sobit_home", Links = 85, MaxTris = 250000, MinHeight = 1.2f, MaxHeight = 1.9f, MinCameraY = 0.9f,
        WheelLinkContains = "wheel_drive_", PanJoint = "head_pan_joint", TiltJoint = "head_tilt_joint", PanLink = "head_pan_link", TiltLink = "head_tilt_link",
        LiftJoint = "body_lift_joint", LiftLink = "body_lift_link",
        ArmJointPrefix = "arm_left_", HandFrame = "hand_left_camera_base_link",
        ArmMoved = new Dictionary<string, double> { ["arm_left_shoulder_tilt_joint"] = -0.75, ["arm_left_upper_roll_joint"] = -1.22, ["arm_left_upper_flex_joint"] = -0.2, ["arm_left_elbow_joint"] = 2.5 },
        ArmReady = new Dictionary<string, double> { ["arm_left_elbow_joint"] = 1.5709 },
        Sides = new[]
        {
            new Side { name = "left", targetsArg = "left", marker = "L", markerX = -0.2f, cameraSuffix = "hand_left_camera/color/image_raw/compressed" },
            new Side { name = "right", targetsArg = "right", marker = "R", markerX = 0.2f, cameraSuffix = "hand_right_camera/color/image_raw/compressed" },
        },
    };

    static RobotSpec Light() => new RobotSpec
    {
        Asset = "SOBIT_LIGHT", Lower = "sobit_light", Links = 55, MaxTris = 200000, MinHeight = 0.9f, MaxHeight = 1.4f, MinCameraY = 0.85f,
        WheelLinkContains = "_drive_wheel_", OdomSuffix = "wheel_controller/odom", MinCardMove = 0.1f, PanJoint = "head_yaw_joint", TiltJoint = "head_pitch_joint", PanLink = "head_yaw_link", TiltLink = "head_pitch_link",
        LiftJoint = null, LiftLink = null,
        ArmJointPrefix = "arm_", HandFrame = "hand_end_effector_link",
        ArmMoved = new Dictionary<string, double> { ["arm_shoulder_pitch_joint"] = -0.5, ["arm_elbow_pitch_joint"] = -0.2, ["arm_wrist_pitch_joint"] = 0.7 },
        ArmReady = new Dictionary<string, double> { ["arm_shoulder_pitch_joint"] = -1.1, ["arm_elbow_pitch_joint"] = -0.6, ["arm_wrist_pitch_joint"] = 0.8 },
        Sides = new[] { new Side { name = "arm", targetsArg = "arm", marker = "A", markerX = -0.2f, cameraSuffix = "hand_camera/color/image_raw/compressed" } },
    };
}
