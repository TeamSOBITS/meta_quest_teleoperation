/// <summary>
/// ROS conventions that hold for every robot (not per-robot values: those live in <see cref="RobotProfile"/>).
/// </summary>
public static class RosNames
{
    // tf2_msgs/TFMessage topic, always absolute.
    public const string Tf = "/tf";
    // sensor_msgs/Joy topic name; the robot namespace goes in front (/<ns>/joy), see RobotProfile.FullTopic.
    public const string Joy = "joy";
    // image_transport: the compressed twin of a raw image topic.
    public const string CompressedSuffix = "/compressed";
    // image_transport: the raw colour image topic's usual last segment (camera name heuristics).
    public const string ImageRaw = "image_raw";
}

/// <summary>
/// What sobits_teleop (the endpoint this app talks to) expects the headset's TF frames to be called.
/// A documented default for robots added on the headset (RobotLibrary.CreateNew) and the fallback of
/// QuestControllerPublisher for a profile that leaves them empty (old added robots); the built-in
/// robots spell the same names out in their profile assets.
/// </summary>
public static class TeleopConventions
{
    public const string BaseFrame = "base_footprint";
    public const string Hmd = "hmd_odom";
    public const string LeftController = "left_controller_odom";
    public const string RightController = "right_controller_odom";
}
