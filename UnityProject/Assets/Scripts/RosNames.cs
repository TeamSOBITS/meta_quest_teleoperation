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
    // sobits_interfaces/VlaStatus topics of the sobits_vla_tools stage nodes, relative to the robot namespace (each
    // node publishes ~/status): the recorder (vla_rosbag_collection) and the policy runner (sobits_vla_deploy).
    public const string CollectionStatus = "vla_rosbag_collection/status";
    public const string DeployStatus = "sobits_vla_deploy/status";
    // RobotProfile.vlaStatusSuffixes of the built-in robots and of robots added on the headset.
    public static readonly string[] VlaStatusDefaults = { CollectionStatus, DeployStatus };
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
