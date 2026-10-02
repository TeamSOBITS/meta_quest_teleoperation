using System;
using UnityEngine;

/// <summary>
/// Per-robot teleoperation settings: ROS namespace, cameras, TF frames and the 3D model's anchors.
/// One asset per robot (Assets/Robots). The robot selection screen stores the
/// chosen profile in <see cref="Selected"/> before loading the teleop scene.
/// Every robot-specific value (frame names, topics, joint limits) lives here; defaults are empty/zero
/// and a feature whose frames are empty is simply off for that robot.
/// </summary>
[CreateAssetMenu(menuName = "Teleop/Robot Profile", fileName = "RobotProfile")]
public class RobotProfile : ScriptableObject, ISerializationCallbackReceiver
{
    // Which side of the robot / viewer a hand, arm or camera belongs to (hand cards, target markers).
    public enum Side { None, Left, Right }

    // What a camera is for: the head camera feeds the first-person view, hand cameras get a card at the gripper.
    public enum CameraRole { Other, Head, Hand }

    [Serializable]
    public class CameraConfig
    {
        // Label shown above the camera view.
        public string displayName;

        // Topic relative to the robot namespace,
        // e.g. "head_camera/color/image_raw/compressed" -> /<robotNamespace>/head_camera/...
        // A topic starting with '/' is used as is (added robots store absolute topics).
        public string topicSuffix;

        // sensor_msgs/Image instead of sensor_msgs/CompressedImage.
        public bool raw;

        // Native resolution of the real camera; sets the view's aspect ratio.
        public Vector2Int resolution = new Vector2Int(640, 480);

        // Maximum display rate. 0 = show every frame.
        public float maxFps = 15f;

        // Size of the camera view relative to the standard view height.
        // Labels keep the shared font sizes; neighbouring blocks move to make room.
        [Min(0.1f)] public float scale = 1f;

        // For cameras mounted rotated or mirrored.
        public bool flipVertical;
        public bool flipHorizontal;

        // Model robots (first-person view / hand cards):
        public CameraRole role;
        public Side side;
        // Exactly one head camera is the first-person camera.
        public bool firstPerson;
        // sensor_msgs/CameraInfo topic relative to the namespace (the first-person camera's field of view).
        public string cameraInfoSuffix;
        // Model frame the camera hangs on: the first-person image quad hangs on it, a hand card is placed at it.
        public string mountFrame;

        public float Aspect => resolution.x > 0 && resolution.y > 0
            ? (float)resolution.x / resolution.y
            : 4f / 3f;
    }

    // One arm of the robot: the hand target broadcast on /tf (sobits_teleop, base frame -> targetFrame)
    // and the model's end effector link the target is drawn against.
    [Serializable]
    public class ArmConfig
    {
        public string name;   // "left", "right", "arm": labels the target marker
        public string targetFrame;
        public string effectorFrame;
        public Side side;     // marker colour; the hand camera of the same side is placed at effectorFrame
    }

    // Pan / tilt head (StatusStrip gauge, first-person anchor). Limits (rad) are the joints' URDF limits
    // (filled by UrdfModelBuilder): pan is symmetric, tilt is up positive.
    [Serializable]
    public class HeadConfig
    {
        public string panFrame, tiltFrame;
        public float panLimitRad, tiltMinRad, tiltMaxRad;
    }

    // Prismatic body lift (StatusStrip gauge); rangeM = the joint's upper limit.
    [Serializable]
    public class LiftConfig
    {
        public string frame;
        public float rangeM;
    }

    // TF child frames the headset publishes under baseFrame (what the robot's teleop node listens for).
    [Serializable]
    public class ControllerFrames
    {
        public string hmd, left, right;
    }

    // Robot chosen on the selection screen. Null when a teleop scene is
    // opened directly in the Editor; scripts then fall back to their own defaults.
    public static RobotProfile Selected;

    // True while a newly added robot is being set up (cameras discovered, layout arranged).
    public static bool SetupMode;

    // Added from the headset (stored as JSON by RobotLibrary) rather than a project asset.
    [NonSerialized] public bool isCustom;

    public string displayName;

    // Shown on the robot's card on the selection screen.
    public Texture2D picture;

    // e.g. "sobit_home". Joy is published on /<robotNamespace>/joy.
    public string robotNamespace;

    // Scene loaded when this robot is selected.
    public string sceneName;

    // The robot's base frame: the headset's TF is published under it, arm targets arrive relative to it.
    public string baseFrame;

    // nav_msgs/Odometry topic relative to the robot namespace (BaseVelocity; empty = no velocity arrow).
    public string odomSuffix;

    public ControllerFrames controllerFrames = new ControllerFrames();

    public CameraConfig[] cameras = Array.Empty<CameraConfig>();

    [Header("First-person view (3D model)")]
    // Life-size model with one RobotLink per URDF link. Null for robots without a model
    // (added robots: RobotLibrary's JSON has no such field), which then have no first-person view.
    public GameObject modelPrefab;
    public HeadConfig head = new HeadConfig();
    public LiftConfig lift = new LiftConfig();
    // Arms with a hand target and an end effector in the model (ArmTargets, HandCamPip).
    public ArmConfig[] arms = Array.Empty<ArmConfig>();
    // Links (by frame-name prefix) hidden in first person: the head the user sits in.
    public string[] firstPersonHiddenLinkPrefixes = Array.Empty<string>();
    // Horizontal field of view (rad) used until camera_info arrives; 0 = FallbackHfov.
    public float defaultHfov;
    public const float FallbackHfov = 1.2f;

    public bool HasModel => modelPrefab != null;

    // Camera of the first-person view: the one marked firstPerson, else the first head camera; null if none.
    public CameraConfig FirstPersonCamera
    {
        get
        {
            if (cameras == null) return null;
            foreach (var c in cameras) if (c != null && c.firstPerson) return c;
            foreach (var c in cameras) if (c != null && c.role == CameraRole.Head) return c;
            return null;
        }
    }

    // Absolute topic for a topic relative to the robot namespace.
    public string FullTopic(string suffix)
    {
        if (suffix.StartsWith("/")) return suffix;
        string ns = string.IsNullOrEmpty(robotNamespace) ? "" : "/" + robotNamespace;
        return ns + "/" + suffix.TrimStart('/');
    }

    public string FullTopic(CameraConfig cam) => FullTopic(cam.topicSuffix);

    // ---- one-release migration of assets saved before the v2 schema -----------------------------------------------
    // The old flat fields stay as private serialized fields so an old asset still loads; their values move
    // into the new structs when those are empty, and the old fields are cleared (a re-save drops them).
#pragma warning disable 0618, 0649
    [SerializeField, Obsolete("use head.panFrame")] string panFrame;
    [SerializeField, Obsolete("use head.tiltFrame")] string tiltFrame;
    [SerializeField, Obsolete("use lift.frame")] string liftFrame;
    [SerializeField, Obsolete("use the first-person camera's mountFrame")] string cameraFrame;
    [SerializeField, Obsolete("use the first-person camera in cameras")] string firstPersonCameraTopicSuffix;
    [SerializeField, Obsolete("use the first-person camera's cameraInfoSuffix")] string cameraInfoSuffix;

    static bool _migrationLogged;

    public void OnBeforeSerialize() { }

    public void OnAfterDeserialize()
    {
        if (string.IsNullOrEmpty(panFrame) && string.IsNullOrEmpty(tiltFrame) && string.IsNullOrEmpty(liftFrame)
            && string.IsNullOrEmpty(cameraFrame) && string.IsNullOrEmpty(firstPersonCameraTopicSuffix)
            && string.IsNullOrEmpty(cameraInfoSuffix)) return;

        if (head == null) head = new HeadConfig();
        if (lift == null) lift = new LiftConfig();
        if (string.IsNullOrEmpty(head.panFrame) && string.IsNullOrEmpty(head.tiltFrame))
        {
            head.panFrame = panFrame;
            head.tiltFrame = tiltFrame;
        }
        if (string.IsNullOrEmpty(lift.frame)) lift.frame = liftFrame;

        // Cameras: the old head camera suffix marks the first-person camera; "hand" in a topic marks a hand camera
        // (and left / right in it its side), as the pre-v2 code guessed it.
        bool hasNew = false;
        if (cameras != null)
            foreach (var c in cameras) if (c != null && (c.firstPerson || c.role != CameraRole.Other)) hasNew = true;
        if (!hasNew && cameras != null)
        {
            string fp = (firstPersonCameraTopicSuffix ?? "").Replace(RosNames.CompressedSuffix, "");
            foreach (var c in cameras)
            {
                if (c == null || string.IsNullOrEmpty(c.topicSuffix)) continue;
                string t = c.topicSuffix.ToLowerInvariant();
                if (fp.Length > 0 && c.topicSuffix.Replace(RosNames.CompressedSuffix, "") == fp)
                {
                    c.role = CameraRole.Head;
                    c.firstPerson = true;
                    if (string.IsNullOrEmpty(c.mountFrame)) c.mountFrame = cameraFrame;
                    if (string.IsNullOrEmpty(c.cameraInfoSuffix)) c.cameraInfoSuffix = cameraInfoSuffix;
                }
                else if (t.Contains("hand"))
                {
                    c.role = CameraRole.Hand;
                    c.side = t.Contains("left") ? Side.Left : t.Contains("right") ? Side.Right : Side.None;
                }
            }
        }
        if (arms != null)
            foreach (var a in arms)
            {
                if (a == null || a.side != Side.None || string.IsNullOrEmpty(a.name)) continue;
                string n = a.name.ToLowerInvariant();
                a.side = n == "left" ? Side.Left : n == "right" ? Side.Right : Side.None;
            }

        panFrame = tiltFrame = liftFrame = cameraFrame = firstPersonCameraTopicSuffix = cameraInfoSuffix = null;
        if (!_migrationLogged)
        {
            _migrationLogged = true;
            Debug.Log("RobotProfile: migrated old flat fields of a profile to the v2 schema (head / lift / camera roles); re-save the asset to drop them");
        }
    }
#pragma warning restore 0618, 0649
}
