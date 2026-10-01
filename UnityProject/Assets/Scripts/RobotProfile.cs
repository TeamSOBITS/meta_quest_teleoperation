using System;
using UnityEngine;

/// <summary>
/// Per-robot teleoperation settings: ROS namespace and the cameras to show.
/// One asset per robot (Assets/Robots). The robot selection screen stores the
/// chosen profile in <see cref="Selected"/> before loading the teleop scene.
/// </summary>
[CreateAssetMenu(menuName = "Teleop/Robot Profile", fileName = "RobotProfile")]
public class RobotProfile : ScriptableObject
{
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

        public float Aspect => resolution.x > 0 && resolution.y > 0
            ? (float)resolution.x / resolution.y
            : 4f / 3f;
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

    public CameraConfig[] cameras = Array.Empty<CameraConfig>();

    [Header("First-person view (3D model)")]
    // Life-size model with one RobotLink per URDF link. Null for robots without a model
    // (added robots: RobotLibrary's JSON has no such field), which then have no first-person view.
    public GameObject modelPrefab;
    // Frame of the head camera (the image quad hangs on it) and of the pan axis (stays fixed).
    public string cameraFrame = "head_camera_color_frame";
    public string panFrame = "head_pan_link";
    // Camera topics relative to the robot namespace (see FullTopic).
    public string firstPersonCameraTopicSuffix = "head_camera/color/image_raw/compressed";
    public string cameraInfoSuffix = "head_camera/camera_info";
    // Links (by frame-name prefix) hidden in first person: the head the user sits in.
    public string[] firstPersonHiddenLinkPrefixes = { "head_", "mic_" };
    // Horizontal field of view (rad) used until camera_info arrives.
    public float defaultHfov = 1.2113f;

    public bool HasModel => modelPrefab != null;

    // Absolute topic for a topic relative to the robot namespace.
    public string FullTopic(string suffix)
    {
        if (suffix.StartsWith("/")) return suffix;
        string ns = string.IsNullOrEmpty(robotNamespace) ? "" : "/" + robotNamespace;
        return ns + "/" + suffix.TrimStart('/');
    }

    public string FullTopic(CameraConfig cam) => FullTopic(cam.topicSuffix);
}
