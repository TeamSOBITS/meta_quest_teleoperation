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
        public string topicSuffix;

        // Native resolution of the real camera; sets the view's aspect ratio.
        public Vector2Int resolution = new Vector2Int(640, 480);

        // Maximum display rate. 0 = show every frame.
        public float maxFps = 15f;

        public float Aspect => resolution.x > 0 && resolution.y > 0
            ? (float)resolution.x / resolution.y
            : 4f / 3f;
    }

    // Robot chosen on the selection screen. Null when a teleop scene is
    // opened directly in the Editor; scripts then fall back to their own defaults.
    public static RobotProfile Selected;

    public string displayName;

    // e.g. "sobit_home". Joy is published on /<robotNamespace>/joy.
    public string robotNamespace;

    // Scene loaded when this robot is selected.
    public string sceneName;

    public CameraConfig[] cameras = Array.Empty<CameraConfig>();

    public string FullTopic(CameraConfig cam)
    {
        string ns = string.IsNullOrEmpty(robotNamespace) ? "" : "/" + robotNamespace;
        return ns + "/" + cam.topicSuffix.TrimStart('/');
    }
}
