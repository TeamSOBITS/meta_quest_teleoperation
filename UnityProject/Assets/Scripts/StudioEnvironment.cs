using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Calm, dark "studio" surroundings shared by the robot selection and robot screens:
/// a flat slate background instead of the bright sky (which competes with camera images
/// and tires the eyes over long sessions) and soft, even ambient light. The scenes keep
/// their dark grid floor for a sense of ground.
/// </summary>
public static class StudioEnvironment
{
    public static readonly Color Background = new Color(0.07f, 0.09f, 0.12f, 1f);
    static readonly Color Ambient = new Color(0.42f, 0.45f, 0.50f, 1f);

    public static void Apply(Camera camera)
    {
        RenderSettings.skybox = null;
        RenderSettings.ambientMode = AmbientMode.Flat;
        RenderSettings.ambientLight = Ambient;
        RenderSettings.fog = false;

        if (camera != null)
        {
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = Background;
        }
    }
}
