using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Calm, dark "studio" surroundings shared by the robot selection and robot screens:
/// a flat slate background instead of the bright sky (which competes with camera images
/// and tires the eyes over long sessions) and soft, even ambient light. The scenes keep
/// their dark grid floor for a sense of ground. Applying it also honours the saved
/// passthrough setting (see <see cref="PassthroughMode"/>), which swaps the background for
/// the real room and hides the floor, so both screens show the same view.
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

        PassthroughMode.Apply(camera, PassthroughMode.Enabled);
    }

    const string KeyLightName = "Model Key Light";

    // Soft warm directional light for the 3D robot model (the flat ambient alone leaves it
    // near-black). Shines from above, in front and to the right. Leaves the scene's own light
    // and the ambient as they are; does nothing when the light already exists.
    public static void EnsureModelKeyLight()
    {
        if (GameObject.Find(KeyLightName) != null) return;
        var go = new GameObject(KeyLightName);
        var light = go.AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 0.8f;
        light.color = new Color(1f, 0.95f, 0.86f, 1f);
        light.shadows = LightShadows.None;
        go.transform.rotation = Quaternion.LookRotation(new Vector3(-0.45f, -0.75f, 0.5f).normalized, Vector3.up);
    }
}
