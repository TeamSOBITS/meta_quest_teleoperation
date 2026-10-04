using UnityEngine;

/// <summary>
/// Material assets of the HUD that need a custom shader (the first-person surround and the lens
/// undistortion), made in the Editor (Tools > HUD > Create assets) and kept in Resources/HudAssets so a
/// build includes their shaders. Code never looks a shader up by name: it uses these materials.
/// Reached through <see cref="HudTheme.Assets"/>; loaded once.
/// </summary>
public class HudAssets : ScriptableObject
{
    public Material surround;     // Hud/Surround, opaque, queue Background
    public Material undistort;    // Hud/UndistortImage, template for the per-camera material

    static HudAssets _instance;

    public static HudAssets Instance
    {
        get
        {
            if (_instance == null) _instance = Resources.Load<HudAssets>("HudAssets");
            if (_instance == null) Debug.LogWarning("HudAssets: Resources/HudAssets is missing (Tools > HUD > Create assets)");
            return _instance;
        }
    }
}
