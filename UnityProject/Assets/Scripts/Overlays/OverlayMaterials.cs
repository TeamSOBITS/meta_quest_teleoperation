using UnityEngine;

/// <summary>
/// Tinted copies of a material the robot model already carries, for the overlay markers and
/// lines. Nothing here looks a shader up by name (Shader.Find fails in a build that did not
/// include the shader): a copy of an existing material keeps its shader.
/// </summary>
public static class OverlayMaterials
{
    // The first material found on the model's renderers, or null.
    public static Material Source(RobotModel model)
    {
        if (model == null) return null;
        foreach (var r in model.GetComponentsInChildren<Renderer>(true))
        {
            if (r is LineRenderer || r is TrailRenderer) continue;
            var m = r.sharedMaterial;
            if (m != null) return m;
        }
        return null;
    }

    // A copy of `source` painted `colour` (URP _BaseColor, built-in _Color), or null without a source.
    public static Material Tinted(Material source, Color colour)
    {
        if (source == null) return null;
        var m = new Material(source);
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", colour);
        if (m.HasProperty("_Color")) m.SetColor("_Color", colour);
        if (m.HasProperty("_BaseMap")) m.SetTexture("_BaseMap", null);
        if (m.HasProperty("_MainTex")) m.SetTexture("_MainTex", null);
        return m;
    }

    // Makes a copy produced by Tinted alpha-blended (URP Lit / Simple Lit: _Surface 1, alpha blend,
    // no depth write, transparent queue). A material without those properties stays opaque.
    public static Material Transparent(Material m)
    {
        if (m == null || !m.HasProperty("_Surface")) return m;
        m.SetFloat("_Surface", 1f);
        if (m.HasProperty("_Blend")) m.SetFloat("_Blend", 0f);
        if (m.HasProperty("_SrcBlend")) m.SetFloat("_SrcBlend", (float)UnityEngine.Rendering.BlendMode.SrcAlpha);
        if (m.HasProperty("_DstBlend")) m.SetFloat("_DstBlend", (float)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        if (m.HasProperty("_ZWrite")) m.SetFloat("_ZWrite", 0f);
        m.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        m.DisableKeyword("_ALPHATEST_ON");
        m.SetOverrideTag("RenderType", "Transparent");
        m.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
        return m;
    }

    // Name of the model's root link (the TF parent frame of the arm targets), or null.
    public static string RootFrame(RobotModel model)
    {
        if (model == null) return null;
        foreach (var link in model.GetComponentsInChildren<RobotLink>(true))
            if (link.isRoot && !string.IsNullOrEmpty(link.frame)) return link.frame;
        return null;
    }
}
