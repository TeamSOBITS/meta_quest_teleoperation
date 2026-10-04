using RosMessageTypes.Sensor;
using UnityEngine;

/// <summary>
/// Lens distortion of the first-person camera. When camera_info carries plumb_bob coefficients
/// D = (k1 k2 p1 p2 k3) that are not all zero, the head card's image is drawn through the
/// Hud/UndistortImage shader (a copy of the HudAssets material with K, D and the image size set), so
/// straight lines in the world are straight in the card, matching the card's pinhole sizing.
/// Limits: only the plumb_bob model (no rational_polynomial / fisheye); the rectified image keeps the
/// camera's K (no optimal new camera matrix), so strongly distorted corners fall outside the source
/// image and are transparent; the sim publishes D = 0, so it only runs for real cameras with a distortion.
/// </summary>
public static class Undistort
{
    public static readonly int KId = Shader.PropertyToID("_K"), DistId = Shader.PropertyToID("_Dist"), Dist2Id = Shader.PropertyToID("_Dist2");

    // plumb_bob (or unnamed) with at least 5 coefficients, any of them non-zero.
    public static bool HasDistortion(string model, double[] d)
    {
        if (d == null || d.Length < 5) return false;
        if (!string.IsNullOrEmpty(model) && model != "plumb_bob") return false;
        for (int i = 0; i < 5; i++) if (d[i] != 0.0) return true;
        return false;
    }

    // A material for this camera, or null (no distortion, not plumb_bob, or no HudAssets material).
    public static Material Create(CameraInfoMsg info)
    {
        var template = HudTheme.Assets != null ? HudTheme.Assets.undistort : null;
        if (info == null || template == null || info.K == null || info.K.Length < 6 || !HasDistortion(info.distortion_model, info.D)) return null;
        var material = new Material(template);
        SetParams(material, info.K[0], info.K[4], info.K[2], info.K[5], info.width, info.height, info.D);
        return material;
    }

    public static void SetParams(Material m, double fx, double fy, double cx, double cy, double width, double height, double[] d)
    {
        m.SetVector(KId, new Vector4((float)fx, (float)fy, (float)cx, (float)cy));
        m.SetVector(DistId, new Vector4((float)d[0], (float)d[1], (float)d[2], (float)d[3]));
        m.SetVector(Dist2Id, new Vector4((float)d[4], (float)width, (float)height, 0f));
    }

    // CPU copy of the shader: for ideal (rectified) image coordinates `uv` (v up, as in textures) the
    // coordinates in the distorted camera image to sample.
    public static Vector2 SourceUv(Vector2 uv, double fx, double fy, double cx, double cy, double width, double height, double[] d)
    {
        double xn = (uv.x * width - cx) / fx, yn = ((1.0 - uv.y) * height - cy) / fy;
        double r2 = xn * xn + yn * yn;
        double radial = 1.0 + r2 * (d[0] + r2 * (d[1] + r2 * d[4]));
        double xd = xn * radial + 2.0 * d[2] * xn * yn + d[3] * (r2 + 2.0 * xn * xn);
        double yd = yn * radial + d[2] * (r2 + 2.0 * yn * yn) + 2.0 * d[3] * xn * yn;
        return new Vector2((float)((xd * fx + cx) / width), (float)(1.0 - (yd * fy + cy) / height));
    }
}
