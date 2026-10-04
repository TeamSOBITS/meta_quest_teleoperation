using System;
using System.Collections;
using System.IO;
using UnityEngine;

/// <summary>
/// Autonomous tests (Settings.DebugCapture, set from the `capture` launch extra): once the screen
/// has been up a while, save what the main camera sees as a PNG for `adb pull`, then clear the flag.
/// </summary>
public static class DebugCapture
{
    const float DelaySeconds = 10f;

    public static IEnumerator Run()
    {
        yield return new WaitForSecondsRealtime(DelaySeconds);
        yield return null;
        Settings.DebugCapture = false;
        Settings.Save();

        var cam = Camera.main;
        if (cam == null) { Debug.LogWarning("FPV: capture failed, no main camera"); yield break; }
        RenderTexture rt = null;
        Texture2D png = null;
        var previousTarget = cam.targetTexture;
        var previousEye = cam.stereoTargetEye;
        var previousActive = RenderTexture.active;
        try
        {
            rt = new RenderTexture(1280, 720, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
            cam.stereoTargetEye = StereoTargetEyeMask.None;
            cam.targetTexture = rt;
            cam.Render();
            RenderTexture.active = rt;
            png = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
            png.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
            png.Apply(false);
            string path = Path.Combine(Application.persistentDataPath, "fpv_capture.png");
            File.WriteAllBytes(path, png.EncodeToPNG());
            DevLog.Log("FPV", $"capture {path}");
        }
        catch (Exception e)
        {
            Debug.LogWarning($"FPV: capture failed: {e.Message}");
        }
        finally
        {
            cam.targetTexture = previousTarget;
            cam.stereoTargetEye = previousEye;
            RenderTexture.active = previousActive;
            if (rt != null) { rt.Release(); UnityEngine.Object.Destroy(rt); }
            if (png != null) UnityEngine.Object.Destroy(png);
        }
    }
}
