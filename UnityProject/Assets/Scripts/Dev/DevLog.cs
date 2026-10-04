using UnityEngine;

/// <summary>
/// Verbose status logs (FPV:, RTT:, Targets:, DEMO:, [ImageSubscriber] cam, [TeleopHud]) that
/// only print while <see cref="DevTools.Enabled"/>; tools/device.sh and the Editor harnesses read them.
/// The line is "category: message", or "category message" for a bracketed category.
/// </summary>
public static class DevLog
{
    public static void Log(string category, string message)
    {
        if (!DevTools.Enabled) return;
        Debug.Log(category.StartsWith("[") ? category + " " + message : category + ": " + message);
    }
}
