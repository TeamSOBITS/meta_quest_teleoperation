using System.Threading.Tasks;
using UnityEngine;

/// <summary>
/// The ROS PC's IP as typed by the user, shared by the robot selection screen and the robot
/// screens. Saved in PlayerPrefs so it survives app restarts.
/// </summary>
public static class RosIpSettings
{
    const string PrefsKey = "RosIPAddress";
    public const int Port = 10000;

    // Saved IP, or `fallback` when none has been typed yet.
    public static string Load(string fallback)
    {
        string ip = PlayerPrefs.GetString(PrefsKey, "");
        return string.IsNullOrEmpty(ip) ? fallback : ip;
    }

    public static bool HasSaved => !string.IsNullOrEmpty(PlayerPrefs.GetString(PrefsKey, ""));

    public static void Save(string ip)
    {
        PlayerPrefs.SetString(PrefsKey, ip);
        PlayerPrefs.Save();  // flush now; the app may be killed from the Quest menu without a clean quit
    }

    // True if the PC at `ip` answers a ping within `timeoutSeconds`.
    // Deliberately not a TCP connection to the ROS endpoint: ros_tcp_endpoint hands its single
    // outgoing (ROS -> Unity) stream to the newest connection, so even a brief probe would cut
    // camera images off from any other Unity client connected to the same endpoint.
    public static async Task<bool> PingAsync(string ip, float timeoutSeconds = 1.5f)
    {
        Ping ping;
        try { ping = new Ping(ip); }
        catch { return false; }

        float deadline = Time.realtimeSinceStartup + timeoutSeconds;
        while (!ping.isDone && Time.realtimeSinceStartup < deadline)
            await Task.Yield();
        bool ok = ping.isDone && ping.time >= 0;
        ping.DestroyPing();
        return ok;
    }
}

/// <summary>
/// Quest system keyboard for typing an IP. Call <see cref="Poll"/> every frame; it returns
/// the new IP once the user confirms a changed, non-empty value, otherwise null.
/// </summary>
public class IpKeyboard
{
    TouchScreenKeyboard _keyboard;
    string _current;

    public bool IsOpen => _keyboard != null;
    // What the user is typing while the keyboard is open.
    public string Text => _keyboard?.text;

    public void Open(string current)
    {
        _current = current;
        TouchScreenKeyboard.hideInput = false;
        _keyboard = TouchScreenKeyboard.Open(current,
            TouchScreenKeyboardType.NumbersAndPunctuation, false, false, false, false);
    }

    public string Poll()
    {
        if (_keyboard == null) return null;
        if (_keyboard.status == TouchScreenKeyboard.Status.Visible) return null;

        string typed = _keyboard.text;
        bool confirmed = _keyboard.status == TouchScreenKeyboard.Status.Done;
        _keyboard = null;
        return confirmed && !string.IsNullOrEmpty(typed) && typed != _current ? typed : null;
    }
}
