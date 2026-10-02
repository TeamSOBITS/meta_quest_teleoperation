using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

/// <summary>
/// Everything the app remembers between runs, typed, in one place (the only user of PlayerPrefs
/// besides the IP wrapper). Keys: "v2/{name}" for global values, "v2/robot/{robot}/{name}" per
/// robot and "v2/robot/{robot}/cam/{topicSuffix with '/' as '_'}/{name}" per camera. Values saved
/// by older versions are copied over once (<see cref="EnsureMigrated"/>); the old keys are kept.
/// </summary>
public static class Settings
{
    public const string DefaultRosIp = "127.0.0.1";
    public const int CurrentVersion = 2;

    const string Prefix = "v2/";

    // Global values.
    public static string RosIp
    {
        get { string ip = PlayerPrefs.GetString(Prefix + "RosIp", ""); return ip.Length > 0 ? ip : DefaultRosIp; }
        set => PlayerPrefs.SetString(Prefix + "RosIp", value ?? "");
    }
    public static bool HasRosIp => PlayerPrefs.GetString(Prefix + "RosIp", "").Length > 0;
    // Name of the robot opened last ("" = none).
    public static string LastRobot
    {
        get => PlayerPrefs.GetString(Prefix + "LastRobot", "");
        set { if (string.IsNullOrEmpty(value)) PlayerPrefs.DeleteKey(Prefix + "LastRobot"); else PlayerPrefs.SetString(Prefix + "LastRobot", value); }
    }
    public static bool Passthrough { get => GetBool("Passthrough", false); set => SetBool("Passthrough", value); }
    public static bool LazyFollow { get => GetBool("LazyFollow", false); set => SetBool("LazyFollow", value); }
    // Compressed image topics (default on); off = the raw twin of each topic.
    public static bool Compressed { get => GetBool("Compressed", true); set => SetBool("Compressed", value); }
    // Set by a launch extra for one test run; cleared when the capture is done.
    public static bool DebugCapture
    {
        get => GetBool("DebugCapture", false);
        set { if (value) SetBool("DebugCapture", true); else PlayerPrefs.DeleteKey(Prefix + "DebugCapture"); }
    }

    // Developer tools (verbose logs, launch extras) in release builds; see DevTools.
    public static bool DevTools { get => GetBool("DevTools", false); set => SetBool("DevTools", value); }

    public static int Version => PlayerPrefs.GetInt(Prefix + "Version", 0);

    // Flush now; the app may be killed from the Quest menu without a clean quit.
    public static void Save() => PlayerPrefs.Save();

    static bool GetBool(string name, bool def) => PlayerPrefs.GetInt(Prefix + name, def ? 1 : 0) == 1;
    static void SetBool(string name, bool value) => PlayerPrefs.SetInt(Prefix + name, value ? 1 : 0);

    public static RobotSettings For(RobotProfile robot) => new RobotSettings(robot);

    // ---- Migration from the pre-v2 keys ----

    // Globals do not need the robot list: copied before any scene reads them (the ROS IP is read in Awake).
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    static void MigrateGlobalsAtStart() => MigrateGlobals();

    static int MigrateGlobals()
    {
        int n = 0;
        n += CopyString("RosIPAddress", Prefix + "RosIp");
        n += CopyString("LastRobot", Prefix + "LastRobot");
        n += CopyInt("Passthrough", Prefix + "Passthrough");
        n += CopyInt("LazyFollow", Prefix + "LazyFollow");
        n += CopyInt("Images/Compressed", Prefix + "Compressed");
        n += CopyInt("DebugCapture", Prefix + "DebugCapture");
        return n;
    }

    // Copies every old key to the new scheme where the new one is absent (the old ones stay, for
    // one release). Per robot it runs once; running it again, or for more robots later, changes nothing else.
    public static void EnsureMigrated(IEnumerable<RobotProfile> robots)
    {
        int n = MigrateGlobals();
        int robotCount = 0;
        foreach (var robot in robots)
        {
            if (robot == null || PlayerPrefs.HasKey(new RobotSettings(robot).MigratedKey)) continue;
            n += new RobotSettings(robot).MigrateOld();
            robotCount++;
        }
        if (Version < CurrentVersion)
        {
            PlayerPrefs.SetInt(Prefix + "Version", CurrentVersion);
            n++;
        }
        if (n == 0) return;
        PlayerPrefs.Save();
        Debug.Log($"[Settings] migrated {n} values ({robotCount} robots) to version {CurrentVersion}");
    }

    public static void EnsureMigrated(RobotProfile robot) => EnsureMigrated(new[] { robot });

    internal static int CopyString(string oldKey, string newKey)
    {
        if (!PlayerPrefs.HasKey(oldKey) || PlayerPrefs.HasKey(newKey)) return 0;
        PlayerPrefs.SetString(newKey, PlayerPrefs.GetString(oldKey, ""));
        return 1;
    }

    internal static int CopyInt(string oldKey, string newKey)
    {
        if (!PlayerPrefs.HasKey(oldKey) || PlayerPrefs.HasKey(newKey)) return 0;
        PlayerPrefs.SetInt(newKey, PlayerPrefs.GetInt(oldKey, 0));
        return 1;
    }
}

/// <summary>What is remembered about one robot: model on/off, camera layout, and its cameras.</summary>
public class RobotSettings
{
    readonly RobotProfile _robot;
    readonly string _prefix;

    internal RobotSettings(RobotProfile robot)
    {
        _robot = robot;
        _prefix = $"v2/robot/{robot.name}/";
    }

    // Marks the robot's old keys as handled (also set by Forget, so a removed robot never comes back).
    internal string MigratedKey => _prefix + "migrated";

    // The robot model is shown (default off).
    public bool ModelOn
    {
        get => PlayerPrefs.GetInt(_prefix + "ModelOn", 0) == 1;
        set => PlayerPrefs.SetInt(_prefix + "ModelOn", value ? 1 : 0);
    }
    public bool HasModelOn => PlayerPrefs.HasKey(_prefix + "ModelOn");

    // Camera layout, "blocks" (default) or "firstperson".
    public string Layout
    {
        get => PlayerPrefs.GetString(_prefix + "Layout", FirstPersonView.LayoutBlocks);
        set => PlayerPrefs.SetString(_prefix + "Layout", value);
    }
    public bool HasLayout => PlayerPrefs.HasKey(_prefix + "Layout");

    public CameraSettings Camera(RobotProfile.CameraConfig cam) => new CameraSettings(_prefix + "cam/" + Sanitize(cam.topicSuffix) + "/");

    static string Sanitize(string topicSuffix) => (topicSuffix ?? "").Replace('/', '_');

    // True once the user has dragged any block of this robot.
    public bool HasAnyPosition
    {
        get
        {
            foreach (var cam in _robot.cameras) if (Camera(cam).Position != null) return true;
            return false;
        }
    }

    // Remove everything saved for this robot (e.g. when it is removed): model, layout, every camera.
    public void Forget()
    {
        PlayerPrefs.DeleteKey(_prefix + "ModelOn");
        PlayerPrefs.DeleteKey(_prefix + "Layout");
        foreach (var cam in _robot.cameras) Camera(cam).Clear();
        PlayerPrefs.SetInt(MigratedKey, 1);
        PlayerPrefs.Save();
    }

    internal int MigrateOld()
    {
        int n = 0;
        // The old single "ViewMode" pref: first person meant model + first-person layout.
        if (PlayerPrefs.GetString($"ViewMode/{_robot.name}", "") == FirstPersonView.LayoutFirstPerson)
        {
            if (!HasModelOn) { ModelOn = true; n++; }
            if (!HasLayout) { Layout = FirstPersonView.LayoutFirstPerson; n++; }
        }
        n += Settings.CopyInt($"RobotModel/{_robot.name}", _prefix + "ModelOn");
        n += Settings.CopyString($"CameraLayout/{_robot.name}", _prefix + "Layout");
        foreach (var cam in _robot.cameras)
        {
            var c = Camera(cam);
            string old = $"/{_robot.name}/{cam.displayName}";
            n += Settings.CopyString("PanelPosition" + old, c.PositionKey);
            n += Settings.CopyInt("PanelVisible" + old, c.VisibleKey);
            n += Settings.CopyString("CameraLabel" + old, c.LabelKey);
        }
        PlayerPrefs.SetInt(MigratedKey, 1);
        return n;
    }
}

/// <summary>What is remembered about one camera of a robot: block position and size, shown/hidden, own name.</summary>
public class CameraSettings
{
    readonly string _prefix;
    internal CameraSettings(string prefix) => _prefix = prefix;

    internal string PositionKey => _prefix + "Position";
    internal string VisibleKey => _prefix + "Visible";
    internal string LabelKey => _prefix + "Label";

    // Where the user dragged the block, relative to the head ("x;y;z;size"); null = automatic layout.
    public Vector3? Position => Parse(out _);
    // Block size; null for older saves (x;y;z) and when nothing is saved.
    public float? Size { get { Parse(out float? size); return size; } }

    Vector3? Parse(out float? size)
    {
        size = null;
        var parts = PlayerPrefs.GetString(PositionKey, "").Split(';');
        if (parts.Length >= 4 && float.TryParse(parts[3], NumberStyles.Float, CultureInfo.InvariantCulture, out float s)) size = s;
        if (parts.Length >= 3 &&
            float.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out float x) &&
            float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float y) &&
            float.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float z))
            return new Vector3(x, y, z);
        return null;
    }

    public void SetPlacement(Vector3 position, float size) =>
        PlayerPrefs.SetString(PositionKey,
            string.Format(CultureInfo.InvariantCulture, "{0};{1};{2};{3}", position.x, position.y, position.z, size));

    public bool Visible
    {
        get => PlayerPrefs.GetInt(VisibleKey, 1) == 1;
        set => PlayerPrefs.SetInt(VisibleKey, value ? 1 : 0);
    }

    // The user's own name for the camera; "" = the original one.
    public string Label
    {
        get => PlayerPrefs.GetString(LabelKey, "");
        set { if (string.IsNullOrEmpty(value)) PlayerPrefs.DeleteKey(LabelKey); else PlayerPrefs.SetString(LabelKey, value); }
    }

    // Back to the default layout: shown, automatic position (the name is kept).
    public void ResetLayout()
    {
        PlayerPrefs.DeleteKey(PositionKey);
        PlayerPrefs.DeleteKey(VisibleKey);
    }

    public void Clear()
    {
        ResetLayout();
        PlayerPrefs.DeleteKey(LabelKey);
    }
}
