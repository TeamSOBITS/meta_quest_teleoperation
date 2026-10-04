/// <summary>
/// Developer switch. On in development builds and the Editor, or in a release build through the
/// "DevTools" preference or a launch extra (`--es dev 1`, this session only). Off in a normal
/// release build: the verbose status logs (<see cref="DevLog"/>) and the launch extras
/// (<see cref="DebugLaunchOptions"/>) stay inactive. Errors and warnings are logged regardless.
/// </summary>
public static class DevTools
{
    // Set for this run by the `dev` launch extra; never saved.
    public static bool Session;

    public static bool Enabled => UnityEngine.Debug.isDebugBuild || Session || Settings.DevTools;
}
