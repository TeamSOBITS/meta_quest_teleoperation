using System;
using System.IO;
using UnityEngine;

// Locates the helper scripts the live suites call (pub.sh, cmdvel.sh, home.sh, targets.sh, teleport.sh).
// Directory: env TELEOP_TOOLS, default <repo>/tools/sim (the project sits in <repo>/UnityProject).
public static class VerifyPaths
{
    public static string ToolsDir
    {
        get
        {
            var d = Environment.GetEnvironmentVariable("TELEOP_TOOLS");
            return string.IsNullOrEmpty(d) ? Path.GetFullPath(Path.Combine(Application.dataPath, "../../tools/sim")) : d;
        }
    }
    // <repo>/tools/models/<robot>/<rel>: builder inputs (the project sits in <repo>/UnityProject; verify.sh mirrors tools/models next to the copy).
    public static string ModelInput(string robot, string rel) =>
        Path.GetFullPath(Path.Combine(Application.dataPath, "../../tools/models", robot, rel));
    public static string Tool(string name) => Path.Combine(ToolsDir, name);
}
