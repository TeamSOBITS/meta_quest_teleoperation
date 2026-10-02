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
    public static string Tool(string name) => Path.Combine(ToolsDir, name);
}
