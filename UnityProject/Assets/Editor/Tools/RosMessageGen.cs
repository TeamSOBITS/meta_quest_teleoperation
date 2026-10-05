// Generates the C# classes of the sobits_interfaces messages this app uses into Assets/RosMessages/.
// Menu: Robots > Generate sobits_interfaces messages. Batch: tools/gen_msgs.sh [SOBITS_INTERFACES_DIR]
// (= -executeMethod RosMessageGen.Run). Input dir: env SOBITS_INTERFACES_DIR, else a docker_containers/*/src/sobits_interfaces.
using System;
using System.IO;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEditor;
using UnityEngine;

public static class RosMessageGen
{
    const string Package = "sobits_interfaces";
    static readonly string[] Messages = { "VlaStatus" };

    static string InputDir()
    {
        string env = Environment.GetEnvironmentVariable("SOBITS_INTERFACES_DIR");
        if (!string.IsNullOrEmpty(env)) return env;
        string repo = Path.GetFullPath(Path.Combine(Application.dataPath, "..", ".."));
        string src = ContainerSrc(repo);   // the ROS container the tools drive (tools/ros_container.sh)
        if (src != null && Directory.Exists(Path.Combine(src, Package))) return Path.Combine(src, Package);
        // No docker: any docker_containers/<workspace>/src/sobits_interfaces beside the repo, one level up or in the
        // home folder; the most recently changed wins.
        string best = null;
        var newest = DateTime.MinValue;
        foreach (var root in new[] { Path.Combine(repo, ".."), Path.Combine(repo, "..", ".."), Environment.GetFolderPath(Environment.SpecialFolder.Personal) })
        {
            string containers = Path.GetFullPath(Path.Combine(root, "docker_containers"));
            if (!Directory.Exists(containers)) continue;
            foreach (var ws in Directory.GetDirectories(containers))
            {
                string dir = Path.Combine(ws, "src", Package);
                string msg = Path.Combine(dir, "msg");
                if (!Directory.Exists(msg)) continue;
                var t = Directory.GetLastWriteTime(msg);
                if (t > newest) { newest = t; best = dir; }
            }
        }
        return best ?? Path.GetFullPath(Path.Combine(repo, "..", "docker_containers", "<workspace>", "src", Package));
    }

    static string ContainerSrc(string repo)
    {
        try
        {
            var p = System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo("bash", "tools/ros_container.sh --src")
            {
                WorkingDirectory = repo, RedirectStandardOutput = true, RedirectStandardError = true, UseShellExecute = false,
            });
            string output = p.StandardOutput.ReadToEnd().Trim();
            p.WaitForExit(10000);
            return p.ExitCode == 0 && output.Length > 0 ? output : null;
        }
        catch (Exception) { return null; }
    }

    [MenuItem("Robots/Generate sobits_interfaces messages")]
    public static void Run()
    {
        bool ok = true;
        try
        {
            string dir = InputDir();
            string output = Path.Combine(Application.dataPath, "RosMessages");
            Debug.Log($"[RosMessageGen] input {dir} -> {output}");
            foreach (var name in Messages)
            {
                string msg = Path.Combine(dir, "msg", name + ".msg");
                if (!File.Exists(msg)) { Debug.LogError($"[RosMessageGen] missing {msg}"); ok = false; continue; }
                foreach (var w in MessageAutoGen.GenerateSingleMessage(msg, output, Package))
                    Debug.LogWarning("[RosMessageGen] " + w);
                Debug.Log($"[RosMessageGen] generated {name}");
            }
        }
        catch (Exception e) { Debug.LogException(e); ok = false; }
        AssetDatabase.Refresh();
        Debug.Log(ok ? "[Verify] PASS RosMessageGen" : "[Verify] FAIL RosMessageGen");
        if (Application.isBatchMode) EditorApplication.Exit(ok ? 0 : 1);
    }
}
