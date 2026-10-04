// Generates the C# classes of the sobits_interfaces messages this app uses into Assets/RosMessages/.
// Menu: Robots > Generate sobits_interfaces messages. Batch: tools/gen_msgs.sh [SOBITS_INTERFACES_DIR]
// (= -executeMethod RosMessageGen.Run). Input dir: env SOBITS_INTERFACES_DIR, else the sibling docker workspace.
using System;
using System.IO;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEditor;
using UnityEngine;

public static class RosMessageGen
{
    const string Package = "sobits_interfaces";
    static readonly string[] Messages = { "VlaRecordStatus" };

    static string InputDir()
    {
        string env = Environment.GetEnvironmentVariable("SOBITS_INTERFACES_DIR");
        if (!string.IsNullOrEmpty(env)) return env;
        string repo = Path.GetFullPath(Path.Combine(Application.dataPath, "..", ".."));
        string rel = Path.Combine("docker_containers", "jazzy_sobit_sciurus_kachaka_ws", "src", "sobits_interfaces");
        // The workspace sits beside the repo or in the home folder (the repo itself is often in ~/Documents).
        foreach (var root in new[] { Path.Combine(repo, ".."), Path.Combine(repo, "..", ".."), Environment.GetFolderPath(Environment.SpecialFolder.Personal) })
        {
            string dir = Path.GetFullPath(Path.Combine(root, rel));
            if (Directory.Exists(dir)) return dir;
        }
        return Path.GetFullPath(Path.Combine(repo, "..", rel));
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
