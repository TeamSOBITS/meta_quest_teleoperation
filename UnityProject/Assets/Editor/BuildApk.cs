using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

// Android APK build. Menu "Robots/Build APK" or:
//   Unity -batchmode -nographics -buildTarget Android -projectPath UnityProject -executeMethod BuildApk.Build
// Output: env BUILD_APK, default <repo>/UnityProject/Builds/SOBITS-Quest-Teleoperation-<bundleVersion>.apk.
// productName / companyName are set for the build and restored afterwards (the verify copy keeps its own
// "...VerifyCopy" name, which isolates the Editor PlayerPrefs).
public static class BuildApk
{
    const string ProductName = "SOBITS Quest Teleoperation";
    const string CompanyName = "SOBITS";

    [MenuItem("Robots/Build APK")]
    public static void Build()
    {
        string oldProduct = PlayerSettings.productName, oldCompany = PlayerSettings.companyName;
        int exit = 0;
        try
        {
            PlayerSettings.productName = ProductName;
            PlayerSettings.companyName = CompanyName;
            string outPath = Environment.GetEnvironmentVariable("BUILD_APK");
            if (string.IsNullOrEmpty(outPath))
                outPath = Path.GetFullPath(Path.Combine(Application.dataPath, "../Builds",
                    $"SOBITS-Quest-Teleoperation-{PlayerSettings.bundleVersion}.apk"));
            Directory.CreateDirectory(Path.GetDirectoryName(outPath));
            var scenes = EditorBuildSettings.scenes.Where(s => s.enabled).Select(s => s.path).ToArray();
            Debug.Log("[Build] scenes: " + string.Join(", ", scenes) + " -> " + outPath);
            var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
            {
                scenes = scenes, locationPathName = outPath, target = BuildTarget.Android, options = BuildOptions.None,
            });
            var s = report.summary;
            Debug.Log($"[Build] result={s.result} errors={s.totalErrors} warnings={s.totalWarnings} size={s.totalSize} time={s.totalTime}");
            foreach (var step in report.steps)
                foreach (var m in step.messages)
                    if (m.type == LogType.Error || m.type == LogType.Exception) Debug.Log("[Build] ERROR " + m.content);
            if (s.result != BuildResult.Succeeded) exit = 1;
        }
        catch (Exception e) { Debug.LogException(e); exit = 1; }
        finally
        {
            PlayerSettings.productName = oldProduct;
            PlayerSettings.companyName = oldCompany;
        }
        if (Application.isBatchMode) EditorApplication.Exit(exit);
        else if (exit != 0) Debug.LogError("[Build] failed");
    }
}
