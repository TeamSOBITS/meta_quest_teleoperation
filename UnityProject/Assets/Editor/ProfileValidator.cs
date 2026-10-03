#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Checks the robot profile assets (Assets/Robots/*.asset) against their model prefabs: every model frame a profile names
/// (head, lift, camera mounts, arm effectors) must be a RobotLink of the prefab, exactly one camera is the first-person
/// camera and it has a camera_info topic. Arm target frames are runtime TF frames (not URDF links) and are not checked.
/// Run: menu "Robots/Validate profiles" or Unity -batchmode -projectPath P -executeMethod ProfileValidator.Run
/// (exit code 1 on failure; each check logs "[Verify] PASS/FAIL" for tools/verify.sh).
/// </summary>
public static class ProfileValidator
{
    // Model frames the profile names (empty names are features that are off).
    public static IEnumerable<string> ModelFrames(RobotProfile p)
    {
        var names = new List<string> { p.head.panFrame, p.head.tiltFrame, p.lift.frame };
        foreach (var c in p.cameras) names.Add(c.mountFrame);
        foreach (var a in p.arms) names.Add(a.effectorFrame);
        return names.Where(n => !string.IsNullOrEmpty(n)).Distinct();
    }

    // The profile's frame names that are not among `links`.
    public static List<string> MissingFrames(RobotProfile p, ICollection<string> links)
        => ModelFrames(p).Where(n => !links.Contains(n)).ToList();

    [MenuItem("Robots/Validate profiles")]
    public static void Run()
    {
        int fails = 0;
        void Check(bool ok, string what) { Debug.Log((ok ? "[Verify] PASS " : "[Verify] FAIL ") + what); if (!ok) fails++; }
        try
        {
            var guids = AssetDatabase.FindAssets("t:RobotProfile", new[] { "Assets/Robots" });
            Check(guids.Length >= 2, $"found {guids.Length} profile assets");
            foreach (var guid in guids)
            {
                string path = AssetDatabase.GUIDToAssetPath(guid);
                var p = AssetDatabase.LoadAssetAtPath<RobotProfile>(path);
                string n = p.name;
                Check(p.HasModel, $"{n}: has a model prefab");
                if (!p.HasModel) continue;
                var links = new HashSet<string>(p.modelPrefab.GetComponentsInChildren<RobotLink>(true).Select(l => l.frame));
                var missing = MissingFrames(p, links);
                Check(missing.Count == 0, $"{n}: every named frame is a link of the prefab" + (missing.Count > 0 ? " (missing: " + string.Join(", ", missing) + ")" : ""));
                int fp = p.cameras.Count(c => c.firstPerson);
                Check(fp == 1, $"{n}: exactly one firstPerson camera ({fp})");
                var cam = p.FirstPersonCamera;
                Check(cam != null && !string.IsNullOrEmpty(cam.cameraInfoSuffix), $"{n}: first-person camera has a cameraInfoSuffix ('{cam?.cameraInfoSuffix}')");
                Check(cam != null && !string.IsNullOrEmpty(cam.mountFrame), $"{n}: first-person camera has a mountFrame");
                Check(!string.IsNullOrEmpty(p.baseFrame) && !string.IsNullOrEmpty(p.controllerFrames.hmd)
                      && !string.IsNullOrEmpty(p.controllerFrames.left) && !string.IsNullOrEmpty(p.controllerFrames.right), $"{n}: baseFrame and controllerFrames set");
                bool head = !string.IsNullOrEmpty(p.head.panFrame) || !string.IsNullOrEmpty(p.head.tiltFrame);
                Check(!head || (p.head.panLimitRad > 0f && p.head.tiltMaxRad > 0f && p.head.tiltMinRad < 0f), $"{n}: head limits filled ({p.head.panLimitRad:F3}, {p.head.tiltMinRad:F3}..{p.head.tiltMaxRad:F3})");
                Check(string.IsNullOrEmpty(p.lift.frame) || p.lift.rangeM > 0f, $"{n}: lift range filled ({p.lift.rangeM:F2})");
                Check(p.arms.All(a => !string.IsNullOrEmpty(a.targetFrame) && !string.IsNullOrEmpty(a.effectorFrame)), $"{n}: every arm names a target and an effector frame");
            }
        }
        catch (Exception e) { Debug.LogException(e); fails++; }
        Debug.Log(fails == 0 ? "[Verify] ProfileValidator: OK" : $"[Verify] ProfileValidator: {fails} failed");
        if (Application.isBatchMode) EditorApplication.Exit(fails == 0 ? 0 : 1);
    }
}
#endif
