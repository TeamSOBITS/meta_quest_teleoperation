using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEngine;

/// <summary>
/// Robots added on the headset ("Add robot"), stored as one JSON file each under
/// persistentDataPath/robots and turned into RobotProfile instances at runtime.
/// Built-in robots stay project assets (Assets/Robots) and are not managed here.
/// </summary>
public static class RobotLibrary
{
    public const string TeleopSceneName = "TeleopScene";
    const string IdPrefix = "custom_";

    static string Folder => Path.Combine(Application.persistentDataPath, "robots");
    static string FileFor(string id) => Path.Combine(Folder, id + ".json");

    [Serializable]
    class CameraData
    {
        public string displayName;
        public string topic;
        public bool raw;
        public Vector2Int resolution;
        public float maxFps = 15f;
        public float scale = 1f;
        public bool flipVertical, flipHorizontal;
    }

    [Serializable]
    class RobotData
    {
        public string displayName;
        public string robotNamespace;
        public List<CameraData> cameras = new List<CameraData>();
    }

    public static List<RobotProfile> LoadAll()
    {
        var robots = new List<RobotProfile>();
        if (!Directory.Exists(Folder)) return robots;
        foreach (var file in Directory.GetFiles(Folder, IdPrefix + "*.json").OrderBy(f => f))
        {
            try
            {
                var data = JsonUtility.FromJson<RobotData>(File.ReadAllText(file));
                if (data != null) robots.Add(ToProfile(Path.GetFileNameWithoutExtension(file), data));
            }
            catch (Exception e)
            {
                Debug.LogWarning($"RobotLibrary: skipping {file}: {e.Message}");
            }
        }
        return robots;
    }

    // A new, not yet saved robot with no cameras; setup mode discovers them.
    public static RobotProfile CreateNew(string displayName)
        => ToProfile(IdFor(displayName), new RobotData { displayName = displayName.Trim() });

    public static void Save(RobotProfile robot)
    {
        var data = new RobotData
        {
            displayName = robot.displayName,
            robotNamespace = robot.robotNamespace,
            cameras = robot.cameras.Select(c => new CameraData
            {
                displayName = c.displayName,
                topic = robot.FullTopic(c),
                raw = c.raw,
                resolution = c.resolution,
                maxFps = c.maxFps,
                scale = c.scale,
                flipVertical = c.flipVertical,
                flipHorizontal = c.flipHorizontal,
            }).ToList(),
        };
        Directory.CreateDirectory(Folder);
        File.WriteAllText(FileFor(robot.name), JsonUtility.ToJson(data, true));
    }

    public static void Delete(RobotProfile robot)
    {
        if (!robot.isCustom) return;
        if (File.Exists(FileFor(robot.name))) File.Delete(FileFor(robot.name));
        ImageSubscriber.ForgetLayout(robot);
    }

    // Names are compared case-insensitively against every robot shown on the selection screen.
    public static bool IsNameTaken(string displayName, IEnumerable<RobotProfile> robots)
    {
        string id = IdFor(displayName);
        return robots.Any(r => string.Equals(r.displayName.Trim(), displayName.Trim(), StringComparison.OrdinalIgnoreCase)
                               || r.name == id);
    }

    static string IdFor(string displayName)
        => IdPrefix + Regex.Replace(displayName.Trim().ToLowerInvariant(), "[^a-z0-9]+", "_").Trim('_');

    static RobotProfile ToProfile(string id, RobotData data)
    {
        var robot = ScriptableObject.CreateInstance<RobotProfile>();
        robot.name = id;
        robot.isCustom = true;
        robot.displayName = data.displayName;
        robot.robotNamespace = data.robotNamespace ?? "";
        robot.sceneName = TeleopSceneName;
        robot.cameras = data.cameras.Select(c => new RobotProfile.CameraConfig
        {
            displayName = c.displayName,
            topicSuffix = c.topic,
            raw = c.raw,
            resolution = c.resolution.x > 0 ? c.resolution : new Vector2Int(640, 480),
            maxFps = c.maxFps,
            scale = c.scale <= 0f ? 1f : c.scale,
            flipVertical = c.flipVertical,
            flipHorizontal = c.flipHorizontal,
        }).ToArray();
        return robot;
    }
}
