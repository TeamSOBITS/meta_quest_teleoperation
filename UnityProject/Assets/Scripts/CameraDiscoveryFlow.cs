using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// <summary>
/// Setup-mode coroutines of <see cref="ImageSubscriber"/> (run by it): discover the camera topics
/// of a robot being added from the endpoint's topic list, check which of them publish, turn them
/// into cameras (<see cref="CameraDiscovery"/> does the topic analysis), and later "Find cameras".
/// </summary>
public class CameraDiscoveryFlow
{
    readonly ImageSubscriber _owner;

    // ROS topic list as reported by the endpoint (topic -> type), filled once discovery starts.
    readonly Dictionary<string, string> _topics = new Dictionary<string, string>();
    bool _listening, _searchNow;

    const float DiscoveryWaitSeconds = 2.5f;
    // How long a candidate camera topic may take to deliver a frame before it counts as idle.
    const float LiveCheckSeconds = 3f;

    public CameraDiscoveryFlow(ImageSubscriber owner) => _owner = owner;

    ROSConnection Ros => _owner.ros;
    RobotProfile Profile => _owner.Profile;

    // Keep the endpoint's topic list in _topics and ask for a fresh copy.
    void RequestTopics()
    {
        if (!_listening)
        {
            Ros.ListenForTopics(t => _topics[t.Topic] = t.RosMessageName, notifyAllExistingTopics: true);
            _listening = true;
        }
        Ros.RefreshTopicsList();
    }

    List<(string, string)> TopicList() => _topics.Select(kv => (kv.Key, kv.Value)).ToList();

    // Wait for the topic list, or less if SearchAgain() was pressed.
    IEnumerator WaitForTopics()
    {
        _searchNow = false;
        float until = Time.time + DiscoveryWaitSeconds;
        while (Time.time < until && !_searchNow) yield return null;
    }

    // Setup mode: ask the ROS endpoint for its topic list until publishing cameras show up
    // (or the user continues without cameras).
    public IEnumerator Discover()
    {
        while (!_owner.IsReady)
        {
            _owner.SetupStatus = "Looking for camera topics…";
            RequestTopics();
            yield return WaitForTopics();
            if (_owner.IsReady) yield break;

            var live = new List<(string, string)>();
            _owner.SetupStatus = "Checking which cameras are publishing…";
            yield return LiveTopics(TopicList(), new HashSet<string>(), live);
            if (_owner.IsReady || ConfigureDiscovered(live)) yield break;
            _owner.SetupStatus = $"No publishing cameras found on {Ros.RosIPAddress} yet. Searching again every few seconds.";
        }
    }

    // Of the camera candidates in `topics` (except `skip`), add to `result` those that deliver a
    // frame within LiveCheckSeconds. The check subscriptions are kept: TeamSOBITS ros_tcp_endpoint
    // has no remove_subscriber command, and ROSConnection.Unsubscribe kills its client connection
    // (the connection resets and the stream to the headset breaks). Live ones become blocks anyway.
    IEnumerator LiveTopics(List<(string, string)> topics, HashSet<string> skip, List<(string, string)> result)
    {
        var candidates = topics.Where(t => CameraDiscovery.IsCameraCandidate(t.Item1, t.Item2) && !skip.Contains(t.Item1)).ToList();
        if (!ImageSubscriber.RequireLiveTopics) { result.AddRange(candidates); yield break; }

        var alive = new HashSet<string>();
        foreach (var (topic, type) in candidates)
        {
            string t = topic;
            if (CameraDiscovery.IsCompressed(type)) Ros.Subscribe<CompressedImageMsg>(t, _ => alive.Add(t));
            else Ros.Subscribe<ImageMsg>(t, _ => alive.Add(t));
        }
        float until = Time.time + LiveCheckSeconds;
        while (Time.time < until && alive.Count < candidates.Count) yield return null;

        result.AddRange(candidates.Where(c => alive.Contains(c.Item1)));
    }

    // Setup mode: search immediately instead of waiting for the next retry.
    public void SearchAgain()
    {
        _owner.SetupStatus = "Looking for camera topics…";
        _searchNow = true;
    }

    // Use the camera topics found in `topics` (topic, ROS type) for the robot being set up.
    // Returns false if there are none yet.
    public bool ConfigureDiscovered(IEnumerable<(string topic, string type)> topics)
    {
        if (_owner.IsReady) return true;
        var found = CameraDiscovery.Select(topics.ToList());
        if (found.Count == 0) return false;

        Profile.robotNamespace = CameraDiscovery.CommonNamespace(found.Select(f => f.topic));
        _owner.NotifyNamespace(Profile.robotNamespace);
        Profile.cameras = found.Select(ToConfig).ToArray();
        _owner.SetupStatus = $"{found.Count} camera{(found.Count == 1 ? "" : "s")} found";
        _owner.BuildPanels();
        return true;
    }

    // Setup mode: keep the robot without cameras (e.g. to drive it with the controllers only).
    // The Joy namespace then comes from the robot's other topics, if they share one.
    public void ContinueWithoutCameras()
    {
        if (_owner.IsReady) return;
        Profile.robotNamespace = CameraDiscovery.RobotNamespace(_topics.Keys);
        _owner.NotifyNamespace(Profile.robotNamespace);
        Profile.cameras = Array.Empty<RobotProfile.CameraConfig>();
        _owner.SetupStatus = "No cameras";
        _owner.BuildPanels();
    }

    // "Find cameras": search the topic list again and add cameras this robot does not have yet.
    // Existing blocks keep their place; `done` receives how many cameras were added.
    public IEnumerator FindNewCameras(Action<int> done)
    {
        RequestTopics();
        yield return WaitForTopics();

        // Known cameras, including the raw/compressed twin of each, so a camera is never added twice.
        var known = new HashSet<string>();
        foreach (var t in Profile.cameras.Select(c => Profile.FullTopic(c)))
        {
            known.Add(t);
            known.Add(t.EndsWith(RosNames.CompressedSuffix) ? t.Substring(0, t.Length - RosNames.CompressedSuffix.Length) : t + RosNames.CompressedSuffix);
        }
        var live = new List<(string, string)>();
        yield return LiveTopics(TopicList(), known, live);
        var fresh = CameraDiscovery.Select(live).Where(f => !known.Contains(f.topic)).ToList();
        if (fresh.Count > 0)
        {
            var names = new HashSet<string>(Profile.cameras.Select(c => c.displayName));
            var added = fresh.Select(ToConfig).ToList();
            foreach (var c in added)
                while (names.Contains(c.displayName)) c.displayName += " 2";   // keep layout keys unique
            if (string.IsNullOrEmpty(Profile.robotNamespace) && Profile.cameras.Length == 0)
            {
                Profile.robotNamespace = CameraDiscovery.CommonNamespace(fresh.Select(f => f.topic));
                _owner.NotifyNamespace(Profile.robotNamespace);
            }
            Profile.cameras = Profile.cameras.Concat(added).ToArray();
            _owner.AddCameras(added);
        }
        done?.Invoke(fresh.Count);
    }

    static RobotProfile.CameraConfig ToConfig(CameraDiscovery.Found f) => new RobotProfile.CameraConfig
    {
        displayName = f.displayName,
        topicSuffix = f.topic,
        raw = f.raw,
        resolution = new Vector2Int(640, 480),   // corrected from the first frame
    };
}
