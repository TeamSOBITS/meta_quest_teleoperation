using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Tells which robots are online for the robot selection screen, from the ROS topic list
/// (asked every few seconds; no image streams are opened).
///
/// A robot is online when its namespace has topics other than the ones this app's own
/// endpoint creates: the endpoint keeps a subscriber on every camera topic ever viewed and a
/// publisher on the Joy topic, so those topics exist even when the robot is off. Anything
/// else under the namespace (joint states, odometry, camera info, ...) comes from the robot.
/// Robots without a namespace cannot be told apart and stay unknown.
///
/// Note: never call ROSConnection.Unsubscribe with TeamSOBITS ros_tcp_endpoint. It has no
/// remove_subscriber command; the request kills its client connection.
///
/// Two ROS connections must never be open at the same time: ROSConnection's reader threads
/// share static read buffers, so overlapping readers corrupt each other's message lengths and
/// allocate gigabytes until the headset kills the app. So this connection opens
/// <see cref="HandoffSeconds"/> after the screen starts (the robot screen's connection has
/// closed by then) and the selection screen waits <see cref="Close"/> before opening a robot.
/// </summary>
public class RobotPresence : MonoBehaviour
{
    const float IntervalSeconds = 4f, ReplySeconds = 3f;
    // Time for a closed connection's reader thread to stop.
    public const float HandoffSeconds = 1f;

    // Robots to check; the selection screen updates this list when robots are added or removed.
    public List<RobotProfile> robots;

    ROSConnection _ros;
    string _ip;
    bool _connected, _closed;
    readonly Dictionary<RobotProfile, bool> _online = new Dictionary<RobotProfile, bool>();

    // true = online, false = offline, null = not known yet or cannot be checked.
    public bool? IsOnline(RobotProfile robot) => _online.TryGetValue(robot, out bool on) ? on : (bool?)null;

    public static bool CanCheck(RobotProfile robot) => !string.IsNullOrEmpty(robot.robotNamespace);

    public static RobotPresence Create(List<RobotProfile> robots, string ip)
    {
        var presence = new GameObject("Robot Presence").AddComponent<RobotPresence>();
        presence.robots = robots;
        presence._ip = ip;
        presence._ros = ROSConnection.GetOrCreateInstance();
        presence._ros.ConnectOnStart = false;
        return presence;
    }

    // A new ROS IP: reconnect and forget the previous results.
    public void SetIp(string ip)
    {
        _ip = ip;
        _online.Clear();
        if (!_connected) return;   // Start connects to the new IP
        _ros.Disconnect();
        _ros.Connect(ip, RosIpSettings.Port);
    }

    // Close the connection and wait until its reader has stopped, before a robot screen connects.
    public IEnumerator Close()
    {
        _closed = true;   // Start may not have run yet (robot opened from an intent in the same frame)
        StopAllCoroutines();
        if (_connected) _ros.Disconnect();
        _connected = false;
        yield return new WaitForSeconds(HandoffSeconds);
    }

    IEnumerator Start()
    {
        yield return new WaitForSeconds(HandoffSeconds);
        if (_closed) yield break;
        _ros.Connect(_ip, RosIpSettings.Port);
        _connected = true;
        while (true)
        {
            Dictionary<string, string> topics = null;
            _ros.GetTopicAndTypeList(t => topics = t);
            float until = Time.time + ReplySeconds;
            while (topics == null && Time.time < until) yield return null;

            foreach (var robot in robots.Where(CanCheck))
                _online[robot] = topics != null && HasRobotTopics(robot, topics);
            yield return new WaitForSeconds(IntervalSeconds);
        }
    }

    static bool HasRobotTopics(RobotProfile robot, Dictionary<string, string> topics)
    {
        string prefix = "/" + robot.robotNamespace.Trim('/') + "/";
        return topics.Any(t => t.Key.StartsWith(prefix)
                               && !CameraDiscovery.IsImage(t.Value)
                               && !t.Value.Replace("/msg/", "/").Equals("sensor_msgs/Joy"));
    }

    // The robot screen opens its own connection; ros_tcp_endpoint serves one client at a time.
    void OnDestroy()
    {
        if (_ros != null && _connected) _ros.Disconnect();
    }
}
