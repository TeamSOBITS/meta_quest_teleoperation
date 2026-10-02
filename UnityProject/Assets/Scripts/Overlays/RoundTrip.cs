using System;
using System.Collections.Generic;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Round-trip latency. While control is on, <see cref="QuestControllerPublisher"/> publishes the
/// headset's own TF (child frame <c>headChildFrame</c>: the profile's controllerFrames.hmd) stamped with the wall-clock
/// UTC time; the endpoint echoes it back on /tf. RTT = now (UTC) - stamp, measured when the echo
/// arrives. Only meaningful while the publisher is actually publishing (control on, connected).
/// Created by TeleopHud with the HUD.
/// </summary>
public class RoundTrip : MonoBehaviour
{
    const float RecentSeconds = 2f, MaxWindowSeconds = 1f, LogSeconds = 5f;
    const float EmaAlpha = 2f / (10f + 1f);   // about the last 10 samples
    const double MaxPlausibleMs = 10000.0;

    static ROSConnection _subscribedOn;   // each robot screen has its own ROSConnection
    static RoundTrip Current;   // the live instance; the one /tf subscription forwards to it

    QuestControllerPublisher _publisher;
    float _lastSampleTime = float.NegativeInfinity, _nextLog;
    bool _hasEma;
    readonly Queue<(float time, float ms)> _window = new Queue<(float, float)>();

    // Smoothed round trip (ms), max over the last second, samples since start.
    public float RttMs { get; private set; }
    public float RttMaxMs
    {
        get
        {
            Trim();
            float max = 0f;
            foreach (var s in _window) max = Mathf.Max(max, s.ms);
            return max;
        }
    }
    public int Samples { get; private set; }
    // A sample arrived in the last 2 s.
    public bool HasRecent => Time.unscaledTime - _lastSampleTime <= RecentSeconds;

    public static RoundTrip Create(QuestControllerPublisher publisher)
    {
        var go = new GameObject("Round Trip");
        var rtt = go.AddComponent<RoundTrip>();
        rtt._publisher = publisher;
        rtt._nextLog = Time.unscaledTime + LogSeconds;
        Current = rtt;
        var ros = ROSConnection.GetOrCreateInstance();
        if (_subscribedOn != ros)
        {
            _subscribedOn = ros;   // subscribe once per connection; a new instance must not add callbacks
            ros.Subscribe<TFMessageMsg>(RosNames.Tf, msg => { if (Current != null) Current.OnTf(msg); });
        }
        return rtt;
    }

    void OnDestroy()
    {
        if (Current == this) Current = null;
    }

    void OnTf(TFMessageMsg msg)
    {
        if (this == null || !isActiveAndEnabled) return;
        if (msg == null || msg.transforms == null || _publisher == null) return;
        if (!_publisher.controlRobot || _publisher.HasConnectionError) return;   // nothing of ours is in flight

        foreach (var t in msg.transforms)
        {
            if (t == null || t.header == null || t.header.stamp == null) continue;
            if (t.child_frame_id != _publisher.headChildFrame) continue;

            // Same epoch math as QuestControllerPublisher.GetRosTime.
            long nowNs = (DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).Ticks * 100;
            long stampNs = (long)t.header.stamp.sec * 1_000_000_000L + t.header.stamp.nanosec;
            double ms = (nowNs - stampNs) / 1e6;
            if (ms < -50.0 || ms > MaxPlausibleMs) continue;   // not ours (old session / other clock)
            AddSample((float)Math.Max(ms, 0.0));
        }
    }

    void AddSample(float ms)
    {
        float now = Time.unscaledTime;
        RttMs = _hasEma ? Mathf.Lerp(RttMs, ms, EmaAlpha) : ms;
        _hasEma = true;
        _lastSampleTime = now;
        Samples++;
        _window.Enqueue((now, ms));
        Trim();
    }

    void Trim()
    {
        float now = Time.unscaledTime;
        while (_window.Count > 0 && now - _window.Peek().time > MaxWindowSeconds) _window.Dequeue();
    }

    void Update()
    {
        if (Time.unscaledTime < _nextLog) return;
        _nextLog = Time.unscaledTime + LogSeconds;
        if (HasRecent) DevLog.Log("RTT", $"{RttMs:F0} ms (max {RttMaxMs:F0})");
    }
}
