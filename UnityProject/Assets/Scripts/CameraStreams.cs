using System;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Sensor;

/// <summary>
/// The image side of <see cref="ImageSubscriber"/>: subscribes the topic of each camera (compressed,
/// or the raw twin when "Compressed images" is off), keeps the newest message per camera, decodes
/// it once per frame at most (<see cref="Tick"/>), and keeps the per-camera statistics. Per-camera
/// lists share the index of the panel list.
/// </summary>
public class CameraStreams
{
    readonly ImageSubscriber _owner;
    readonly IReadOnlyList<CameraPanel> _panels;

    // Per camera, same index as the panels (lists, because cameras can be added later).
    readonly List<Texture2D> _textures = new List<Texture2D>();
    readonly List<byte[]> _rawBuffers = new List<byte[]>();
    readonly List<double> _lastRenderTime = new List<double>();
    readonly List<bool> _warnedFormat = new List<bool>();
    // Latest-frame decode: the subscription callbacks only keep the newest message per camera;
    // Tick (LateUpdate) decodes it (once per frame at most). An older message that is replaced before it
    // was decoded counts as dropped, so a backlog never makes the app show stale frames.
    readonly List<CompressedImageMsg> _pendingCompressed = new List<CompressedImageMsg>();
    readonly List<ImageMsg> _pendingRaw = new List<ImageMsg>();
    readonly List<int> _dropped = new List<int>();
    readonly List<float> _decodeMs = new List<float>();
    readonly List<double> _lastFrameTime = new List<double>();   // Time.unscaledTime of the last shown frame
    readonly List<float> _fps = new List<float>();
    readonly List<int> _framesInWindow = new List<int>();
    readonly List<float> _fpsWindowStart = new List<float>();
    readonly List<int> _loggedDropped = new List<int>();
    readonly List<int> _received = new List<int>(), _loggedReceived = new List<int>();
    float _lastDropLog;
    readonly System.Diagnostics.Stopwatch _decodeWatch = new System.Diagnostics.Stopwatch();
    const float FpsWindowSeconds = 1f, LogSeconds = 10f;
    float _nextLog;

    // Keep decoding a camera although its block is hidden (first-person view uses it).
    readonly HashSet<int> _forceDecode = new HashSet<int>();

    // Raised before a decoded frame is shown: the façade adapts the view to the frame's real size.
    public Action<int, Texture2D> FrameDecoded;
    // Raised after a camera's texture was shown; carries the current texture instance (raw decoding can replace it).
    public Action<int, Texture2D> FrameReady;

    public CameraStreams(ImageSubscriber owner, IReadOnlyList<CameraPanel> panels)
    {
        _owner = owner;
        _panels = panels;
    }

    // Topic that is subscribed for `cam`, and whether it carries raw images.
    public string EffectiveTopic(RobotProfile.CameraConfig cam, out bool raw)
    {
        string topic = _owner.Profile.FullTopic(cam);
        raw = cam.raw;
        if (!ImageSubscriber.Compressed && topic.EndsWith(RosNames.CompressedSuffix))
        {
            topic = topic.Substring(0, topic.Length - RosNames.CompressedSuffix.Length);
            raw = true;
        }
        return topic;
    }

    // Start the stream of the camera whose panel was just added at the end of the panel list.
    public void Add(RobotProfile.CameraConfig cam, string topic, bool raw)
    {
        int index = _textures.Count;
        _textures.Add(new Texture2D(cam.resolution.x, cam.resolution.y, TextureFormat.RGB24, false));
        _rawBuffers.Add(null);
        _lastRenderTime.Add(0.0);
        _warnedFormat.Add(false);
        _pendingCompressed.Add(null);
        _pendingRaw.Add(null);
        _dropped.Add(0);
        _decodeMs.Add(0f);
        _lastFrameTime.Add(-1.0);
        _fps.Add(0f);
        _framesInWindow.Add(0);
        _fpsWindowStart.Add(Time.unscaledTime);
        _loggedDropped.Add(0);
        _received.Add(0);
        _loggedReceived.Add(0);

        if (raw)
            _owner.ros.Subscribe<ImageMsg>(topic, msg => OnRawMessage(msg, index));
        else
            _owner.ros.Subscribe<CompressedImageMsg>(topic, msg => OnCompressedMessage(msg, index));
    }

    public void ForceDecode(int index, bool on)
    {
        if (index < 0 || index >= _panels.Count) return;
        if (on) _forceDecode.Add(index); else _forceDecode.Remove(index);
    }

    // Hidden cameras skip decoding entirely; others are limited to their maxFps.
    bool IsWanted(int index) => _panels[index].Visible || _forceDecode.Contains(index);

    bool ShouldRender(int index)
    {
        float fps = _panels[index].Config.maxFps;
        if (fps <= 0f) return true;
        double now = Time.unscaledTimeAsDouble;
        if (now - _lastRenderTime[index] < 1.0 / fps) return false;
        _lastRenderTime[index] = now;
        return true;
    }

    bool OwnerGone => _owner == null || !_owner.isActiveAndEnabled;

    // Callbacks only store the newest message; decoding happens in Tick.
    void OnCompressedMessage(CompressedImageMsg msg, int index)
    {
        if (OwnerGone) return;
        if (msg == null || msg.data == null || msg.data.Length == 0 || index >= _panels.Count || !IsWanted(index)) return;
        _received[index]++;
        if (_pendingCompressed[index] != null) _dropped[index]++;
        _pendingCompressed[index] = msg;
    }

    void OnRawMessage(ImageMsg msg, int index)
    {
        if (OwnerGone) return;
        if (msg == null || index >= _panels.Count || !IsWanted(index)) return;
        _received[index]++;
        if (_pendingRaw[index] != null) _dropped[index]++;
        _pendingRaw[index] = msg;
    }

    // Called from LateUpdate.
    public void Tick()
    {
        for (int i = 0; i < _panels.Count; i++)
        {
            var compressed = _pendingCompressed[i];
            var raw = _pendingRaw[i];
            if (compressed == null && raw == null) continue;
            if (!IsWanted(i))
            {
                _pendingCompressed[i] = null;   // hidden meanwhile: nothing to decode
                _pendingRaw[i] = null;
                continue;
            }
            if (!ShouldRender(i)) continue;     // keep the newest for the next allowed frame
            _pendingCompressed[i] = null;
            _pendingRaw[i] = null;

            _decodeWatch.Restart();
            if (compressed != null) RenderCompressedTexture(compressed, i);
            else RenderRawTexture(raw, i);
            _decodeWatch.Stop();
            _decodeMs[i] = (float)_decodeWatch.Elapsed.TotalMilliseconds;
        }

        float now = Time.unscaledTime;
        for (int i = 0; i < _panels.Count; i++)
            if (now - _fpsWindowStart[i] >= FpsWindowSeconds)
            {
                _fps[i] = _framesInWindow[i] / (now - _fpsWindowStart[i]);
                _framesInWindow[i] = 0;
                _fpsWindowStart[i] = now;
            }

        if (now >= _nextLog)
        {
            _nextLog = now + LogSeconds;
            for (int i = 0; i < _panels.Count; i++)
            {
                // Drops beyond the maxFps thinning mean the backlog really grew. Otherwise log at most once a minute.
                int drops = _dropped[i] - _loggedDropped[i];
                int received = _received[i] - _loggedReceived[i];
                float maxFps = _panels[i].Config.maxFps;
                float expected = maxFps > 0f ? Mathf.Max(0f, received - maxFps * LogSeconds) : 0f;
                bool grew = drops > expected + 5f;
                bool minuteDue = drops > 0 && now - _lastDropLog >= 60f;
                _loggedDropped[i] = _dropped[i];
                _loggedReceived[i] = _received[i];
                if (grew || minuteDue)
                {
                    _lastDropLog = now;
                    DevLog.Log("[ImageSubscriber]", $"cam {i}: fps {_fps[i]:F1}, dropped {_dropped[i]}, decode {_decodeMs[i]:F1} ms");
                }
            }
        }
    }

    // Messages replaced before they were decoded (includes maxFps thinning), since start.
    public int DroppedFrames(int index) => index >= 0 && index < _dropped.Count ? _dropped[index] : 0;
    // Duration of the last decode (ms).
    public float DecodeMs(int index) => index >= 0 && index < _decodeMs.Count ? _decodeMs[index] : 0f;
    // Time.unscaledTime of the last shown frame, -1 before the first.
    public double LastFrameTime(int index) => index >= 0 && index < _lastFrameTime.Count ? _lastFrameTime[index] : -1.0;
    // Shown frames per second over the last second.
    public float Fps(int index) => index >= 0 && index < _fps.Count ? _fps[index] : 0f;

    void RenderCompressedTexture(CompressedImageMsg msg, int index)
    {
        if (!_textures[index].LoadImage(msg.data))
        {
            Debug.LogWarning($"Failed to decode compressed image on {_panels[index].Topic}. format={msg.format}");
            return;
        }
        ShowFrame(index);
    }

    void RenderRawTexture(ImageMsg msg, int index)
    {
        var tex = _textures[index];
        var buf = _rawBuffers[index];
        bool ok = RawImageDecoder.TryDecode(msg, ref tex, ref buf);
        _textures[index] = tex;
        _rawBuffers[index] = buf;
        if (!ok)
        {
            if (!_warnedFormat[index])
                Debug.LogWarning($"Unsupported image on {_panels[index].Topic}: encoding={msg.encoding} {msg.width}x{msg.height}");
            _warnedFormat[index] = true;
            return;
        }
        ShowFrame(index);
    }

    void ShowFrame(int index)
    {
        var tex = _textures[index];
        FrameDecoded?.Invoke(index, tex);
        _panels[index].SetTexture(tex);
        _lastFrameTime[index] = Time.unscaledTime;
        _framesInWindow[index]++;
        FrameReady?.Invoke(index, tex);
    }
}
