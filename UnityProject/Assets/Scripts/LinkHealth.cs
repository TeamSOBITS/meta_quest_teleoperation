using System;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// How healthy the link to the robot looks, from three signals: the age of the newest frame of a camera,
/// the round trip (<see cref="RoundTrip"/>, only while it measures) and the ROS connection error flag.
///   Good      newest frame younger than 0.5 s and round trip under 150 ms
///   Degraded  anything between (frame under 2 s old, round trip under 400 ms or beyond)
///   Lost      newest frame 2 s old or older, or the ROS connection reports an error
/// The first-person head card, the camera blocks (their outline ring) and the status strip show it.
/// </summary>
public class LinkHealth
{
    public enum Level { Good, Degraded, Lost }

    public const float GoodAgeS = 0.5f, LostAgeS = 2f, GoodRttMs = 150f;

    readonly ImageSubscriber _images;
    readonly int _camera;
    readonly Func<RoundTrip> _rtt;

    public LinkHealth(ImageSubscriber images, int cameraIndex, Func<RoundTrip> rtt)
    {
        _images = images;
        _camera = cameraIndex;
        _rtt = rtt;
    }

    // Pure rule. ageS < 0: unknown (no frame yet, camera off) and ignored; rttMs < 0: not measured.
    public static Level Evaluate(float ageS, float rttMs, bool connectionError)
    {
        if (connectionError || (ageS >= 0f && ageS >= LostAgeS)) return Level.Lost;
        bool ageGood = ageS < 0f || ageS < GoodAgeS;
        bool rttGood = rttMs < 0f || rttMs < GoodRttMs;
        return ageGood && rttGood ? Level.Good : Level.Degraded;
    }

    public static bool ConnectionError => ROSConnection.GetOrCreateInstance().HasConnectionError;

    // Seconds since the camera's newest frame; -1 without a camera, before its first frame or while it is switched off.
    public float AgeS
    {
        get
        {
            if (_images == null || _camera < 0 || _camera >= _images.Panels.Count) return -1f;
            double last = _images.LastFrameTime(_camera);
            if (last < 0.0 || !_images.IsOn(_images.Panels[_camera])) return -1f;
            return (float)(Time.unscaledTime - last);
        }
    }

    // Smoothed round trip (ms), -1 while none was measured lately.
    public float RttMs
    {
        get
        {
            var rtt = _rtt != null ? _rtt() : null;
            return rtt != null && rtt.HasRecent ? rtt.RttMs : -1f;
        }
    }

    public Level Current => Evaluate(AgeS, RttMs, ConnectionError);

    public static Color Colour(Level level) => level == Level.Good ? HudTheme.Good : level == Level.Degraded ? HudTheme.Warn : HudTheme.Bad;
    public static string Text(Level level) => level == Level.Good ? "link ok" : level == Level.Degraded ? "degraded" : "lost";
}
