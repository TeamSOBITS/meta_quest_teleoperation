using UnityEngine;

/// <summary>
/// Pure rules for the recorder status badge (the data layer is <see cref="RecordStatus"/>).
/// The feed: sobits_vla_tools' sobits_vla_rosbag_collection publishes sobits_interfaces/VlaRecordStatus on
/// /&lt;ns&gt;/vla_rosbag_collection/record_status: a 1 Hz heartbeat (event NONE) plus one message at once per event
/// (started, paused, resumed, saved, discarded, deleted, error, task set, rejected).
///   Hidden     no message yet, or the newest one is 3 s old or older (the node is not running)
///   Idle       alive, state STOPPED
///   Recording  alive, state RECORDING (elapsed time keeps counting between heartbeats)
///   Paused     alive, state PAUSED
///   Error      alive, state ERROR (or any state this app does not know)
/// Events show a short toast for <see cref="ToastS"/> seconds (see <see cref="Toast"/>).
/// </summary>
public static class RecordStatusRule
{
    public const float StaleS = 3f, ToastS = 4f, PulseHz = 1f;

    public enum Look { Hidden, Idle, Recording, Paused, Error }

    // Message state / event numbers (VlaRecordStatus.msg).
    const byte StateStopped = 0, StateRecording = 1, StatePaused = 2;
    const byte EventSaved = 4, EventDiscarded = 5, EventDeleted = 6, EventError = 7, EventTaskSet = 8, EventRejected = 9;

    public static bool IsAlive(float ageS) => ageS >= 0f && ageS < StaleS;

    public static Look Evaluate(bool hasMessage, float ageS, byte state)
    {
        if (!hasMessage || !IsAlive(ageS)) return Look.Hidden;
        switch (state)
        {
            case StateStopped: return Look.Idle;
            case StateRecording: return Look.Recording;
            case StatePaused: return Look.Paused;
            default: return Look.Error;
        }
    }

    public static Color Colour(Look look)
    {
        switch (look)
        {
            case Look.Idle: return HudTheme.Muted;
            case Look.Recording: return HudTheme.Record;
            case Look.Paused: return HudTheme.Warn;
            case Look.Error: return HudTheme.Bad;
            default: return Color.clear;
        }
    }

    public static string Label(Look look, float elapsedS)
    {
        switch (look)
        {
            case Look.Recording: return "REC " + FormatElapsed(elapsedS);
            case Look.Paused: return "PAUSED " + FormatElapsed(elapsedS);
            case Look.Idle: return "IDLE";
            case Look.Error: return "ERROR";
            default: return "";
        }
    }

    public static string FormatElapsed(float s)
    {
        int t = s > 0f ? Mathf.FloorToInt(s) : 0;
        return $"{t / 3600:00}:{t / 60 % 60:00}:{t % 60:00}";
    }

    // Toast text of an event, null for the ones that show none (heartbeat, started, paused, resumed).
    public static string Toast(byte evt, string detail, float elapsedS)
    {
        bool has = !string.IsNullOrEmpty(detail);
        switch (evt)
        {
            case EventSaved: return "Saved · " + FormatElapsed(elapsedS);
            case EventDiscarded:
                if (detail == "too_short") return "Discarded: too short";
                if (detail == "integrity_failed") return "Discarded: integrity failed";
                return has ? "Discarded: " + detail : "Discarded";
            case EventDeleted: return "Deleted";
            case EventError: return has ? "Error: " + detail : "Error";
            case EventTaskSet: return has ? "Task: " + detail : "Task set";
            case EventRejected: return has ? detail : "Rejected";
            default: return null;
        }
    }

    // Colour of an event's toast: saved Good, discarded / error Bad, deleted / rejected Warn, task set Accent.
    public static Color ToastColour(byte evt)
    {
        switch (evt)
        {
            case EventSaved: return HudTheme.Good;
            case EventDiscarded:
            case EventError: return HudTheme.Bad;
            case EventTaskSet: return HudTheme.Accent;
            default: return HudTheme.Warn;   // deleted, rejected
        }
    }

    // Opacity of the pulsing REC dot at time t (seconds): 0.55 .. 1.
    public static float PulseAlpha(float t) => 0.55f + 0.45f * (0.5f + 0.5f * Mathf.Sin(2f * Mathf.PI * PulseHz * t));
}
