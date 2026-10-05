using UnityEngine;

/// <summary>
/// Pure rules for the VLA stage status badge (the data layer is <see cref="VlaStatus"/>).
/// The feed: sobits_vla_tools' stage nodes publish sobits_interfaces/VlaStatus on ~/status, the recorder on
/// /&lt;ns&gt;/vla_rosbag_collection/status (stage COLLECTION), the policy runner on /&lt;ns&gt;/sobits_vla_deploy/status
/// (stage DEPLOY): a 1 Hz heartbeat (event NONE) plus one message at once per event.
///   Hidden     no message yet, or the newest one is 3 s old or older (no stage node is running)
///   Idle       alive, state STOPPED
///   Recording  alive, state RECORDING (elapsed time keeps counting between heartbeats)
///   Paused     alive, state PAUSED
///   Playing    alive, state PLAYING (deploy: the policy runs; the episode clock counts between heartbeats once started)
///   Resetting  alive, state RESETTING (deploy: the world reset after an episode)
///   Error      alive, state ERROR (or any state this app does not know)
/// Deploy adds a deadman chip (<see cref="Deadman"/>), the policy's short name in the task line (<see cref="TaskLine"/>)
/// and the episode's steps and inference rate (<see cref="Stats"/>).
/// Events show a short toast for <see cref="ToastS"/> seconds (see <see cref="Toast"/>).
/// Both nodes may publish at once (they never run an episode at the same time): see <see cref="Supersedes"/>.
/// </summary>
public static class VlaStatusRule
{
    public const float StaleS = 3f, ToastS = 4f, PulseHz = 1f;

    public enum Look { Hidden, Idle, Recording, Paused, Playing, Resetting, Error }

    // Message stage / state / event numbers (VlaStatus.msg).
    public const byte StageCollection = 0, StageDeploy = 1;
    const byte StateStopped = 0, StateRecording = 1, StatePaused = 2, StatePlaying = 3, StateResetting = 5;
    const byte EventNone = 0;
    const byte EventSaved = 4, EventDiscarded = 5, EventDeleted = 6, EventError = 7, EventTaskSet = 8, EventRejected = 9;

    public static bool IsAlive(float ageS) => ageS >= 0f && ageS < StaleS;

    // Whether a new message replaces the current one (current: its stage and age in s, < 0 = none). The newest
    // wins, except that an idle heartbeat (STOPPED, event NONE) of the OTHER stage never displaces a live message: with
    // both nodes up and idle the badge stays with the stage heard first instead of flipping every heartbeat, and an idle
    // recorder never hides a running deploy episode (or the other way round). Any event or non-idle state takes over.
    public static bool Supersedes(byte newStage, byte newState, byte newEvent, byte curStage, float curAgeS)
    {
        if (newStage == curStage || !IsAlive(curAgeS)) return true;
        return !(newState == StateStopped && newEvent == EventNone);
    }

    public static Look Evaluate(bool hasMessage, float ageS, byte state)
    {
        if (!hasMessage || !IsAlive(ageS)) return Look.Hidden;
        switch (state)
        {
            case StateStopped: return Look.Idle;
            case StateRecording: return Look.Recording;
            case StatePaused: return Look.Paused;
            case StatePlaying: return Look.Playing;
            case StateResetting: return Look.Resetting;
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
            case Look.Playing: return HudTheme.Accent;
            case Look.Resetting: return HudTheme.Warn;
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
            case Look.Playing: return "PLAY " + FormatElapsed(elapsedS);
            case Look.Resetting: return "RESETTING";
            case Look.Idle: return "IDLE";
            case Look.Error: return "ERROR";
            default: return "";
        }
    }

    // Whether the state dot pulses: while recording, and while playing with actions flowing (`driving`: the deadman is
    // engaged, or there is none). A held robot (deadman released) shows a steady dot.
    public static bool Pulses(Look look, bool driving) => look == Look.Recording || (look == Look.Playing && driving);

    public const int PolicyMaxChars = 24;

    // Short policy name: the last path segment of the repo id ("team-sobits/x-smolvla_fft" -> "x-smolvla_fft"); longer than
    // PolicyMaxChars -> "…" + its last PolicyMaxChars - 1 characters (the end tells the variants apart).
    public static string PolicyShort(string policy)
    {
        if (string.IsNullOrEmpty(policy)) return "";
        string s = policy.TrimEnd('/');
        int slash = s.LastIndexOf('/');
        if (slash >= 0) s = s.Substring(slash + 1);
        return s.Length > PolicyMaxChars ? "\u2026" + s.Substring(s.Length - (PolicyMaxChars - 1)) : s;
    }

    // Second line under the state. Collection: "task: pick cup" / "no task". Deploy: the task and the policy,
    // "pick cup · smolvla_fft" / "no task · smolvla_fft" (just the task without a policy).
    public static string TaskLine(byte stage, bool taskSet, string task, string policy)
    {
        string name = string.IsNullOrEmpty(task) ? "set" : task;
        if (stage != StageDeploy) return taskSet ? "task: " + name : "no task";
        string p = PolicyShort(policy);
        string t = taskSet ? name : "no task";
        return p.Length > 0 ? t + " \u00B7 " + p : t;
    }

    // Deploy deadman chip: none unless the deadman is enabled and the policy plays; else "GRIP driving" (Good, pulsing
    // dot) while the grip is held, "GRIP released" (Warn, steady) while the robot is held.
    public static (string text, Color colour, bool pulse)? Deadman(bool enabled, bool engaged, byte state)
    {
        if (!enabled || state != StatePlaying) return null;
        return engaged ? ("GRIP driving", HudTheme.Good, true) : ("GRIP released", HudTheme.Warn, false);
    }

    // Deploy episode figures: "240 steps · 8.1 Hz" (control steps, action-chunk inferences per second).
    public static string Stats(uint steps, float inferenceHz) => $"{steps} steps \u00B7 {inferenceHz:F1} Hz";

    public static string FormatElapsed(float s)
    {
        int t = s > 0f ? Mathf.FloorToInt(s) : 0;
        return $"{t / 3600:00}:{t / 60 % 60:00}:{t % 60:00}";
    }

    // Toast text of an event, null for the ones that show none (heartbeat, started, paused, resumed, and the deploy
    // events 10-14: stopped, episode done, engaged, released, reset done; the deploy outcome is not toasted).
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
