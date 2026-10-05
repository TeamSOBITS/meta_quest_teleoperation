using System;
using System.Collections.Generic;
using RosMessageTypes.SobitsInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// VLA stage status feed (sobits_interfaces/VlaStatus from sobits_vla_tools) for the HUD: the recorder
/// (vla_rosbag_collection, STAGE_COLLECTION) and the policy runner (sobits_vla_deploy, STAGE_DEPLOY) each publish it on
/// ~/status (RobotProfile.vlaStatusSuffixes). One feed for both topics: the newest message wins (see
/// <see cref="VlaStatusRule.Supersedes"/> for the one exception), with its age, a toast text for the newest event, and
/// the elapsed time that keeps counting between the 1 Hz heartbeats while recording / playing. Display only: nothing
/// is published back. Created by TeleopHud (one per robot screen); the rules (what to show, texts) are in
/// <see cref="VlaStatusRule"/>.
/// </summary>
public class VlaStatus : MonoBehaviour
{
    static ROSConnection _subscribedOn;   // each robot screen has its own ROSConnection
    static readonly HashSet<string> _subscribedTopics = new HashSet<string>();   // on _subscribedOn (there is no Unsubscribe)
    static VlaStatus Current;   // the live instance; the subscriptions forward to it

    VlaStatusMsg _msg;
    float _receivedAt = float.NegativeInfinity;
    // Per stage (each node counts its own events): the last event_seq seen, and whether one was seen.
    readonly uint[] _lastSeq = new uint[2];
    readonly bool[] _hasSeq = new bool[2];

    /// <summary>Raised on every accepted message (heartbeat or event).</summary>
    public event Action Changed;

    public static VlaStatus Latest => Current;

    public bool HasMessage => _msg != null;
    /// <summary>Seconds since the newest message arrived, -1 when none yet.</summary>
    public float AgeS => _msg == null ? -1f : Time.unscaledTime - _receivedAt;
    /// <summary>A stage node is publishing (newest message younger than <see cref="VlaStatusRule.StaleS"/>).</summary>
    public bool Available => VlaStatusRule.IsAlive(AgeS);
    /// <summary>VlaStatusMsg.STAGE_COLLECTION (recorder) or STAGE_DEPLOY (policy runner).</summary>
    public byte Stage => _msg?.stage ?? VlaStatusMsg.STAGE_COLLECTION;
    public bool IsDeploy => Stage == VlaStatusMsg.STAGE_DEPLOY;
    public byte State => _msg?.state ?? 0;
    public bool TaskSet => _msg != null && _msg.task_set;
    public string TaskName => _msg?.task_name ?? "";
    public string EpisodeName => _msg?.episode_name ?? "";
    public uint EventSeq => _msg?.event_seq ?? 0;
    // Deploy only (zero / empty from the recorder).
    public string Policy => _msg?.policy ?? "";
    public bool DeadmanEnabled => _msg != null && _msg.deadman_enabled;
    public bool DeadmanEngaged => _msg != null && _msg.deadman_engaged;
    public uint Steps => _msg?.steps ?? 0;
    public float InferenceHz => _msg?.inference_hz ?? 0f;
    /// <summary>How the last deploy episode ended (set with EVENT_STOPPED / EVENT_EPISODE_DONE, "" while playing).</summary>
    public string Outcome => _msg?.outcome ?? "";

    /// <summary>Recorded / episode time: the message's value plus the time since it while recording or playing (frozen
    /// otherwise). Deploy: the clock starts at PLAY or at the first deadman engagement, so it ticks only once
    /// elapsed_sec &gt; 0, the deadman is engaged, or there is no deadman.</summary>
    public float ElapsedS
    {
        get
        {
            if (_msg == null) return 0f;
            bool running = State == VlaStatusMsg.STATE_RECORDING
                           || (State == VlaStatusMsg.STATE_PLAYING && (_msg.elapsed_sec > 0f || _msg.deadman_engaged || !_msg.deadman_enabled));
            return running && Available ? _msg.elapsed_sec + AgeS : _msg.elapsed_sec;
        }
    }
    public VlaStatusRule.Look Look => VlaStatusRule.Evaluate(HasMessage, AgeS, State);

    /// <summary>Text of the newest toast-worthy event (stays after it expires, see <see cref="ToastShown"/>).</summary>
    public string ToastText { get; private set; }
    /// <summary>Event number of <see cref="ToastText"/> (its colour: <see cref="VlaStatusRule.ToastColour"/>).</summary>
    public byte ToastEvent { get; private set; }
    public float ToastUntil { get; private set; }
    public bool ToastShown => ToastText != null && Time.unscaledTime < ToastUntil;

    public static VlaStatus Create(RobotProfile profile)
    {
        if (profile == null) return null;
        var go = new GameObject("Vla Status");
        var s = go.AddComponent<VlaStatus>();
        Current = s;
        var suffixes = profile.vlaStatusSuffixes;
        if (suffixes == null || suffixes.Length == 0) return s;   // no stage node configured: never shows
        var ros = ROSConnection.GetOrCreateInstance();
        if (_subscribedOn != ros) { _subscribedOn = ros; _subscribedTopics.Clear(); }
        foreach (var suffix in suffixes)
        {
            if (string.IsNullOrEmpty(suffix)) continue;
            string topic = profile.FullTopic(suffix);
            if (!_subscribedTopics.Add(topic)) continue;   // subscribe once per connection and topic
            ros.Subscribe<VlaStatusMsg>(topic, m => { if (Current != null) Current.OnMessage(m); });
        }
        return s;
    }

    void OnDestroy()
    {
        if (Current == this) Current = null;
    }

    internal void OnMessage(VlaStatusMsg m)
    {
        if (this == null || m == null) return;
        float now = Time.unscaledTime;
        int k = m.stage == VlaStatusMsg.STAGE_DEPLOY ? 1 : 0;
        // The first message of a stage only seeds its sequence: the transient-local topic replays the last event on connect, old news.
        bool newEvent = _hasSeq[k] && m.event_seq != _lastSeq[k];
        _lastSeq[k] = m.event_seq;
        _hasSeq[k] = true;
        if (_msg != null && !VlaStatusRule.Supersedes(m.stage, m.state, m.@event, _msg.stage, AgeS)) return;
        _msg = m;
        _receivedAt = now;
        if (newEvent)
        {
            string toast = VlaStatusRule.Toast(m.@event, m.detail, m.elapsed_sec);
            if (toast != null) { ToastText = toast; ToastEvent = m.@event; ToastUntil = now + VlaStatusRule.ToastS; }
        }
        if (m.@event != 0)
            DevLog.Log("VLA", $"stage={m.stage} event={m.@event} seq={m.event_seq} state={m.state} elapsed={m.elapsed_sec:F1} detail='{m.detail}'"
                              + (m.stage == VlaStatusMsg.STAGE_DEPLOY ? $" deadman={(m.deadman_enabled ? m.deadman_engaged ? "engaged" : "released" : "off")} steps={m.steps} outcome='{m.outcome}'" : ""));
        Changed?.Invoke();
    }
}
