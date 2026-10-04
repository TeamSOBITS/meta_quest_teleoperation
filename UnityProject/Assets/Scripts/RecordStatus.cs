using System;
using RosMessageTypes.SobitsInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Recorder status feed (sobits_interfaces/VlaRecordStatus from sobits_vla_tools' sobits_vla_rosbag_collection) for the HUD:
/// the newest message, its age, a toast text for the newest event, and the elapsed time that keeps counting
/// between the 1 Hz heartbeats while recording. Display only: nothing is published back. Created by TeleopHud
/// (one per robot screen); the rules (what to show, texts) are in <see cref="RecordStatusRule"/>.
/// </summary>
public class RecordStatus : MonoBehaviour
{
    static ROSConnection _subscribedOn;   // each robot screen has its own ROSConnection
    static string _subscribedTopic;
    static RecordStatus Current;   // the live instance; the one subscription forwards to it

    VlaRecordStatusMsg _msg;
    float _receivedAt = float.NegativeInfinity;
    uint _lastSeq;
    bool _hasSeq;

    /// <summary>Raised on every message (heartbeat or event).</summary>
    public event Action Changed;

    public static RecordStatus Latest => Current;

    public bool HasMessage => _msg != null;
    /// <summary>Seconds since the newest message arrived, -1 when none yet.</summary>
    public float AgeS => _msg == null ? -1f : Time.unscaledTime - _receivedAt;
    /// <summary>The recorder node is publishing (newest message younger than <see cref="RecordStatusRule.StaleS"/>).</summary>
    public bool Available => RecordStatusRule.IsAlive(AgeS);
    public byte State => _msg?.state ?? 0;
    public bool TaskSet => _msg != null && _msg.task_set;
    public string TaskName => _msg?.task_name ?? "";
    public string EpisodeName => _msg?.episode_name ?? "";
    public uint EventSeq => _msg?.event_seq ?? 0;
    /// <summary>Recorded time: the message's value plus the time since it while recording (frozen otherwise).</summary>
    public float ElapsedS
    {
        get
        {
            if (_msg == null) return 0f;
            return State == VlaRecordStatusMsg.STATE_RECORDING && Available ? _msg.elapsed_sec + AgeS : _msg.elapsed_sec;
        }
    }
    public RecordStatusRule.Look Look => RecordStatusRule.Evaluate(HasMessage, AgeS, State);

    /// <summary>Text of the newest toast-worthy event (stays after it expires, see <see cref="ToastShown"/>).</summary>
    public string ToastText { get; private set; }
    /// <summary>Event number of <see cref="ToastText"/> (its colour: <see cref="RecordStatusRule.ToastColour"/>).</summary>
    public byte ToastEvent { get; private set; }
    public float ToastUntil { get; private set; }
    public bool ToastShown => ToastText != null && Time.unscaledTime < ToastUntil;

    public static RecordStatus Create(RobotProfile profile)
    {
        if (profile == null) return null;
        var go = new GameObject("Record Status");
        var s = go.AddComponent<RecordStatus>();
        Current = s;
        if (string.IsNullOrEmpty(profile.recordStatusSuffix)) return s;   // no recorder configured: never shows
        string topic = profile.FullTopic(profile.recordStatusSuffix);
        var ros = ROSConnection.GetOrCreateInstance();
        if (_subscribedOn != ros || _subscribedTopic != topic)
        {
            _subscribedOn = ros;   // subscribe once per connection (there is no Unsubscribe)
            _subscribedTopic = topic;
            ros.Subscribe<VlaRecordStatusMsg>(topic, m => { if (Current != null) Current.OnMessage(m); });
        }
        return s;
    }

    void OnDestroy()
    {
        if (Current == this) Current = null;
    }

    internal void OnMessage(VlaRecordStatusMsg m)
    {
        if (this == null || m == null) return;
        float now = Time.unscaledTime;
        _msg = m;
        _receivedAt = now;
        // The first message only seeds the sequence: the transient-local topic replays the last event on connect, old news.
        if (_hasSeq && m.event_seq != _lastSeq)
        {
            string toast = RecordStatusRule.Toast(m.@event, m.detail, m.elapsed_sec);
            if (toast != null) { ToastText = toast; ToastEvent = m.@event; ToastUntil = now + RecordStatusRule.ToastS; }
        }
        _lastSeq = m.event_seq;
        _hasSeq = true;
        if (m.@event != 0)
            DevLog.Log("REC", $"event={m.@event} seq={m.event_seq} state={m.state} elapsed={m.elapsed_sec:F1} detail='{m.detail}'");
        Changed?.Invoke();
    }
}
