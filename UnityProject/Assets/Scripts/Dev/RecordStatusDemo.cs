using System.Collections;
using RosMessageTypes.SobitsInterfaces;
using UnityEngine;

/// <summary>
/// Fake recorder feed for the HUD without the ROS side (device.sh launch --recstatus, extra `recstatus`). 1 s steps:
/// idle x2, task set "pick cup", started, 10 heartbeats recording, paused x3 (time frozen), resumed x3, saved, idle x3,
/// started x2, discarded (too_short), started, deleted, error "demo error", then it stops feeding (the badge vanishes
/// after <see cref="RecordStatusRule.StaleS"/>). Dev only (<see cref="DevTools.Enabled"/>).
/// </summary>
public static class RecordStatusDemo
{
    public static void Start(MonoBehaviour host, RecordStatus target)
    {
        if (!DevTools.Enabled || host == null || target == null) return;
        host.StartCoroutine(Run(target));
    }

    static IEnumerator Run(RecordStatus target)
    {
        var wait = new WaitForSecondsRealtime(1f);
        uint seq = 0;
        float elapsed = 0f;
        string task = "", episode = "";

        void Send(byte state, byte evt, string detail, string message)
        {
            if (target == null) return;
            if (evt != 0) seq++;
            target.OnMessage(new VlaRecordStatusMsg
            {
                state = state, @event = evt, event_seq = seq, task_set = task != "", task_name = task,
                episode_name = episode, elapsed_sec = elapsed, detail = detail ?? "", message = message,
            });
        }
        string Hms(float s) => RecordStatusRule.FormatElapsed(s);
        void Idle() => Send(VlaRecordStatusMsg.STATE_STOPPED, VlaRecordStatusMsg.EVENT_NONE, "", "idle");
        void Rec(byte evt) => Send(VlaRecordStatusMsg.STATE_RECORDING, evt, "", "recording " + Hms(elapsed));
        void Pause(byte evt) => Send(VlaRecordStatusMsg.STATE_PAUSED, evt, "", "paused " + Hms(elapsed));
        void Begin() { elapsed = 0f; episode = "episode_demo_" + (seq + 1); Rec(VlaRecordStatusMsg.EVENT_STARTED); }

        DevLog.Log("REC", "demo feed start");
        for (int i = 0; i < 2; i++) { Idle(); yield return wait; }
        task = "pick cup";
        Send(VlaRecordStatusMsg.STATE_STOPPED, VlaRecordStatusMsg.EVENT_TASK_SET, task, "task_set: " + task); yield return wait;
        Begin(); yield return wait;
        for (int i = 0; i < 10; i++) { elapsed += 1f; Rec(VlaRecordStatusMsg.EVENT_NONE); yield return wait; }
        Pause(VlaRecordStatusMsg.EVENT_PAUSED); yield return wait;
        for (int i = 0; i < 2; i++) { Pause(VlaRecordStatusMsg.EVENT_NONE); yield return wait; }
        Rec(VlaRecordStatusMsg.EVENT_RESUMED); yield return wait;
        for (int i = 0; i < 2; i++) { elapsed += 1f; Rec(VlaRecordStatusMsg.EVENT_NONE); yield return wait; }
        Send(VlaRecordStatusMsg.STATE_STOPPED, VlaRecordStatusMsg.EVENT_SAVED, "", "saved"); yield return wait;
        for (int i = 0; i < 3; i++) { Idle(); yield return wait; }
        Begin(); yield return wait;
        elapsed += 1f; Rec(VlaRecordStatusMsg.EVENT_NONE); yield return wait;
        Send(VlaRecordStatusMsg.STATE_STOPPED, VlaRecordStatusMsg.EVENT_DISCARDED, "too_short", "discarded"); yield return wait;
        Begin(); yield return wait;
        elapsed += 1f; Rec(VlaRecordStatusMsg.EVENT_NONE); yield return wait;
        Send(VlaRecordStatusMsg.STATE_STOPPED, VlaRecordStatusMsg.EVENT_DELETED, "", "deleted"); yield return wait;
        Send(VlaRecordStatusMsg.STATE_ERROR, VlaRecordStatusMsg.EVENT_ERROR, "demo error", "error");
        DevLog.Log("REC", "demo feed end (badge hides after " + RecordStatusRule.StaleS + " s)");
    }
}
