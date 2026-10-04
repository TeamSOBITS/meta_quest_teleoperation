using System.Collections;
using RosMessageTypes.SobitsInterfaces;
using UnityEngine;

/// <summary>
/// Fake recorder feed for the HUD without the ROS side (device.sh launch --recstatus, extra `recstatus`). 1 s steps:
/// idle x2, task set "pick cup", started, 10 heartbeats recording, paused x3 (time frozen), resumed x3, saved, idle x3,
/// started x2, discarded (too_short), started, deleted, error "demo error", then it stops feeding (the badge vanishes
/// after <see cref="VlaStatusRule.StaleS"/>). Dev only (<see cref="DevTools.Enabled"/>).
/// </summary>
public static class VlaStatusDemo
{
    public static void Start(MonoBehaviour host, VlaStatus target)
    {
        if (!DevTools.Enabled || host == null || target == null) return;
        host.StartCoroutine(Run(target));
    }

    static IEnumerator Run(VlaStatus target)
    {
        var wait = new WaitForSecondsRealtime(1f);
        uint seq = 0;
        float elapsed = 0f;
        string task = "", episode = "";

        void Send(byte state, byte evt, string detail, string message)
        {
            if (target == null) return;
            if (evt != 0) seq++;
            target.OnMessage(new VlaStatusMsg
            {
                stage = VlaStatusMsg.STAGE_COLLECTION, state = state, @event = evt, event_seq = seq, task_set = task != "", task_name = task,
                episode_name = episode, elapsed_sec = elapsed, detail = detail ?? "", message = message,
            });
        }
        string Hms(float s) => VlaStatusRule.FormatElapsed(s);
        void Idle() => Send(VlaStatusMsg.STATE_STOPPED, VlaStatusMsg.EVENT_NONE, "", "idle");
        void Rec(byte evt) => Send(VlaStatusMsg.STATE_RECORDING, evt, "", "recording " + Hms(elapsed));
        void Pause(byte evt) => Send(VlaStatusMsg.STATE_PAUSED, evt, "", "paused " + Hms(elapsed));
        void Begin() { elapsed = 0f; episode = "episode_demo_" + (seq + 1); Rec(VlaStatusMsg.EVENT_STARTED); }

        DevLog.Log("VLA", "demo feed start");
        for (int i = 0; i < 2; i++) { Idle(); yield return wait; }
        task = "pick cup";
        Send(VlaStatusMsg.STATE_STOPPED, VlaStatusMsg.EVENT_TASK_SET, task, "task_set: " + task); yield return wait;
        Begin(); yield return wait;
        for (int i = 0; i < 10; i++) { elapsed += 1f; Rec(VlaStatusMsg.EVENT_NONE); yield return wait; }
        Pause(VlaStatusMsg.EVENT_PAUSED); yield return wait;
        for (int i = 0; i < 2; i++) { Pause(VlaStatusMsg.EVENT_NONE); yield return wait; }
        Rec(VlaStatusMsg.EVENT_RESUMED); yield return wait;
        for (int i = 0; i < 2; i++) { elapsed += 1f; Rec(VlaStatusMsg.EVENT_NONE); yield return wait; }
        Send(VlaStatusMsg.STATE_STOPPED, VlaStatusMsg.EVENT_SAVED, "", "saved"); yield return wait;
        for (int i = 0; i < 3; i++) { Idle(); yield return wait; }
        Begin(); yield return wait;
        elapsed += 1f; Rec(VlaStatusMsg.EVENT_NONE); yield return wait;
        Send(VlaStatusMsg.STATE_STOPPED, VlaStatusMsg.EVENT_DISCARDED, "too_short", "discarded"); yield return wait;
        Begin(); yield return wait;
        elapsed += 1f; Rec(VlaStatusMsg.EVENT_NONE); yield return wait;
        Send(VlaStatusMsg.STATE_STOPPED, VlaStatusMsg.EVENT_DELETED, "", "deleted"); yield return wait;
        Send(VlaStatusMsg.STATE_ERROR, VlaStatusMsg.EVENT_ERROR, "demo error", "error");
        DevLog.Log("VLA", "demo feed end (badge hides after " + VlaStatusRule.StaleS + " s)");
    }
}
