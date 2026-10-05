using System.Collections;
using RosMessageTypes.SobitsInterfaces;
using UnityEngine;

/// <summary>
/// Fake VLA status feed for the HUD without the ROS side (device.sh launch --recstatus [deploy], extra `recstatus`
/// = 1 | deploy). 1 s steps, then it stops feeding (the badge vanishes after <see cref="VlaStatusRule.StaleS"/>).
/// Collection (recorder): idle x2, task set "pick cup", started, 10 heartbeats recording, paused x3 (time frozen),
/// resumed x3, saved, idle x3, started x2, discarded (too_short), started, deleted, error "demo error".
/// Deploy (policy runner, deadman on): idle x2, task set, started with the grip released x3 (clock not started),
/// engaged + 10 heartbeats (steps +10, 8.1 Hz), released x2, engaged x5, stopped success_lift, resetting x4,
/// episode done, idle x3, started (grip held) + 5 heartbeats, stopped timeout, resetting x2, episode done.
/// Dev only (<see cref="DevTools.Enabled"/>).
/// </summary>
public static class VlaStatusDemo
{
    public const string ModeCollection = "1", ModeDeploy = "deploy";
    public const string DemoPolicy = "team-sobits/sobit_home_left_sim-pnp_pear_bowl-abs-200-smolvla_fft";

    public static void Start(MonoBehaviour host, VlaStatus target, string mode = ModeCollection)
    {
        if (!DevTools.Enabled || host == null || target == null) return;
        host.StartCoroutine(mode == ModeDeploy ? RunDeploy(target) : Run(target));
    }

    static IEnumerator RunDeploy(VlaStatus target)
    {
        var wait = new WaitForSecondsRealtime(1f);
        uint seq = 0, steps = 0;
        float elapsed = 0f, hz = 0f;
        string task = "", episode = "", outcome = "";
        bool engaged = false;
        int episodes = 0;

        void Send(byte state, byte evt, string detail, string message)
        {
            if (target == null) return;
            if (evt != 0) seq++;
            target.OnMessage(new VlaStatusMsg
            {
                stage = VlaStatusMsg.STAGE_DEPLOY, state = state, @event = evt, event_seq = seq, task_set = task != "", task_name = task,
                episode_name = episode, elapsed_sec = elapsed, detail = detail ?? "", message = message,
                policy = DemoPolicy, deadman_enabled = true, deadman_engaged = engaged, steps = steps, inference_hz = hz, outcome = outcome,
            });
        }
        string Hms(float s) => VlaStatusRule.FormatElapsed(s);
        void Idle() => Send(VlaStatusMsg.STATE_STOPPED, VlaStatusMsg.EVENT_NONE, "", "idle");
        void Play(byte evt) => Send(VlaStatusMsg.STATE_PLAYING, evt, "", "playing " + Hms(elapsed) + (engaged ? "" : " (released)"));
        void Drive(byte evt) { elapsed += 1f; steps += 10; hz = 8.1f; Play(evt); }
        void Begin(bool grip)
        {
            elapsed = 0f; steps = 0; hz = 0f; outcome = ""; engaged = grip;
            episode = "episode_" + ++episodes;
            Play(VlaStatusMsg.EVENT_STARTED);
        }
        IEnumerator Finish(string how, int resets)
        {
            engaged = false; hz = 0f; outcome = how;
            Send(VlaStatusMsg.STATE_RESETTING, VlaStatusMsg.EVENT_STOPPED, how, "resetting"); yield return wait;
            for (int i = 0; i < resets; i++) { Send(VlaStatusMsg.STATE_RESETTING, VlaStatusMsg.EVENT_NONE, "", "resetting"); yield return wait; }
            Send(VlaStatusMsg.STATE_STOPPED, VlaStatusMsg.EVENT_EPISODE_DONE, how, "done: " + how); yield return wait;
        }

        DevLog.Log("VLA", "demo deploy feed start");
        for (int i = 0; i < 2; i++) { Idle(); yield return wait; }
        task = "pick cup";
        Send(VlaStatusMsg.STATE_STOPPED, VlaStatusMsg.EVENT_TASK_SET, task, "task_set: " + task); yield return wait;
        Begin(false); yield return wait;
        for (int i = 0; i < 2; i++) { Play(VlaStatusMsg.EVENT_NONE); yield return wait; }
        engaged = true; Play(VlaStatusMsg.EVENT_ENGAGED); yield return wait;
        for (int i = 0; i < 10; i++) { Drive(VlaStatusMsg.EVENT_NONE); yield return wait; }
        engaged = false; hz = 0f; elapsed += 1f; Play(VlaStatusMsg.EVENT_RELEASED); yield return wait;
        elapsed += 1f; Play(VlaStatusMsg.EVENT_NONE); yield return wait;
        engaged = true; Drive(VlaStatusMsg.EVENT_ENGAGED); yield return wait;
        for (int i = 0; i < 4; i++) { Drive(VlaStatusMsg.EVENT_NONE); yield return wait; }
        foreach (var w in Iterate(Finish("success_lift", 4))) yield return w;
        for (int i = 0; i < 3; i++) { Idle(); yield return wait; }
        Begin(true); yield return wait;
        for (int i = 0; i < 5; i++) { Drive(VlaStatusMsg.EVENT_NONE); yield return wait; }
        foreach (var w in Iterate(Finish("timeout", 2))) yield return w;
        DevLog.Log("VLA", "demo deploy feed end (badge hides after " + VlaStatusRule.StaleS + " s)");
    }

    static IEnumerable Iterate(IEnumerator e) { while (e.MoveNext()) yield return e.Current; }

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

        DevLog.Log("VLA", "demo collection feed start");
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
