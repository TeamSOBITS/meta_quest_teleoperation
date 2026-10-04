// Verify harness: Experiments batches 2-3: hand cams, arm targets, base velocity, head lock, FPV card, camera toggles.
// Env VERIFY_ROBOT=SOBIT_LIGHT runs it against SOBIT LIGHT (RobotSpec; sim: tools/sim.sh light).
// Needs the live sim on 127.0.0.1:10000 and no other ROS client (stop the app on the headset).
// Run: tools/verify.sh --suite ExperimentsVerify2   (or Unity -batchmode -projectPath <copy> -executeMethod ExperimentsVerify2.Run)
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reflection;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.UI;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;

// Copy-only harness: teleoperation experiments batches 2-3 (hand cams, arm targets, base velocity,
// head lock, lowered 0.7 bar, FPV card, camera toggles while blocks are hidden) live against the sim; never edits project code.
public static class ExperimentsVerify2
{
    static IEnumerator _run;
    static int _failures, _passes;
    static string ShotDir
    {
        get
        {
            var d = Environment.GetEnvironmentVariable("EXP_SHOTS");
            if (string.IsNullOrEmpty(d)) d = Path.GetFullPath(Path.Combine(Application.dataPath, "../../../exp_shots"));
            Directory.CreateDirectory(d);
            return d;
        }
    }
    static string Script(string name) => VerifyPaths.Tool(name);
    static RobotSpec Spec => RobotSpec.Current;

    public static void Run()
    {
        EditorSettings.enterPlayModeOptionsEnabled = true;
        EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.DisableDomainReload;
        Application.logMessageReceived += (m, st, t) => { lock (_logs) _logs.Add(m); };
        _run = Main();
        EditorApplication.update += Tick;
        EditorApplication.update += InjectTick;
    }

    static readonly List<string> _logs = new List<string>();
    static int LogCount(string s) { lock (_logs) return _logs.Count(l => l.Contains(s)); }

    static double _waitUntil; static int _waitFrame;
    static void Tick()
    {
        if (EditorApplication.timeSinceStartup < _waitUntil) return;
        if (Application.isPlaying && Time.frameCount < _waitFrame) return;
        try
        {
            if (!_run.MoveNext())
            {
                EditorApplication.update -= Tick;
                Log(_failures == 0 ? $"ALL CHECKS PASSED ({_passes})" : $"{_failures} CHECK(S) FAILED, {_passes} passed");
                EditorApplication.Exit(_failures == 0 ? 0 : 1);
            }
        }
        catch (Exception e)
        {
            Debug.LogException(e);
            Log("FAIL exception " + e.Message);
            EditorApplication.Exit(3);
        }
    }
    static object Frames(int n) { _waitFrame = Time.frameCount + n; _waitUntil = EditorApplication.timeSinceStartup + 0.05; return null; }
    static object Seconds(double s) { _waitUntil = EditorApplication.timeSinceStartup + s; return null; }
    static double Now => EditorApplication.timeSinceStartup;
    static void Log(string m) => Debug.Log("[Verify] " + m);
    static bool Check(bool ok, string what) { if (ok) _passes++; else _failures++; Log((ok ? "PASS " : "FAIL ") + what); return ok; }
    static T Field<T>(object o, string name) => (T)o.GetType().GetField(name, BindingFlags.NonPublic | BindingFlags.Instance).GetValue(o);
    static string V(Vector3 v) => $"({v.x:F3},{v.y:F3},{v.z:F3})";

    // ---------------- ROS helpers ----------------
    static readonly Dictionary<string, double> _q = new Dictionary<string, double>();
    static int _jsCount;
    static void OnJointStates(JointStateMsg m)
    {
        if (m?.name == null || m.position == null) return;
        _jsCount++;
        for (int i = 0; i < m.name.Length && i < m.position.Length; i++) _q[m.name[i]] = m.position[i];
    }
    static double Q(string j) => _q.TryGetValue(j, out var v) ? v : double.NaN;

    static bool _inject; static double _nextInject; static int _injected;
    static void InjectTick()
    {
        if (!_inject || !Application.isPlaying) return;
        if (EditorApplication.timeSinceStartup < _nextInject) return;
        _nextInject = EditorApplication.timeSinceStartup + 1.0 / 30.0;
        long ns = (DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).Ticks * 100;
        var t = new TransformStampedMsg
        {
            header = new HeaderMsg { frame_id = RobotProfile.Selected?.baseFrame, stamp = new TimeMsg { sec = (int)(ns / 1_000_000_000), nanosec = (uint)(ns % 1_000_000_000) } },
            child_frame_id = RobotProfile.Selected?.controllerFrames.hmd,
            transform = new TransformMsg { translation = new Vector3Msg(0, 0, 1.2), rotation = new QuaternionMsg(0, 0, 0, 1) }
        };
        ROSConnection.GetOrCreateInstance().Publish(RosNames.Tf, new TFMessageMsg(new[] { t }));
        _injected++;
    }

    static Process Sh(string script, string args)
    {
        Log($"    {script} {args}");
        return Process.Start(new ProcessStartInfo("/bin/bash", $"\"{Script(script)}\" {args}") { UseShellExecute = false, CreateNoWindow = true });
    }
    static Process Pub(string args) => Sh("pub.sh", args);

    static IEnumerator Settle(Process p, Dictionary<string, double> targets, double timeout = 25)
    {
        double end = Now + timeout;
        while (Now < end)
        {
            bool done = (p == null || p.HasExited) && targets.All(t => Math.Abs(Q(t.Key) - t.Value) < (t.Key.Contains("lift") ? 0.003 : 0.01));
            if (done) break;
            yield return Seconds(0.2);
        }
        Log($"    settle: {string.Join(", ", targets.Select(t => $"{t.Key} q={Q(t.Key):F4} target={t.Value}"))}");
    }
    static IEnumerator WaitExit(Process p, double timeout = 30)
    {
        double end = Now + timeout;
        while (!p.HasExited && Now < end) yield return Seconds(0.1);
    }

    static CameraPanel _toggleOff;
    static float _odomLin, _odomAng; static int _odomCount; static double _stoppedAt;
    // Sends a zero cmd_vel (the sim base keeps its last command) and records when odom says the base is still.
    static double _movingFalseAt;
    static IEnumerator StopBase(BaseVelocity bv)
    {
        var sp = Sh("cmdvel.sh", "stop 6");
        double s0 = Now, stillSince = -1; _stoppedAt = -1;
        while (Now - s0 < 10)
        {
            // The sim's odom twist is jittery (single zero samples while moving): still = 0.4 s of zeros.
            bool still = Mathf.Abs(_odomLin) < 0.005f && Mathf.Abs(_odomAng) < 0.005f;
            if (!still) stillSince = -1;
            else if (stillSince < 0) stillSince = Now;
            else if (Now - stillSince >= 0.4) { _stoppedAt = stillSince; break; }
            yield return Seconds(0.02);
        }
        // First time the arrow logic says "not moving" after the base went still.
        _movingFalseAt = -1;
        while (_stoppedAt > 0 && Now - _stoppedAt < 4)
        {
            if (!bv.Moving) { _movingFalseAt = Now; break; }
            yield return Seconds(0.02);
        }
        Log($"    zero cmd_vel: odom still {(_stoppedAt > 0 ? _stoppedAt - s0 : -1):F2} s after starting the CLI");
        var w = WaitExit(sp); while (w.MoveNext()) yield return w.Current;
    }

    // Hands mode: each card >= 0.2 m from the head->hand line, left card left of the right card in
    // head space, card y = hand y - 0.05 (+-0.02).
    static void HandsPlacement(string tag, Transform head, Transform lc, Transform rc, Transform le, Transform re)
    {
        if (rc == null)   // one arm: the card is outboard to the viewer's right of the hand, below it
        {
            Vector3 d1 = (le.position - head.position).normalized;
            float dist = Vector3.Cross(lc.position - head.position, d1).magnitude;
            float x1 = head.InverseTransformPoint(lc.position).x, hx = head.InverseTransformPoint(le.position).x;
            float y1 = lc.position.y - (le.position.y - 0.05f);
            Check(dist >= 0.2f, $"4 [{tag}]: card to head->hand line {dist:F3} m (>= 0.2)");
            Check(x1 > hx, $"4 [{tag}]: single hand camera: card x {x1:F3} right of the hand x {hx:F3} in head space");
            Check(Mathf.Abs(y1) <= 0.02f, $"4 [{tag}]: card y - (hand y - 0.05): {y1:F3} (+-0.02)");
            return;
        }
        float Dist(Transform card, Transform hand)
        {
            Vector3 d = (hand.position - head.position).normalized;
            return Vector3.Cross(card.position - head.position, d).magnitude;
        }
        float dl = Dist(lc, le), dr = Dist(rc, re);
        float xl = head.InverseTransformPoint(lc.position).x, xr = head.InverseTransformPoint(rc.position).x;
        float yl = lc.position.y - (le.position.y - 0.05f), yr = rc.position.y - (re.position.y - 0.05f);
        Check(dl >= 0.2f && dr >= 0.2f, $"4 [{tag}]: card to head->hand line: L {dl:F3} m, R {dr:F3} m (>= 0.2)");
        Check(xl < xr, $"4 [{tag}]: head-space x: left card {xl:F3} < right card {xr:F3} (hands x L {head.InverseTransformPoint(le.position).x:F3} R {head.InverseTransformPoint(re.position).x:F3})");
        Check(Mathf.Abs(yl) <= 0.02f && Mathf.Abs(yr) <= 0.02f, $"4 [{tag}]: card y - (hand y - 0.05): L {yl:F3}, R {yr:F3} (+-0.02)");
    }

    static Quaternion Look(Transform head, Vector3 target) => Quaternion.LookRotation(target - head.position, Vector3.up);

    static string StripText(StatusStrip s) => s == null ? "" : string.Join(" | ", s.GetComponentsInChildren<TextMeshProUGUI>(true).Select(t => t.text));
    static Quaternion Down(Transform head, float deg) => Quaternion.Euler(deg, head.eulerAngles.y, 0f);

    // ---------------- Main ----------------
    static IEnumerator Main()
    {
        Log("===== experiments batches 2-3");
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.1");
        PlayerPrefs.SetInt($"RobotModel/{Spec.Asset}", 1); PlayerPrefs.SetString($"CameraLayout/{Spec.Asset}", "firstperson");
        PlayerPrefs.Save();

        {
            var h = Sh("home.sh", ""); var w = WaitExit(h, 90); while (w.MoveNext()) yield return w.Current;
            var pre = new[] { Pub("head 0.0 0.0"), Spec.HasLift ? Pub("lift 0.2") : null, Pub("armhome") }.Where(x => x != null).ToArray();
            while (pre.Any(p => !p.HasExited)) yield return Seconds(0.2);
            yield return Seconds(2.0);
        }

        var profile = AssetDatabase.LoadAssetAtPath<RobotProfile>(Spec.ProfilePath);
        EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
        RobotProfile.Selected = profile;
        Object.FindFirstObjectByType<QuestControllerPublisher>().controlRobot = false;
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Seconds(0.1);
        var publisher = Object.FindFirstObjectByType<QuestControllerPublisher>();
        publisher.controlRobot = false;
        yield return Frames(2);

        var ros = ROSConnection.GetOrCreateInstance();
        Check(Object.FindObjectsByType<ROSConnection>(FindObjectsSortMode.None).Length == 1, "exactly one ROSConnection");
        ros.Subscribe<JointStateMsg>(Spec.JointStates, OnJointStates);
        ros.Subscribe<RosMessageTypes.Nav.OdometryMsg>(Spec.Odom, m => { _odomLin = (float)m.twist.twist.linear.x; _odomAng = (float)m.twist.twist.angular.z; _odomCount++; });
        var hud = Object.FindFirstObjectByType<TeleopHud>();
        var images = Object.FindFirstObjectByType<ImageSubscriber>();

        FirstPersonView fpv = null; RobotModel model = null;
        double end = Now + 40;
        while (Now < end)
        {
            fpv = Object.FindFirstObjectByType<FirstPersonView>();
            model = fpv != null ? fpv.Model : null;
            if (model != null && model.AcceptedTransforms > 0 && fpv.FramesReceived > 0 && _jsCount > 0) break;
            yield return Seconds(0.25);
        }
        if (!Check(hud.FirstPerson && model != null && model.AcceptedTransforms > 0 && fpv.FramesReceived > 0 && _jsCount > 0,
                   $"first person live: TF {model?.AcceptedTransforms}, frames {fpv?.FramesReceived}, js {_jsCount}"))
            yield break;
        double LiftErr() => Spec.HasLift ? Math.Abs(Q(Spec.LiftJoint) - 0.2) : 0.0;
        for (int attempt = 0; attempt < 3 && (Math.Abs(Q(Spec.PanJoint)) > 0.01 || Math.Abs(Q(Spec.TiltJoint)) > 0.01 || LiftErr() > 0.005); attempt++)
        {
            var hp = Pub("head 0.0 0.0"); var lp = Spec.HasLift ? Pub("lift 0.2") : hp;
            var targets0 = new Dictionary<string, double> { [Spec.PanJoint] = 0, [Spec.TiltJoint] = 0 };
            if (Spec.HasLift) targets0[Spec.LiftJoint] = 0.2;
            var se = Settle(lp, targets0, 12); while (se.MoveNext()) yield return se.Current;
        }
        Check(Math.Abs(Q(Spec.PanJoint)) < 0.01 && Math.Abs(Q(Spec.TiltJoint)) < 0.01, $"start pose: head pan {Q(Spec.PanJoint):F3} tilt {Q(Spec.TiltJoint):F3}, lift {(Spec.HasLift ? Q(Spec.LiftJoint).ToString("F3") : "none")}");
        fpv.Recenter();
        yield return Seconds(1.5);
        var head = FirstPersonView.Head;
        var strip = Object.FindFirstObjectByType<StatusStrip>();
        Log($"    head {V(head.position)} yaw {head.eulerAngles.y:F1}; model root {V(model.Root.position)} yaw {model.Root.eulerAngles.y:F1}; root frame '{OverlayMaterials.RootFrame(model)}'");

        // ================= 4. hand cams =================
        int nCards = Spec.Sides.Length;
        int li = images.IndexOf(Spec.Sides[0].cameraSuffix);
        int ri = Spec.DualArm ? images.IndexOf(Spec.Sides[1].cameraSuffix) : -1;
        var pip = Object.FindFirstObjectByType<HandCamPip>();
        Check(pip != null && pip.CardCount == nCards, $"4: HandCamPip exists, CardCount {pip?.CardCount} == {nCards}");
        var views = pip.GetComponentsInChildren<RawImage>(true).Where(r => r.name == "View").ToArray();
        end = Now + 5;
        while (Now < end && views.Any(v => v.texture == null)) yield return Seconds(0.1);
        Check(views.Length == nCards && views.All(v => v.texture != null), $"4: all cards got a texture within 5 s ({views.Count(v => v.texture != null)}/{views.Length})");
        int dl0 = images.DroppedFrames(li), dr0 = ri >= 0 ? images.DroppedFrames(ri) : 0;
        yield return Seconds(3.0);
        Check(images.Fps(li) > 5f && (ri < 0 || images.Fps(ri) > 5f),
              $"4: hand cams decoding with blocks hidden: L fps {images.Fps(li):F1} decode {images.DecodeMs(li):F2} ms dropped {dl0}->{images.DroppedFrames(li)}{(ri >= 0 ? $"; R fps {images.Fps(ri):F1} decode {images.DecodeMs(ri):F2} ms dropped {dr0}->{images.DroppedFrames(ri)}" : "")}");
        var cards = pip.GetComponentsInChildren<Canvas>(true).Where(c => c.name.StartsWith("Hand Cam ")).ToArray();
        var leftCard = (Spec.DualArm ? cards.First(c => c.name.Contains("Left")) : cards[0]).transform;
        var rightCard = Spec.DualArm ? cards.First(c => c.name.Contains("Right")).transform : null;
        var leftEff = model.Frame(profile.arms[0].effectorFrame);
        var rightEff0 = Spec.DualArm ? model.Frame(profile.arms[1].effectorFrame) : null;
        HandsPlacement("start", head, leftCard, rightCard, leftEff, rightEff0);
        Vector3 toHead = (head.position - leftCard.position).normalized;
        Check(Vector3.Dot(-leftCard.forward, toHead) > 0.95f, $"4: left card faces the head (dot {Vector3.Dot(-leftCard.forward, toHead):F3})");
        Vector3 lc0 = leftCard.position;
        var p = Pub("arm");
        var e = Settle(p, Spec.ArmMoved); while (e.MoveNext()) yield return e.Current;
        yield return Seconds(1.0);
        float moved = Vector3.Distance(leftCard.position, lc0);
        Check(moved > Spec.MinCardMove, $"4: after pub arm the first card moved {moved:F3} m (> {Spec.MinCardMove}): {V(lc0)} -> {V(leftCard.position)}");
        HandsPlacement("arm", head, leftCard, rightCard, leftEff, rightEff0);
        {   // both arms at home for the picture: the cards outboard of the grippers, the head image between them
            var ph = Pub("armhome");
            var eh = Settle(ph, Spec.ArmReady); while (eh.MoveNext()) yield return eh.Current;
            yield return Seconds(1.0);
            HandsPlacement("armhome", head, leftCard, rightCard, leftEff, rightEff0);
            Log($"    hand card below the eye: first {Vector3.Angle(Vector3.ProjectOnPlane(leftCard.position - head.position, Vector3.up), leftCard.position - head.position):F0} deg");
            Shot("04_handcams_hands", head.position, Down(head, 42f), 90f);
        }

        // round 3: "corners" hand-cam mode removed -> its checks dropped
        // round 3: no "handcams" toggle; the cards go with the first-person layout. Blocks layout (model on):
        // PiP + head card destroyed, every ForceDecode cleared; back to first person: 2 cards again.
        pip.gameObject.SetActive(false);   // picture only: no cards in front of the arms
        Shot("10_key_light", head.position, Look(head, rightEff0 != null ? (leftEff.position + rightEff0.position) / 2f : leftEff.position), 80f);   // arms lit, no cards in front of them
        pip.gameObject.SetActive(true);
        hud.SetCameraLayout("blocks");
        yield return Frames(3);
        var forced = Field<HashSet<int>>(Field<object>(images, "_streams"), "_forceDecode");
        Check(Object.FindFirstObjectByType<HandCamPip>() == null && leftCard == null && fpv != null && fpv.Model != null && !fpv.ImageShown, "4: blocks layout -> PiP and cards destroyed, model kept, no head card");
        Check(!forced.Contains(li) && !forced.Contains(ri) && !forced.Contains(fpv.CameraIndex), $"4: ForceDecode cleared for hand cams and head (forced = [{string.Join(",", forced)}])");
        hud.SetCameraLayout("firstperson");
        yield return Frames(3);
        pip = Object.FindFirstObjectByType<HandCamPip>();
        strip = Object.FindFirstObjectByType<StatusStrip>();
        Check(pip != null && pip.CardCount == nCards && fpv.ImageShown, $"4: first-person layout again -> {nCards} cards, head card");

        // ================= 5. arm targets =================
        var targets = Object.FindFirstObjectByType<ArmTargets>();
        Check(targets != null && !targets.HasLeft && !targets.HasRight, "5: ArmTargets exists, no targets yet");
        var rightEff = rightEff0;
        for (int si = 0; si < nCards; si++)
        {
            var sd = Spec.Sides[si];
            string side = sd.name;
            bool left = si == 0;
            var tp = Sh("targets.sh", $"{sd.targetsArg} 8");
            double t0 = Now;
            Func<bool> has = () => left ? targets.HasLeft : targets.HasRight;
            while (!has() && Now - t0 < 10) yield return Seconds(0.05);
            double seenAfter = Now - t0;
            Check(has(), $"5: {side}: Has{(left ? "Left" : "Right")} true {seenAfter:F2} s after starting `ros2 topic pub` (CLI start-up included)");
            yield return Seconds(1.0);
            var marker = model.Root.Find("Target " + sd.marker);
            Vector3 local = model.Root.InverseTransformPoint(marker.position);
            Vector3 want = new Vector3(sd.markerX, 0.9f, 0.4f);
            Check((local - want).magnitude < 0.01f, $"5: {side}: marker in model-root space {V(local)} ~ {V(want)}");
            var eff = left ? leftEff : rightEff;
            float err = left ? targets.LeftErrorM : targets.RightErrorM;
            float d = Vector3.Distance(marker.position, eff.position);
            Check(err > 0f && Mathf.Abs(err - d) < 0.01f, $"5: {side}: error {err:F3} m == |marker - effector| {d:F3}");
            var line = targets.GetComponentsInChildren<LineRenderer>(true).First(l => l.name.EndsWith(" " + sd.marker));
            Check(line.enabled && Vector3.Distance(line.GetPosition(0), marker.position) < 0.001f && Vector3.Distance(line.GetPosition(1), eff.position) < 0.001f, $"5: {side}: line enabled from marker to effector");
            var w = WaitExit(tp); while (w.MoveNext()) yield return w.Current;
            double stop = Now;
            while (has() && Now - stop < 3) yield return Seconds(0.05);
            Check(!has() && Now - stop < 1.6, $"5: {side}: hidden {Now - stop:F2} s after the publisher stopped; marker active {marker.gameObject.activeSelf}");
        }
        p = Pub("armhome");
        var tb = Sh("targets.sh", "both 14");
        e = Settle(p, Spec.ArmReady); while (e.MoveNext()) yield return e.Current;
        Func<bool> allTargets = () => targets.HasLeft && (!Spec.DualArm || targets.HasRight);
        end = Now + 8;
        while (Now < end && !allTargets()) yield return Seconds(0.1);
        yield return Seconds(1.0);
        Check(allTargets(), $"5: all targets shown; errors L {targets.LeftErrorM:F3} m{(Spec.DualArm ? $" R {targets.RightErrorM:F3} m" : "")}");
        pip.gameObject.SetActive(false);   // picture only: the cards hang in front of the markers
        yield return Frames(3);
        {
            var ml = model.Root.Find("Target " + Spec.Sides[0].marker).position;
            var mr = Spec.DualArm ? model.Root.Find("Target " + Spec.Sides[1].marker).position : ml;
            Shot("05_targets", head.position, Look(head, (ml + mr + leftEff.position + (rightEff != null ? rightEff.position : leftEff.position)) / 4f), 80f);
        }
        pip.gameObject.SetActive(true);
        { var w = WaitExit(tb); while (w.MoveNext()) yield return w.Current; }

        // ================= 6. base velocity (v2: filled meshes + value chip) =================
        var bv = Object.FindFirstObjectByType<BaseVelocity>();
        Check(bv != null && !bv.Moving, $"6: BaseVelocity exists, not moving (lin {bv?.Linear}, ang {bv?.Angular:F3})");
        var arrow = Field<MeshRenderer>(bv, "_arrowR"); var arc = Field<MeshRenderer>(bv, "_arcR");
        var arrowT = Field<Transform>(bv, "_arrowT"); var chipT = Field<Transform>(bv, "_chipT");
        var chipText = Field<TextMeshProUGUI>(bv, "_chipText");
        Check(arrow != null && arc != null && bv.GetComponentsInChildren<LineRenderer>(true).Length == 0,
              $"6: arrow and arc are MeshRenderers ({arrow?.GetType().Name}, {arc?.GetType().Name}), no LineRenderer under BaseVelocity");
        Check(arrow.sharedMaterial != null && arrow.sharedMaterial.renderQueue >= 3000 && arrow.sharedMaterial.IsKeywordEnabled("_SURFACE_TYPE_TRANSPARENT"),
              $"6: arrow material '{arrow.sharedMaterial?.shader?.name}' transparent (queue {arrow.sharedMaterial?.renderQueue}, colour a {arrow.sharedMaterial?.color.a:F2})");
        Check(!arrow.enabled && !arc.enabled && !chipT.gameObject.activeSelf, "6: still -> arrow, arc, chip hidden");
        var cp = Sh("cmdvel.sh", "lin 7");
        double c0 = Now; double movingAt = -1; float bestLin = 0f; bool arrowOk = false; float dirDot = 0f; string chipSeen = "";
        while (!cp.HasExited && Now - c0 < 10 && !(movingAt >= 0 && arrowOk && Now - c0 - movingAt > 2.0))
        {
            if (bv.Moving && movingAt < 0) movingAt = Now - c0;
            if (bv.Moving)
            {
                if (Mathf.Abs(bv.Linear.x - 0.15f) < Mathf.Abs(bestLin - 0.15f)) bestLin = bv.Linear.x;
                if (arrow.enabled && arrow.gameObject.activeInHierarchy && arrowT.GetComponent<MeshFilter>().sharedMesh.bounds.size.z > 0.3f)
                {
                    Vector3 fwd = Vector3.ProjectOnPlane(model.Root.forward, Vector3.up).normalized;
                    dirDot = Vector3.Dot(arrowT.forward, fwd); arrowOk = true;
                    if (chipT.gameObject.activeInHierarchy) chipSeen = chipText.text;
                }
            }
            yield return Seconds(0.05);
        }
        Check(movingAt >= 0, $"6: Moving true {movingAt:F2} s after starting the cmd_vel CLI");
        Check(Mathf.Abs(bestLin - 0.15f) < 0.05f, $"6: Linear.x reached {bestLin:F3} (~0.15 +-0.05)");
        Check(arrowOk && dirDot > 0.9f, $"6: arrow mesh shown during the burst, along the root's +Z (dot {dirDot:F3})");
        Check(chipSeen.Contains("m/s"), $"6: value chip shown with '{chipSeen}' (contains m/s)");
        Log($"    burst stopped early; odom twist now lin {_odomLin:F3} ang {_odomAng:F3} (the sim base keeps the last cmd_vel)");
        var e6 = StopBase(bv); while (e6.MoveNext()) yield return e6.Current;
        Check(_stoppedAt > 0 && _movingFalseAt > 0 && !bv.Moving && !arrow.enabled && !chipT.gameObject.activeSelf && _movingFalseAt - _stoppedAt < 2.0, $"6: Moving false {_movingFalseAt - _stoppedAt:F2} s after odom reported the base still; arrow + chip hidden");
        cp = Sh("cmdvel.sh", "ang 7");
        c0 = Now; float bestAng = 0f; bool arcOk = false; string chipAng = "";
        double angAt = -1;
        while (!cp.HasExited && Now - c0 < 10 && !(angAt >= 0 && Now - angAt > 2.0))
        {
            if (bv.Moving && angAt < 0) angAt = Now;
            if (bv.Moving && Mathf.Abs(bv.Angular - 0.5f) < Mathf.Abs(bestAng - 0.5f)) bestAng = bv.Angular;
            if (arc.enabled && arc.gameObject.activeInHierarchy && arc.GetComponent<MeshFilter>().sharedMesh.vertexCount > 0) { arcOk = true; chipAng = chipText.text; }
            yield return Seconds(0.05);
        }
        Check(Mathf.Abs(bestAng - 0.5f) < 0.15f && arcOk, $"6: Angular reached {bestAng:F3} (~0.5 +-0.15), turn arc mesh shown {arcOk}, chip '{chipAng}'");
        e6 = StopBase(bv); while (e6.MoveNext()) yield return e6.Current;
        Check(_stoppedAt > 0 && _movingFalseAt > 0 && !bv.Moving && !arc.enabled && _movingFalseAt - _stoppedAt < 2.0, $"6: still again {_movingFalseAt - _stoppedAt:F2} s after odom reported the turn stopped");
        { var y = Sh("home.sh", ""); var w = WaitExit(y, 90); while (w.MoveNext()) yield return w.Current; }
        yield return Seconds(1.5);
        // picture: linear + angular burst, looking down ~40 deg
        cp = Sh("cmdvel.sh", "both 6");
        c0 = Now; bool shot6 = false;
        while (!cp.HasExited && Now - c0 < 10 && !shot6)
        {
            if (arrow.enabled && arc.enabled && bv.Linear.x > 0.12f && Mathf.Abs(bv.Angular) > 0.35f && Now - c0 > 3.0)
            {
                shot6 = true;
                Log($"    06 shot: lin {bv.Linear.x:F3} ang {bv.Angular:F3} chip '{chipText.text}'");
                Shot("06_basevel", head.position, Down(head, 40f), 80f);
                Shot("06_basevel_fov100", head.position, Down(head, 40f), 100f);
                Shot("06_basevel_down55", head.position, Down(head, 55f), 80f);
            }
            yield return Seconds(0.05);
        }
        Check(shot6, "6: arrow + arc shown together during a linear + angular burst (picture taken)");
        e6 = StopBase(bv); while (e6.MoveNext()) yield return e6.Current;
        { var y = Sh("home.sh", ""); var w = WaitExit(y, 90); while (w.MoveNext()) yield return w.Current; }
        yield return Seconds(1.5);
        Log($"    yaw restored: model root yaw {model.Root.eulerAngles.y:F1}");

        // ================= 10. lowered bar (always 0.7 scale), shown by the menu toggle =================
        var barT = Field<Transform>(hud, "_bar");
        var hudBar = barT.GetComponent<HudBar>();
        Vector3 normalPos = Field<Vector3>(Field<object>(hud, "_view"), "_barNormalPos");
        Vector3 builtScale = Vector3.one * (HudBar.CompactScale / HudUi.MmPerMetre);
        Check(hud.BarLowered && !barT.gameObject.activeSelf, $"10: first person: BarLowered {hud.BarLowered}, bar hidden");
        int toggles0 = LogCount("[TeleopHud] menu -> bar");
        typeof(TeleopHud).GetMethod("ToggleBar", BindingFlags.NonPublic | BindingFlags.Instance).Invoke(hud, null);
        yield return Frames(3);
        Check(barT.gameObject.activeSelf && LogCount("[TeleopHud] menu -> bar") == toggles0 + 1, "10: menu toggle -> bar shown, toggled once");
        Check(Vector3.Distance(barT.localScale, builtScale) < 1e-6f, $"10: bar scale {barT.localScale.x * 1000f:F3}/1000 == 0.7/1000 in first person too");
        Check(Vector3.Distance(barT.localPosition, normalPos + Vector3.down * 0.55f) < 1e-4f, $"10: bar local pos {V(barT.localPosition)} == normal {V(normalPos)} - 0.55 m y");
        yield return Seconds(0.5);
        {
            var vRt = Field<RectTransform>(fpv, "_viewRt");
            var bc = new Vector3[4]; ((RectTransform)barT).GetWorldCorners(bc);
            var qc = new Vector3[4]; vRt.GetWorldCorners(qc);
            // gap between two screen-space edges over their shared x range (>0: edge `lo` lies below edge `hi`)
            float EdgeGap(Vector3 l0, Vector3 l1, Vector3 h0, Vector3 h1)
            {
                if (l0.x > l1.x) (l0, l1) = (l1, l0); if (h0.x > h1.x) (h0, h1) = (h1, h0);
                float x0 = Mathf.Max(l0.x, h0.x), x1 = Mathf.Min(l1.x, h1.x), g = float.MaxValue;
                if (x1 < x0) return g;
                for (int k = 0; k <= 20; k++)
                {
                    float x = Mathf.Lerp(x0, x1, k / 20f);
                    float yl = Mathf.Lerp(l0.y, l1.y, Mathf.InverseLerp(l0.x, l1.x, x)), yh = Mathf.Lerp(h0.y, h1.y, Mathf.InverseLerp(h0.x, h1.x, x));
                    g = Mathf.Min(g, yh - yl);
                }
                return g;
            }
            foreach (var (nm, rot, fov) in new[] { ("shot", Quaternion.Euler(24f, head.eulerAngles.y + 7f, 0f), 50f), ("head", head.rotation, 90f) })
            {
                var go = new GameObject("ProjCam"); var cam = go.AddComponent<Camera>(); var rt = new RenderTexture(1280, 720, 24);
                go.transform.SetPositionAndRotation(head.position, rot); cam.stereoTargetEye = StereoTargetEyeMask.None; cam.targetTexture = rt; cam.fieldOfView = fov; cam.aspect = 16f / 9f;
                Vector3 P(Vector3 w) => cam.WorldToScreenPoint(w);
                float gBar = EdgeGap(P(bc[1]), P(bc[2]), P(qc[0]), P(qc[3]));
                Log($"    10: {nm} view: bar top ({P(bc[1]).x:F0},{P(bc[1]).y:F0})-({P(bc[2]).x:F0},{P(bc[2]).y:F0}) quad bottom ({P(qc[0]).x:F0},{P(qc[0]).y:F0})-({P(qc[3]).x:F0},{P(qc[3]).y:F0})");
                cam.targetTexture = null; Object.DestroyImmediate(rt); Object.DestroyImmediate(go);
                Check(gBar > 0f, $"10: {nm} view (1280x720): bar top edge below the image quad's bottom edge, min gap {gBar:F0} px over the shared x range");
            }
        }
        Shot("10_bar_compact", head.position, Quaternion.Euler(24f, head.eulerAngles.y + 7f, 0f), 50f);
        typeof(TeleopHud).GetMethod("ToggleBar", BindingFlags.NonPublic | BindingFlags.Instance).Invoke(hud, null);
        yield return Frames(3);
        Check(!barT.gameObject.activeSelf, "10: second toggle -> bar hidden again");

        // ================= 3 (FPV card): head-camera card + label =================
        {
            var cardRt = Field<RectTransform>(fpv, "_cardRt"); var nameT = Field<TextMeshProUGUI>(fpv, "_name");
            var viewRt = Field<RectTransform>(fpv, "_viewRt");
            string want = images.Panels[fpv.CameraIndex].Label;
            Check(cardRt != null && cardRt.gameObject.activeInHierarchy && nameT != null && nameT.text == want, $"FPV card: exists, label '{nameT?.text}' == head panel Label '{want}'");
            var cc = new Vector3[4]; var vc = new Vector3[4]; var nc = new Vector3[4];
            cardRt.GetWorldCorners(cc); viewRt.GetWorldCorners(vc); nameT.rectTransform.GetWorldCorners(nc);
            Vector3 L(Vector3 w) => cardRt.InverseTransformPoint(w);
            Rect R(Vector3[] c) { var a0 = L(c[0]); var a2 = L(c[2]); return Rect.MinMaxRect(Mathf.Min(a0.x, a2.x), Mathf.Min(a0.y, a2.y), Mathf.Max(a0.x, a2.x), Mathf.Max(a0.y, a2.y)); }
            Rect rc = R(cc), rv = R(vc), rn = R(nc);
            bool encl(Rect o, Rect i) => i.xMin >= o.xMin - 1e-3f && i.xMax <= o.xMax + 1e-3f && i.yMin >= o.yMin - 1e-3f && i.yMax <= o.yMax + 1e-3f;
            Check(encl(rc, rv), $"FPV card: card rect {rc} encloses the image {rv}");
            Check(encl(rc, rn) && rn.yMin >= rv.yMax - 1e-3f, $"FPV card: label {rn} inside the card and above the image");
            Log($"    FPV card size {rc.width:F0} x {rc.height:F0} (canvas units), label font {nameT.fontSize:F1}, label preferred width {nameT.preferredWidth:F0}");
            var c0w = (cc[0] + cc[2]) / 2f;
            var top = (nc[1] + nc[2]) / 2f;
            Shot("02b_fpv_card", head.position, Look(head, c0w), 72f);
        }

        // ================= toggle bug: camera toggles while the blocks are hidden =================
        {
            var headPanel = images.Panels[fpv.CameraIndex];
            bool AllInactive() => images.Panels.All(pp => !pp.gameObject.activeSelf);
            // through the bar's camera toggles (what the user clicks), like the bug report
            Toggle BarToggle(CameraPanel cp) => barT.GetComponentsInChildren<Toggle>(true).FirstOrDefault(t => t.GetComponentInChildren<TextMeshProUGUI>()?.text == cp.Label);
            var headTgl = BarToggle(headPanel);
            Check(headTgl != null && headTgl.isOn, $"toggle: bar toggle for '{headPanel.Label}' found, on ({headTgl?.isOn}) while hidden");
            headTgl.isOn = false;
            yield return Frames(2);
            Check(AllInactive() && !images.IsOn(headPanel) && !images.BlocksShown, $"toggle: hidden blocks, head off -> all {images.Panels.Count} panels inactive, IsOn(head) {images.IsOn(headPanel)}");
            headTgl.isOn = true;
            yield return Frames(2);
            Check(AllInactive() && images.IsOn(headPanel), $"toggle: head on again while hidden -> all panels still inactive, IsOn(head) {images.IsOn(headPanel)}");
            Check(fpv.FramesReceived > 0 && Field<HashSet<int>>(Field<object>(images, "_streams"), "_forceDecode").Contains(fpv.CameraIndex), "toggle: head camera still force-decoded for the FPV image");
            images.ResetLayout();
            yield return Frames(2);
            Check(AllInactive() && images.Panels.All(images.IsOn), $"toggle: ResetLayout while hidden -> all inactive, all IsOn ({images.Panels.Count(images.IsOn)}/{images.Panels.Count})");
            var off = images.Panels[li];
            Check(BarToggle(off) != null && BarToggle(off).isOn, "toggle: after ResetLayout the bar toggles show on");
            BarToggle(off).isOn = false;
            yield return Frames(2);
            Check(AllInactive() && !images.IsOn(off), $"toggle: '{off.Label}' off while hidden -> all inactive, IsOn false");
            _toggleOff = off;
        }

        // ================= 8. head lock =================
        var canvas = Field<RectTransform>(fpv, "_canvasRt");
        var camFrame = Field<Transform>(fpv, "_camFrame");
        Check(canvas.name == "FPV Image" && canvas.parent == camFrame, $"8: '{canvas.name}' under the camera frame '{camFrame?.name}'");
        // round 3: "headlock" removed -> its checks dropped

        // ================= 10. leave first person =================
        hud.SetRobotModel(false);
        yield return Frames(3);
        Check(!hud.BarLowered && Vector3.Distance(barT.localPosition, normalPos) < 1e-4f && Vector3.Distance(barT.localScale, builtScale) < 1e-6f && barT.gameObject.activeSelf,
              $"10: blocks mode -> bar at its built pose {V(barT.localPosition)}, scale {barT.localScale.x * 1000f:F3}/1000 (0.7)");
        {
            var off = _toggleOff;
            var others = images.Panels.Where(pp => pp != off).ToList();
            Check(!off.gameObject.activeSelf && !images.IsOn(off) && others.All(pp => pp.gameObject.activeSelf && images.IsOn(pp)),
                  $"toggle: after SetBlocksShown(true) '{off.Label}' stays hidden, the other {others.Count} shown ({others.Count(pp => pp.gameObject.activeSelf)})");
            var tgl = barT.GetComponentsInChildren<Toggle>(true).FirstOrDefault(t => t.GetComponentInChildren<TextMeshProUGUI>()?.text == off.Label);
            Check(tgl != null && !tgl.isOn, $"toggle: bar toggle for '{off.Label}' shows off ({tgl?.isOn})");
            images.SetCameraVisible(off, true);
            yield return Frames(2);
            Check(off.gameObject.activeSelf, "toggle: turned back on in blocks mode -> shown");
        }
        Check(Object.FindFirstObjectByType<HandCamPip>() == null && Object.FindFirstObjectByType<ArmTargets>() == null && Object.FindFirstObjectByType<BaseVelocity>() == null,
              "4/5/6: leaving first person destroys PiP, targets, base velocity");
        Check(model == null || model.Root.Find("Target " + Spec.Sides[0].marker) == null, "5: target markers gone with the model");

        // ================= second robot screen: statics survive, ROSConnection is new =================
        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Seconds(0.1);
        { var saved = Settings.For(profile); saved.ModelOn = true; saved.Layout = "firstperson"; Settings.Save(); }   // already migrated in this run: seed the new keys
        EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
        RobotProfile.Selected = profile;
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Seconds(0.1);
        publisher = Object.FindFirstObjectByType<QuestControllerPublisher>();
        publisher.controlRobot = false;
        end = Now + 40;
        while (Now < end)
        {
            fpv = Object.FindFirstObjectByType<FirstPersonView>();
            if (fpv != null && fpv.Model != null && fpv.Model.AcceptedTransforms > 0) break;
            yield return Seconds(0.25);
        }
        Check(fpv != null && fpv.Model != null, "2nd screen: first person live again");
        yield return Seconds(1.0);
        var rtt = Object.FindFirstObjectByType<RoundTrip>();
        publisher.controlRobot = true;
        _inject = true;
        int s0 = rtt.Samples;
        yield return Seconds(3.0);
        _inject = false;
        Check(rtt.Samples - s0 > 10, $"2nd screen: RTT samples {rtt.Samples - s0} on the new ROSConnection (subscription redone)");
        publisher.controlRobot = false;
        targets = Object.FindFirstObjectByType<ArmTargets>();
        var t2 = Sh("targets.sh", Spec.Sides[0].targetsArg + " 9");
        double tt0 = Now;
        while (!targets.HasLeft && Now - tt0 < 12) yield return Seconds(0.1);
        Check(targets.HasLeft, $"2nd screen: arm target received on the new ROSConnection ({Now - tt0:F2} s after starting the CLI)");
        { var w = WaitExit(t2); while (w.MoveNext()) yield return w.Current; }
        Check(Object.FindObjectsByType<ROSConnection>(FindObjectsSortMode.None).Length == 1, "2nd screen: exactly one ROSConnection");
        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Seconds(0.1);
    }

    static void Shot(string name, Vector3 pos, Quaternion rot, float fov)
    {
        var go = new GameObject("VerifyCam");
        var cam = go.AddComponent<Camera>();
        go.transform.SetPositionAndRotation(pos, rot);
        var main = Camera.main;
        cam.fieldOfView = fov; cam.aspect = 16f / 9f;
        cam.nearClipPlane = 0.05f; cam.farClipPlane = 200f;
        cam.stereoTargetEye = StereoTargetEyeMask.None;
        if (main != null) { cam.clearFlags = main.clearFlags; cam.backgroundColor = main.backgroundColor; }
        var rt = new RenderTexture(1280, 720, 24);
        cam.targetTexture = rt;
        cam.Render();
        RenderTexture.active = rt;
        var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;
        File.WriteAllBytes(Path.Combine(ShotDir, name + ".png"), tex.EncodeToPNG());
        cam.targetTexture = null; Object.DestroyImmediate(rt); Object.DestroyImmediate(go); Object.DestroyImmediate(tex);
        Log($"    screenshot {name}.png (fov {fov:F1})");
    }
}
