// Verify harness: First-person view: static prefab checks plus live joint following (RunStatic / RunLive variants).
// Needs the live sim (HOME) on 127.0.0.1:10000 and no other ROS client (stop the app on the headset).
// Run: tools/verify.sh --suite FirstPersonVerify   (or Unity -batchmode -projectPath <copy> -executeMethod FirstPersonVerify.Run)
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Xml;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;

// Copy-only harness: first-person view (static prefab checks + live joint following against the sim).
//   Run       = static + live
//   RunStatic = prefab checks only
//   RunLive   = live checks only
public static class FirstPersonVerify
{
    const string PrefabPath = "Assets/Robots/Models/SOBIT_HOME.prefab";
    const string UrdfPath = "Assets/Robots/Models/sobit_home/sobit_home.urdf";
    const string LodDir = "Assets/Robots/Models/sobit_home/meshes_lod";

    static IEnumerator _run;
    static int _failures, _passes;
    static string ShotDir
    {
        get
        {
            var d = Environment.GetEnvironmentVariable("VERIFY_SHOTS");
            if (string.IsNullOrEmpty(d)) d = Path.GetFullPath(Path.Combine(Application.dataPath, "../../shots"));
            Directory.CreateDirectory(d);
            return d;
        }
    }
    static string PubScript => VerifyPaths.Tool("pub.sh");

    public static void Run() => Start(true, true);
    public static void RunStatic() => Start(true, false);
    public static void RunLive() => Start(false, true);

    static void Start(bool st, bool live)
    {
        EditorSettings.enterPlayModeOptionsEnabled = true;
        EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.DisableDomainReload;
        _run = Main(st, live);
        EditorApplication.update += Tick;
    }

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
    static void Log(string m) => Debug.Log("[Verify] " + m);
    static bool Check(bool ok, string what) { if (ok) _passes++; else _failures++; Log((ok ? "PASS " : "FAIL ") + what); return ok; }

    static IEnumerator Main(bool st, bool live)
    {
        if (st) StaticChecks();
        if (live) { var e = Live(); while (e.MoveNext()) yield return e.Current; }
    }

    // ---------------- URDF ----------------

    class Joint { public string name, type, parent, child; public Vector3 pos; public Quaternion rot = Quaternion.identity; public Vector3 axis = Vector3.forward; }
    static Dictionary<string, Joint> _joints;   // by name
    static List<string> _urdfLinks;

    static Vector3 V(string s)
    {
        var p = (s ?? "0 0 0").Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries).Select(x => float.Parse(x, System.Globalization.CultureInfo.InvariantCulture)).ToArray();
        return new Vector3(p[0], p[1], p[2]);
    }
    static Vector3 RosToUnity(Vector3 v) => new Vector3(-v.y, v.z, v.x);
    static Quaternion Rpy(Vector3 rpy)
        => Quaternion.AngleAxis(-rpy.z * Mathf.Rad2Deg, Vector3.up) * Quaternion.AngleAxis(rpy.y * Mathf.Rad2Deg, Vector3.right) * Quaternion.AngleAxis(-rpy.x * Mathf.Rad2Deg, Vector3.forward);

    static void LoadUrdf()
    {
        if (_joints != null) return;
        var doc = new XmlDocument();
        doc.Load(UrdfPath);
        _joints = new Dictionary<string, Joint>();
        _urdfLinks = new List<string>();
        foreach (XmlElement l in doc.DocumentElement.SelectNodes("link")) _urdfLinks.Add(l.GetAttribute("name"));
        foreach (XmlElement j in doc.DocumentElement.SelectNodes("joint"))
        {
            var jt = new Joint { name = j.GetAttribute("name"), type = j.GetAttribute("type"),
                parent = ((XmlElement)j.SelectSingleNode("parent")).GetAttribute("link"),
                child = ((XmlElement)j.SelectSingleNode("child")).GetAttribute("link") };
            if (j.SelectSingleNode("origin") is XmlElement o)
            {
                jt.pos = RosToUnity(V(o.HasAttribute("xyz") ? o.GetAttribute("xyz") : null));
                jt.rot = Rpy(V(o.HasAttribute("rpy") ? o.GetAttribute("rpy") : null));
            }
            if (j.SelectSingleNode("axis") is XmlElement a) jt.axis = RosToUnity(V(a.GetAttribute("xyz"))).normalized;
            else jt.axis = RosToUnity(new Vector3(1, 0, 0));
            _joints[jt.name] = jt;
        }
    }

    // Expected local pose of a joint's child link at joint position q.
    static void Expected(Joint j, double q, out Vector3 pos, out Quaternion rot)
    {
        pos = j.pos; rot = j.rot;
        if (j.type == "revolute" || j.type == "continuous") rot = j.rot * Quaternion.AngleAxis(-(float)q * Mathf.Rad2Deg, j.axis);
        else if (j.type == "prismatic") pos = j.pos + j.rot * (j.axis * (float)q);
    }

    // ---------------- Static ----------------

    static void StaticChecks()
    {
        Log("===== static prefab checks");
        LoadUrdf();
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        if (!Check(prefab != null, "prefab exists " + PrefabPath)) return;
        var go = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        go.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
        var root = go.transform;

        var links = go.GetComponentsInChildren<RobotLink>(true);
        Check(links.Length == 85 && links.Length == _urdfLinks.Count, $"link count {links.Length} (expected 85, URDF {_urdfLinks.Count})");
        var byFrame = links.ToDictionary(l => l.frame);
        Check(_urdfLinks.All(byFrame.ContainsKey), "every URDF link has a RobotLink");
        var bad = new List<string>();
        int roots = 0;
        foreach (var l in links)
        {
            var parentLink = l.transform.parent != null ? l.transform.parent.GetComponent<RobotLink>() : null;
            if (l.isRoot) { roots++; if (!string.IsNullOrEmpty(l.parentFrame) || parentLink != null) bad.Add(l.frame + "(root)"); }
            else if (parentLink == null || parentLink.frame != l.parentFrame) bad.Add($"{l.frame}: parentFrame={l.parentFrame} transformParent={(parentLink ? parentLink.frame : "none")}");
        }
        Check(bad.Count == 0 && roots == 1, $"parentFrame matches Transform parent for all links, one root ({roots}) {string.Join("; ", bad.Take(5))}");
        // URDF joint origins baked: fixed and movable joints at q=0
        var originErr = new List<string>(); float worstP = 0, worstR = 0;
        foreach (var j in _joints.Values)
        {
            if (!byFrame.TryGetValue(j.child, out var c)) continue;
            Expected(j, 0, out var p, out var r);
            float dp = (c.transform.localPosition - p).magnitude, dr = Quaternion.Angle(c.transform.localRotation, r);
            worstP = Mathf.Max(worstP, dp); worstR = Mathf.Max(worstR, dr);
            if (dp > 1e-4f || dr > 0.1f) originErr.Add($"{j.name} dp={dp:F4} dr={dr:F2}");
        }
        Check(originErr.Count == 0, $"joint origins baked = URDF (worst {worstP * 1000:F2} mm, {worstR:F3} deg) {string.Join("; ", originErr.Take(5))}");

        long tris = 0; int meshes = 0;
        foreach (var mf in go.GetComponentsInChildren<MeshFilter>(true))
        {
            if (mf.sharedMesh == null) continue;
            meshes++;
            for (int i = 0; i < mf.sharedMesh.subMeshCount; i++) tris += mf.sharedMesh.GetIndexCount(i) / 3;
        }
        Check(tris <= 250000, $"triangles {tris} <= 250000 ({meshes} mesh renderers)");

        var rends = go.GetComponentsInChildren<Renderer>(true);
        Bounds all = rends[0].bounds; foreach (var r in rends) all.Encapsulate(r.bounds);
        Log($"    union bounds min={all.min:F3} max={all.max:F3} size={all.size:F3}");
        Check(all.size.y >= 1.2f && all.size.y <= 1.9f, $"height {all.size.y:F3} in [1.2,1.9]");
        Check(all.min.y >= -0.05f && all.min.y <= 0.1f, $"min y {all.min.y:F3} in [-0.05,0.1]");
        bool first = true; Bounds body = default;
        foreach (var r in rends)
        {
            var owner = r.GetComponentInParent<RobotLink>();
            if (owner == null || owner.frame.StartsWith("arm_") || owner.frame.StartsWith("hand_")) continue;
            if (first) { body = r.bounds; first = false; } else body.Encapsulate(r.bounds);
        }
        Check(body.size.x < 0.8f && body.size.z < 0.8f, $"base/torso (no arm_/hand_) extent x={body.size.x:F3} z={body.size.z:F3} < 0.8");
        float lx = byFrame["arm_left_base_link"].transform.position.x, rx = byFrame["arm_right_base_link"].transform.position.x;
        Check(lx < 0 && rx > 0, $"arm_left_base x={lx:F3} < 0 < arm_right_base x={rx:F3}");
        var cam = byFrame["head_camera_color_frame"].transform;
        float camAngle = Vector3.Angle(cam.forward, root.forward);
        Check(cam.position.y > 0.9f && camAngle < 5f, $"head_camera_color_frame y={cam.position.y:F3} > 0.9, forward angle {camAngle:F2} < 5 (pos {cam.position:F3})");
        Log("    wheel_ links y: " + string.Join(", ", links.Where(l => l.frame.StartsWith("wheel_")).Select(l => $"{l.frame}={l.transform.position.y:F3}")));
        // Steering bases/steering links sit above the wheels by URDF (0.319 / 0.223 m); the wheels themselves must be low.
        var wheels = links.Where(l => l.frame.StartsWith("wheel_drive_")).ToList();
        var highWheels = wheels.Where(l => l.transform.position.y >= 0.2f).Select(l => $"{l.frame}={l.transform.position.y:F3}").ToList();
        Check(wheels.Count > 0 && highWheels.Count == 0, $"{wheels.Count} wheel_drive_ links y < 0.2 {string.Join(", ", highWheels)}");

        var far = new List<(float d, string what)>();
        foreach (var r in rends)
        {
            var owner = r.GetComponentInParent<RobotLink>();
            float d = (r.bounds.center - owner.transform.position).magnitude;
            far.Add((d, $"{owner.frame}/{r.name}({(r.GetComponent<MeshFilter>()?.sharedMesh?.name)})={d:F3}"));
        }
        far.Sort((a, b) => b.d.CompareTo(a.d));
        Check(far[0].d <= 0.35f, $"renderer centres within 0.35 m of link origin; worst: {string.Join(", ", far.Take(4).Select(f => f.what))}");

        // STL winding: signed volume (ROS axes, before the Unity mirror) must be positive.
        var stls = Directory.GetFiles(LodDir, "*.stl", SearchOption.AllDirectories);
        var neg = new List<string>(); var vols = new List<string>();
        foreach (var f in stls)
        {
            double v = StlSignedVolume(f, out int n);
            vols.Add($"{Path.GetFileName(f)}:{v:E2}");
            if (!(v > 0)) neg.Add($"{f.Substring(LodDir.Length + 1)} vol={v:E3} tris={n}");
        }
        Check(stls.Length > 0 && neg.Count == 0, $"signed volume > 0 for all {stls.Length} LOD STLs {string.Join("; ", neg)}");
        Object.DestroyImmediate(go);
    }

    static double StlSignedVolume(string path, out int n)
    {
        var b = File.ReadAllBytes(path);
        n = BitConverter.ToInt32(b, 80);
        if (84 + 50L * n != b.Length) throw new Exception($"{path}: not a binary STL ({b.Length} bytes, {n} tris)");
        double vol = 0;
        for (int i = 0; i < n; i++)
        {
            int o = 84 + 50 * i + 12;
            Vector3 a = new Vector3(BitConverter.ToSingle(b, o), BitConverter.ToSingle(b, o + 4), BitConverter.ToSingle(b, o + 8));
            Vector3 c = new Vector3(BitConverter.ToSingle(b, o + 12), BitConverter.ToSingle(b, o + 16), BitConverter.ToSingle(b, o + 20));
            Vector3 d = new Vector3(BitConverter.ToSingle(b, o + 24), BitConverter.ToSingle(b, o + 28), BitConverter.ToSingle(b, o + 32));
            vol += Vector3.Dot(a, Vector3.Cross(c, d)) / 6.0;
        }
        return vol;
    }

    // ---------------- Live ----------------

    static readonly Dictionary<string, double> _q = new Dictionary<string, double>();
    static int _jsCount;
    static bool _jsSubscribed;
    static void OnJointStates(JointStateMsg m)
    {
        if (m?.name == null || m.position == null) return;
        _jsCount++;
        for (int i = 0; i < m.name.Length && i < m.position.Length; i++) _q[m.name[i]] = m.position[i];
    }

    static Process Pub(string args)
    {
        Log($"    pub {args}");
        var p = Process.Start(new ProcessStartInfo("/bin/bash", $"\"{PubScript}\" {args}") { UseShellExecute = false, CreateNoWindow = true });
        return p;
    }

    static double Q(string joint) => _q.TryGetValue(joint, out var v) ? v : double.NaN;

    // Wait for the publisher to finish and the joints to reach their targets (then 1 s more).
    static IEnumerator Settle(Process p, Dictionary<string, double> targets, double timeout = 25)
    {
        double end = EditorApplication.timeSinceStartup + timeout;
        while (EditorApplication.timeSinceStartup < end)
        {
            bool done = (p == null || p.HasExited) && targets.All(t => Math.Abs(Q(t.Key) - t.Value) < (t.Key.Contains("lift") ? 0.003 : 0.01));
            if (done) break;
            yield return Seconds(0.2);
        }
        bool ok = targets.All(t => Math.Abs(Q(t.Key) - t.Value) < 0.03);
        Log($"    settle: {string.Join(", ", targets.Select(t => $"{t.Key} q={Q(t.Key):F4} target={t.Value}"))} {(ok ? "" : "(NOT REACHED)")}");
        yield return Seconds(1.0);
    }

    // Compare every non-fixed joint of the model with joint_states.
    static void CompareJoints(RobotModel model, string tag, IEnumerable<string> only, float maxDeg = 1.5f, float maxMm = 5f)
    {
        var errs = new List<(float, string)>();
        int n = 0;
        foreach (var name in only)
        {
            var j = _joints[name];
            var t = model.Frame(j.child);
            if (t == null || !_q.ContainsKey(name)) { errs.Add((999, name + " missing")); continue; }
            Expected(j, _q[name], out var p, out var r);
            float dr = Quaternion.Angle(t.localRotation, r), dp = (t.localPosition - p).magnitude * 1000f;
            n++;
            bool bad = dr > maxDeg || dp > maxMm;
            errs.Add((dr + dp / 10f, $"{name} q={_q[name]:F3} dRot={dr:F2}deg dPos={dp:F1}mm{(bad ? " BAD" : "")}"));
        }
        errs.Sort((a, b) => b.Item1.CompareTo(a.Item1));
        bool ok = !errs.Any(e => e.Item2.EndsWith("BAD") || e.Item2.EndsWith("missing"));
        Check(ok, $"{tag}: {n} joints match joint_states (<{maxDeg} deg, <{maxMm} mm); worst: {string.Join("; ", errs.Take(3).Select(e => e.Item2))}");
    }

    static T Field<T>(object o, string name) => (T)o.GetType().GetField(name, BindingFlags.NonPublic | BindingFlags.Instance).GetValue(o);

    static IEnumerator Live()
    {
        Log("===== live first-person checks");
        LoadUrdf();
        PlayerPrefs.DeleteAll();
        PlayerPrefs.SetString("RosIPAddress", "127.0.0.1");
        PlayerPrefs.SetInt("RobotModel/SOBIT_HOME", 1); PlayerPrefs.SetString("CameraLayout/SOBIT_HOME", "firstperson");
        PlayerPrefs.SetInt("DebugCapture", 1);
        PlayerPrefs.Save();
        string capturePath = Path.Combine(Application.persistentDataPath, "fpv_capture.png");
        if (File.Exists(capturePath)) File.Delete(capturePath);

        // Start with the head turned so the first-TF recenter is meaningful.
        var pre = Pub("head -0.4 0.0");
        var preLift = Pub("lift 0.5");
        var preArm = Pub("armhome");
        while (!pre.HasExited || !preLift.HasExited || !preArm.HasExited) yield return Seconds(0.2);
        yield return Seconds(2.0);

        var profile = AssetDatabase.LoadAssetAtPath<RobotProfile>("Assets/Robots/SOBIT_HOME.asset");
        EditorSceneManager.OpenScene("Assets/Scenes/TeleopScene.unity");
        RobotProfile.Selected = profile;
        var pubEdit = Object.FindFirstObjectByType<QuestControllerPublisher>();
        pubEdit.controlRobot = false;   // nothing published to sobits_teleop
        EditorApplication.EnterPlaymode();
        while (!Application.isPlaying) yield return Seconds(0.1);
        var publisher = Object.FindFirstObjectByType<QuestControllerPublisher>();
        publisher.controlRobot = false;
        Check(publisher.controlRobot == false, "controlRobot off in play mode");
        yield return Frames(2);

        var ros = ROSConnection.GetOrCreateInstance();
        if (!_jsSubscribed) { ros.Subscribe<JointStateMsg>("/sobit_home/joint_states", OnJointStates); _jsSubscribed = true; }
        var hud = Object.FindFirstObjectByType<TeleopHud>();
        var images = Object.FindFirstObjectByType<ImageSubscriber>();
        Check(hud != null && hud.FirstPerson, "TeleopHud starts in first person from PlayerPrefs");

        FirstPersonView fpv = null; RobotModel model = null;
        double end = EditorApplication.timeSinceStartup + 40;
        while (EditorApplication.timeSinceStartup < end)
        {
            fpv = Object.FindFirstObjectByType<FirstPersonView>();
            model = fpv != null ? fpv.Model : null;
            if (model != null && model.AcceptedTransforms > 0 && fpv.FramesReceived > 0 && _jsCount > 0) break;
            yield return Seconds(0.25);
        }
        if (!Check(model != null && model.AcceptedTransforms > 0 && fpv.FramesReceived > 0 && _jsCount > 0,
            $"model receives TF ({model?.AcceptedTransforms}) and image frames ({fpv?.FramesReceived}), joint_states {_jsCount}; connErr={ros.HasConnectionError}"))
        {
            ros.GetTopicAndTypeList(list => Log("    topics: " + string.Join(", ", list.Keys.Take(40))));
            yield return Seconds(3);
            yield break;
        }
        yield return Seconds(2.0);
        var head = Camera.main.transform;
        var root = model.Root;
        var camFrame = model.Frame(profile.cameraFrame);
        var pan = model.Frame("head_pan_link");
        var tilt = model.Frame("head_tilt_link");
        var lift = model.Frame("body_lift_link");

        // First-TF recenter: the camera frame sits at the head and looks where the head looks.
        {
            float d = (camFrame.position - head.position).magnitude;
            Vector3 a = camFrame.forward, b = head.forward; a.y = 0; b.y = 0;
            float yaw = Vector3.Angle(a, b);
            Check(d < 0.01f && yaw < 2f, $"after first TF: camera frame at the head ({d * 1000:F1} mm) and yaw aligned ({yaw:F2} deg) with pan q={Q("head_pan_joint"):F3}");
        }
        Check(model.TfHz > 10f, $"TfHz {model.TfHz:F1} > 10");
        Check(images.Panels.All(p => !p.Visible), $"all {images.Panels.Count} camera blocks hidden in first person");
        Check(GameObject.Find("HUD Bar") == null || !GameObject.Find("HUD Bar").activeInHierarchy, "HUD bar hidden in first person");

        var headJoints = new[] { "head_pan_joint", "head_tilt_joint" };
        var armJoints = _joints.Values.Where(j => j.name.StartsWith("arm_left_") && j.type != "fixed").Select(j => j.name).ToArray();
        var allMoving = _joints.Values.Where(j => j.type != "fixed" && _q.ContainsKey(j.name)).Select(j => j.name).ToList();
        CompareJoints(model, "initial pose", allMoving);

        // Head target 1.
        var p1 = Pub("head 0.5 0.3");
        var e = Settle(p1, new Dictionary<string, double> { ["head_pan_joint"] = 0.5, ["head_tilt_joint"] = 0.3 }); while (e.MoveNext()) yield return e.Current;
        {
            float qp = (float)Q("head_pan_joint"), qt = (float)Q("head_tilt_joint");
            float ap = Quaternion.Angle(pan.localRotation, Quaternion.AngleAxis(-qp * Mathf.Rad2Deg, Vector3.up));
            var tj = _joints["head_tilt_joint"];
            float at = Quaternion.Angle(tilt.localRotation, tj.rot * Quaternion.AngleAxis(-qt * Mathf.Rad2Deg, Vector3.right));
            float dpos = (tilt.localPosition - new Vector3(0f, 0.131f, 0.05f)).magnitude;
            Check(ap < 1.5f, $"head 1: pan q={qp:F3} model error {ap:F2} deg < 1.5");
            Check(at < 1.5f, $"head 1: tilt q={qt:F3} model error {at:F2} deg < 1.5 (origin rot {tj.rot.eulerAngles:F1})");
            Check(dpos < 0.002f, $"head_tilt_link localPosition {tilt.localPosition:F4} ~ (0,0.131,0.05) err {dpos * 1000:F2} mm");
            Check(Math.Abs(qp - 0.5) < 0.03 && Math.Abs(qt - 0.3) < 0.03, "head 1 reached the target in the sim");
            Check(model.TfHz > 10f, $"TfHz {model.TfHz:F1} > 10");
        }
        CompareJoints(model, "head 1", headJoints);
        Shot("fpv_view_head1", head);
        Diagnose(model, fpv, head);

        // Explicit Recenter with the head turned.
        hud.Recenter();
        yield return Frames(3);
        {
            float d = (camFrame.position - head.position).magnitude;
            Vector3 a = camFrame.forward, b = head.forward; a.y = 0; b.y = 0;
            Check(d < 0.01f && Vector3.Angle(a, b) < 2f, $"Recenter: camera frame at head ({d * 1000:F1} mm), yaw diff {Vector3.Angle(a, b):F2} deg");
        }
        Shot("fpv_view_recentered", head);
        var downPose = new GameObject("Down30").transform;
        downPose.SetPositionAndRotation(head.position, Quaternion.Euler(30f, head.eulerAngles.y, 0f));
        Shot("fpv_view_recentered_down30", downPose);
        Object.DestroyImmediate(downPose.gameObject);

        // Lift: model local y follows; pan anchor stays fixed while the lift moves.
        Vector3 anchor = Field<Vector3>(fpv, "_anchorPoint");
        double qLift0 = Q("body_lift_joint"); float yLift0 = lift.localPosition.y;
        float maxAnchorErr = 0f, maxAnchorDrift = 0f; Vector3 rootStart = root.position;
        var p2 = Pub("lift 0.2");
        double lend = EditorApplication.timeSinceStartup + 25;
        while (EditorApplication.timeSinceStartup < lend)
        {
            maxAnchorErr = Mathf.Max(maxAnchorErr, (pan.position - anchor).magnitude);
            maxAnchorDrift = Mathf.Max(maxAnchorDrift, (Field<Vector3>(fpv, "_anchorPoint") - anchor).magnitude);
            if (p2.HasExited && Math.Abs(Q("body_lift_joint") - 0.2) < 0.003) break;
            yield return Frames(1);
        }
        for (int i = 0; i < 20; i++) { maxAnchorErr = Mathf.Max(maxAnchorErr, (pan.position - anchor).magnitude); yield return Frames(1); }
        {
            double qLift1 = Q("body_lift_joint"); float yLift1 = lift.localPosition.y;
            double dq = qLift1 - qLift0; float dy = yLift1 - yLift0;
            Check(Math.Abs(dq - (-0.3)) < 0.03, $"lift reached 0.2 in the sim (q {qLift0:F3} -> {qLift1:F3})");
            Check(Math.Abs(dy - dq) < 0.005, $"body_lift_link local y change {dy:F4} = joint change {dq:F4} (+-5 mm)");
            Check(maxAnchorErr < 0.002f && maxAnchorDrift < 1e-5f, $"pan anchor held while lifting: max |pan - anchor| {maxAnchorErr * 1000:F2} mm, anchor drift {maxAnchorDrift * 1000:F3} mm; root moved {(root.position - rootStart).y:F3} m in y");
            Check(Mathf.Abs((root.position - rootStart).y - (float)(-dq)) < 0.01f, "model root moved up by the lift drop (user stays at the head)");
        }

        // Arm left to initial_pose.
        var hand = model.Frame("hand_left_camera_base_link");
        Vector3 handBefore = root.InverseTransformPoint(hand.position);
        var p3 = Pub("armleft");
        var armTargets = new Dictionary<string, double> { ["arm_left_shoulder_tilt_joint"] = -0.75, ["arm_left_upper_roll_joint"] = -1.22, ["arm_left_upper_flex_joint"] = -0.2, ["arm_left_elbow_joint"] = 2.5 };
        e = Settle(p3, armTargets); while (e.MoveNext()) yield return e.Current;
        Vector3 handAfter = root.InverseTransformPoint(hand.position);
        Check((handAfter - handBefore).magnitude > 0.05f, $"hand_left_camera_base_link moved {(handAfter - handBefore).magnitude:F3} m in root space ({handBefore:F3} -> {handAfter:F3})");
        CompareJoints(model, "arm left", armJoints);
        CompareJoints(model, "all joints after arm", allMoving);

        // Head target 2.
        var p4 = Pub("head -0.4 0.0");
        e = Settle(p4, new Dictionary<string, double> { ["head_pan_joint"] = -0.4, ["head_tilt_joint"] = 0.0 }); while (e.MoveNext()) yield return e.Current;
        {
            float qp = (float)Q("head_pan_joint"), qt = (float)Q("head_tilt_joint");
            float ap = Quaternion.Angle(pan.localRotation, Quaternion.AngleAxis(-qp * Mathf.Rad2Deg, Vector3.up));
            float at = Quaternion.Angle(tilt.localRotation, _joints["head_tilt_joint"].rot * Quaternion.AngleAxis(-qt * Mathf.Rad2Deg, Vector3.right));
            Check(Math.Abs(qp + 0.4) < 0.03 && ap < 1.5f && at < 1.5f, $"head 2: pan q={qp:F3} err {ap:F2} deg, tilt q={qt:F3} err {at:F2} deg");
        }

        // Image quad.
        int f0 = fpv.FramesReceived;
        yield return Seconds(3.0);
        int f1 = fpv.FramesReceived;
        Check(f1 > f0 && images.Panels.All(p => !p.Visible), $"image frames keep arriving with blocks hidden ({f0} -> {f1} in 3 s)");
        var viewRt = Field<RectTransform>(fpv, "_viewRt");
        bool hasInfo = Field<bool>(fpv, "_hasInfo");
        float w = viewRt.sizeDelta.x / 1000f, h = viewRt.sizeDelta.y / 1000f;
        float wHfov = 2f * 1.5f * Mathf.Tan(1.2113f / 2f);
        if (hasInfo)
        {
            double fx = Field<double>(fpv, "_fx"), iw = Field<double>(fpv, "_infoW");
            float wInfo = (float)(1.5 * iw / fx);
            Check(Mathf.Abs(w - wInfo) / wInfo < 0.02f, $"quad width {w:F3} m matches camera_info ({wInfo:F3} m, fx={fx:F1}, w={iw}); hfov formula {wHfov:F3} m (diff {(w - wHfov) / wHfov * 100:F1} %)");
        }
        else Check(Mathf.Abs(w - wHfov) / wHfov < 0.02f, $"quad width {w:F3} m ~ 2*1.5*tan(hfov/2) = {wHfov:F3} m (no camera_info received)");
        Log($"    quad {w:F3} x {h:F3} m, source={(hasInfo ? "camera_info" : "default hfov")}");
        var view = Field<UnityEngine.UI.RawImage>(fpv, "_view");
        Check(view.texture != null && view.color == Color.white, $"quad shows a texture ({view.texture?.width}x{view.texture?.height})");
        var canvasRt = Field<RectTransform>(fpv, "_canvasRt");
        Check(canvasRt.parent == camFrame && (canvasRt.localPosition - new Vector3(0, 0, 1.5f)).magnitude < 1e-4f, "quad hangs 1.5 m in front of the camera frame");

        // Screenshots.
        Shot("fpv_view", head);
        double capEnd = EditorApplication.timeSinceStartup + 20;
        while (!File.Exists(capturePath) && EditorApplication.timeSinceStartup < capEnd) yield return Seconds(0.5);
        if (Check(File.Exists(capturePath), $"DebugCapture PNG written: {capturePath}"))
            File.Copy(capturePath, Path.Combine(ShotDir, "fpv_capture_editor.png"), true);
        Check(PlayerPrefs.GetInt("DebugCapture", 0) == 0, "DebugCapture flag cleared after the capture");

        // Toggle back to blocks and again to first person.
        hud.SetRobotModel(false);
        yield return Frames(3);
        Check(!hud.FirstPerson && images.Panels.All(p => p.Visible), $"blocks reappear ({images.Panels.Count(p => p.Visible)}/{images.Panels.Count})");
        Check(Object.FindObjectsByType<RobotModel>(FindObjectsSortMode.None).Length == 0 && Object.FindFirstObjectByType<FirstPersonView>() == null, "model gone in blocks mode");
        Check(PlayerPrefs.GetInt("RobotModel/SOBIT_HOME", -1) == 0 && PlayerPrefs.GetString("CameraLayout/SOBIT_HOME") == "firstperson", "RobotModel pref = 0, CameraLayout pref kept = firstperson");
        Shot("fpv_blocks_after_toggle", head);
        Exception ex = null;
        try { hud.SetRobotModel(true); } catch (Exception x) { ex = x; }
        yield return Seconds(3.0);
        var fpv2 = Object.FindFirstObjectByType<FirstPersonView>();
        Check(ex == null && fpv2 != null && fpv2.Model != null && fpv2 != fpv, $"first person again: model recreated (exception: {ex?.Message ?? "none"})");
        Check(fpv2 != null && fpv2.Model.AcceptedTransforms > 0 && fpv2.FramesReceived > 0 && images.Panels.All(p => !p.Visible),
            $"second FPV gets TF ({fpv2?.Model?.AcceptedTransforms}) and frames ({fpv2?.FramesReceived}), blocks hidden");
        Check(Object.FindObjectsByType<RobotModel>(FindObjectsSortMode.None).Length == 1, "exactly one model");
        yield return Frames(5);
        if (fpv2 != null && fpv2.Model != null) CompareJoints(fpv2.Model, "second model", allMoving);
        Shot("fpv_view_again", head);

        EditorApplication.ExitPlaymode();
        while (Application.isPlaying) yield return Seconds(0.1);
    }

    static void Diagnose(RobotModel model, FirstPersonView fpv, Transform head)
    {
        var rends = model.GetComponentsInChildren<Renderer>(true);
        Bounds b = rends[0].bounds; foreach (var r in rends) b.Encapsulate(r.bounds);
        Log($"    diag: head pos {head.position:F3} rot {head.eulerAngles:F1}; model root {model.Root.position:F3} rot {model.Root.eulerAngles:F1}; enabled renderers {rends.Count(r => r.enabled)}/{rends.Length}; model bounds min {b.min:F3} max {b.max:F3}; layer {model.gameObject.layer}; cullingMask {Camera.main.cullingMask}");
        var canvasRt = Field<RectTransform>(fpv, "_canvasRt");
        var c = new Vector3[4]; Field<RectTransform>(fpv, "_viewRt").GetWorldCorners(c);
        Log($"    diag: quad corners {string.Join(" ", c.Select(v => v.ToString("F2")))}");
        foreach (var t in Object.FindObjectsByType<Transform>(FindObjectsSortMode.None))
            if (t.name == "Grid" || t.name == "Ground")
            {
                var r = t.GetComponent<Renderer>();
                Log($"    diag: floor {t.name} pos {t.position:F3} scale {t.lossyScale:F2} active {t.gameObject.activeInHierarchy} shader {r?.sharedMaterial?.shader?.name} queue {r?.sharedMaterial?.renderQueue}");
            }
        var mr = model.GetComponentInChildren<MeshRenderer>();
        Log($"    diag: model material {mr.sharedMaterial?.shader?.name} queue {mr.sharedMaterial?.renderQueue}");
        var down = new GameObject("DownPose").transform;
        down.SetPositionAndRotation(head.position, Quaternion.Euler(50f, head.eulerAngles.y, 0f));
        Shot("fpv_diag_down", down);
        var side = new GameObject("SidePose").transform;
        Vector3 fwd = Vector3.ProjectOnPlane(head.forward, Vector3.up).normalized;
        Vector3 rightV = Vector3.Cross(Vector3.up, fwd);
        side.position = head.position + rightV * 3.5f - fwd * 0.5f + Vector3.up * 0.3f;
        side.rotation = Quaternion.LookRotation((b.center + fwd * 0.8f) - side.position, Vector3.up);
        Shot("fpv_diag_side", side);
        Object.DestroyImmediate(down.gameObject); Object.DestroyImmediate(side.gameObject);
    }

    static void Shot(string name, Transform pose)
    {
        var go = new GameObject("VerifyCam");
        var cam = go.AddComponent<Camera>();
        go.transform.SetPositionAndRotation(pose.position, pose.rotation);
        var main = Camera.main;
        cam.fieldOfView = Mathf.Max(70f, main != null ? main.fieldOfView : 70f); cam.aspect = 16f / 9f;
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
        Log("    screenshot " + name + ".png");
    }
}
