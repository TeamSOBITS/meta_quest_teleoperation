using System.Collections.Generic;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

/// <summary>
/// Arm target markers: sobits_teleop broadcasts the hand targets on /tf as base frame ->
/// left_target_link / right_target_link. A small sphere is drawn at each target (in the model's
/// root frame) with a line to the model's end effector, so the distance is where the arm still has
/// to go. A marker is hidden when its transform was not seen for 1 s. Created by TeleopHud
/// while the robot model is on, destroyed when it is turned off.
/// </summary>
[DefaultExecutionOrder(250)]
public class ArmTargets : MonoBehaviour
{
    const string TfTopic = "/tf";
    const float SeenSeconds = 1f, LogSeconds = 5f, MarkerM = 0.03f, LineWidthM = 0.004f;

    class Side
    {
        public string childFrame, effectorFrame, label;
        public Transform effector, marker;
        public LineRenderer line;
        public bool seen;
        public Vector3 position;
        public float lastTime = float.NegativeInfinity;
        public bool Visible => Time.unscaledTime - lastTime <= SeenSeconds;
        public float Error;
    }

    static ROSConnection _subscribedOn;   // each robot screen has its own ROSConnection
    static ArmTargets Current;   // the live instance; the one /tf subscription forwards to it

    readonly List<Side> _sides = new List<Side>();
    RobotModel _model;
    string _rootFrame;
    float _nextLog;

    public bool HasLeft => _sides.Count > 0 && _sides[0].Visible;
    public bool HasRight => _sides.Count > 1 && _sides[1].Visible;
    // Distance marker - end effector (m); meaningful while HasLeft / HasRight.
    public float LeftErrorM => _sides.Count > 0 ? _sides[0].Error : 0f;
    public float RightErrorM => _sides.Count > 1 ? _sides[1].Error : 0f;

    public static ArmTargets Create(RobotModel model)
    {
        if (model == null) return null;
        string root = OverlayMaterials.RootFrame(model);
        var source = OverlayMaterials.Source(model);
        if (root == null || source == null)
        {
            Debug.LogWarning("Targets: model has no root link or no material; markers skipped");
            return null;
        }
        var go = new GameObject("Arm Targets");
        var t = go.AddComponent<ArmTargets>();
        t._model = model;
        t._rootFrame = root;
        t.AddSide(source, "left_target_link", "hand_left_end_effector_link", "L", HudUi.AccentColor);
        t.AddSide(source, "right_target_link", "hand_right_end_effector_link", "R", HudUi.WarnColor);
        t._nextLog = Time.unscaledTime + LogSeconds;
        Current = t;
        var ros = ROSConnection.GetOrCreateInstance();
        if (_subscribedOn != ros)
        {
            _subscribedOn = ros;   // subscribe once per connection; toggling must not add callbacks
            ros.Subscribe<TFMessageMsg>(TfTopic, msg => { if (Current != null) Current.OnTf(msg); });
        }
        return t;
    }

    void AddSide(Material source, string child, string effectorFrame, string label, Color colour)
    {
        var side = new Side { childFrame = child, effectorFrame = effectorFrame, label = label };
        side.effector = _model.Frame(effectorFrame);
        if (side.effector == null)
            Debug.LogWarning($"Targets: frame '{effectorFrame}' not in the model; no line for the {label} target");

        var material = OverlayMaterials.Tinted(source, colour);
        var marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        marker.name = "Target " + label;
        var collider = marker.GetComponent<Collider>();
        if (collider != null) Destroy(collider);
        marker.GetComponent<MeshRenderer>().sharedMaterial = material;
        marker.transform.SetParent(_model.Root, false);
        marker.transform.localScale = Vector3.one * MarkerM;
        marker.SetActive(false);
        side.marker = marker.transform;

        var line = new GameObject("Target Line " + label).AddComponent<LineRenderer>();
        line.transform.SetParent(transform, false);
        line.sharedMaterial = material;
        line.useWorldSpace = true;
        line.positionCount = 2;
        line.startWidth = line.endWidth = LineWidthM;
        line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        line.enabled = false;
        side.line = line;
        _sides.Add(side);
    }

    void OnTf(TFMessageMsg msg)
    {
        if (this == null || !isActiveAndEnabled) return;
        if (msg == null || msg.transforms == null) return;
        foreach (var t in msg.transforms)
        {
            if (t == null || t.child_frame_id == null || t.header == null || t.transform == null) continue;
            if (t.header.frame_id != _rootFrame) continue;
            foreach (var s in _sides)
            {
                if (t.child_frame_id != s.childFrame) continue;
                s.position = t.transform.translation.From<FLU>();
                s.lastTime = Time.unscaledTime;
            }
        }
    }

    void LateUpdate()
    {
        // After RobotModel (150) and FirstPersonView (200): the effector and root are final.
        foreach (var s in _sides)
        {
            bool visible = s.Visible;
            s.marker.gameObject.SetActive(visible);
            s.line.enabled = visible && s.effector != null;
            if (!visible) continue;
            s.marker.localPosition = s.position;
            if (s.effector != null)
            {
                s.line.SetPosition(0, s.marker.position);
                s.line.SetPosition(1, s.effector.position);
                s.Error = Vector3.Distance(s.marker.position, s.effector.position);
            }
        }

        if (Time.unscaledTime < _nextLog) return;
        _nextLog = Time.unscaledTime + LogSeconds;
        if (HasLeft || HasRight)
            Debug.Log($"Targets: L err {(HasLeft ? LeftErrorM.ToString("F2") : "-")} m, " +
                      $"R err {(HasRight ? RightErrorM.ToString("F2") : "-")} m");
    }

    void OnDestroy()
    {
        if (Current == this) Current = null;
        foreach (var s in _sides)
            if (s.marker != null) Destroy(s.marker.gameObject);   // hangs under the model, not under us
    }
}
