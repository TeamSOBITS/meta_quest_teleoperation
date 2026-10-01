using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using UnityEngine;

/// <summary>
/// Base velocity arrow on the floor at the model's base, from the odometry twist: an arrow along
/// the driving direction (longer when faster) and an arc swinging to the left or right for the
/// turn rate. Hidden while the base is still. Created by TeleopHud in first person while the
/// "basevel" experiment is on, destroyed when either turns off.
/// Directions: ROS x (forward) is the root's Unity +Z, ROS y (left) is Unity -X.
/// </summary>
[DefaultExecutionOrder(250)]
public class BaseVelocity : MonoBehaviour
{
    const float EmaAlpha = 0.3f, StaleSeconds = 1f;
    const float HideLinear = 0.02f, HideAngular = 0.05f;
    const float LiftM = 0.02f, MinLengthM = 0.3f, PerMps = 1.0f, MaxLengthM = 1.2f, HeadM = 0.12f;
    const float ArrowWidthM = 0.03f, ArcWidthM = 0.02f, ArcRadiusM = 0.55f, ArcRadPerRadS = 0.8f, ArcMaxRad = 1.6f;
    // The arrow starts this far out from the base centre: seen from the head, the robot's own base
    // and torso hide the floor up to about 0.3 m (the first picture showed only the arrow head).
    const float StartM = 0.35f;
    const int ArcPoints = 14;

    static ROSConnection _subscribedOn;   // each robot screen has its own ROSConnection
    static string _subscribedTopic;
    static BaseVelocity Current;   // the live instance; the one odom subscription forwards to it

    RobotModel _model;
    LineRenderer _arrow, _arc;
    float _lastTime = float.NegativeInfinity;
    bool _hasValue;

    // Smoothed base velocity: linear (x forward, y left, m/s) and angular (z, rad/s).
    public Vector2 Linear { get; private set; }
    public float Angular { get; private set; }
    public bool Moving => Time.unscaledTime - _lastTime <= StaleSeconds
                          && (Linear.magnitude >= HideLinear || Mathf.Abs(Angular) >= HideAngular);

    public static BaseVelocity Create(RobotModel model, RobotProfile profile)
    {
        if (model == null || profile == null) return null;
        var source = ExperimentMaterials.Source(model);
        if (source == null)
        {
            Debug.LogWarning("BaseVel: model has no material; arrow skipped");
            return null;
        }
        var go = new GameObject("Base Velocity");
        var v = go.AddComponent<BaseVelocity>();
        v._model = model;
        v._arrow = v.MakeLine("Arrow", source, HudUi.AccentColor, ArrowWidthM, 5);
        v._arc = v.MakeLine("Turn", source, HudUi.WarnColor, ArcWidthM, ArcPoints);
        Current = v;
        string topic = profile.FullTopic("odom");
        var ros = ROSConnection.GetOrCreateInstance();
        if (_subscribedOn != ros || _subscribedTopic != topic)
        {
            _subscribedOn = ros;   // subscribe once per connection; toggling must not add callbacks
            _subscribedTopic = topic;
            ros.Subscribe<OdometryMsg>(topic, msg => { if (Current != null) Current.OnOdom(msg); });
        }
        return v;
    }

    LineRenderer MakeLine(string name, Material source, Color colour, float width, int points)
    {
        var line = new GameObject(name).AddComponent<LineRenderer>();
        line.transform.SetParent(transform, false);
        line.sharedMaterial = ExperimentMaterials.Tinted(source, colour);
        line.useWorldSpace = true;
        line.positionCount = points;
        line.startWidth = line.endWidth = width;
        line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        line.enabled = false;
        return line;
    }

    void OnOdom(OdometryMsg msg)
    {
        if (this == null || !isActiveAndEnabled) return;
        if (msg == null || msg.twist == null || msg.twist.twist == null) return;
        var lin = msg.twist.twist.linear;
        var ang = msg.twist.twist.angular;
        if (lin == null || ang == null) return;
        var l = new Vector2((float)lin.x, (float)lin.y);
        float w = (float)ang.z;
        if (!_hasValue) { Linear = l; Angular = w; _hasValue = true; }
        else
        {
            Linear = Vector2.Lerp(Linear, l, EmaAlpha);
            Angular = Mathf.Lerp(Angular, w, EmaAlpha);
        }
        _lastTime = Time.unscaledTime;
    }

    void OnDestroy()
    {
        if (Current == this) Current = null;
    }

    void LateUpdate()
    {
        bool moving = Moving;
        float speed = Linear.magnitude;
        bool arrow = moving && speed >= HideLinear;
        bool arc = moving && Mathf.Abs(Angular) >= HideAngular;
        _arrow.enabled = arrow;
        _arc.enabled = arc;
        var root = _model.Root;
        Vector3 origin = root.position + Vector3.up * LiftM;

        if (arrow)
        {
            // Root-local direction on the floor: ROS x -> +Z, ROS y -> -X.
            var dirLocal = new Vector3(-Linear.y, 0f, Linear.x).normalized;
            Vector3 dir = Vector3.ProjectOnPlane(root.TransformDirection(dirLocal), Vector3.up).normalized;
            Vector3 side = Vector3.Cross(Vector3.up, dir);
            float length = Mathf.Min(MinLengthM + PerMps * speed, MaxLengthM);
            Vector3 start = origin + dir * StartM;
            Vector3 tip = start + dir * length;
            Vector3 back = tip - dir * HeadM;
            _arrow.SetPosition(0, start);
            _arrow.SetPosition(1, tip);
            _arrow.SetPosition(2, back + side * HeadM * 0.6f);
            _arrow.SetPosition(3, tip);
            _arrow.SetPosition(4, back - side * HeadM * 0.6f);
        }
        if (arc)
        {
            // From straight ahead, towards the left for positive angular z (ROS left = Unity -X).
            float sweep = Mathf.Clamp(Angular * ArcRadPerRadS, -ArcMaxRad, ArcMaxRad);
            Vector3 fwd = Vector3.ProjectOnPlane(root.forward, Vector3.up).normalized;
            Vector3 left = Vector3.Cross(Vector3.up, fwd) * -1f;
            for (int i = 0; i < ArcPoints; i++)
            {
                float a = sweep * i / (ArcPoints - 1);
                _arc.SetPosition(i, origin + (fwd * Mathf.Cos(a) + left * Mathf.Sin(a)) * ArcRadiusM);
            }
        }
    }
}
