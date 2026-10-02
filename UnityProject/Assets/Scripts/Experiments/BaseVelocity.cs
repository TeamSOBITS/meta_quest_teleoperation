using System.Collections.Generic;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Base velocity on the floor at the model's base, from the odometry twist (v2: filled meshes):
/// a flat arrow along the driving direction (longer when faster), a filled arc band with an
/// arrowhead swinging to the left or right for the turn rate, and a small floating chip with the
/// numbers ("0.15 m/s", "0.50 rad/s"). All hidden while the base is still. Created by TeleopHud in
/// first person while the "basevel" experiment is on, destroyed when either turns off.
/// Directions: ROS x (forward) is the root's Unity +Z, ROS y (left) is Unity -X.
/// </summary>
[DefaultExecutionOrder(250)]
public class BaseVelocity : MonoBehaviour
{
    const float EmaAlpha = 0.3f, StaleSeconds = 1f;
    const float HideLinear = 0.02f, HideAngular = 0.05f;
    const float LiftM = 0.02f, ArcLiftM = 0.025f;
    const float MinLengthM = 0.35f, PerMps = 1.0f, MaxLengthM = 1.2f;
    const float ArrowWidthM = 0.08f, HeadM = 0.14f, HeadHalfWidthM = 0.09f;
    const float ArcRadiusM = 0.55f, ArcBandM = 0.06f, ArcSpanDeg = 60f, ArcHeadM = 0.12f, ArcHeadHalfWidthM = 0.07f;
    const int ArcSegments = 14;
    const float Alpha = 0.75f;
    // The arrow starts this far out from the base centre: seen from the head, the robot's own base
    // and torso hide the floor up to about 0.3 m (the first picture showed only the arrow head).
    const float StartM = 0.35f;
    const float ChipRiseM = 0.15f, ChipDistanceM = 1.5f, ChipPadMm = 14f;

    static ROSConnection _subscribedOn;   // each robot screen has its own ROSConnection
    static string _subscribedTopic;
    static BaseVelocity Current;   // the live instance; the one odom subscription forwards to it

    RobotModel _model;
    Transform _arrowT, _arcT, _chipT;
    MeshRenderer _arrowR, _arcR;
    Mesh _arrowMesh, _arcMesh;
    int _arcSign;   // turn direction the arc mesh is built for (+1 left, -1 right), 0 = not built
    readonly Vector3[] _arrowVerts = new Vector3[7];
    TextMeshProUGUI _chipText;
    RectTransform _chipRt, _chipBg;
    string _chipShown;
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
        v.BuildMeshes(source);
        v.BuildChip();
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

    void BuildMeshes(Material source)
    {
        var arrow = new GameObject("Arrow");
        _arrowT = arrow.transform;
        _arrowT.SetParent(transform, false);
        _arrowMesh = new Mesh { name = "Base velocity arrow" };
        _arrowMesh.vertices = _arrowVerts;
        _arrowMesh.triangles = new[] { 0, 1, 2, 0, 2, 3, 4, 5, 6 };   // clockwise seen from above
        _arrowMesh.RecalculateNormals();
        _arrowR = AddRenderer(arrow, _arrowMesh, ExperimentMaterials.Transparent(
            ExperimentMaterials.Tinted(source, new Color(HudUi.AccentColor.r, HudUi.AccentColor.g, HudUi.AccentColor.b, Alpha))));

        var arc = new GameObject("Turn");
        _arcT = arc.transform;
        _arcT.SetParent(transform, false);
        _arcMesh = new Mesh { name = "Base velocity turn" };
        _arcR = AddRenderer(arc, _arcMesh, ExperimentMaterials.Transparent(
            ExperimentMaterials.Tinted(source, new Color(HudUi.WarnColor.r, HudUi.WarnColor.g, HudUi.WarnColor.b, Alpha))));
    }

    static MeshRenderer AddRenderer(GameObject go, Mesh mesh, Material material)
    {
        go.AddComponent<MeshFilter>().sharedMesh = mesh;
        var r = go.AddComponent<MeshRenderer>();
        r.sharedMaterial = material;
        r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        r.receiveShadows = false;
        r.enabled = false;
        return r;
    }

    // Floating value chip: rounded PanelColor card with the numbers in the body font, facing the head.
    void BuildChip()
    {
        var go = new GameObject("Value Chip", typeof(RectTransform));
        go.transform.SetParent(transform, false);
        var canvas = go.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.sortingOrder = HudUi.CanvasSortingOrder - 5;
        canvas.worldCamera = Camera.main;
        _chipRt = (RectTransform)go.transform;
        _chipRt.localScale = Vector3.one / HudUi.MmPerMetre;
        _chipT = go.transform;
        var bg = HudUi.Round(HudUi.Box(_chipRt, "Card", HudUi.PanelColor), 14f);
        _chipBg = bg.rectTransform;
        _chipText = HudUi.Label(_chipRt, "Value", "", HudUi.FontAt(HudUi.BodyFontSize, ChipDistanceM));
        _chipText.textWrappingMode = TextWrappingModes.NoWrap;
        _chipText.verticalAlignment = VerticalAlignmentOptions.Middle;
        HudUi.Stretch(_chipText.rectTransform);
        go.SetActive(false);
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
        foreach (var r in new[] { _arrowR, _arcR })
            if (r != null && r.sharedMaterial != null) Destroy(r.sharedMaterial);
        if (_arrowMesh != null) Destroy(_arrowMesh);
        if (_arcMesh != null) Destroy(_arcMesh);
    }

    // Triangle with its normal up (clockwise seen from above, the front face).
    static void Tri(List<int> t, List<Vector3> v, int a, int b, int c)
    {
        if (Vector3.Cross(v[b] - v[a], v[c] - v[a]).y >= 0f) { t.Add(a); t.Add(b); t.Add(c); }
        else { t.Add(a); t.Add(c); t.Add(b); }
    }

    // Arc band in the object's local space (+Z forward, +X right, y = 0): angle a runs from straight
    // ahead towards the left (point = (-sin a, 0, cos a) x radius). It spans ArcSpanDeg centred on
    // forward and ends in an arrowhead on the side of the turn (sign +1 = left).
    void BuildArc(int sign)
    {
        float half = ArcSpanDeg * 0.5f * Mathf.Deg2Rad;
        float start = -sign * half, end = sign * half;
        float bandEnd = end - sign * ArcHeadM / ArcRadiusM;   // the band stops where the head begins
        var v = new List<Vector3>();
        var t = new List<int>();
        for (int i = 0; i <= ArcSegments; i++)
        {
            float a = Mathf.Lerp(start, bandEnd, i / (float)ArcSegments);
            var dir = new Vector3(-Mathf.Sin(a), 0f, Mathf.Cos(a));
            v.Add(dir * (ArcRadiusM - ArcBandM / 2f));
            v.Add(dir * (ArcRadiusM + ArcBandM / 2f));
        }
        for (int i = 0; i < ArcSegments; i++)
        {
            int k = 2 * i;
            Tri(t, v, k, k + 1, k + 3);
            Tri(t, v, k, k + 3, k + 2);
        }
        var endDir = new Vector3(-Mathf.Sin(end), 0f, Mathf.Cos(end));
        var baseDir = new Vector3(-Mathf.Sin(bandEnd), 0f, Mathf.Cos(bandEnd));
        int h = v.Count;
        v.Add(baseDir * (ArcRadiusM - ArcHeadHalfWidthM));
        v.Add(baseDir * (ArcRadiusM + ArcHeadHalfWidthM));
        v.Add(endDir * ArcRadiusM);
        Tri(t, v, h, h + 1, h + 2);
        _arcMesh.Clear();
        _arcMesh.SetVertices(v);
        _arcMesh.SetTriangles(t, 0);
        _arcMesh.RecalculateNormals();
        _arcMesh.RecalculateBounds();
        _arcSign = sign;
    }

    // Arrow in the object's local space: shaft quad from z = 0, triangular head ending at z = length.
    void UpdateArrowMesh(float length)
    {
        float sl = length - HeadM, w = ArrowWidthM / 2f;
        _arrowVerts[0] = new Vector3(-w, 0f, 0f);
        _arrowVerts[1] = new Vector3(-w, 0f, sl);
        _arrowVerts[2] = new Vector3(w, 0f, sl);
        _arrowVerts[3] = new Vector3(w, 0f, 0f);
        _arrowVerts[4] = new Vector3(-HeadHalfWidthM, 0f, sl);
        _arrowVerts[5] = new Vector3(0f, 0f, length);
        _arrowVerts[6] = new Vector3(HeadHalfWidthM, 0f, sl);
        _arrowMesh.vertices = _arrowVerts;
        _arrowMesh.RecalculateNormals();
        _arrowMesh.RecalculateBounds();
    }

    void LateUpdate()
    {
        bool moving = Moving;
        float speed = Linear.magnitude;
        bool arrow = moving && speed >= HideLinear;
        bool arc = moving && Mathf.Abs(Angular) >= HideAngular;
        _arrowR.enabled = arrow;
        _arcR.enabled = arc;
        _chipT.gameObject.SetActive(moving);
        var root = _model.Root;
        Vector3 floor = root.position + Vector3.up * LiftM;
        Vector3 fwd = Vector3.ProjectOnPlane(root.forward, Vector3.up).normalized;
        // The chip hovers above the arrow tip (or above the arc's arrowhead end when only the arc shows).
        Vector3 chipAt = floor + fwd * StartM;

        if (arrow)
        {
            // Root-local direction on the floor: ROS x -> +Z, ROS y -> -X.
            var dirLocal = new Vector3(-Linear.y, 0f, Linear.x).normalized;
            Vector3 dir = Vector3.ProjectOnPlane(root.TransformDirection(dirLocal), Vector3.up).normalized;
            float length = Mathf.Min(MinLengthM + PerMps * speed, MaxLengthM);
            _arrowT.SetPositionAndRotation(floor + dir * StartM, Quaternion.LookRotation(dir, Vector3.up));
            UpdateArrowMesh(length);
            chipAt = floor + dir * (StartM + length);
        }
        if (arc)
        {
            // Left for positive angular z.
            int sign = Angular > 0f ? 1 : -1;
            if (sign != _arcSign) BuildArc(sign);
            _arcT.SetPositionAndRotation(root.position + Vector3.up * ArcLiftM, Quaternion.LookRotation(fwd, Vector3.up));
            if (!arrow)   // arc end only when the linear arrow is hidden
            {
                float endA = sign * ArcSpanDeg * 0.5f * Mathf.Deg2Rad;
                chipAt = _arcT.TransformPoint(new Vector3(-Mathf.Sin(endA), 0f, Mathf.Cos(endA)) * ArcRadiusM);
            }
        }
        if (moving)
        {
            var at = chipAt + Vector3.up * ChipRiseM;
            at.y = Mathf.Max(at.y, root.position.y + ChipRiseM);   // never closer than 0.15 m to the floor
            UpdateChip(speed, arrow, arc, at);
        }
    }

    void UpdateChip(float speed, bool arrow, bool arc, Vector3 at)
    {
        string text = arrow && arc ? $"{speed:F2} m/s  {Angular:F1} rad/s"
                    : arrow ? $"{speed:F2} m/s" : $"{Angular:F1} rad/s";
        if (text != _chipShown)
        {
            _chipShown = text;
            _chipText.text = text;
            var size = _chipText.GetPreferredValues(text);
            _chipRt.sizeDelta = new Vector2(size.x + 2f * ChipPadMm, size.y + 1.2f * ChipPadMm);
            HudUi.Stretch(_chipBg);
        }
        _chipT.position = at;
        var head = FirstPersonView.Head;
        // Canvas front faces -Z, so look away from the head.
        if (head != null) _chipT.rotation = Quaternion.LookRotation(at - head.position, Vector3.up);
    }
}
