using System;
using System.Collections.Generic;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

/// <summary>
/// Life-size 3D model of the robot that follows its joints live. The model comes from
/// <see cref="RobotProfile.modelPrefab"/>: one GameObject per URDF link (each a child of its parent
/// link, tagged with a <see cref="RobotLink"/>), with every fixed joint already baked in.
///
/// Moving joints arrive on /tf (robot_state_publisher publishes every non-fixed joint in each
/// message). A transform is only used when its parent and child frames are exactly the link's
/// URDF parent and name; this drops odom -> base_footprint, the headset's and controllers' own
/// frames echoed back by the endpoint, and other robots' frames. The latest pose per link is
/// applied in LateUpdate (the pose is the full parent -> child transform, so it replaces the
/// link's local pose). /tf_static is not used: it is latched, and the endpoint's subscriber may
/// never receive it.
///
/// Subscribes once on the scene's ROSConnection; the callback ignores messages once this
/// component is gone (the connector has no safe Unsubscribe, see RobotPresence).
/// </summary>
[DefaultExecutionOrder(150)]
public class RobotModel : MonoBehaviour
{
    const string TfTopic = "/tf";
    const float HzWindowSeconds = 1f;

    class Link
    {
        public RobotLink info;
        public Transform transform;
        public bool hasPose;
        public Vector3 position;
        public Quaternion rotation;
    }

    readonly Dictionary<string, Link> _links = new Dictionary<string, Link>();
    int _messagesInWindow;
    bool _firstAccepted;
    float _windowStart;

    public Transform Root => transform;
    // /tf messages per second that carried at least one transform of this robot (1 s window).
    public float TfHz { get; private set; }
    // Transforms applied to links so far.
    public int AcceptedTransforms { get; private set; }
    public int LinkCount => _links.Count;
    // Raised once, with the first accepted transform.
    public event Action Updated;

    public Transform Frame(string name)
        => !string.IsNullOrEmpty(name) && _links.TryGetValue(name, out var l) ? l.transform : null;

    // Creates the model from the profile's prefab under `parent` and starts following /tf.
    // Returns null (after logging) when the robot has no model.
    public static RobotModel Create(RobotProfile profile, Transform parent)
    {
        if (profile == null || profile.modelPrefab == null)
        {
            Debug.LogWarning("FPV: robot has no model prefab");
            return null;
        }
        var go = Instantiate(profile.modelPrefab, parent, false);
        go.name = profile.modelPrefab.name;
        var model = go.AddComponent<RobotModel>();
        model.Init();
        return model;
    }

    void Init()
    {
        foreach (var info in GetComponentsInChildren<RobotLink>(true))
        {
            if (string.IsNullOrEmpty(info.frame) || _links.ContainsKey(info.frame)) continue;
            _links[info.frame] = new Link { info = info, transform = info.transform };
        }
        if (_links.Count == 0)
            Debug.LogWarning($"FPV: model {name} has no RobotLink components; nothing will move");

        _windowStart = Time.unscaledTime;
        var ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TFMessageMsg>(TfTopic, OnTf);
    }

    void OnTf(TFMessageMsg msg)
    {
        if (this == null || !isActiveAndEnabled) return;
        if (msg == null || msg.transforms == null) return;

        bool any = false;
        foreach (var t in msg.transforms)
        {
            if (t == null || t.child_frame_id == null || t.header == null || t.transform == null) continue;
            if (!_links.TryGetValue(t.child_frame_id, out var link)) continue;
            if (link.info.isRoot || t.header.frame_id != link.info.parentFrame) continue;

            link.position = t.transform.translation.From<FLU>();
            link.rotation = t.transform.rotation.From<FLU>();
            link.hasPose = true;
            AcceptedTransforms++;
            any = true;
        }
        if (!any) return;

        _messagesInWindow++;
        if (!_firstAccepted)
        {
            _firstAccepted = true;
            Updated?.Invoke();
        }
    }

    void LateUpdate()
    {
        foreach (var link in _links.Values)
        {
            if (!link.hasPose) continue;
            link.transform.localPosition = link.position;
            link.transform.localRotation = link.rotation;
        }

        float now = Time.unscaledTime;
        if (now - _windowStart >= HzWindowSeconds)
        {
            TfHz = _messagesInWindow / (now - _windowStart);
            _messagesInWindow = 0;
            _windowStart = now;
        }
    }

    // Show / hide the visuals of every link whose frame starts with one of `prefixes`
    // (a link's own visuals only, not those of its child links).
    public void SetLinksVisible(IEnumerable<string> prefixes, bool visible)
    {
        if (prefixes == null) return;
        var list = new List<string>(prefixes);
        foreach (var r in GetComponentsInChildren<Renderer>(true))
        {
            var owner = r.GetComponentInParent<RobotLink>();
            if (owner == null || string.IsNullOrEmpty(owner.frame)) continue;
            foreach (var p in list)
                if (!string.IsNullOrEmpty(p) && owner.frame.StartsWith(p, StringComparison.Ordinal))
                {
                    r.enabled = visible;
                    break;
                }
        }
    }
}
