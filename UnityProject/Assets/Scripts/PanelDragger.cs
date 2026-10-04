using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit.Inputs;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

/// <summary>
/// Lets the user move camera blocks with a controller (trigger) or hand (pinch) while robot control is off
/// (so the trigger can't also reach the robot): point the ray at a block, hold the trigger,
/// and the block follows the ray around the head at its current distance. Blocks stay
/// head-locked and facing the eye; the new position is saved on release.
/// </summary>
public class PanelDragger : MonoBehaviour
{
    public QuestControllerPublisher publisher;
    public ImageSubscriber images;

    // Two-handed resize: while one trigger/pinch holds a block, pressing with the other hand
    // scales the view with the distance between the two hands/controllers.
    const float MinSize = 0.3f, MaxSize = 3f;

    bool _enabled;
    CameraPanel _dragged;
    static PanelDragger _instance;

    // A block is being moved or resized (the hand menu gesture is ignored meanwhile:
    // a left-hand pinch is part of a two-hand resize).
    public static bool Dragging => _instance != null && _instance._dragged != null;

    void Awake() => _instance = this;
    XRBaseInputInteractor _interactor, _second;
    float _startDistance, _startSize;
    Quaternion _rayToPanel;   // rotation from the ray direction to the panel direction, head space
    float _radius;

    Transform Head => images.panelParent;

    void Update()
    {
        bool allowed = !publisher.controlRobot;
        if (allowed != _enabled) SetEnabled(allowed);
        if (!allowed) return;

        if (_dragged != null)
        {
            if (_interactor == null || !Held(_interactor))
                EndDrag();
            else if (_second != null)
                Resize();
            else
            {
                TryBeginResize();
                if (_second == null) Follow();
            }
        }

        foreach (var panel in images.Panels)
        {
            if (panel == _dragged) continue;
            panel.SetHighlight(panel.Interactable.isHovered ? CameraPanel.Highlight.Hover : CameraPanel.Highlight.None);

            if (_dragged != null || !panel.Interactable.isHovered) continue;
            foreach (var hover in panel.Interactable.interactorsHovering)
            {
                // Trigger (the interactor's Activate input) pressed while pointing at this block.
                // (Not while the ray is on a button, e.g. Rename: that press is a click, not a drag.)
                if (hover is XRBaseInputInteractor interactor && PressedThisFrame(interactor)
                    && !(interactor is NearFarInteractor nf && nf.TryGetCurrentUIRaycastResult(out _)))
                {
                    BeginDrag(panel, interactor);
                    break;
                }
            }
        }
    }

    void SetEnabled(bool on)
    {
        _enabled = on;
        if (!on && _dragged != null) EndDrag();
        foreach (var panel in images.Panels)
        {
            panel.Interactable.enabled = on;
            panel.SetEditable(on);
            panel.SetHighlight(CameraPanel.Highlight.None);
        }
    }

    void BeginDrag(CameraPanel panel, XRBaseInputInteractor interactor)
    {
        _fingersClosed.Clear();
        _dragged = panel;
        _interactor = interactor;
        Vector3 panelDir = panel.transform.localPosition;
        _radius = panelDir.magnitude;
        _rayToPanel = Quaternion.FromToRotation(RayDirection(), panelDir.normalized);
        panel.SetHighlight(CameraPanel.Highlight.Drag);
    }

    void Follow()
    {
        Vector3 dir = _rayToPanel * RayDirection();
        Vector3 pos = dir * _radius;
        _dragged.transform.localPosition = pos;
        _dragged.transform.localRotation = Quaternion.LookRotation(pos);
    }

    void EndDrag()
    {
        images.SavePosition(_dragged);
        _dragged.SetHighlight(CameraPanel.Highlight.None);
        _dragged = null;
        _interactor = null;
        _second = null;
    }

    // While one hand/controller holds a block, the other one pressing (pinch or trigger) anywhere,
    // except on a button, starts resizing. Requiring its ray on the same block was unreliable
    // with hands: once one hand holds the block, the other hand's ray rarely registers on it.
    void TryBeginResize()
    {
        foreach (var other in OtherInteractors())
        {
            if (!PressedThisFrame(other)) continue;
            if (other is NearFarInteractor nf && nf.TryGetCurrentUIRaycastResult(out _)) continue;
            float d = ControllerDistance();
            if (d <= 0.01f) return;   // hands/controllers not both tracked
            _second = other;
            _fingersClosed.Remove(other);
            _startDistance = d;
            _startSize = _dragged.Size;
            Debug.Log($"[PanelDragger] resize start: {_dragged.Config.displayName} size {_startSize:F2} distance {d:F2} m");
            return;
        }
    }

    readonly System.Collections.Generic.List<NearFarInteractor> _interactors = new();

    // Active hand/controller interactors other than the one holding the block (the input modality
    // manager deactivates controller or hand objects that are not in use).
    System.Collections.Generic.IEnumerable<XRBaseInputInteractor> OtherInteractors()
    {
        if (_interactors.Count == 0)
            _interactors.AddRange(FindObjectsByType<NearFarInteractor>(FindObjectsInactive.Include, FindObjectsSortMode.None));
        foreach (var i in _interactors)
            if (i != null && i != _interactor && i.isActiveAndEnabled) yield return i;
    }

    // Size follows the controllers' distance; the block stays where it is while resizing.
    void Resize()
    {
        if (_second == null || !Held(_second))
        {
            // Back to moving with the first controller, from where the block is now.
            Debug.Log($"[PanelDragger] resize end: {_dragged.Config.displayName} size {_dragged.Size:F2} distance {ControllerDistance():F2} m");
            _second = null;
            _rayToPanel = Quaternion.FromToRotation(RayDirection(), _dragged.transform.localPosition.normalized);
            return;
        }
        float d = ControllerDistance();
        if (d <= 0f) return;   // lost tracking for a moment: keep the current size
        float size = Mathf.Clamp(_startSize * d / _startDistance, MinSize, MaxSize);
        if (Mathf.Abs(size - _dragged.Size) > 0.005f) _dragged.SetSize(size);
    }

    // Grab input: the trigger (Activate) on controllers; with tracked hands a pinch, which XRI
    // reports as Select (hands have no Activate).
    static bool UsingHands =>
        XRInputModalityManager.currentInputMode.Value == XRInputModalityManager.InputMode.TrackedHand;

    static bool PressedThisFrame(XRBaseInputInteractor i)
        => UsingHands ? i.selectInput.ReadWasPerformedThisFrame() : i.activateInput.ReadWasPerformedThisFrame();

    // With hands, release is read from the fingers: XRI's pinch only releases once the hand is
    // opened wide, so a block kept following the hand after the fingers had parted.
    const float PinchReleaseDistance = 0.035f;   // thumb tip to index tip (m)

    // Hands whose fingertips have closed since the pinch began. Until then XRI's pinch state is
    // used: it can report a pinch while the tips are still a few centimetres apart, and the
    // finger check alone would release that grab immediately.
    static readonly System.Collections.Generic.HashSet<XRBaseInputInteractor> _fingersClosed = new();

    static bool Held(XRBaseInputInteractor i)
    {
        if (!UsingHands) return i.activateInput.ReadIsPerformed();
        bool xriHeld = i.selectInput.ReadIsPerformed();
        float d = PinchDistance(i.handedness);
        if (d < 0f) return xriHeld;                       // hand not tracked this frame
        if (d < PinchReleaseDistance) { _fingersClosed.Add(i); return true; }
        if (_fingersClosed.Contains(i)) { _fingersClosed.Remove(i); return false; }   // fingers parted
        return xriHeld;
    }

    // Thumb tip to index tip of the interactor's hand (m), or -1 if that hand is not tracked.
    static float PinchDistance(InteractorHandedness handedness)
    {
        var hands = HudInput.Hands;
        if (hands == null || handedness == InteractorHandedness.None) return -1f;
        var hand = handedness == InteractorHandedness.Left ? hands.leftHand : hands.rightHand;
        if (hand.isTracked &&
            hand.GetJoint(UnityEngine.XR.Hands.XRHandJointID.ThumbTip).TryGetPose(out Pose thumb) &&
            hand.GetJoint(UnityEngine.XR.Hands.XRHandJointID.IndexTip).TryGetPose(out Pose index))
            return Vector3.Distance(thumb.position, index.position);
        return -1f;
    }

    // Distance between the two hands (palm joints) or controllers (device positions), in metres.
    // Read from tracking data: the hands' ray interactor objects do not follow the hands closely.
    static float ControllerDistance()
    {
        if (UsingHands)
        {
            var hands = HudInput.Hands;
            if (hands != null && hands.leftHand.isTracked && hands.rightHand.isTracked &&
                hands.leftHand.GetJoint(UnityEngine.XR.Hands.XRHandJointID.Palm).TryGetPose(out Pose l) &&
                hands.rightHand.GetJoint(UnityEngine.XR.Hands.XRHandJointID.Palm).TryGetPose(out Pose r))
                return Vector3.Distance(l.position, r.position);
            return -1f;
        }
        var left = UnityEngine.XR.InputDevices.GetDeviceAtXRNode(UnityEngine.XR.XRNode.LeftHand);
        var right = UnityEngine.XR.InputDevices.GetDeviceAtXRNode(UnityEngine.XR.XRNode.RightHand);
        if (left.TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out Vector3 lp) &&
            right.TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out Vector3 rp))
            return Vector3.Distance(lp, rp);
        return -1f;
    }

    // Controller ray direction in head space.
    Vector3 RayDirection()
    {
        Transform origin = _interactor is IXRRayProvider ray ? ray.GetOrCreateRayOrigin() : _interactor.transform;
        return Head.InverseTransformDirection(origin.forward);
    }
}
