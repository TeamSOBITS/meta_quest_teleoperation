using UnityEngine;
using UnityEngine.XR.Hands;
using UnityEngine.XR.Interaction.Toolkit.Inputs;

/// <summary>
/// Menu input of the robot screen: the left controller's menu button, or with hand tracking the
/// hand menu gesture (left palm facing you + pinch), shows / hides the HUD bar. Ignored while a
/// block is being dragged (the pinch of the drag would toggle the bar). Back to robots is on the bar.
/// </summary>
public class HudInput
{
    // --- Hand menu gesture: left palm facing the head + thumb and index pinched. ---
    // Read from the hand joints (XR Hands): Meta's aim "menu pressed" flag did not arrive on the
    // Quest 3S, so it is only kept as a second source.
    const float PinchDistance = 0.02f;     // thumb tip to index tip (m)
    const float PalmFacingDot = 0.5f;      // cos of max angle between palm normal and the head
    static XRHandSubsystem _hands;
    static readonly System.Collections.Generic.List<XRHandSubsystem> _found = new();

    bool _wasPressed;

    public static XRHandSubsystem Hands
    {
        get
        {
            if (_hands != null && _hands.running) return _hands;
            SubsystemManager.GetSubsystems(_found);
            _hands = _found.Find(h => h.running);
            return _hands;
        }
    }

    public static bool HandsTracked => Hands != null && Hands.leftHand.isTracked;

    // True once per press. `head` is the transform the joint poses are relative to (the HUD parent).
    public bool BarTogglePressed(Transform head)
    {
        bool pressed = ControllerMenuPressed() || HandMenuGesture(head);
        bool edge = pressed && !_wasPressed && !PanelDragger.Dragging;
        _wasPressed = pressed;
        return edge;
    }

    static bool ControllerMenuPressed()
        => UnityEngine.XR.InputDevices.GetDeviceAtXRNode(UnityEngine.XR.XRNode.LeftHand)
               .TryGetFeatureValue(UnityEngine.XR.CommonUsages.menuButton, out bool pressed) && pressed;

    static bool HandMenuGesture(Transform head)
    {
        if (PanelDragger.Dragging) return false;
        if (MetaAimHand.left != null && ((ulong)MetaAimHand.left.aimFlags.ReadValue() & (ulong)MetaAimFlags.MenuPressed) != 0)
            return true;
        if (!HandsTracked || head == null) return false;

        var hand = Hands.leftHand;
        if (!hand.GetJoint(XRHandJointID.Palm).TryGetPose(out Pose palm) ||
            !hand.GetJoint(XRHandJointID.ThumbTip).TryGetPose(out Pose thumb) ||
            !hand.GetJoint(XRHandJointID.IndexTip).TryGetPose(out Pose index))
            return false;
        if (Vector3.Distance(thumb.position, index.position) > PinchDistance) return false;

        // Joint poses are relative to the XR Origin's tracking space, i.e. the camera's parent.
        Vector3 headPos = head.localPosition;
        Vector3 palmNormal = palm.rotation * Vector3.down;   // +Y points out of the back of the hand
        return Vector3.Dot(palmNormal, (headPos - palm.position).normalized) > PalmFacingDot;
    }
}
