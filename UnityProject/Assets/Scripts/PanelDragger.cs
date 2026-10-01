using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

/// <summary>
/// Lets the user move camera blocks with a controller while Joy publishing is off
/// (so the trigger can't also reach the robot): point the ray at a block, hold the trigger,
/// and the block follows the ray around the head at its current distance. Blocks stay
/// head-locked and facing the eye; the new position is saved on release.
/// </summary>
public class PanelDragger : MonoBehaviour
{
    public QuestControllerPublisher publisher;
    public ImageSubscriber images;

    // Two-controller resize: while one trigger holds a block, pointing the other controller at
    // it and pulling its trigger scales the view with the distance between the controllers.
    const float MinSize = 0.3f, MaxSize = 3f;

    bool _enabled;
    CameraPanel _dragged;
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
            if (_interactor == null || !_interactor.activateInput.ReadIsPerformed())
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
                if (hover is XRBaseInputInteractor interactor && interactor.activateInput.ReadWasPerformedThisFrame()
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

    void TryBeginResize()
    {
        foreach (var hover in _dragged.Interactable.interactorsHovering)
        {
            if (hover is XRBaseInputInteractor other && other != _interactor && other.activateInput.ReadWasPerformedThisFrame())
            {
                _second = other;
                _startDistance = Mathf.Max(ControllerDistance(), 0.01f);
                _startSize = _dragged.Size;
                return;
            }
        }
    }

    // Size follows the controllers' distance; the block stays where it is while resizing.
    void Resize()
    {
        if (_second == null || !_second.activateInput.ReadIsPerformed())
        {
            // Back to moving with the first controller, from where the block is now.
            _second = null;
            _rayToPanel = Quaternion.FromToRotation(RayDirection(), _dragged.transform.localPosition.normalized);
            return;
        }
        float size = Mathf.Clamp(_startSize * ControllerDistance() / _startDistance, MinSize, MaxSize);
        if (Mathf.Abs(size - _dragged.Size) > 0.005f) _dragged.SetSize(size);
    }

    float ControllerDistance() => Vector3.Distance(_interactor.transform.position, _second.transform.position);

    // Controller ray direction in head space.
    Vector3 RayDirection()
    {
        Transform origin = _interactor is IXRRayProvider ray ? ray.GetOrCreateRayOrigin() : _interactor.transform;
        return Head.InverseTransformDirection(origin.forward);
    }
}
