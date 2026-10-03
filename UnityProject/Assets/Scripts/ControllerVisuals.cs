using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Hands;
using UnityEngine.XR.Interaction.Toolkit.Inputs;
using UnityEngine.XR.Interaction.Toolkit.Interactors.Visuals;

/// <summary>
/// First person: controller and hand visuals are hidden while the bar is hidden.
/// Only what is drawn is switched off: renderers (controller models, hand mesh, line, reticle,
/// pinch / poke visuals), the ray's XRInteractorLineVisual (it re-enables its LineRenderer every
/// frame) and the XRHandMeshController (it re-enables the hand mesh on tracking changes). The
/// GameObjects and interactors stay active, so input, the menu button and the hand gesture work.
/// </summary>
public class ControllerVisuals
{
    class Hidden { public Renderer renderer; public Behaviour behaviour; public bool restore; }

    readonly List<Hidden> _hidden = new List<Hidden>();
    readonly HashSet<Object> _hiddenKnown = new HashSet<Object>();
    bool _dirty;

    public bool IsHidden { get; private set; }
    // Renderers (and visual behaviours) currently switched off by this.
    public int Count => _hidden.Count;

    public void SetHidden(bool hide)
    {
        if (hide == IsHidden) return;
        IsHidden = hide;
        if (hide)
        {
            XRInputModalityManager.currentInputMode.Subscribe(OnInputModeChanged);
            Scan();
        }
        else
        {
            XRInputModalityManager.currentInputMode.Unsubscribe(OnInputModeChanged);
            // Renderers first: the hand mesh controller then re-applies the tracking state.
            foreach (var h in _hidden)
                if (h.renderer != null && h.restore) h.renderer.enabled = true;
            foreach (var h in _hidden)
                if (h.behaviour != null && h.restore) h.behaviour.enabled = true;
            _hidden.Clear();
            _hiddenKnown.Clear();
        }
        DevLog.Log("[TeleopHud]", $"controller visuals {(hide ? "hidden" : "shown")}" + (hide ? $" ({_hidden.Count} objects)" : ""));
    }

    // The modality switched between controllers and hands: other objects are active now; look again.
    void OnInputModeChanged(XRInputModalityManager.InputMode _) => _dirty = true;

    void Scan()
    {
        var modality = Object.FindFirstObjectByType<XRInputModalityManager>();
        if (modality == null) return;
        foreach (var root in new[] { modality.leftController, modality.rightController, modality.leftHand, modality.rightHand })
        {
            if (root == null) continue;
            foreach (var r in root.GetComponentsInChildren<Renderer>(true))
                if (_hiddenKnown.Add(r)) { _hidden.Add(new Hidden { renderer = r, restore = r.enabled }); r.enabled = false; }
            foreach (var l in root.GetComponentsInChildren<XRInteractorLineVisual>(true))
                if (_hiddenKnown.Add(l)) { _hidden.Add(new Hidden { behaviour = l, restore = l.enabled }); l.enabled = false; }
            foreach (var m in root.GetComponentsInChildren<XRHandMeshController>(true))
                if (_hiddenKnown.Add(m)) { _hidden.Add(new Hidden { behaviour = m, restore = m.enabled }); m.enabled = false; }
        }
    }

    // Called every LateUpdate.
    public void Tick()
    {
        if (!IsHidden) return;
        if (_dirty) { _dirty = false; Scan(); }
        // Something drew again (e.g. a controller model switched on): keep it off, and remember to restore it.
        foreach (var h in _hidden)
            if (h.renderer != null && h.renderer.enabled) { h.renderer.enabled = false; h.restore = true; }
    }
}
