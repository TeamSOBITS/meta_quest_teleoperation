using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Toggle that applies its hover/press tint to the check mark as well as the box,
/// so the whole control darkens while the controller ray points at it.
/// </summary>
public class HudToggle : Toggle
{
    protected override void DoStateTransition(SelectionState state, bool instant)
    {
        base.DoStateTransition(state, instant);
        if (graphic == null) return;

        Color tint = state switch
        {
            SelectionState.Highlighted => colors.highlightedColor,
            SelectionState.Pressed     => colors.pressedColor,
            SelectionState.Disabled    => colors.disabledColor,
            _                          => colors.normalColor,
        };
        // Colour only: the check mark's alpha is how Toggle shows on/off.
        graphic.CrossFadeColor(tint * colors.colorMultiplier, instant ? 0f : colors.fadeDuration, true, false);
    }
}
