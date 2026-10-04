using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Short notice for a VLA stage event (recorder: "Saved · 00:01:05", "Discarded: too short", "Deleted", "Task: pick cup", ...):
/// <see cref="VlaStatus.ToastText"/> for <see cref="VlaStatusRule.ToastS"/> seconds in the event's colour
/// (<see cref="VlaStatusRule.ToastColour"/>), a pill with a dot like the status strip's chips, fading in and out.
/// Non-interactive, head-locked, shown whether the bar (menu) is open or not:
///   first person   just above the status strip (under the head camera image's centre, never over it)
///   blocks, bar hidden   just above the strip, where the bar's bottom row was
///   blocks, bar shown    just above the bar's top edge
/// Same size as the strip's text in both layouts (it takes the strip's scale). One per robot screen, made by TeleopHud;
/// it lives on TeleopHud's object (so it ticks while its canvas, "Vla Toast", is inactive) and follows the strip's /
/// the bar's parent, so lazy follow and ReparentHud carry it along (TeleopHud also calls <see cref="Place"/> there).
/// </summary>
public class VlaToast : MonoBehaviour
{
    // mm at the strip's scale (font = the strip's, sized for 1.2 m).
    const float HeightMm = 64f, PadMm = 24f, DotMm = 14f, GapMm = 12f, AboveMm = 14f, StripDistance = 1.2f;
    const float BackgroundAlpha = 0.9f;

    TeleopHud _hud;
    RectTransform _canvas;
    CanvasGroup _group;
    Image _tint, _dot;
    TextMeshProUGUI _label;
    string _shownText;
    float _shownUntil;

    public RectTransform Rect => _canvas;
    public bool Shown => _canvas != null && _canvas.gameObject.activeSelf;
    public TextMeshProUGUI Label => _label;
    public Color Colour => _label != null ? _label.color : Color.clear;

    public static VlaToast Create(TeleopHud hud)
    {
        var toast = hud.gameObject.AddComponent<VlaToast>();
        toast._hud = hud;
        toast.Build();
        return toast;
    }

    void Build()
    {
        float font = HudUi.FontAt(HudTheme.BodyFont, StripDistance);
        _canvas = HudUi.CreateCanvas("Vla Toast", _hud.hudParent, new Vector3(0f, 0f, StripDistance), new Vector2(400f, HeightMm), interactive: false);
        _canvas.GetComponent<Canvas>().sortingOrder = HudTheme.SortingOrder + 2;   // above the strip and the bar
        _group = _canvas.gameObject.AddComponent<CanvasGroup>();
        _group.interactable = false;
        _group.blocksRaycasts = false;
        // Panel underneath so the tint reads the same on the image, the studio or passthrough.
        HudUi.Stretch(HudUi.Round(HudUi.Box(_canvas, "Background", HudTheme.WithAlpha(HudTheme.Panel, BackgroundAlpha)), HeightMm / 2f).rectTransform);
        _tint = HudUi.Round(HudUi.Box(_canvas, "Tint", Color.clear), HeightMm / 2f);
        HudUi.Stretch(_tint.rectTransform);
        _dot = HudUi.Round(HudUi.Box(_canvas, "Dot", Color.white), DotMm / 2f);
        HudUi.Place(_dot.rectTransform, PadMm, (HeightMm - DotMm) / 2f, DotMm, DotMm);
        _label = HudUi.Label(_canvas, "Label", "", font, TextAlignmentOptions.Left);
        _label.fontStyle = FontStyles.Bold;
        _label.textWrappingMode = TextWrappingModes.NoWrap;
        _label.verticalAlignment = VerticalAlignmentOptions.Middle;
        _canvas.gameObject.SetActive(false);
    }

    void LateUpdate()
    {
        var rs = _hud != null ? _hud.VlaStatus : null;
        bool show = rs != null && rs.ToastShown;
        if (show != _canvas.gameObject.activeSelf) _canvas.gameObject.SetActive(show);
        if (!show) { _shownText = null; return; }
        if (rs.ToastText != _shownText || rs.ToastUntil != _shownUntil) Show(rs);

        float now = Time.unscaledTime, start = rs.ToastUntil - VlaStatusRule.ToastS;
        _group.alpha = Mathf.Clamp01((now - start) / HudTheme.VlaToastFadeInS) * Mathf.Clamp01((rs.ToastUntil - now) / HudTheme.VlaToastFadeOutS);
        Place();
    }

    void Show(VlaStatus rs)
    {
        _shownText = rs.ToastText;
        _shownUntil = rs.ToastUntil;
        var c = VlaStatusRule.ToastColour(rs.ToastEvent);
        _tint.color = HudTheme.PillBackground(c, strong: true);
        _dot.color = c;
        _label.color = c;
        _label.text = rs.ToastText;
        float textW = Mathf.Ceil(_label.GetPreferredValues(rs.ToastText).x) + 2f;
        float left = PadMm + DotMm + GapMm;
        HudUi.Place(_label.rectTransform, left, 0f, textW, HeightMm);
        _canvas.sizeDelta = new Vector2(left + textW + PadMm, HeightMm);
    }

    // Above the strip (first person, or blocks with the bar hidden), else above the bar; same parent as that, the strip's scale.
    public void Place()
    {
        if (_hud == null || _canvas == null) return;
        var strip = _hud.Strip != null ? (RectTransform)_hud.Strip.transform : null;
        var bar = _hud.Bar as RectTransform;
        bool barShown = bar != null && bar.gameObject.activeSelf;
        RectTransform anchor = strip != null && (_hud.FirstPerson || !barShown) ? strip : bar != null ? bar : strip;
        if (anchor == null) return;
        Vector3 scale = strip != null ? strip.localScale : Vector3.one * (StatusStrip.BlocksScale / HudUi.MmPerMetre);

        if (_canvas.parent != anchor.parent) _canvas.SetParent(anchor.parent, false);
        _canvas.localScale = scale;
        float offset = anchor.sizeDelta.y * anchor.localScale.y / 2f + (AboveMm + HeightMm / 2f) * scale.y;
        var pos = anchor.localPosition + anchor.localRotation * Vector3.up * offset;
        _canvas.localPosition = pos;
        _canvas.localRotation = Quaternion.LookRotation(pos);
    }
}
