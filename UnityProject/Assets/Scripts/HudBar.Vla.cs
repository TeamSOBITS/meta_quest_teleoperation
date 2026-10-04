using TMPro;
using UnityEngine;
using UnityEngine.UI;

// The header's VLA pill (VLA stage status, see VlaStatus / VlaStatusRule): between the ROS IP and the connection
// pill, the same label and colour as the status strip's REC group ("REC 00:01:05" with a pulsing dot, "PAUSED ...",
// "IDLE", "ERROR"). Inactive unless a stage node publishes; while shown the IP label gives it room (its text shrinks to
// fit, then the ellipsis cuts it).
// Polled in Update like the rest of the header, so a rebuilt bar holds no state. The rest is in HudBar.cs.
public partial class HudBar
{
    const float RecordPillFontRatio = 0.9f;   // of the body font
    const float RecordDotRatio = 0.4f;        // dot diameter / pill height
    const float MinIpFontRatio = 0.6f;        // smallest IP text while the pill is shown, of its normal size

    Image _recPill, _recPillDot;
    TextMeshProUGUI _recPillText;
    float _ipFullWidth, _recRight, _recTop, _recHeight, _recPad;
    VlaStatusRule.Look? _shownRecLook;

    // `right`: the pill's right edge (mm); `ipWidth`: the IP label's width without the pill.
    void BuildVlaPill(RectTransform root, float top, float height, float right, float ipWidth)
    {
        float font = HudTheme.BodyFontBase * HudUi.MmPerMetre * RecordPillFontRatio;
        (_recPill, _recPillText) = HudUi.Pill(root, "Vla Pill", "", font, height, Color.clear, Color.white, bold: true);
        _recPad = height * 0.35f;
        float dot = height * RecordDotRatio;
        _recPillDot = HudUi.Round(HudUi.Box(_recPill.transform, "Dot", HudTheme.Record), dot / 2f);
        var d = _recPillDot.rectTransform;
        d.anchorMin = d.anchorMax = d.pivot = new Vector2(0f, 0.5f);
        d.sizeDelta = new Vector2(dot, dot);
        d.anchoredPosition = new Vector2(_recPad, 0f);
        _recPillText.margin = new Vector4(_recPad + dot + _recPad * 0.6f, 0f, _recPad, 0f);
        _recRight = right; _recTop = top; _recHeight = height;
        _ipFullWidth = ipWidth;
        // While the pill takes room, a long IP shrinks (down to MinIpFontRatio) before the ellipsis cuts it.
        _ip.enableAutoSizing = true;
        _ip.fontSizeMax = _ip.fontSize;
        _ip.fontSizeMin = _ip.fontSize * MinIpFontRatio;
        _recPill.gameObject.SetActive(false);
    }

    public bool VlaPillShown => _recPill != null && _recPill.gameObject.activeSelf;

    void UpdateVlaPill()
    {
        if (_recPill == null) return;
        var rs = VlaStatus.Latest;
        var look = rs != null ? rs.Look : VlaStatusRule.Look.Hidden;
        bool show = look != VlaStatusRule.Look.Hidden;
        if (_shownRecLook != look)
        {
            _shownRecLook = look;
            _recPill.gameObject.SetActive(show);
            float ipWidth = _ipFullWidth;
            if (show)
            {
                var c = VlaStatusRule.Colour(look);
                _recPill.color = HudTheme.PillBackground(c, strong: look == VlaStatusRule.Look.Recording);
                _recPillText.color = c;
                // Digits share one width, so the width only changes with the state (the preferred width includes the margins).
                float w = Mathf.Ceil(_recPillText.GetPreferredValues(VlaStatusRule.Label(look, 0f)).x) + 4f;
                HudUi.Place(_recPill.rectTransform, _recRight - w, _recTop, w, _recHeight);
                ipWidth = Mathf.Max(0f, _ipFullWidth - w - GapMm);
            }
            _ip.rectTransform.sizeDelta = new Vector2(ipWidth, _ip.rectTransform.sizeDelta.y);
        }
        if (!show) return;
        _recPillText.text = VlaStatusRule.Label(look, rs.ElapsedS);
        _recPillDot.color = HudTheme.WithAlpha(VlaStatusRule.Colour(look),
                                               look == VlaStatusRule.Look.Recording ? VlaStatusRule.PulseAlpha(Time.unscaledTime) : 1f);
    }
}
