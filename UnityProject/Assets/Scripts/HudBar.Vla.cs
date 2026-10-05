using TMPro;
using UnityEngine;
using UnityEngine.UI;

// The header's VLA pill (VLA stage status, see VlaStatus / VlaStatusRule): between the ROS IP and the connection
// pill, the same label and colour as the status strip's VLA group ("REC 00:01:05" with a pulsing dot, "PAUSED ...",
// "PLAY 00:00:12", "RESETTING", "IDLE", "ERROR"). The bar has no room for the strip's GRIP chip, so while the policy
// plays with a deadman the dot takes the chip's colour (Good driving, Warn released). Inactive unless a stage node publishes; while shown the IP label gives it room (its text shrinks to
// fit, then the ellipsis cuts it).
// Polled in Update like the rest of the header, so a rebuilt bar holds no state. The rest is in HudBar.cs.
public partial class HudBar
{
    const float VlaPillFontRatio = 0.9f;   // of the body font
    const float VlaDotRatio = 0.4f;        // dot diameter / pill height
    const float MinIpFontRatio = 0.6f;        // smallest IP text while the pill is shown, of its normal size

    Image _vlaPill, _vlaPillDot;
    TextMeshProUGUI _vlaPillText;
    float _ipFullWidth, _vlaRight, _vlaTop, _vlaHeight, _vlaPad;
    VlaStatusRule.Look? _shownVlaLook;

    // `right`: the pill's right edge (mm); `ipWidth`: the IP label's width without the pill.
    void BuildVlaPill(RectTransform root, float top, float height, float right, float ipWidth)
    {
        float font = HudTheme.BodyFontBase * HudUi.MmPerMetre * VlaPillFontRatio;
        (_vlaPill, _vlaPillText) = HudUi.Pill(root, "Vla Pill", "", font, height, Color.clear, Color.white, bold: true);
        _vlaPad = height * 0.35f;
        float dot = height * VlaDotRatio;
        _vlaPillDot = HudUi.Round(HudUi.Box(_vlaPill.transform, "Dot", HudTheme.Record), dot / 2f);
        var d = _vlaPillDot.rectTransform;
        d.anchorMin = d.anchorMax = d.pivot = new Vector2(0f, 0.5f);
        d.sizeDelta = new Vector2(dot, dot);
        d.anchoredPosition = new Vector2(_vlaPad, 0f);
        _vlaPillText.margin = new Vector4(_vlaPad + dot + _vlaPad * 0.6f, 0f, _vlaPad, 0f);
        _vlaRight = right; _vlaTop = top; _vlaHeight = height;
        _ipFullWidth = ipWidth;
        // While the pill takes room, a long IP shrinks (down to MinIpFontRatio) before the ellipsis cuts it.
        _ip.enableAutoSizing = true;
        _ip.fontSizeMax = _ip.fontSize;
        _ip.fontSizeMin = _ip.fontSize * MinIpFontRatio;
        _vlaPill.gameObject.SetActive(false);
    }

    public bool VlaPillShown => _vlaPill != null && _vlaPill.gameObject.activeSelf;

    void UpdateVlaPill()
    {
        if (_vlaPill == null) return;
        var rs = VlaStatus.Latest;
        var look = rs != null ? rs.Look : VlaStatusRule.Look.Hidden;
        bool show = look != VlaStatusRule.Look.Hidden;
        if (_shownVlaLook != look)
        {
            _shownVlaLook = look;
            _vlaPill.gameObject.SetActive(show);
            float ipWidth = _ipFullWidth;
            if (show)
            {
                var c = VlaStatusRule.Colour(look);
                _vlaPill.color = HudTheme.PillBackground(c, strong: look == VlaStatusRule.Look.Recording || look == VlaStatusRule.Look.Playing);
                _vlaPillText.color = c;
                // Digits share one width, so the width only changes with the state (the preferred width includes the margins).
                float w = Mathf.Ceil(_vlaPillText.GetPreferredValues(VlaStatusRule.Label(look, 0f)).x) + 4f;
                HudUi.Place(_vlaPill.rectTransform, _vlaRight - w, _vlaTop, w, _vlaHeight);
                ipWidth = Mathf.Max(0f, _ipFullWidth - w - GapMm);
            }
            _ip.rectTransform.sizeDelta = new Vector2(ipWidth, _ip.rectTransform.sizeDelta.y);
        }
        if (!show) return;
        _vlaPillText.text = VlaStatusRule.Label(look, rs.ElapsedS);
        var chip = rs.IsDeploy ? VlaStatusRule.Deadman(rs.DeadmanEnabled, rs.DeadmanEngaged, rs.State) : null;
        _vlaPillDot.color = HudTheme.WithAlpha(chip.HasValue ? chip.Value.colour : VlaStatusRule.Colour(look),
                                               VlaStatusRule.Pulses(look, rs.DeadmanEngaged || !rs.DeadmanEnabled) ? VlaStatusRule.PulseAlpha(Time.unscaledTime) : 1f);
    }
}
