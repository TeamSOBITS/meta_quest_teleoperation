using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// <summary>
/// "Experiments" panel to the right of the HUD bar: one toggle per registered experiment
/// (<see cref="ExperimentSettings"/>). A child of the bar, so it is shown/hidden with it and
/// follows it when the bar is reparented (lazy follow); it faces the eye like a bar of its own.
/// </summary>
public class ExperimentsPanel : MonoBehaviour
{
    const float WidthMm = 1900f, PaddingMm = 50f, GapMm = 20f, RadiusMm = 60f, RowHeightMm = 150f;
    const float TitleHeightMm = 220f, SideGapMm = 60f;

    readonly Dictionary<string, Toggle> _toggles = new Dictionary<string, Toggle>();

    // `bar` is the HUD bar canvas (mm units); the panel sits just beyond its right edge, same bottom.
    public static ExperimentsPanel Create(Transform bar)
    {
        var barRt = bar as RectTransform;
        if (barRt == null) return null;

        var entries = new List<(string key, string label, bool on)>(ExperimentSettings.All);
        float body = HudUi.BodyFontSize * HudUi.MmPerMetre;
        float heightMm = PaddingMm + TitleHeightMm + entries.Count * RowHeightMm
                         + Mathf.Max(0, entries.Count - 1) * GapMm + PaddingMm;

        var go = new GameObject("Experiments Panel", typeof(RectTransform));
        go.transform.SetParent(bar, false);
        var canvas = go.AddComponent<Canvas>();   // nested: renders with the bar's world canvas
        canvas.overrideSorting = false;
        go.AddComponent<TrackedDeviceGraphicRaycaster>();

        var rt = (RectTransform)go.transform;
        rt.sizeDelta = new Vector2(WidthMm, heightMm);
        float x = barRt.sizeDelta.x / 2f + SideGapMm + WidthMm / 2f;
        float y = -barRt.sizeDelta.y / 2f + heightMm / 2f;
        rt.localPosition = new Vector3(x, y, 0f);
        // Turn about the bar's vertical axis so the panel faces the eye (bar is at ReferenceDistance).
        rt.localRotation = Quaternion.LookRotation(new Vector3(x, 0f, HudUi.ReferenceDistance * HudUi.MmPerMetre));

        var panel = go.AddComponent<ExperimentsPanel>();
        panel.Build(rt, entries, body);
        ExperimentSettings.Changed += panel.OnChanged;
        return panel;
    }

    void Build(RectTransform root, List<(string key, string label, bool on)> entries, float body)
    {
        HudUi.Stretch(HudUi.Round(HudUi.Box(root, "Background", HudUi.PanelColor), RadiusMm).rectTransform);

        var title = HudUi.Label(root, "Title", "Experiments", HudUi.TitleFontSize * HudUi.MmPerMetre,
                                TextAlignmentOptions.Left);
        title.fontStyle = FontStyles.Bold;
        title.textWrappingMode = TextWrappingModes.NoWrap;
        HudUi.Place(title.rectTransform, PaddingMm, PaddingMm * 0.5f, WidthMm - 2 * PaddingMm, TitleHeightMm);

        float top = PaddingMm + TitleHeightMm;
        foreach (var (key, label, on) in entries)
        {
            string k = key;
            var toggle = HudUi.Toggle(root, label, body, on, v => ExperimentSettings.Set(k, v));
            float rowW = WidthMm - 2 * PaddingMm;
            if (key == ExperimentSettings.HandCams)
            {
                // Small button at the row's right end cycles where the cards go: at hands / corners.
                const float buttonW = 300f;
                TMP_Text text = null;
                var button = HudUi.Button(root, HandModeText(), body * 0.85f, () =>
                {
                    HandCamPip.Mode = HandCamPip.Mode == HandCamPip.ModeHands ? HandCamPip.ModeCorners : HandCamPip.ModeHands;
                    if (text != null) text.text = HandModeText();
                });
                text = button.GetComponentInChildren<TMP_Text>();
                if (text != null) text.textWrappingMode = TextWrappingModes.NoWrap;
                HudUi.Place((RectTransform)button.transform, PaddingMm + rowW - buttonW, top + RowHeightMm * 0.1f, buttonW, RowHeightMm * 0.8f);
                rowW -= buttonW + GapMm;
            }
            HudUi.Place((RectTransform)toggle.transform, PaddingMm, top, rowW, RowHeightMm);
            _toggles[key] = toggle;
            top += RowHeightMm + GapMm;
        }
    }

    static string HandModeText() => HandCamPip.Mode == HandCamPip.ModeCorners ? "corners" : "at hands";

    // Keep the toggles in step when a setting is changed from elsewhere.
    void OnChanged(string key, bool on)
    {
        if (this == null) return;
        if (_toggles.TryGetValue(key, out var t) && t != null) t.SetIsOnWithoutNotify(on);
    }

    void OnDestroy() => ExperimentSettings.Changed -= OnChanged;
}
