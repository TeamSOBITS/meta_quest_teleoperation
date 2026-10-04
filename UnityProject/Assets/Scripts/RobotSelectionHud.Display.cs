using TMPro;
using UnityEngine;
using UnityEngine.UI;

// The "Display" row at the bottom of the selection screen (see RobotSelectionHud):
//   Display   [A-]  Text 100 %  [A+]   [x] High contrast
// A change is saved at once (HudTheme.SetTextScale / SetHighContrast) and the screen is rebuilt with it; the robot screens
// pick it up the next time they are built.
public partial class RobotSelectionHud
{
    // Widths in body fonts, so the row keeps its proportions at every text size.
    const float DisplayCaptionW = 4f, DisplayStepW = 3.2f, DisplayValueW = 6.4f, DisplayToggleW = 9.5f, DisplayGapW = 0.6f;

    public static string TextScaleText(float scale) => $"Text {Mathf.RoundToInt(scale * 100f)} %";

    void BuildDisplayRow(RectTransform root, float top, float widthMm, float body, float height)
    {
        float gap = body * DisplayGapW;
        float total = body * (DisplayCaptionW + 2f * DisplayStepW + DisplayValueW + DisplayToggleW) + 4f * gap;
        float x = (widthMm - total) / 2f;

        var caption = HudUi.Label(root, "Display Caption", "Display", body, TextAlignmentOptions.Left);
        caption.color = HudTheme.Muted;
        caption.textWrappingMode = TextWrappingModes.NoWrap;
        HudUi.Place(caption.rectTransform, x, top, body * DisplayCaptionW, height);
        x += body * DisplayCaptionW + gap;

        float scale = Settings.TextScale;
        var smaller = HudUi.Button(root, "A\u2212", body, () => SetTextScale(Settings.TextScale - Settings.TextScaleStep));
        smaller.interactable = scale > Settings.MinTextScale + 0.01f;
        HudUi.Place((RectTransform)smaller.transform, x, top, body * DisplayStepW, height);
        x += body * DisplayStepW + gap;

        var value = HudUi.Label(root, "Text Size", TextScaleText(scale), body);
        value.textWrappingMode = TextWrappingModes.NoWrap;
        HudUi.Place(value.rectTransform, x, top, body * DisplayValueW, height);
        x += body * DisplayValueW + gap;

        var larger = HudUi.Button(root, "A+", body, () => SetTextScale(Settings.TextScale + Settings.TextScaleStep));
        larger.interactable = scale < Settings.MaxTextScale - 0.01f;
        HudUi.Place((RectTransform)larger.transform, x, top, body * DisplayStepW, height);
        x += body * DisplayStepW + gap;

        var contrast = HudUi.Toggle(root, "High contrast", body, HudTheme.HighContrast, on =>
        {
            HudTheme.SetHighContrast(on);
            Build();
        });
        HudUi.Place((RectTransform)contrast.transform, x, top, body * DisplayToggleW, height);
    }

    void SetTextScale(float scale)
    {
        HudTheme.SetTextScale(scale);
        Build();
    }
}
