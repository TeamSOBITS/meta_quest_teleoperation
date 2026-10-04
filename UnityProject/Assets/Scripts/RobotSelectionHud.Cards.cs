using TMPro;
using UnityEngine;
using UnityEngine.UI;

// The robot cards of the selection screen (see RobotSelectionHud).
public partial class RobotSelectionHud
{
    Button BuildCard(RectTransform parent, RobotProfile robot, float pictureW, float pictureH, float body, float title)
    {
        var bg = HudUi.Round(HudUi.Box(parent, "Card " + robot.displayName, HudTheme.Surface, raycastTarget: true), RadiusMm);
        var button = bg.gameObject.AddComponent<Button>();
        button.targetGraphic = bg;
        button.colors = HudTheme.Hover;
        button.onClick.AddListener(() => Select(robot));

        float top = CardPaddingMm;
        var frame = HudUi.Round(HudUi.Box(bg.transform, "Picture", PictureColor), RadiusMm * PictureRadiusRatio);
        HudUi.Place(frame.rectTransform, CardPaddingMm, top, pictureW, pictureH);
        if (robot.picture == null)
        {
            // Added robots have no picture: show their initials instead.
            frame.color = HudTheme.WithAlpha(HudTheme.Accent * InitialsBackgroundRatio, 1f);
            var initials = HudUi.Label(frame.transform, "Initials", Initials(robot.displayName), title * 3f);
            initials.fontStyle = FontStyles.Bold;
            initials.color = HudTheme.Accent;
            HudUi.Stretch(initials.rectTransform);
        }
        else
        {
            var pic = new GameObject("Image", typeof(RectTransform)).AddComponent<RawImage>();
            pic.transform.SetParent(frame.transform, false);
            pic.texture = robot.picture;
            pic.raycastTarget = false;
            // Fit inside the frame, keeping the picture's aspect ratio.
            float aspect = (float)robot.picture.width / robot.picture.height;
            float w = pictureW * 0.9f, h = pictureH * 0.9f;
            if (w / h > aspect) w = h * aspect; else h = w / aspect;
            var prt = pic.rectTransform;
            prt.anchorMin = prt.anchorMax = new Vector2(0.5f, 0.5f);
            prt.sizeDelta = new Vector2(w, h);
        }
        if (robot.name == Settings.LastRobot)
        {
            float tagH = body * 1.6f;
            var (tag, tagText) = HudUi.Pill(frame.transform, "Last used", "Last used", body * 0.9f, tagH,
                                            HudTheme.WithAlpha(HudTheme.Accent, TagAlpha), TagTextColor, bold: true);
            float tagW = tagText.GetPreferredValues("Last used").x + 1.6f * body;
            HudUi.Place(tag.rectTransform, pictureW - tagW - PictureMarginMm, PictureMarginMm, tagW, tagH);
        }
        if (robot.isCustom)
            BuildRemoveButton(frame.rectTransform, robot, body);
        top += pictureH + 40f;

        // Name, with a dot at the end of the row: green when the robot is online.
        float dot = body * 0.9f;
        var name = HudUi.Label(bg.transform, "Name", robot.displayName, title, TextAlignmentOptions.Left);
        name.fontStyle = FontStyles.Bold;
        name.textWrappingMode = TextWrappingModes.NoWrap;
        Shrink(name, title);
        HudUi.Place(name.rectTransform, CardPaddingMm, top, pictureW - dot - 30f, title * 1.3f);
        if (RobotPresence.CanCheck(robot))
        {
            var status = HudUi.Round(HudUi.Box(bg.transform, "Online", OfflineDotColor), dot / 2f);
            HudUi.Place(status.rectTransform, CardPaddingMm + pictureW - dot, top + (title * 1.3f - dot) / 2f, dot, dot);
            status.gameObject.SetActive(false);   // shown once the first check is done
            _dots[robot] = status;
        }
        top += title * 1.3f;

        int cams = robot.cameras != null ? robot.cameras.Length : 0;
        string ns = string.IsNullOrEmpty(robot.robotNamespace) ? "no namespace" : "/" + robot.robotNamespace;
        var meta = HudUi.Label(bg.transform, "Details",
            $"{ns}  ·  {cams} camera{(cams == 1 ? "" : "s")}", body, TextAlignmentOptions.Left);
        meta.color = HudTheme.Muted;
        Shrink(meta, body);
        HudUi.Place(meta.rectTransform, CardPaddingMm, top, pictureW, body * 1.4f);
        return button;
    }

    // "Remove" on an added robot's card; a second press within a few seconds confirms.
    void BuildRemoveButton(RectTransform frame, RobotProfile robot, float body)
    {
        var button = HudUi.Button(frame, "Remove", body * 0.9f, null);
        var label = button.GetComponentInChildren<TextMeshProUGUI>();
        HudUi.Place((RectTransform)button.transform, PictureMarginMm, PictureMarginMm, RemoveButtonWidthMm * HudTheme.FontScale, body * 1.7f);
        float armedUntil = -1f;
        button.onClick.AddListener(() =>
        {
            if (Time.time < armedUntil)
            {
                RobotLibrary.Delete(robot);
                if (Settings.LastRobot == robot.name) Settings.LastRobot = "";
                Build();
                return;
            }
            armedUntil = Time.time + RemoveConfirmSeconds;
            label.text = "Press again";
            label.color = HudTheme.Bad;
        });
    }

    // One line that shrinks (to MinShrink x the font) before it is cut off, so large text sizes still show the whole name.
    const float MinShrink = 0.6f;
    static void Shrink(TextMeshProUGUI label, float font)
    {
        label.textWrappingMode = TextWrappingModes.NoWrap;
        label.overflowMode = TextOverflowModes.Ellipsis;
        label.enableAutoSizing = true;
        label.fontSizeMax = font;
        label.fontSizeMin = font * MinShrink;
    }

    static string Initials(string name)
    {
        var words = name.Split(new[] { ' ', '_', '-' }, System.StringSplitOptions.RemoveEmptyEntries);
        string s = words.Length >= 2 ? $"{words[0][0]}{words[1][0]}" : name.Length > 0 ? name.Substring(0, Mathf.Min(2, name.Length)) : "?";
        return s.ToUpperInvariant();
    }
}
