using System;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// The one place for the HUD's look: distances, font sizes, colours and the sizes the bar, the
/// selection screen and the camera cards share (all canvases use 1 unit = 1 mm). File-specific
/// layout numbers stay in their files as named constants.
/// </summary>
public static class HudTheme
{
    // Materials with custom shaders (first-person surround, undistortion); see HudAssets.
    public static HudAssets Assets => HudAssets.Instance;

    // --- Distances and fonts ---

    // Distance the camera blocks sit at; sizes below are tuned for it (metres).
    public const float ReferenceDistance = 4.3f;
    // Gap between camera blocks and the lowest y the auto layout may use (keeps blocks above the HUD bar), metres.
    public const float BlockGap = 0.15f, LayoutMinBottom = -1.75f;
    // Design font sizes (metres at ReferenceDistance; HudUi.FontAt scales them for other distances).
    public const float TitleFontBase = 0.14f;   // camera names, ROS IP
    public const float BodyFontBase  = 0.09f;   // topics, control panel entries
    // What screens use: the design sizes x the user's text size (Settings.TextScale). Layout that derives from them
    // (cards, selection screen, strip) grows with the text. The HUD bar is built at the design sizes and scaled as a whole by
    // FontScale instead (HudBar), so its columns keep fitting their labels.
    public static float TitleFont => TitleFontBase * FontScale;
    public static float BodyFont  => BodyFontBase * FontScale;

    // --- Display settings (text size, high contrast): cached; Reload re-reads them ---

    static float _fontScale = -1f;
    static int _highContrast = -1;
    public static float FontScale { get { if (_fontScale < 0f) _fontScale = Settings.TextScale; return _fontScale; } }
    public static bool HighContrast { get { if (_highContrast < 0) _highContrast = Settings.HighContrast ? 1 : 0; return _highContrast == 1; } }
    // Raised after the text size or the contrast changed; screens built earlier rebuild (selection screen) or keep their look until the next build.
    public static event Action Changed;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    public static void Reload() { _fontScale = -1f; _highContrast = -1; }

    public static void SetTextScale(float scale)
    {
        Settings.TextScale = scale;
        Settings.Save();
        Reload();
        Changed?.Invoke();
    }

    public static void SetHighContrast(bool on)
    {
        Settings.HighContrast = on;
        Settings.Save();
        Reload();
        Changed?.Invoke();
    }

    // HUD canvases draw after transparent scenery (e.g. a grid floor). UI does not write depth,
    // so without this a large transparent floor sorted later is blended on top of the panels.
    public const int SortingOrder = 100;

    // --- Colours ---

    // A step lighter than StudioEnvironment.Background so cards read as surfaces on it.
    // High contrast: opaque panels (alpha 1) instead of 0.92, brighter secondary text, stronger dividers.
    public static float CardAlpha => HighContrast ? 1f : 0.92f;
    public static Color Panel => new Color(0.14f, 0.16f, 0.20f, CardAlpha);
    public static readonly Color Control = new Color(0.25f, 0.27f, 0.32f, 1f);
    public static readonly Color Surface = new Color(0.21f, 0.23f, 0.28f, 1f);    // cards on a Panel (selection screen)
    public static readonly Color Accent  = new Color(0.30f, 0.65f, 1f, 1f);
    public static Color Muted => new Color(1f, 1f, 1f, HighContrast ? 0.85f : 0.6f);   // secondary text
    public static readonly Color Good    = new Color(0.38f, 0.88f, 0.50f, 1f);
    public static readonly Color Bad     = new Color(1f, 0.38f, 0.38f, 1f);
    public static readonly Color Warn    = new Color(1f, 0.71f, 0.28f, 1f);
    public static readonly Color Record  = new Color(0.96f, 0.26f, 0.21f, 1f);   // recording badge (red dot, REC)
    public static Color Divider => new Color(1f, 1f, 1f, HighContrast ? 0.4f : 0.12f);
    public static readonly Color Waiting = new Color(0.15f, 0.15f, 0.15f, 1f);   // camera view before the first frame
    public static readonly Color StaleTint = new Color(0.45f, 0.45f, 0.45f, 1f); // camera view while frames stop
    public static readonly Color BadgeBackground = new Color(0f, 0f, 0f, 0.55f);

    // Link health outline (camera cards): alpha of the Warn / Bad colour.
    public const float LinkRingAlpha = 0.9f;

    public static Color WithAlpha(Color c, float alpha) => new Color(c.r, c.g, c.b, alpha);

    // Pill (chip) background for a tint colour: 18 % of the tint, or 25 % for a stronger one.
    public const float PillAlpha = 0.18f, StrongPillAlpha = 0.25f;
    public static Color PillBackground(Color tint, bool strong = false) => WithAlpha(tint, strong ? StrongPillAlpha : PillAlpha);

    // Darken controls while the controller ray points at them (and more while pressed).
    // Selected = normal, so a control doesn't stay tinted after it has been clicked.
    public static readonly Color HoverTint = new Color(0.62f, 0.62f, 0.62f, 1f), PressedTint = new Color(0.45f, 0.45f, 0.45f, 1f);
    public static readonly ColorBlock Hover = new ColorBlock
    {
        normalColor      = Color.white,
        highlightedColor = HoverTint,
        pressedColor     = PressedTint,
        selectedColor    = Color.white,
        disabledColor    = new Color(0.6f, 0.6f, 0.6f, 0.5f),
        colorMultiplier  = 1f,
        fadeDuration     = 0.08f,
    };

    // --- Panels shared by the bar, the selection screen and the setup card (mm) ---

    public const float PanelRadius = 60f;                // corner radius of a backdrop / bar
    public const float Padding = 50f, Gap = 40f;         // inside a panel, between its controls
    public const float EditButtonWidth = 420f;           // the "Edit" button of the ROS IP row

    // --- Camera card (a block, the first-person head card, a hand card) ---

    // Dark card behind the whole block so labels read on any background.
    public const float CardPadding = 40f, CardRadius = 60f, CardOutlineMargin = 30f;
    // Standard view height of a camera at scale 1 and the gap between its name and the view (metres).
    public const float ViewHeight = 1.2f, LabelGap = 0.05f;
    // Highlight ring around a block while hovered / dragged: alpha of the accent colour.
    public const float HoverRingAlpha = 0.35f, DragRingAlpha = 0.7f;

    // --- Recording status (status strip REC group, bar header pill, event toast; see RecordStatusRule) ---

    // The event toast fades in / out over these times (s) within its RecordStatusRule.ToastS.
    public const float RecordToastFadeInS = 0.15f, RecordToastFadeOutS = 0.4f;

    // fps / stale badge, relative to the view it decorates: font height = FontRatio x the view's
    // height clamped to [MinFontMm, MaxFontMm]; corner radius = font; margin from the top-right
    // corner = MarginRatio x the view's width.
    public const float BadgeFontRatio = 0.07f, BadgeMarginRatio = 0.03f, BadgeMinFontMm = 8f, BadgeMaxFontMm = 120f;
}
