using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// <summary>
/// The card every camera image sits on: a world-space canvas (1 unit = 1 mm) with a highlight ring
/// around a dark rounded card, the camera's name above the view, the view (a RawImage that shows
/// "Waiting for ..." until the first frame, with the fps / stale <see cref="CameraBadge"/> in its corner)
/// and a Rename button above the card's top-right corner (hidden until <see cref="SetEditable"/>).
/// Used by the camera blocks (CameraPanel), the first-person head card (FirstPersonView) and the hand
/// cards (HandCamPip); a <see cref="Style"/> says how big the card's parts are.
///
///   (Rename)                     card, `Pad` mm around the content
///   +--------------------+
///   |   name (NameFont)  |       `NameGap` between the name and the view
///   | +----------------+ |
///   | |      view      | |       <see cref="SetSize"/> sets the view's size in mm
///   | +----------------+ |
///   |   topic (optional) |       <see cref="AddTopic"/>: camera blocks in setup mode
///   +--------------------+
///
/// The card is centred on the canvas; with <see cref="Style.PinView"/> the view is centred on it instead
/// (shifted by the offset given to SetSize) and the card grows around the view.
/// </summary>
public class CameraCard : MonoBehaviour
{
    /// <summary>Sizes in mm. <see cref="Block"/> is the camera blocks' look at a scale (1 = block distance).</summary>
    public struct Style
    {
        public float PadMm, RadiusMm;
        public float OutlineMm;                  // highlight ring margin around the card; 0 = no ring
        public float NameFontMm, NameGapMm;
        public float NameHeightMm;               // 0 = as high as the wrapped name needs
        public float NameRaiseMm;                // moves the name up inside its slot
        public bool NameFullWidth;               // name as wide as the card instead of the view
        public bool NameCompact;                 // muted, one line, shrinks to fit (hand cards)
        public string WaitingText;
        public float WaitingFontMm, WaitingInsetMm;
        public float RenameFontMm;
        public int SortingOrder;
        public bool PinView;

        public static Style Block(float scale = 1f) => new Style
        {
            PadMm = HudTheme.CardPadding * scale,
            RadiusMm = HudTheme.CardRadius * scale,
            OutlineMm = HudTheme.CardOutlineMargin * scale,
            NameFontMm = HudTheme.TitleFont * HudUi.MmPerMetre * scale,
            NameGapMm = HudTheme.LabelGap * HudUi.MmPerMetre * scale,
            WaitingText = "Waiting",
            WaitingFontMm = HudTheme.BodyFont * HudUi.MmPerMetre * scale,
            WaitingInsetMm = HudTheme.BodyFont * HudUi.MmPerMetre * scale,
            RenameFontMm = HudTheme.BodyFont * HudUi.MmPerMetre * scale,
            SortingOrder = HudTheme.SortingOrder,
        };
    }

    // Sizes of the whole card for a view size (mm); AboveViewCentre / BelowViewCentre reach to its top / bottom edge.
    public struct Metrics { public float Width, Height, AboveViewCentre, BelowViewCentre; }

    const float RenameWidthRatio = 4.6f, RenameHeightRatio = 1.6f;   // of the Rename font
    const float CompactNameMinRatio = 0.5f;
    const float LostFontRatio = 1.6f;   // "LAST FRAME" pill font = the badge's x this

    Style _style;
    Image _outline, _card;
    TextMeshProUGUI _name, _topic;
    Button _rename;
    Vector2 _viewMm, _shiftMm;
    Color _highlight = Color.clear, _linkTint = Color.clear;   // the outline shows the highlight, else the link tint
    Image _lostBg;
    TextMeshProUGUI _lostText;

    public RectTransform Rect => (RectTransform)transform;
    public RawImage View { get; private set; }
    public RectTransform ViewRect => View.rectTransform;
    public RectTransform CardRect => _card.rectTransform;
    public TextMeshProUGUI NameLabel => _name;
    public TextMeshProUGUI Waiting { get; private set; }
    public TextMeshProUGUI Topic => _topic;
    public CameraBadge Badge { get; private set; }
    public Button RenameButton => _rename;
    public RobotProfile.CameraConfig Config { get; set; }   // its flips apply to the texture
    public Vector2 ViewSize => _viewMm;
    public Metrics Size { get; private set; }
    public string Label => _name.text;
    public bool HasFrame { get; private set; }

    public event Action Renamed;

    // A new canvas under `parent`; the caller places it.
    public static CameraCard Create(Transform parent, string objName, Style style, RobotProfile.CameraConfig config, string label)
    {
        var go = new GameObject(objName, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        return Attach(go, style, config, label);
    }

    // The card on an existing GameObject (the camera block keeps its collider and interactable there).
    public static CameraCard Attach(GameObject go, Style style, RobotProfile.CameraConfig config, string label)
    {
        var canvas = go.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.sortingOrder = style.SortingOrder;
        canvas.worldCamera = Camera.main;
        go.AddComponent<TrackedDeviceGraphicRaycaster>();   // for the Rename button
        ((RectTransform)go.transform).localScale = Vector3.one / HudUi.MmPerMetre;
        var card = go.AddComponent<CameraCard>();
        card.Build(style, config, label);
        return card;
    }

    void Build(Style style, RobotProfile.CameraConfig config, string label)
    {
        _style = style;
        Config = config;

        // Drawn first, so they sit behind the labels and view: highlight ring around the card, then the card.
        if (style.OutlineMm > 0f)
            _outline = HudUi.Ring(HudUi.Box(transform, "Outline", Color.clear), style.OutlineMm / HudUi.RingThicknessRatio);
        _card = HudUi.Round(HudUi.Box(transform, "Card", HudTheme.Panel), style.RadiusMm);

        _name = HudUi.Label(transform, "Name", label, style.NameFontMm);
        if (style.NameCompact)
        {
            _name.color = HudTheme.Muted;
            _name.enableAutoSizing = true;   // renamed cameras can have long names
            _name.fontSizeMin = style.NameFontMm * CompactNameMinRatio;
            _name.fontSizeMax = style.NameFontMm;
            _name.textWrappingMode = TextWrappingModes.NoWrap;
        }

        View = new GameObject("View", typeof(RectTransform)).AddComponent<RawImage>();
        View.transform.SetParent(transform, false);
        View.color = HudTheme.Waiting;
        View.raycastTarget = false;

        Waiting = HudUi.Label(View.transform, "Waiting", style.WaitingText, style.WaitingFontMm);
        Waiting.color = HudTheme.Muted;
        HudUi.Stretch(Waiting.rectTransform, style.WaitingInsetMm);

        Badge = CameraBadge.Create(View.transform, Vector2.zero);

        _rename = HudUi.Button(transform, "Rename", style.RenameFontMm, () => Renamed?.Invoke());
        var rrt = (RectTransform)_rename.transform;
        rrt.anchorMin = rrt.anchorMax = rrt.pivot = new Vector2(0.5f, 0.5f);
        rrt.sizeDelta = new Vector2(style.RenameFontMm * RenameWidthRatio, style.RenameFontMm * RenameHeightRatio);
        _rename.gameObject.SetActive(false);
    }

    // The topic line under the view (camera blocks in setup mode). Topics have no spaces, so allow line breaks after each '/'.
    public void AddTopic(string topic, float fontMm)
    {
        _topic = HudUi.Label(transform, "Topic", topic.Replace("/", "/​"), fontMm);
        _topic.transform.SetSiblingIndex(View.transform.GetSiblingIndex());   // behind the view, like the name
    }

    // Card sizes if the view were `viewW` x `viewH` mm.
    public Metrics Measure(float viewW, float viewH)
    {
        float nameH = NameHeight(viewW), topicH = TopicHeight(viewW);
        float pad = _style.PadMm, gap = _style.NameGapMm;
        float below = viewH / 2f + (_topic != null ? gap + topicH : 0f) + pad;
        float above = pad + nameH + gap + viewH / 2f;
        return new Metrics { Width = viewW + 2f * pad, Height = above + below, AboveViewCentre = above, BelowViewCentre = below };
    }

    // Resize the view (mm); fonts and gaps stay fixed, labels re-wrap. `shift` moves a pinned view off the canvas centre.
    public void SetSize(float viewW, float viewH, Vector2 shift = default)
    {
        _viewMm = new Vector2(viewW, viewH);
        _shiftMm = shift;
        Layout();
    }

    void Layout()
    {
        float viewW = _viewMm.x, viewH = _viewMm.y;
        var m = Size = Measure(viewW, viewH);
        float pad = _style.PadMm, gap = _style.NameGapMm;
        float nameH = NameHeight(viewW), topicH = TopicHeight(viewW);

        // Offsets from the card's centre to its top, the name, the view and the topic.
        float top = m.Height / 2f;
        float viewCentre = top - pad - nameH - gap - viewH / 2f;
        var centre = _style.PinView ? _shiftMm - new Vector2(0f, viewCentre) : Vector2.zero;
        Rect.sizeDelta = _style.PinView ? _viewMm : new Vector2(m.Width, m.Height);

        Put(CardRect, centre, new Vector2(m.Width, m.Height));
        if (_outline != null) Put(_outline.rectTransform, centre, new Vector2(m.Width, m.Height) + Vector2.one * 2f * _style.OutlineMm);
        float nameW = _style.NameFullWidth ? m.Width : viewW;
        Put(_name.rectTransform, centre + new Vector2(0f, top - pad - nameH / 2f + _style.NameRaiseMm), new Vector2(nameW, nameH));
        Put(ViewRect, centre + new Vector2(0f, viewCentre), _viewMm);
        if (_topic != null) Put(_topic.rectTransform, centre + new Vector2(0f, viewCentre - viewH / 2f - gap - topicH / 2f), new Vector2(viewW, topicH));
        Badge.Fit(_viewMm);

        // Rename: above the card's top-right corner, right-aligned.
        var rrt = (RectTransform)_rename.transform;
        rrt.anchoredPosition = centre + new Vector2(m.Width / 2f - rrt.sizeDelta.x / 2f, top + rrt.sizeDelta.y / 2f + pad * 0.5f);
    }

    static void Put(RectTransform rt, Vector2 centre, Vector2 size)
    {
        rt.anchorMin = rt.anchorMax = rt.pivot = new Vector2(0.5f, 0.5f);
        rt.sizeDelta = size;
        rt.anchoredPosition = centre;
    }

    float NameHeight(float viewW)
        => _style.NameHeightMm > 0f ? _style.NameHeightMm : _name.GetPreferredValues(_name.text, viewW, Mathf.Infinity).y;

    float TopicHeight(float viewW)
        => _topic != null ? _topic.GetPreferredValues(_topic.text, viewW, Mathf.Infinity).y : 0f;

    // Name shown above the view (a longer one may wrap, making the card taller).
    public void SetLabel(string label)
    {
        _name.text = label;
        if (_viewMm.sqrMagnitude > 0f) Layout();
    }

    // Layout mode: the Rename button is available.
    public void SetEditable(bool editable) => _rename.gameObject.SetActive(editable);

    // Highlight ring colour (blocks); clear = none.
    public void SetHighlight(Color colour)
    {
        _highlight = colour;
        ApplyOutline();
    }

    void ApplyOutline()
    {
        if (_outline != null) _outline.color = _highlight.a > 0f ? _highlight : _linkTint;
    }

    public LinkHealth.Level Link { get; private set; }
    public bool LostLabelShown => _lostBg != null && _lostBg.gameObject.activeSelf;
    public string LostLabelText => _lostText != null ? _lostText.text : "";

    // Link health of this camera: the outline ring turns Warn (degraded) / Bad (lost) unless a highlight
    // is showing; with `lostLabel` a centred "LAST FRAME 3.2 s" pill appears over the view while lost.
    public void SetLink(LinkHealth.Level level, float ageS, bool lostLabel)
    {
        Link = level;
        _linkTint = level == LinkHealth.Level.Good ? Color.clear : HudTheme.WithAlpha(LinkHealth.Colour(level), HudTheme.LinkRingAlpha);
        ApplyOutline();
        if (!lostLabel) return;
        bool lost = level == LinkHealth.Level.Lost;
        if (_lostBg == null)
        {
            if (!lost) return;
            float font = Badge.FontMm * LostFontRatio;
            (_lostBg, _lostText) = HudUi.Pill(View.transform, "Lost", "", font, font * 1.7f, HudTheme.BadgeBackground, HudTheme.Bad, bold: true);
            var rt = _lostBg.rectTransform;
            rt.anchorMin = rt.anchorMax = rt.pivot = new Vector2(0.5f, 0.5f);
            rt.anchoredPosition = Vector2.zero;
        }
        _lostBg.gameObject.SetActive(lost);
        if (!lost) return;
        _lostText.text = ageS >= 0f ? $"LAST FRAME {ageS:F1} s" : "NO CONNECTION";
        var size = _lostText.GetPreferredValues(_lostText.text);
        _lostBg.rectTransform.sizeDelta = new Vector2(size.x + _lostText.fontSize * 1.2f, _lostText.fontSize * 1.7f);
    }

    // Show a camera frame with the camera's flips; the first one replaces the "Waiting for ..." text.
    public void SetTexture(Texture texture)
    {
        HasFrame = true;
        Waiting.gameObject.SetActive(false);
        View.texture = texture;
        View.color = Color.white;
        bool flipH = Config != null && Config.flipHorizontal, flipV = Config != null && Config.flipVertical;
        View.uvRect = new Rect(flipH ? 1f : 0f, flipV ? 1f : 0f, flipH ? -1f : 1f, flipV ? -1f : 1f);
    }

    // Dim the image while its frames have stopped (the next SetTexture undoes it).
    public void SetStale(bool stale)
    {
        if (stale) View.color = HudTheme.StaleTint;
    }
}
