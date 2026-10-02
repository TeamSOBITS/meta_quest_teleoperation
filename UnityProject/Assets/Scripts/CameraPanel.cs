using TMPro;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// <summary>
/// One camera block: camera name above the view, the camera view, topic name below.
/// Built in code so every block, for every robot, shares the same font sizes and spacing.
///
/// Layout (world-space canvas, 1 canvas unit = 1 mm):
///
///   (all on a dark rounded card)
///      Camera Name        name label, fixed font, wraps within the view width
///      [gap]
///   +--------------+
///   |  camera view |      height = standard height x camera scale, width = height x aspect ratio
///   +--------------+
///      [gap]
///   /ns/topic/name      topic label, smaller fixed font, wraps within the view width
/// </summary>
public class CameraPanel : MonoBehaviour
{
    // Shared look for all camera blocks (metres).
    public const float ViewHeight     = 1.2f;
    public const float NameFontSize   = HudUi.TitleFontSize;
    public const float TopicFontSize  = HudUi.BodyFontSize;
    public const float LabelGap       = 0.05f;

    const float MmPerMetre = HudUi.MmPerMetre;
    static readonly Color WaitingColor = new Color(0.15f, 0.15f, 0.15f, 1f);
    const float OutlineMarginMm = 30f;
    // Dark card behind the whole block so labels read on any background (same style as the HUD bar).
    const float CardPaddingMm = 40f, CardRadiusMm = 60f;

    public enum Highlight { None, Hover, Drag }

    // Camera state shown on the view: "Waiting for ..." before the first frame, then the
    // displayed frame rate, or "stale" (and a dimmed image) when frames stop arriving.
    public const float StaleAfterSeconds = 1.0f;
    const float FpsWindowSeconds = 0.5f, BadgeRefreshSeconds = 0.2f;
    static readonly Color StaleTint = new Color(0.45f, 0.45f, 0.45f, 1f);

    public RobotProfile.CameraConfig Config { get; private set; }
    public string Topic { get; private set; }

    // Total block size in metres for the current Size.
    public float Width  { get; private set; }
    public float Height { get; private set; }

    // Distance from the view's centre to the top / bottom edge of the block (metres).
    public float AboveViewCentre { get; private set; }
    public float BelowViewCentre { get; private set; }

    RawImage _view;
    Image _outline, _card;
    TextMeshProUGUI _name, _topic, _waiting, _badgeText;
    Image _badge;
    float _lastFrameTime = -1f, _windowStart, _fps, _nextBadgeRefresh;
    int _framesInWindow;

    public enum FeedState { Waiting, Live, Stale }
    public FeedState State { get; private set; } = FeedState.Waiting;
    BoxCollider _collider;
    const float ColliderOffsetMm = 5f;   // behind the canvas (its front faces -Z, towards the eye)

    // Current view size factor (1 = profile size); set by the auto layout.
    public float Size { get; private set; } = 1f;

    // Ray target for dragging; enabled only while dragging is allowed (see PanelDragger).
    public XRSimpleInteractable Interactable { get; private set; }

    public static CameraPanel Create(Transform parent, RobotProfile.CameraConfig config, string topic)
    {
        var go = new GameObject("CameraPanel " + config.displayName, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        var panel = go.AddComponent<CameraPanel>();
        panel.Build(config, topic);
        return panel;
    }

    void Build(RobotProfile.CameraConfig config, string topic)
    {
        Config = config;
        Topic  = topic;

        var canvas = gameObject.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.sortingOrder = HudUi.CanvasSortingOrder;
        canvas.worldCamera = Camera.main;
        gameObject.AddComponent<TrackedDeviceGraphicRaycaster>();   // for the Rename button
        ((RectTransform)transform).localScale = Vector3.one / MmPerMetre;

        // Drawn first, so they sit behind the labels and view: highlight ring around the card, then the card.
        _outline = HudUi.Ring(HudUi.Box(transform, "Outline", Color.clear), OutlineMarginMm / HudUi.RingThicknessRatio);
        HudUi.Stretch(_outline.rectTransform, -OutlineMarginMm);
        _card = HudUi.Round(HudUi.Box(transform, "Card", HudUi.PanelColor), CardRadiusMm);
        HudUi.Stretch(_card.rectTransform);

        // Labels: same width as the view; text wraps instead of widening the block.
        // Topics have no spaces, so allow line breaks after each '/'.
        _name  = CreateLabel("Name",  config.displayName,          NameFontSize);
        _topic = CreateLabel("Topic", topic.Replace("/", "/\u200B"), TopicFontSize);

        _view = new GameObject("View", typeof(RectTransform)).AddComponent<RawImage>();
        _view.transform.SetParent(transform, false);
        _view.color = WaitingColor;
        _view.raycastTarget = false;

        float body = TopicFontSize * MmPerMetre;
        _waiting = HudUi.Label(_view.transform, "Waiting",
            $"Waiting for\n<color=#FFFFFF>{ShortTopic(config.topicSuffix).Replace("/", "/\u200B")}</color>", body);
        _waiting.color = HudUi.MutedText;
        HudUi.Stretch(_waiting.rectTransform, body);

        _badge = HudUi.Round(HudUi.Box(_view.transform, "Badge", new Color(0f, 0f, 0f, 0.55f)), body);
        var brt = _badge.rectTransform;
        brt.anchorMin = brt.anchorMax = brt.pivot = new Vector2(1f, 1f);
        brt.anchoredPosition = new Vector2(-0.5f * body, -0.5f * body);
        _badgeText = HudUi.Label(_badge.transform, "Label", "", body * 0.85f);
        _badgeText.textWrappingMode = TextWrappingModes.NoWrap;
        HudUi.Stretch(_badgeText.rectTransform);
        _badge.gameObject.SetActive(false);

        // Rename: shown in layout mode only, above the card's top-right corner.
        _rename = HudUi.Button(transform, "Rename", TopicFontSize * MmPerMetre, () => RenameRequested?.Invoke(this));
        var rrt = (RectTransform)_rename.transform;
        rrt.anchorMin = rrt.anchorMax = rrt.pivot = new Vector2(1f, 1f);
        rrt.sizeDelta = new Vector2(TopicFontSize * MmPerMetre * 4.6f, TopicFontSize * MmPerMetre * 1.6f);
        rrt.anchoredPosition = new Vector2(0f, rrt.sizeDelta.y + CardPaddingMm * 0.5f);   // above the card's top edge, right-aligned
        _rename.gameObject.SetActive(false);

        // Collider over the whole block (canvas units are mm), just behind the canvas so a ray on
        // the Rename button hits the button first; then the interactable that uses it.
        _collider = gameObject.AddComponent<BoxCollider>();
        _collider.center = new Vector3(0f, 0f, ColliderOffsetMm);
        Interactable = gameObject.AddComponent<HoverOnlyInteractable>();
        Interactable.enabled = false;

        SetSize(1f);
    }

    // Block dimensions (metres) if the view were drawn at `size` x its profile scale.
    public struct Metrics { public float Width, AboveViewCentre, BelowViewCentre; }

    public Metrics Measure(float size)
    {
        Sizes(size, out float viewW, out float viewH, out float nameH, out float topicH);
        float gap = LabelGap * MmPerMetre, pad = CardPaddingMm;
        return new Metrics
        {
            Width = (viewW + 2f * pad) / MmPerMetre,
            AboveViewCentre = (pad + nameH + gap + viewH / 2f) / MmPerMetre,
            BelowViewCentre = (viewH / 2f + gap + topicH + pad) / MmPerMetre,
        };
    }

    // Resize the view to `size` x its profile scale. Fonts and gaps stay fixed; labels re-wrap.
    public void SetSize(float size)
    {
        Size = size;
        Sizes(size, out float viewW, out float viewH, out float nameH, out float topicH);
        float gap = LabelGap * MmPerMetre, pad = CardPaddingMm;
        float totalH = pad + nameH + gap + viewH + gap + topicH + pad;
        float totalW = viewW + 2f * pad;
        ((RectTransform)transform).sizeDelta = new Vector2(totalW, totalH);

        // Stack from the top of the card downwards.
        float top = totalH / 2f - pad;
        Place(_name.rectTransform, top, nameH, viewW);
        top -= nameH + gap;
        Place(_view.rectTransform, top, viewH, viewW);
        top -= viewH + gap;
        Place(_topic.rectTransform, top, topicH, viewW);

        _collider.size = new Vector3(totalW, totalH, 2f);

        var m = Measure(size);
        Width  = m.Width;
        Height = totalH / MmPerMetre;
        AboveViewCentre = m.AboveViewCentre;
        BelowViewCentre = m.BelowViewCentre;
    }

    // Sizes in mm for a given size factor.
    void Sizes(float size, out float viewW, out float viewH, out float nameH, out float topicH)
    {
        viewH = ViewHeight * Config.scale * size * MmPerMetre;
        viewW = viewH * Config.Aspect;
        nameH  = _name.GetPreferredValues(_name.text, viewW, Mathf.Infinity).y;
        topicH = _topic.GetPreferredValues(_topic.text, viewW, Mathf.Infinity).y;
    }

    // Topic without its namespace (absolute topics of added robots start with /<ns>/).
    static string ShortTopic(string topic)
    {
        if (!topic.StartsWith("/")) return topic;
        int slash = topic.IndexOf('/', 1);
        return slash > 0 ? topic.Substring(slash + 1) : topic.TrimStart('/');
    }

    TextMeshProUGUI CreateLabel(string objName, string text, float fontSizeMetres)
        => HudUi.Label(transform, objName, text, fontSizeMetres * MmPerMetre);

    // Anchor a child to the block's centre and place its top edge at y = top.
    static void Place(RectTransform rt, float top, float height, float width)
    {
        rt.anchorMin = rt.anchorMax = new Vector2(0.5f, 0.5f);
        rt.pivot = new Vector2(0.5f, 1f);
        rt.sizeDelta = new Vector2(width, height);
        rt.anchoredPosition = new Vector2(0f, top);
    }

    public bool Visible
    {
        get => gameObject.activeSelf;
        set => gameObject.SetActive(value);
    }

    // Name shown above the view; may differ from Config.displayName (which keys saved layouts).
    public string Label => _name.text;
    public event System.Action<CameraPanel> RenameRequested;
    Button _rename;

    public void SetLabel(string label)
    {
        _name.text = label;
        SetSize(Size);   // a longer name may wrap
    }

    // Layout mode: the Rename button is available.
    public void SetEditable(bool editable) => _rename.gameObject.SetActive(editable);

    public void SetHighlight(Highlight state)
    {
        var c = HudUi.AccentColor;
        _outline.color = state switch
        {
            Highlight.Hover => new Color(c.r, c.g, c.b, 0.35f),
            Highlight.Drag  => new Color(c.r, c.g, c.b, 0.7f),
            _               => Color.clear,
        };
    }

    public void SetTexture(Texture texture)
    {
        float now = Time.unscaledTime;
        if (_lastFrameTime < 0f)
        {
            _waiting.gameObject.SetActive(false);
            _badge.gameObject.SetActive(true);
            _windowStart = now;
        }
        _lastFrameTime = now;
        _framesInWindow++;

        _view.texture = texture;
        _view.color = Color.white;
        _view.uvRect = new Rect(
            Config.flipHorizontal ? 1f : 0f, Config.flipVertical ? 1f : 0f,
            Config.flipHorizontal ? -1f : 1f, Config.flipVertical ? -1f : 1f);
    }

    void Update()
    {
        if (_lastFrameTime < 0f) return;  // still waiting for the first frame

        float now = Time.unscaledTime;
        if (now - _windowStart >= FpsWindowSeconds)
        {
            _fps = _framesInWindow / (now - _windowStart);
            _framesInWindow = 0;
            _windowStart = now;
        }
        if (now < _nextBadgeRefresh) return;
        _nextBadgeRefresh = now + BadgeRefreshSeconds;

        float age = now - _lastFrameTime;
        if (age > StaleAfterSeconds)
        {
            State = FeedState.Stale;
            _view.color = StaleTint;
            SetBadge($"stale {age:F1} s", HudUi.WarnColor);
        }
        else
        {
            State = FeedState.Live;
            SetBadge($"{Mathf.RoundToInt(_fps)} fps", HudUi.GoodColor);
        }
    }

    void SetBadge(string text, Color color)
    {
        if (_badgeText.text == text) return;
        _badgeText.text = text;
        _badgeText.color = color;
        var size = _badgeText.GetPreferredValues(text);
        _badge.rectTransform.sizeDelta = new Vector2(size.x + _badgeText.fontSize, size.y * 1.15f);
    }
}

/// <summary>
/// A block is only hovered, never selected: PanelDragger reads the trigger/pinch itself. With
/// tracked hands a pinch is XRI's Select, and selecting the block would snap the hand's ray to
/// the block's centre and take the pinch away from the Rename button.
/// </summary>
public class HoverOnlyInteractable : XRSimpleInteractable
{
    public override bool IsSelectableBy(UnityEngine.XR.Interaction.Toolkit.Interactors.IXRSelectInteractor interactor) => false;
}
