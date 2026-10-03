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
///                       (setup mode only, where the names are auto-generated; see ImageSubscriber.InSetup)
/// </summary>
public class CameraPanel : MonoBehaviour
{
    // Shared look for all camera blocks (metres).
    public const float ViewHeight     = HudTheme.ViewHeight;
    public static float NameFontSize  => HudTheme.TitleFont;
    public static float TopicFontSize => HudTheme.BodyFont;
    public const float LabelGap       = HudTheme.LabelGap;

    const float MmPerMetre = HudUi.MmPerMetre;

    public enum Highlight { None, Hover, Drag }

    // Camera state shown on the view: "Waiting for ..." before the first frame, then the
    // displayed frame rate, or "stale" (and a dimmed image) when frames stop arriving.
    public const float StaleAfterSeconds = 1.0f;
    const float FpsWindowSeconds = 0.5f;

    public RobotProfile.CameraConfig Config { get; private set; }
    // Width / height of the view: the real frame size once a frame arrived (built-in robots do not
    // persist it), else the configured resolution.
    public float Aspect => _liveAspect > 0f ? _liveAspect : Config.Aspect;
    float _liveAspect;
    public void SetLiveAspect(float aspect) { _liveAspect = aspect; }
    public string Topic { get; private set; }

    // Total block size in metres for the current Size.
    public float Width  { get; private set; }
    public float Height { get; private set; }

    // Distance from the view's centre to the top / bottom edge of the block (metres).
    public float AboveViewCentre { get; private set; }
    public float BelowViewCentre { get; private set; }

    CameraCard _card;
    // The card's parts (kept as fields for the harness).
    RawImage _view;
    TextMeshProUGUI _name;
    CameraBadge _badge;
    Button _rename;
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

    // showTopic: the topic line below the view (setup mode only).
    public static CameraPanel Create(Transform parent, RobotProfile.CameraConfig config, string topic, bool showTopic = false)
    {
        var go = new GameObject("CameraPanel " + config.displayName, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        var panel = go.AddComponent<CameraPanel>();
        panel.Build(config, topic, showTopic);
        return panel;
    }

    void Build(RobotProfile.CameraConfig config, string topic, bool showTopic)
    {
        Config = config;
        Topic  = topic;

        // Waiting text: the topic without its namespace; topics have no spaces, so allow line breaks after each '/'.
        var style = CameraCard.Style.Block();
        style.WaitingText = $"Waiting for\n<color=#FFFFFF>{ShortTopic(topic).Replace("/", "/\u200B")}</color>";
        _card = CameraCard.Attach(gameObject, style, config, config.displayName);
        _card.Renamed += () => RenameRequested?.Invoke(this);
        _view = _card.View;
        _name = _card.NameLabel;
        _badge = _card.Badge;
        _rename = _card.RenameButton;
        if (showTopic) _card.AddTopic(topic, TopicFontSize * MmPerMetre);

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
        var m = _card.Measure(ViewMm(size).x, ViewMm(size).y);
        return new Metrics
        {
            Width = m.Width / MmPerMetre,
            AboveViewCentre = m.AboveViewCentre / MmPerMetre,
            BelowViewCentre = m.BelowViewCentre / MmPerMetre,
        };
    }

    // Resize the view to `size` x its profile scale. Fonts and gaps stay fixed; labels re-wrap.
    public void SetSize(float size)
    {
        Size = size;
        var view = ViewMm(size);
        _card.SetSize(view.x, view.y);
        var m = _card.Size;
        _collider.size = new Vector3(m.Width, m.Height, 2f);
        Width  = m.Width / MmPerMetre;
        Height = m.Height / MmPerMetre;
        AboveViewCentre = m.AboveViewCentre / MmPerMetre;
        BelowViewCentre = m.BelowViewCentre / MmPerMetre;
    }

    // View size in mm for a given size factor.
    Vector2 ViewMm(float size)
    {
        float viewH = ViewHeight * Config.scale * size * MmPerMetre;
        return new Vector2(viewH * Aspect, viewH);
    }

    // Topic without its namespace (absolute topics of added robots start with /<ns>/).
    static string ShortTopic(string topic)
    {
        if (!topic.StartsWith("/")) return topic;
        int slash = topic.IndexOf('/', 1);
        return slash > 0 ? topic.Substring(slash + 1) : topic.TrimStart('/');
    }

    public bool Visible
    {
        get => gameObject.activeSelf;
        set => gameObject.SetActive(value);
    }

    // Name shown above the view; may differ from Config.displayName (which keys saved layouts).
    public string Label => _card.Label;
    public event System.Action<CameraPanel> RenameRequested;

    public void SetLabel(string label)
    {
        _card.SetLabel(label);
        SetSize(Size);   // a longer name may wrap
    }

    // Layout mode: the Rename button is available.
    public void SetEditable(bool editable) => _card.SetEditable(editable);

    public void SetHighlight(Highlight state)
    {
        _card.SetHighlight(state switch
        {
            Highlight.Hover => HudTheme.WithAlpha(HudTheme.Accent, HudTheme.HoverRingAlpha),
            Highlight.Drag  => HudTheme.WithAlpha(HudTheme.Accent, HudTheme.DragRingAlpha),
            _               => Color.clear,
        });
    }

    public void SetTexture(Texture texture)
    {
        float now = Time.unscaledTime;
        if (_lastFrameTime < 0f)
        {
            _badge.Show(true);
            _windowStart = now;
        }
        _lastFrameTime = now;
        _framesInWindow++;

        _card.SetTexture(texture);
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
        _nextBadgeRefresh = now + CameraBadge.RefreshSeconds;

        float age = now - _lastFrameTime;
        _card.SetLink(LinkHealth.Evaluate(age, -1f, LinkHealth.ConnectionError), age, false);
        if (age > StaleAfterSeconds)
        {
            State = FeedState.Stale;
            _card.SetStale(true);
            _badge.Set($"stale {age:F1} s", HudTheme.Warn);
        }
        else
        {
            State = FeedState.Live;
            _badge.Set($"{Mathf.RoundToInt(_fps)} fps", HudTheme.Good);
        }
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

/// <summary>
/// The small "15 fps" / "stale 1.2 s" badge in the top-right corner of a camera view: shared by the
/// camera blocks (CameraPanel, which measures the rate itself) and the first-person views
/// (FirstPersonView head card, HandCamPip cards), which feed it from ImageSubscriber via
/// <see cref="Tick"/>. `fontMm` is the body font size in canvas units of the card it sits on.
/// </summary>
public class CameraBadge
{
    public const float RefreshSeconds = 0.2f;   // ~5 Hz

    readonly Image _bg;
    readonly TextMeshProUGUI _text;
    float _next;
    bool _stale;

    public GameObject gameObject => _bg.gameObject;
    public string Text => _text.text;
    public bool Visible => _bg.gameObject.activeSelf;
    public bool Stale => _stale;

    // Size relative to the view it decorates: font height = 7 % of the view's height, clamped to
    // [MinFontMm, MaxFontMm]; corner radius = font; margin from the top-right corner = 3 % of the view's width.
    public const float FontRatio = HudTheme.BadgeFontRatio, MarginRatio = HudTheme.BadgeMarginRatio,
                       MinFontMm = HudTheme.BadgeMinFontMm, MaxFontMm = HudTheme.BadgeMaxFontMm;

    CameraBadge(Image bg, TextMeshProUGUI text) { _bg = bg; _text = text; }

    public RectTransform Rect => _bg.rectTransform;
    public float FontMm { get; private set; }

    // Hidden until the first frame (Show / Tick turn it on). `viewMm` is the view's size in canvas units.
    public static CameraBadge Create(Transform view, Vector2 viewMm)
    {
        var bg = HudUi.Round(HudUi.Box(view, "Badge", HudTheme.BadgeBackground), 10f);
        var rt = bg.rectTransform;
        rt.anchorMin = rt.anchorMax = rt.pivot = new Vector2(1f, 1f);
        var text = HudUi.Label(bg.transform, "Label", "", 10f);
        text.textWrappingMode = TextWrappingModes.NoWrap;
        HudUi.Stretch(text.rectTransform);
        bg.gameObject.SetActive(false);
        var badge = new CameraBadge(bg, text);
        badge.Fit(viewMm);
        return badge;
    }

    // Re-apply the size for a view of `viewMm` (call whenever the view is resized).
    public void Fit(Vector2 viewMm)
    {
        float font = Mathf.Clamp(FontRatio * viewMm.y, MinFontMm, MaxFontMm);
        FontMm = font;
        float margin = MarginRatio * viewMm.x;
        _bg.rectTransform.anchoredPosition = new Vector2(-margin, -margin);
        _text.fontSize = font * 0.85f;
        HudUi.Round(_bg, font);
        Resize();
    }

    void Resize()
    {
        if (string.IsNullOrEmpty(_text.text)) { _bg.rectTransform.sizeDelta = new Vector2(_text.fontSize * 3f, _text.fontSize * 1.4f); return; }
        var size = _text.GetPreferredValues(_text.text);
        _bg.rectTransform.sizeDelta = new Vector2(size.x + _text.fontSize, size.y * 1.15f);
    }

    public void Show(bool on) => _bg.gameObject.SetActive(on);

    public void Set(string text, Color color)
    {
        if (_text.text == text) return;
        _text.text = text;
        _text.color = color;
        Resize();
    }

    // Poll ImageSubscriber's rate and last-frame time (about 5 Hz). No frame yet: hidden.
    // Returns true while the feed is stale.
    public bool Tick(ImageSubscriber images, int index)
    {
        if (images == null || Time.unscaledTime < _next) return _stale;
        _next = Time.unscaledTime + RefreshSeconds;
        double last = images.LastFrameTime(index);
        if (last < 0.0) { Show(false); return _stale = false; }
        Show(true);
        float age = (float)(Time.unscaledTime - last);
        _stale = age > CameraPanel.StaleAfterSeconds;
        if (_stale) Set($"stale {age:F1} s", HudTheme.Warn);
        else Set($"{Mathf.RoundToInt(images.Fps(index))} fps", HudTheme.Good);
        return _stale;
    }
}
