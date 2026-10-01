using TMPro;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

/// <summary>
/// One camera block: camera name above the view, the camera view, topic name below.
/// Built in code so every block, for every robot, shares the same font sizes and spacing.
///
/// Layout (world-space canvas, 1 canvas unit = 1 mm):
///
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

    public enum Highlight { None, Hover, Drag }

    public RobotProfile.CameraConfig Config { get; private set; }
    public string Topic { get; private set; }

    // Total block size in metres, available after Create().
    public float Width  { get; private set; }
    public float Height { get; private set; }

    // Distance from the view's centre to the top / bottom edge of the block (metres).
    public float AboveViewCentre { get; private set; }
    public float BelowViewCentre { get; private set; }

    RawImage _view;
    Image _outline;

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

        // Drawn first, so it sits behind the labels and view.
        _outline = HudUi.Round(HudUi.Box(transform, "Outline", Color.clear), 2f * OutlineMarginMm);

        float viewH = ViewHeight * config.scale * MmPerMetre;
        float viewW = viewH * config.Aspect;
        float gap   = LabelGap * MmPerMetre;

        // Labels: same width as the view; text wraps instead of widening the block.
        // Topics have no spaces, so allow line breaks after each '/'.
        var name  = CreateLabel("Name",  config.displayName,          NameFontSize,  viewW);
        var topicLabel = CreateLabel("Topic", topic.Replace("/", "/​"), TopicFontSize, viewW);
        float nameH  = name.GetPreferredValues(name.text, viewW, Mathf.Infinity).y;
        float topicH = topicLabel.GetPreferredValues(topicLabel.text, viewW, Mathf.Infinity).y;

        float totalH = nameH + gap + viewH + gap + topicH;
        var root = (RectTransform)transform;
        root.sizeDelta  = new Vector2(viewW, totalH);
        root.localScale = Vector3.one / MmPerMetre;

        // Stack from the top of the block downwards.
        float top = totalH / 2f;
        Place(name.rectTransform, top, nameH, viewW);
        top -= nameH + gap;

        _view = new GameObject("View", typeof(RectTransform)).AddComponent<RawImage>();
        _view.transform.SetParent(transform, false);
        _view.color = WaitingColor;
        _view.raycastTarget = false;
        Place(_view.rectTransform, top, viewH, viewW);
        top -= viewH + gap;

        Place(topicLabel.rectTransform, top, topicH, viewW);

        HudUi.Stretch(_outline.rectTransform, -OutlineMarginMm);

        // Collider over the whole block (canvas units are mm), then the interactable that uses it.
        var collider = gameObject.AddComponent<BoxCollider>();
        collider.size = new Vector3(viewW, totalH, 20f);
        Interactable = gameObject.AddComponent<XRSimpleInteractable>();
        Interactable.enabled = false;

        Width  = viewW  / MmPerMetre;
        Height = totalH / MmPerMetre;
        AboveViewCentre = (nameH + gap + viewH / 2f) / MmPerMetre;
        BelowViewCentre = (viewH / 2f + gap + topicH) / MmPerMetre;
    }

    TextMeshProUGUI CreateLabel(string objName, string text, float fontSizeMetres, float width)
    {
        var label = HudUi.Label(transform, objName, text, fontSizeMetres * MmPerMetre);
        label.rectTransform.sizeDelta = new Vector2(width, 0f);
        return label;
    }

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
        _view.texture = texture;
        _view.color = Color.white;
        _view.uvRect = new Rect(
            Config.flipHorizontal ? 1f : 0f, Config.flipVertical ? 1f : 0f,
            Config.flipHorizontal ? -1f : 1f, Config.flipVertical ? -1f : 1f);
    }
}
