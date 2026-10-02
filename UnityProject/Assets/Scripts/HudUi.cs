using System;
using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// <summary>
/// Small toolkit for the code-built, head-locked HUD (camera blocks, ROS IP block, control panel).
/// Canvases use 1 unit = 1 mm. Font sizes are given for <see cref="HudTheme.ReferenceDistance"/> and scaled
/// with each canvas's distance, so text looks the same size whether a panel is near or far. Sizes and
/// colours live in <see cref="HudTheme"/>.
/// </summary>
public static class HudUi
{
    public const float MmPerMetre = 1000f;

    // Rounded-rectangle sprite for sliced Images; corner radius is set per Image via Round().
    const int RoundedSpriteSize = 64, RoundedSpriteBorder = 16;
    // Ring (outline) thickness as a fraction of its corner radius.
    public const float RingThicknessRatio = 0.25f;
    static Sprite _roundedSprite, _ringSprite;

    // Canvas sized width x height mm, placed at localPosition under parent and facing the parent's origin.
    // Interactive canvases get a raycaster so XR controller rays can press their controls.
    public static RectTransform CreateCanvas(string name, Transform parent, Vector3 localPosition,
                                             Vector2 sizeMm, bool interactive)
    {
        var go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        var canvas = go.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.sortingOrder = HudTheme.SortingOrder;
        if (interactive)
        {
            canvas.worldCamera = Camera.main;
            go.AddComponent<TrackedDeviceGraphicRaycaster>();
        }

        var rt = (RectTransform)go.transform;
        rt.sizeDelta = sizeMm;
        rt.localScale = Vector3.one / MmPerMetre;
        rt.localPosition = localPosition;
        rt.localRotation = Quaternion.LookRotation(localPosition);
        return rt;
    }

    // Font size in canvas units for text that should look like `sizeAtReference` seen from ReferenceDistance.
    public static float FontAt(float sizeAtReference, float distance)
        => sizeAtReference * distance / HudTheme.ReferenceDistance * MmPerMetre;

    public static TextMeshProUGUI Label(Transform parent, string name, string text, float fontSize,
                                        TextAlignmentOptions alignment = TextAlignmentOptions.Center)
    {
        var label = new GameObject(name, typeof(RectTransform)).AddComponent<TextMeshProUGUI>();
        label.transform.SetParent(parent, false);
        label.text = text;
        label.fontSize = fontSize;
        label.alignment = alignment;
        label.textWrappingMode = TextWrappingModes.Normal;
        label.overflowMode = TextOverflowModes.Overflow;
        label.raycastTarget = false;
        return label;
    }

    public static Image Box(Transform parent, string name, Color color, bool raycastTarget = false)
    {
        var image = new GameObject(name, typeof(RectTransform)).AddComponent<Image>();
        image.transform.SetParent(parent, false);
        image.color = color;
        image.raycastTarget = raycastTarget;
        return image;
    }

    // Give an Image rounded corners of `radius` canvas units (mm).
    public static Image Round(Image image, float radius)
    {
        image.sprite = RoundedSprite();
        image.type = Image.Type.Sliced;
        // Corner size in canvas units = sprite border px / multiplier (sprite and canvas both 100 px per unit).
        image.pixelsPerUnitMultiplier = RoundedSpriteBorder / Mathf.Max(radius, 0.01f);
        return image;
    }

    // Rounded outline only (transparent inside), `radius` mm corners, radius x RingThicknessRatio thick.
    // Used for highlight outlines so they don't tint the see-through card behind them.
    public static Image Ring(Image image, float radius)
    {
        image.sprite = RingSprite();
        image.type = Image.Type.Sliced;
        image.fillCenter = false;
        image.pixelsPerUnitMultiplier = RoundedSpriteBorder / Mathf.Max(radius, 0.01f);
        return image;
    }

    // Unity's == (not ??=): runtime-made sprites can be destroyed (e.g. leaving Play mode or
    // unloading unused assets) while the static field still holds the dead reference.
    static Sprite RoundedSprite()
    {
        if (_roundedSprite == null) _roundedSprite = MakeRoundedSprite(0f);
        return _roundedSprite;
    }

    static Sprite RingSprite()
    {
        if (_ringSprite == null) _ringSprite = MakeRoundedSprite(RoundedSpriteBorder * RingThicknessRatio);
        return _ringSprite;
    }

    // Rounded rectangle; with thickness > 0 only a ring of that many pixels is opaque.
    static Sprite MakeRoundedSprite(float thickness)
    {
        int n = RoundedSpriteSize; float r = RoundedSpriteBorder;
        var tex = new Texture2D(n, n, TextureFormat.RGBA32, false) { wrapMode = TextureWrapMode.Clamp };
        var pixels = new Color32[n * n];
        for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++)
        {
            // Distance outside the rounded rectangle, anti-aliased over one pixel.
            float dx = Mathf.Max(r - (x + 0.5f), (x + 0.5f) - (n - r), 0f);
            float dy = Mathf.Max(r - (y + 0.5f), (y + 0.5f) - (n - r), 0f);
            float d = Mathf.Sqrt(dx * dx + dy * dy);       // 0 inside the straight part
            float a = Mathf.Clamp01(r - d + 0.5f);
            if (thickness > 0f)
            {
                // Distance to the nearest edge, inside the shape: keep only the outer band.
                float edge = Mathf.Min(Mathf.Min(x + 0.5f, n - x - 0.5f), Mathf.Min(y + 0.5f, n - y - 0.5f));
                float inner = d > 0f ? r - d : edge;
                a *= Mathf.Clamp01(thickness - inner + 0.5f);
            }
            pixels[y * n + x] = new Color32(255, 255, 255, (byte)(a * 255));
        }
        tex.SetPixels32(pixels);
        tex.Apply(false, true);
        return Sprite.Create(tex, new Rect(0, 0, n, n), new Vector2(0.5f, 0.5f), 100f, 0,
                             SpriteMeshType.FullRect, new Vector4(r, r, r, r));
    }

    public static Button Button(Transform parent, string text, float fontSize, UnityAction onClick)
    {
        var bg = Box(parent, "Button " + text, HudTheme.Control, raycastTarget: true);
        Round(bg, fontSize * 0.35f);
        var button = bg.gameObject.AddComponent<Button>();
        button.targetGraphic = bg;
        button.colors = HudTheme.Hover;
        if (onClick != null) button.onClick.AddListener(onClick);
        var label = Label(bg.transform, "Label", text, fontSize);
        Stretch(label.rectTransform);
        return button;
    }

    // Rounded chip with a centred, non-wrapping label: `height` mm tall (fully rounded), `fontSize` mm text.
    // Size and position it with Place; set its look later with SetPill.
    public static (Image background, TextMeshProUGUI label) Pill(Transform parent, string name, string text, float fontSize,
                                                                 float height, Color background, Color textColor, bool bold = false)
    {
        var bg = Round(Box(parent, name, background), height / 2f);
        var label = Label(bg.transform, "Label", text, fontSize);
        label.color = textColor;
        if (bold) label.fontStyle = FontStyles.Bold;
        label.textWrappingMode = TextWrappingModes.NoWrap;
        Stretch(label.rectTransform);
        return (bg, label);
    }

    // Chip tinted with `tint`: text in the colour, background a faint version of it (HudTheme.PillBackground).
    public static (Image background, TextMeshProUGUI label) Pill(Transform parent, string name, string text, float fontSize,
                                                                 float height, Color tint, bool strong = false, bool bold = false)
        => Pill(parent, name, text, fontSize, height, HudTheme.PillBackground(tint, strong), tint, bold);

    // Row with a check box on the left and the label to its right; the whole row is clickable.
    public static Toggle Toggle(Transform parent, string text, float fontSize, bool isOn, Action<bool> onChanged)
    {
        var row = Box(parent, "Toggle " + text, Color.clear, raycastTarget: true);
        var toggle = row.gameObject.AddComponent<HudToggle>();

        float boxSize = fontSize * 1.1f;
        var box = Round(Box(row.transform, "Box", HudTheme.Control), boxSize * 0.2f);
        var boxRt = box.rectTransform;
        boxRt.anchorMin = boxRt.anchorMax = new Vector2(0f, 0.5f);
        boxRt.pivot = new Vector2(0f, 0.5f);
        boxRt.sizeDelta = new Vector2(boxSize, boxSize);
        boxRt.anchoredPosition = Vector2.zero;

        var check = Round(Box(box.transform, "Check", HudTheme.Accent), boxSize * 0.12f);
        Stretch(check.rectTransform, boxSize * 0.2f);

        var label = Label(row.transform, "Label", text, fontSize, TextAlignmentOptions.Left);
        Stretch(label.rectTransform);
        label.rectTransform.offsetMin = new Vector2(boxSize * 1.5f, 0f);

        toggle.targetGraphic = box;
        toggle.graphic = check;
        toggle.colors = HudTheme.Hover;
        // Instant on/off: a fading check mark would be cut short by HudToggle's hover tint fade.
        toggle.toggleTransition = UnityEngine.UI.Toggle.ToggleTransition.None;
        toggle.isOn = isOn;
        toggle.onValueChanged.AddListener(v => onChanged(v));
        return toggle;
    }

    // Fill the parent rect, optionally inset by `inset` on every side.
    public static void Stretch(RectTransform rt, float inset = 0f)
    {
        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.offsetMin = new Vector2(inset, inset);
        rt.offsetMax = new Vector2(-inset, -inset);
    }

    // Place a child by its top-left corner, in mm from the parent's top-left corner.
    public static void Place(RectTransform rt, float left, float top, float width, float height)
    {
        rt.anchorMin = rt.anchorMax = new Vector2(0f, 1f);
        rt.pivot = new Vector2(0f, 1f);
        rt.sizeDelta = new Vector2(width, height);
        rt.anchoredPosition = new Vector2(left, -top);
    }

    // Place a child as a full-width row whose top edge is `top` mm below the parent's top edge.
    public static void Row(RectTransform rt, float top, float height, float sidePadding)
    {
        rt.anchorMin = new Vector2(0f, 1f);
        rt.anchorMax = new Vector2(1f, 1f);
        rt.pivot = new Vector2(0.5f, 1f);
        rt.offsetMin = new Vector2(sidePadding, 0f);
        rt.offsetMax = new Vector2(-sidePadding, 0f);
        rt.sizeDelta = new Vector2(rt.sizeDelta.x, height);
        rt.anchoredPosition = new Vector2(0f, -top);
    }
}
