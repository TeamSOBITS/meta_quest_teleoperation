using System;
using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// <summary>
/// Small toolkit for the code-built, head-locked HUD (camera blocks, ROS IP block, control panel).
/// Canvases use 1 unit = 1 mm. Font sizes are given for <see cref="ReferenceDistance"/> and scaled
/// with each canvas's distance, so text looks the same size whether a panel is near or far.
/// </summary>
public static class HudUi
{
    // Distance the camera blocks sit at; sizes below are tuned for it (metres).
    public const float ReferenceDistance = 4.3f;
    public const float TitleFontSize = 0.14f;   // camera names, ROS IP
    public const float BodyFontSize  = 0.09f;   // topics, control panel entries

    public const float MmPerMetre = 1000f;

    public static readonly Color PanelColor   = new Color(0.08f, 0.08f, 0.10f, 0.85f);
    public static readonly Color ControlColor = new Color(0.25f, 0.27f, 0.32f, 1f);
    public static readonly Color AccentColor  = new Color(0.30f, 0.65f, 1f, 1f);

    // Canvas sized width x height mm, placed at localPosition under parent and facing the parent's origin.
    // Interactive canvases get a raycaster so XR controller rays can press their controls.
    public static RectTransform CreateCanvas(string name, Transform parent, Vector3 localPosition,
                                             Vector2 sizeMm, bool interactive)
    {
        var go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        var canvas = go.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
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
        => sizeAtReference * distance / ReferenceDistance * MmPerMetre;

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

    public static Button Button(Transform parent, string text, float fontSize, UnityAction onClick)
    {
        var bg = Box(parent, "Button " + text, ControlColor, raycastTarget: true);
        var button = bg.gameObject.AddComponent<Button>();
        button.targetGraphic = bg;
        button.onClick.AddListener(onClick);
        var label = Label(bg.transform, "Label", text, fontSize);
        Stretch(label.rectTransform);
        return button;
    }

    // Row with a check box on the left and the label to its right; the whole row is clickable.
    public static Toggle Toggle(Transform parent, string text, float fontSize, bool isOn, Action<bool> onChanged)
    {
        var row = Box(parent, "Toggle " + text, new Color(0f, 0f, 0f, 0f), raycastTarget: true);
        var toggle = row.gameObject.AddComponent<Toggle>();

        float boxSize = fontSize * 1.1f;
        var box = Box(row.transform, "Box", ControlColor);
        var boxRt = box.rectTransform;
        boxRt.anchorMin = boxRt.anchorMax = new Vector2(0f, 0.5f);
        boxRt.pivot = new Vector2(0f, 0.5f);
        boxRt.sizeDelta = new Vector2(boxSize, boxSize);
        boxRt.anchoredPosition = Vector2.zero;

        var check = Box(box.transform, "Check", AccentColor);
        Stretch(check.rectTransform, boxSize * 0.2f);

        var label = Label(row.transform, "Label", text, fontSize, TextAlignmentOptions.Left);
        Stretch(label.rectTransform);
        label.rectTransform.offsetMin = new Vector2(boxSize * 1.5f, 0f);

        toggle.targetGraphic = box;
        toggle.graphic = check;
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
