using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// <summary>
/// Hand cameras in first person: one small card per hand camera of the robot (a camera whose topic
/// suffix contains "hand"; RawImage on a rounded PanelColor box, a muted label with the camera's name).
/// A card floats outboard of and a little below its gripper (the end effector link of the profile's
/// arm: the one named "left" / "right" in the camera's topic, else the only arm), to the side of the line of sight
/// so it never covers what the head camera shows there, and faces the head. Created by TeleopHud in
/// the first-person camera layout, destroyed when the layout or the robot model is switched off.
/// In layout mode (<see cref="SetEditable"/>) each card has a Rename button above its top-right corner.
/// </summary>
public class HandCamPip : MonoBehaviour
{
    const float CardWidthM = 0.22f, OutboardM = 0.28f, BelowHandM = 0.05f;
    const float PaddingMm = 8f, RadiusMm = 14f, LabelMm = 30f;
    static readonly Color WaitingColor = new Color(0.15f, 0.15f, 0.15f, 1f);

    class Card
    {
        public int index;
        public bool left, on = true;
        public GameObject go;
        public CameraBadge badge;
        public Transform link;
        public RectTransform rt;
        public RawImage view;
        public TextMeshProUGUI waiting;
        public float aspect;
        public RobotProfile.CameraConfig config;
        public bool sized;
        public TextMeshProUGUI name;
        public Button rename;
    }

    ImageSubscriber _images;
    readonly List<Card> _cards = new List<Card>();

    public int CardCount => _cards.Count;
    // Cards whose camera is on in the bar (the others are hidden and not decoded).
    public int ActiveCardCount { get { int n = 0; foreach (var c in _cards) if (c.go.activeSelf) n++; return n; } }

    public static HandCamPip Create(ImageSubscriber images, RobotModel model, RobotProfile profile)
    {
        if (images == null || model == null || profile == null) return null;
        var go = new GameObject("Hand Cam PiP");
        var pip = go.AddComponent<HandCamPip>();
        pip._images = images;
        foreach (var panel in images.Panels)
            if (IsHandCamera(panel.Config)) pip.Add(model, profile, panel.Config);
        if (pip._cards.Count == 0)
        {
            Debug.LogWarning("HandCams: robot has no hand camera; nothing to show");
            Destroy(go);
            return null;
        }
        images.FrameReady += pip.OnFrame;
        images.CameraVisibilityChanged += pip.OnCameraVisibility;
        images.LabelsChanged += pip.OnLabels;
        return pip;
    }

    static bool IsHandCamera(RobotProfile.CameraConfig config)
        => config != null && config.topicSuffix != null && config.topicSuffix.ToLowerInvariant().Contains("hand");

    // The arm a hand camera belongs to: named like the camera ("left" / "right"), else the only arm.
    static RobotProfile.ArmConfig ArmOf(RobotProfile profile, RobotProfile.CameraConfig config)
    {
        string suffix = config.topicSuffix.ToLowerInvariant();
        foreach (var arm in profile.arms)
            if (!string.IsNullOrEmpty(arm.name) && suffix.Contains(arm.name.ToLowerInvariant())) return arm;
        return profile.arms.Length == 1 ? profile.arms[0] : null;
    }

    // The card's side comes from the camera's name; a hand camera without "left" / "right" goes to the viewer's right.
    void Add(RobotModel model, RobotProfile profile, RobotProfile.CameraConfig config)
    {
        int index = _images.IndexOf(config.topicSuffix);
        if (index < 0) return;
        var arm = ArmOf(profile, config);
        string label = config.displayName;
        if (arm == null) { Debug.LogWarning($"HandCams: no arm for the {label} camera, skipping it"); return; }
        var link = model.Frame(arm.effectorFrame);
        if (link == null) { Debug.LogWarning($"HandCams: frame '{arm.effectorFrame}' not in the model, skipping the {label} camera"); return; }

        var card = new Card { index = index, left = SideOf(config.topicSuffix, config, false), link = link, config = config, aspect = config.Aspect };
        var go = new GameObject("Hand Cam " + label, typeof(RectTransform));
        card.go = go;
        go.transform.SetParent(transform, false);
        var canvas = go.AddComponent<Canvas>();
        go.AddComponent<TrackedDeviceGraphicRaycaster>();   // for the Rename button
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.sortingOrder = HudUi.CanvasSortingOrder - 5;   // above the FPV image, behind the HUD
        canvas.worldCamera = Camera.main;
        card.rt = (RectTransform)go.transform;
        card.rt.localScale = Vector3.one / HudUi.MmPerMetre;

        var bg = HudUi.Round(HudUi.Box(card.rt, "Card", HudUi.PanelColor), RadiusMm);
        HudUi.Stretch(bg.rectTransform);
        card.view = new GameObject("View", typeof(RectTransform)).AddComponent<RawImage>();
        card.view.transform.SetParent(card.rt, false);
        card.view.color = WaitingColor;
        card.view.raycastTarget = false;
        card.waiting = HudUi.Label(card.view.transform, "Waiting", "Waiting", LabelMm * 0.8f);
        card.waiting.color = HudUi.MutedText;
        HudUi.Stretch(card.waiting.rectTransform);
        card.badge = CameraBadge.Create(card.view.transform, card.view.rectTransform.sizeDelta);
        var name = card.name = HudUi.Label(card.rt, "Name", _images.Panels[index].Label, LabelMm);
        name.color = HudUi.MutedText;
        name.textWrappingMode = TextWrappingModes.NoWrap;
        name.enableAutoSizing = true;   // renamed cameras can have long names
        name.fontSizeMin = LabelMm * 0.5f;
        name.fontSizeMax = LabelMm;
        name.rectTransform.anchorMin = new Vector2(0f, 1f);
        name.rectTransform.anchorMax = new Vector2(1f, 1f);
        name.rectTransform.pivot = new Vector2(0.5f, 1f);
        name.rectTransform.anchoredPosition = new Vector2(0f, -PaddingMm * 0.5f);
        name.rectTransform.sizeDelta = new Vector2(0f, LabelMm * 1.2f);

        // Rename: layout mode only, above the card's top-right corner (same look as a camera block's).
        int panelIndex = index;
        card.rename = HudUi.Button(card.rt, "Rename", LabelMm, () => _images.BeginRename(_images.Panels[panelIndex]));
        var rrt = (RectTransform)card.rename.transform;
        rrt.anchorMin = rrt.anchorMax = rrt.pivot = new Vector2(1f, 1f);
        rrt.sizeDelta = new Vector2(LabelMm * 4.6f, LabelMm * 1.6f);
        rrt.anchoredPosition = new Vector2(0f, rrt.sizeDelta.y + PaddingMm * 0.5f);
        card.rename.gameObject.SetActive(false);

        _cards.Add(card);
        ApplySize(card);
        SetCardOn(card, _images.IsOn(_images.Panels[index]));
    }

    // Card side from the camera's identity: "left" in the topic -> true, "right" -> false, else `fallback`.
    public static bool SideOf(string topic, RobotProfile.CameraConfig config, bool fallback)
    {
        string t = ((topic ?? "") + " " + (config?.topicSuffix ?? "")).ToLowerInvariant();
        bool l = t.Contains("left"), r = t.Contains("right");
        return l != r ? l : fallback;
    }

    // Layout mode: the cards' Rename buttons are available.
    public void SetEditable(bool editable)
    {
        foreach (var c in _cards) c.rename.gameObject.SetActive(editable);
    }

    // A camera was renamed: show the new name.
    void OnLabels()
    {
        if (this == null) return;
        foreach (var c in _cards) c.name.text = _images.Panels[c.index].Label;
    }

    void OnCameraVisibility(int index, bool on)
    {
        if (this == null) return;
        foreach (var c in _cards)
            if (c.index == index) SetCardOn(c, on);
    }

    // Camera toggled in the bar: show / hide this card and stop / resume decoding its frames.
    void SetCardOn(Card c, bool on)
    {
        c.on = on;
        c.go.SetActive(on);
        _images.ForceDecode(c.index, on);
    }

    // (Re)size the card for the camera's aspect ratio.
    void ApplySize(Card c)
    {
        float viewW = CardWidthM * HudUi.MmPerMetre - 2f * PaddingMm;
        float viewH = viewW / Mathf.Max(c.aspect, 0.1f);
        float top = PaddingMm + LabelMm * 1.2f;
        c.rt.sizeDelta = new Vector2(viewW + 2f * PaddingMm, top + viewH + PaddingMm);
        var v = c.view.rectTransform;
        v.anchorMin = v.anchorMax = new Vector2(0.5f, 1f);
        v.pivot = new Vector2(0.5f, 1f);
        v.sizeDelta = new Vector2(viewW, viewH);
        v.anchoredPosition = new Vector2(0f, -top);
        c.badge?.Fit(v.sizeDelta);
    }

    void OnFrame(int index, Texture2D tex)
    {
        if (this == null || !isActiveAndEnabled || tex == null) return;
        foreach (var c in _cards)
        {
            if (c.index != index) continue;
            c.waiting.gameObject.SetActive(false);
            c.view.color = Color.white;
            c.view.texture = tex;   // raw decoding can replace the instance
            c.view.uvRect = new Rect(                     // same flips as CameraPanel.SetTexture
                c.config.flipHorizontal ? 1f : 0f, c.config.flipVertical ? 1f : 0f,
                c.config.flipHorizontal ? -1f : 1f, c.config.flipVertical ? -1f : 1f);
            if (!c.sized && tex.height > 0)
            {
                c.sized = true;
                c.aspect = (float)tex.width / tex.height;
                ApplySize(c);
            }
        }
    }

    void Update()
    {
        foreach (var c in _cards)
            if (c.on) c.badge.Tick(_images, c.index);
    }

    void LateUpdate()
    {
        var head = FirstPersonView.Head;
        if (head == null) return;
        foreach (var c in _cards)
        {
            if (!c.on || c.link == null) continue;
            c.rt.position = HandsPosition(c, head);
            // Canvas front faces -Z, so look away from the head.
            c.rt.rotation = Quaternion.LookRotation(c.rt.position - head.position, Vector3.up);
        }
    }

    // Outboard and low: `outward` is the horizontal direction perpendicular to the line of sight
    // (head -> hand); cross(up, forward) points to the viewer's right. The card goes to the side
    // the camera's identity gives (left camera: viewer's left, right camera: right), never the hand's
    // position, so the cards do not swap sides when the arms cross.
    Vector3 HandsPosition(Card c, Transform head)
    {
        Vector3 hand = c.link.position;
        Vector3 outward = Vector3.Cross(Vector3.up, hand - head.position);
        outward.y = 0f;
        if (outward.sqrMagnitude < 1e-6f) outward = Vector3.ProjectOnPlane(head.right, Vector3.up);
        outward.Normalize();
        Vector3 pos = hand + outward * ((c.left ? -1f : 1f) * OutboardM);
        pos.y = hand.y - BelowHandM;
        return pos;
    }

    void OnDestroy()
    {
        if (_images == null) return;
        _images.FrameReady -= OnFrame;
        _images.CameraVisibilityChanged -= OnCameraVisibility;
        _images.LabelsChanged -= OnLabels;
        foreach (var c in _cards)
        {
            _images.ForceDecode(c.index, false);   // the head camera's stays
            if (c.rt != null) Destroy(c.rt.gameObject);
        }
    }
}
