using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Hand cameras in first person: one small card per hand camera of the robot (RawImage on a
/// rounded PanelColor box, a muted "Left hand" / "Right hand" label). In mode "hands" a card floats
/// outboard of and a little below its gripper (the model's hand_*_end_effector_link), to the side
/// of the line of sight so it never covers what the head camera shows there, and faces the head;
/// in mode "corners"
/// both cards are head-locked at the bottom corners of the view. Created by TeleopHud in first
/// person while the "handcams" experiment is on, destroyed when either turns off.
/// The mode is a second pref, "Exp/handcams.mode" (<see cref="Mode"/>).
/// </summary>
public class HandCamPip : MonoBehaviour
{
    public const string ModeHands = "hands", ModeCorners = "corners";
    const string ModePref = "Exp/handcams.mode";

    const float CardWidthHandsM = 0.22f, CardWidthCornersM = 0.30f, OutboardM = 0.28f, BelowHandM = 0.05f, SideDeadZoneM = 0.03f;
    const float CornerDistanceM = 0.9f, CornerXM = 0.52f, CornerYM = -0.35f;   // x: clear of the status strip
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
        public string builtMode;
    }

    ImageSubscriber _images;
    readonly List<Card> _cards = new List<Card>();

    public static string Mode
    {
        get => PlayerPrefs.GetString(ModePref, ModeHands) == ModeCorners ? ModeCorners : ModeHands;
        set
        {
            string v = value == ModeCorners ? ModeCorners : ModeHands;
            if (Mode == v) return;
            PlayerPrefs.SetString(ModePref, v);
            PlayerPrefs.Save();
            Debug.Log($"[Experiments] handcams.mode -> {v}");
        }
    }

    public int CardCount => _cards.Count;
    // Cards whose camera is on in the bar (the others are hidden and not decoded).
    public int ActiveCardCount { get { int n = 0; foreach (var c in _cards) if (c.go.activeSelf) n++; return n; } }

    public static HandCamPip Create(ImageSubscriber images, RobotModel model, RobotProfile profile)
    {
        if (images == null || model == null || profile == null) return null;
        var go = new GameObject("Hand Cam PiP");
        var pip = go.AddComponent<HandCamPip>();
        pip._images = images;
        pip.Add(model, profile, true, "hand_left_camera/color/image_raw/compressed", "hand_left_end_effector_link", "Left hand");
        pip.Add(model, profile, false, "hand_right_camera/color/image_raw/compressed", "hand_right_end_effector_link", "Right hand");
        if (pip._cards.Count == 0)
        {
            Debug.LogWarning("HandCams: robot has no hand camera; nothing to show");
            Destroy(go);
            return null;
        }
        images.FrameReady += pip.OnFrame;
        images.CameraVisibilityChanged += pip.OnCameraVisibility;
        return pip;
    }

    void Add(RobotModel model, RobotProfile profile, bool left, string suffix, string frame, string label)
    {
        int index = _images.IndexOf(suffix);
        if (index < 0) return;
        var link = model.Frame(frame);
        if (link == null) { Debug.LogWarning($"HandCams: frame '{frame}' not in the model, skipping the {label} camera"); return; }

        var config = _images.Panels[index].Config;
        var card = new Card { index = index, left = left, link = link, config = config, aspect = config.Aspect };
        var go = new GameObject("Hand Cam " + label, typeof(RectTransform));
        card.go = go;
        go.transform.SetParent(transform, false);
        var canvas = go.AddComponent<Canvas>();
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
        card.badge = CameraBadge.Create(card.view.transform, LabelMm);
        var name = HudUi.Label(card.rt, "Name", label, LabelMm);
        name.color = HudUi.MutedText;
        name.textWrappingMode = TextWrappingModes.NoWrap;
        // Layout is applied in ApplyMode (card size depends on the mode).
        name.rectTransform.anchorMin = new Vector2(0f, 1f);
        name.rectTransform.anchorMax = new Vector2(1f, 1f);
        name.rectTransform.pivot = new Vector2(0.5f, 1f);
        name.rectTransform.anchoredPosition = new Vector2(0f, -PaddingMm * 0.5f);
        name.rectTransform.sizeDelta = new Vector2(0f, LabelMm * 1.2f);

        _cards.Add(card);
        ApplyMode(card);
        SetCardOn(card, _images.IsOn(_images.Panels[index]));
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

    // (Re)size the card for the current mode and parent it (hands: free, corners: under the head).
    void ApplyMode(Card c)
    {
        string mode = Mode;
        c.builtMode = mode;
        float widthM = mode == ModeCorners ? CardWidthCornersM : CardWidthHandsM;
        float viewW = widthM * HudUi.MmPerMetre - 2f * PaddingMm;
        float viewH = viewW / Mathf.Max(c.aspect, 0.1f);
        float top = PaddingMm + LabelMm * 1.2f;
        c.rt.sizeDelta = new Vector2(viewW + 2f * PaddingMm, top + viewH + PaddingMm);
        var v = c.view.rectTransform;
        v.anchorMin = v.anchorMax = new Vector2(0.5f, 1f);
        v.pivot = new Vector2(0.5f, 1f);
        v.sizeDelta = new Vector2(viewW, viewH);
        v.anchoredPosition = new Vector2(0f, -top);

        var head = FirstPersonView.Head;
        c.rt.SetParent(mode == ModeCorners && head != null ? head : transform, false);
        c.rt.localScale = Vector3.one / HudUi.MmPerMetre;
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
                ApplyMode(c);
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
        string mode = Mode;
        foreach (var c in _cards)
        {
            if (!c.on) continue;
            if (c.builtMode != mode) ApplyMode(c);
            if (head == null) continue;
            if (mode == ModeCorners)
            {
                c.rt.localPosition = new Vector3(c.left ? -CornerXM : CornerXM, CornerYM, CornerDistanceM);
                c.rt.localRotation = Quaternion.LookRotation(c.rt.localPosition);
            }
            else if (c.link != null)
            {
                c.rt.position = HandsPosition(c, head);
                // Canvas front faces -Z, so look away from the head.
                c.rt.rotation = Quaternion.LookRotation(c.rt.position - head.position, Vector3.up);
            }
        }
    }

    // Outboard and low: `outward` is the horizontal direction perpendicular to the line of sight
    // (head -> hand); cross(up, forward) points to the viewer's right. The card goes to the side
    // the hand is on in head space (its own side by default; the hand's real side when it has
    // crossed over), so the left hand's card ends up on the viewer's left.
    Vector3 HandsPosition(Card c, Transform head)
    {
        Vector3 hand = c.link.position;
        Vector3 outward = Vector3.Cross(Vector3.up, hand - head.position);
        outward.y = 0f;
        if (outward.sqrMagnitude < 1e-6f) outward = Vector3.ProjectOnPlane(head.right, Vector3.up);
        outward.Normalize();
        float x = head.InverseTransformPoint(hand).x;   // + = viewer's right
        float side = Mathf.Abs(x) > SideDeadZoneM ? Mathf.Sign(x) : c.left ? -1f : 1f;
        Vector3 pos = hand + outward * (side * OutboardM);
        pos.y = hand.y - BelowHandM;
        return pos;
    }

    void OnDestroy()
    {
        if (_images == null) return;
        _images.FrameReady -= OnFrame;
        _images.CameraVisibilityChanged -= OnCameraVisibility;
        foreach (var c in _cards)
        {
            _images.ForceDecode(c.index, false);   // the head camera's stays
            if (c.rt != null) Destroy(c.rt.gameObject);   // corner cards hang under the head, not under us
        }
    }
}
