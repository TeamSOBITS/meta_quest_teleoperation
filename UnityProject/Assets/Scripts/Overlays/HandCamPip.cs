using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// <summary>
/// Hand cameras in first person: one small <see cref="CameraCard"/> per hand camera of the robot (role Hand in the
/// profile; a rounded Panel box with a muted label with the camera's name, no highlight ring).
/// A card floats outboard of and a little below its mount frame (the camera's mountFrame, else the end effector
/// of the arm on the camera's side, else the only arm's), to the camera's side of the line of sight
/// so it never covers what the head camera shows there, and faces the head. Created by TeleopHud in
/// the first-person camera layout, destroyed when the layout or the robot model is switched off.
/// In layout mode (<see cref="SetEditable"/>) each card has a Rename button above its top-right corner.
/// </summary>
public class HandCamPip : MonoBehaviour
{
    const float CardWidthM = 0.22f, OutboardM = 0.28f, BelowHandM = 0.05f;
    const float PaddingMm = 8f, RadiusMm = 14f, LabelMm = 30f;
    const float LabelHeightRatio = 1.2f, WaitingFontRatio = 0.8f;   // of LabelMm
    const int SortingOrderBelowHud = 5;

    class Card
    {
        public int index;
        public bool left, on = true;
        public GameObject go;
        public CameraCard card;
        public CameraBadge badge;
        public Transform link;
        public RectTransform rt;
        public RawImage view;
        public float aspect;
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
            if (panel.Config != null && panel.Config.role == RobotProfile.CameraRole.Hand) pip.Add(model, profile, panel.Config);
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

    // Frame a hand camera's card is placed at: its mountFrame, else the end effector of the arm on its side
    // (or the only arm); null when the profile names none.
    static string MountFrameOf(RobotProfile profile, RobotProfile.CameraConfig config)
    {
        if (!string.IsNullOrEmpty(config.mountFrame)) return config.mountFrame;
        if (config.side != RobotProfile.Side.None)
            foreach (var arm in profile.arms)
                if (arm.side == config.side) return arm.effectorFrame;
        return profile.arms.Length == 1 ? profile.arms[0].effectorFrame : null;
    }

    // The card's side comes from the camera's side in the profile; a hand camera without one goes to the viewer's right.
    void Add(RobotModel model, RobotProfile profile, RobotProfile.CameraConfig config)
    {
        int index = _images.IndexOf(config.topicSuffix);
        if (index < 0) return;
        string mount = MountFrameOf(profile, config);
        string label = config.displayName;
        if (string.IsNullOrEmpty(mount)) { Debug.LogWarning($"HandCams: no mount frame for the {label} camera, skipping it"); return; }
        var link = model.Frame(mount);
        if (link == null) { Debug.LogWarning($"HandCams: frame '{mount}' not in the model, skipping the {label} camera"); return; }

        var card = new Card { index = index, left = config.side == RobotProfile.Side.Left, link = link, aspect = config.Aspect };
        var style = new CameraCard.Style
        {
            PadMm = PaddingMm, RadiusMm = RadiusMm,
            NameFontMm = LabelMm, NameHeightMm = LabelMm * LabelHeightRatio, NameRaiseMm = PaddingMm * 0.5f,
            NameFullWidth = true, NameCompact = true,
            WaitingText = "Waiting", WaitingFontMm = LabelMm * WaitingFontRatio,
            RenameFontMm = LabelMm,
            SortingOrder = HudTheme.SortingOrder - SortingOrderBelowHud,   // above the FPV image, behind the HUD
        };
        card.card = CameraCard.Create(transform, "Hand Cam " + label, style, config, _images.Panels[index].Label);
        int panelIndex = index;
        card.card.Renamed += () => _images.BeginRename(_images.Panels[panelIndex]);
        card.go = card.card.gameObject;
        card.rt = card.card.Rect;
        card.view = card.card.View;
        card.badge = card.card.Badge;
        card.name = card.card.NameLabel;
        card.rename = card.card.RenameButton;

        _cards.Add(card);
        ApplySize(card);
        SetCardOn(card, _images.IsOn(_images.Panels[index]));
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
        foreach (var c in _cards) c.card.SetLabel(_images.Panels[c.index].Label);
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
        c.card.SetSize(viewW, viewW / Mathf.Max(c.aspect, 0.1f));
    }

    void OnFrame(int index, Texture2D tex)
    {
        if (this == null || !isActiveAndEnabled || tex == null) return;
        foreach (var c in _cards)
        {
            if (c.index != index) continue;
            c.card.SetTexture(tex);   // raw decoding can replace the instance
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
