using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Head-locked bar under the camera blocks, the same for every robot:
///
///   +--------------------------------------------------------------+
///   | ROS IP  192.168.11.20            ( connected )      [ Edit ] |
///   |--------------------------------------------------------------|
///   | [x] Publish Joy  |  [x] Head Camera   [x] Hand Camera  [Reset |
///   |                  |  [x] Front Camera  [ ] Back Camera  layout]|
///   +--------------------------------------------------------------+
/// </summary>
public class HudBar : MonoBehaviour
{
    // Sizes in mm on a canvas at HudUi.ReferenceDistance.
    const float WidthMm = 3600f, PaddingMm = 50f, GapMm = 40f, RadiusMm = 60f;
    const float HeaderHeightMm = 220f, RowHeightMm = 150f;
    const float JoyColumnMm = 850f, ResetColumnMm = 520f, EditWidthMm = 420f, PillWidthMm = 640f;
    const int CameraColumns = 2;
    // Space between the lowest camera block and the bar (metres). Blocks are turned to face
    // the eye, which brings their lower outer corners slightly down in view; this gap absorbs it.
    const float GapBelowCamerasM = 0.17f;

    QuestControllerPublisher _publisher;
    TextMeshProUGUI _ip;
    Image _pill;
    TextMeshProUGUI _pillText;
    bool? _shownConnected;

    public static HudBar Create(Transform parent, QuestControllerPublisher publisher, ImageSubscriber images)
    {
        int cameraRows = Mathf.Max(1, Mathf.CeilToInt(images.Panels.Count / (float)CameraColumns));
        float controlsH = cameraRows * RowHeightMm + (cameraRows - 1) * GapMm;
        float heightMm = PaddingMm + HeaderHeightMm + GapMm + 4f + GapMm + controlsH + PaddingMm;

        // Top edge just below the lowest point the camera blocks may reach.
        float topY = images.minBottom - GapBelowCamerasM;
        var position = new Vector3(0f, topY - heightMm / HudUi.MmPerMetre / 2f, HudUi.ReferenceDistance);

        var root = HudUi.CreateCanvas("HUD Bar", parent, position, new Vector2(WidthMm, heightMm), interactive: true);
        var bar = root.gameObject.AddComponent<HudBar>();
        bar._publisher = publisher;
        bar.Build(root, images, heightMm);
        return bar;
    }

    void Build(RectTransform root, ImageSubscriber images, float heightMm)
    {
        float title = HudUi.TitleFontSize * HudUi.MmPerMetre;
        float body  = HudUi.BodyFontSize  * HudUi.MmPerMetre;
        float inner = WidthMm - 2 * PaddingMm;

        HudUi.Stretch(HudUi.Round(HudUi.Box(root, "Background", HudUi.PanelColor), RadiusMm).rectTransform);

        // --- Header: ROS IP, connection pill, Edit ---
        float top = PaddingMm;
        var caption = HudUi.Label(root, "Caption", "ROS IP", body, TextAlignmentOptions.Left);
        caption.color = HudUi.MutedText;
        caption.textWrappingMode = TextWrappingModes.NoWrap;
        float captionW = caption.GetPreferredValues("ROS IP").x;
        HudUi.Place(caption.rectTransform, PaddingMm, top, captionW, HeaderHeightMm);

        float ipLeft = PaddingMm + captionW + GapMm;
        float ipWidth = inner - captionW - GapMm - PillWidthMm - GapMm - EditWidthMm - GapMm;
        _ip = HudUi.Label(root, "IP", "", title, TextAlignmentOptions.Left);
        _ip.textWrappingMode = TextWrappingModes.NoWrap;
        _ip.overflowMode = TextOverflowModes.Ellipsis;
        HudUi.Place(_ip.rectTransform, ipLeft, top, ipWidth, HeaderHeightMm);

        float pillH = body * 1.7f;
        _pill = HudUi.Round(HudUi.Box(root, "Status", Color.clear), pillH / 2f);
        HudUi.Place(_pill.rectTransform, ipLeft + ipWidth + GapMm, top + (HeaderHeightMm - pillH) / 2f, PillWidthMm, pillH);
        _pillText = HudUi.Label(_pill.transform, "Label", "", body);
        HudUi.Stretch(_pillText.rectTransform);

        var edit = HudUi.Button(root, "Edit", body, _publisher.OpenIpKeyboard);
        float editH = HeaderHeightMm - 2 * 30f;
        HudUi.Place((RectTransform)edit.transform, WidthMm - PaddingMm - EditWidthMm, top + 30f, EditWidthMm, editH);
        top += HeaderHeightMm + GapMm;

        var divider = HudUi.Box(root, "Divider", new Color(1f, 1f, 1f, 0.12f));
        HudUi.Place(divider.rectTransform, PaddingMm, top, inner, 4f);
        top += 4f + GapMm;

        // --- Controls: Publish Joy | camera toggles | Reset layout ---
        var joy = HudUi.Toggle(root, "Publish Joy", body, _publisher.publishJoy, on => _publisher.publishJoy = on);
        HudUi.Place((RectTransform)joy.transform, PaddingMm, top, JoyColumnMm, RowHeightMm);

        float camLeft = PaddingMm + JoyColumnMm + GapMm;
        float camWidth = inner - JoyColumnMm - GapMm - ResetColumnMm - GapMm;
        float colW = (camWidth - (CameraColumns - 1) * GapMm) / CameraColumns;
        for (int i = 0; i < images.Panels.Count; i++)
        {
            var cam = images.Panels[i];
            var t = HudUi.Toggle(root, cam.Config.displayName, body, cam.Visible, on => images.SetCameraVisible(cam, on));
            HudUi.Place((RectTransform)t.transform,
                camLeft + (i % CameraColumns) * (colW + GapMm),
                top + (i / CameraColumns) * (RowHeightMm + GapMm),
                colW, RowHeightMm);
        }

        var reset = HudUi.Button(root, "Reset layout", body, images.ResetLayout);
        var resetLabel = reset.GetComponentInChildren<TextMeshProUGUI>();
        resetLabel.margin = new Vector4(20f, 0f, 20f, 0f);
        HudUi.Place((RectTransform)reset.transform, WidthMm - PaddingMm - ResetColumnMm, top, ResetColumnMm,
            heightMm - top - PaddingMm);
    }

    void Update()
    {
        _ip.text = _publisher.DisplayedIp;

        bool connected = !_publisher.HasConnectionError;
        if (_shownConnected == connected) return;
        _shownConnected = connected;
        var c = connected ? HudUi.GoodColor : HudUi.BadColor;
        _pill.color = new Color(c.r, c.g, c.b, 0.18f);
        _pillText.color = c;
        _pillText.text = connected ? "connected" : "not connected";
    }
}
