using TMPro;
using UnityEngine;

/// <summary>
/// Head-locked panel at the lower right of the view:
///   - Publish Joy on/off (on by default)
///   - one show/hide toggle per camera
///   - Reset layout
/// </summary>
public class ControlPanel : MonoBehaviour
{
    // ~1 m in front, lower right (about 24 deg right, 14 deg down).
    public static readonly Vector3 Position = new Vector3(0.45f, -0.25f, 1.0f);
    const float WidthMm = 340f, PaddingMm = 15f, RowGapMm = 8f;

    public static ControlPanel Create(Transform parent, QuestControllerPublisher publisher, ImageSubscriber images)
    {
        float distance = Position.magnitude;
        float title = HudUi.FontAt(HudUi.TitleFontSize, distance);
        float body  = HudUi.FontAt(HudUi.BodyFontSize, distance);
        float rowH  = body * 1.6f;

        int cameraCount = images.Panels.Count;
        float heightMm = PaddingMm + title * 1.4f                       // "Controls"
                       + rowH + RowGapMm                                 // Publish Joy
                       + body * 1.6f                                     // "Cameras"
                       + cameraCount * (rowH + RowGapMm)
                       + RowGapMm + rowH * 1.2f + PaddingMm;             // Reset layout

        var root = HudUi.CreateCanvas("Control Panel", parent, Position, new Vector2(WidthMm, heightMm), interactive: true);
        var panel = root.gameObject.AddComponent<ControlPanel>();
        HudUi.Stretch(HudUi.Box(root, "Background", HudUi.PanelColor).rectTransform);

        float top = PaddingMm;
        void Add(RectTransform rt, float h) { HudUi.Row(rt, top, h, PaddingMm); top += h + RowGapMm; }

        Add(HudUi.Label(root, "Title", "Controls", title, TextAlignmentOptions.Left).rectTransform, title * 1.4f - RowGapMm);

        Add((RectTransform)HudUi.Toggle(root, "Publish Joy", body, publisher.publishJoy,
            on => publisher.publishJoy = on).transform, rowH);

        var camerasLabel = HudUi.Label(root, "Cameras", "Cameras", body, TextAlignmentOptions.Left);
        camerasLabel.color = new Color(1f, 1f, 1f, 0.6f);
        Add(camerasLabel.rectTransform, body * 1.6f - RowGapMm);

        foreach (var cam in images.Panels)
        {
            var c = cam;
            Add((RectTransform)HudUi.Toggle(root, c.Config.displayName, body, c.Visible,
                on => c.Visible = on).transform, rowH);
        }

        top += RowGapMm;
        Add((RectTransform)HudUi.Button(root, "Reset layout", body, images.ResetLayout).transform, rowH * 1.2f);

        return panel;
    }
}
