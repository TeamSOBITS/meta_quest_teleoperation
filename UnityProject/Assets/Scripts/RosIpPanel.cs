using TMPro;
using UnityEngine;

/// <summary>
/// Head-locked block at the bottom centre of the view: current ROS IP, connection
/// state, and an Edit button that opens the Quest keyboard to type a new IP.
/// </summary>
public class RosIpPanel : MonoBehaviour
{
    // Below the camera blocks (which stay above ImageSubscriber.minBottom).
    public static readonly Vector3 Position = new Vector3(0f, -1.86f, HudUi.ReferenceDistance);
    const float WidthMm = 1900f, HeightMm = 260f, PaddingMm = 30f, ButtonWidthMm = 380f;

    QuestControllerPublisher _publisher;
    TextMeshProUGUI _ipLabel;

    public static RosIpPanel Create(Transform parent, QuestControllerPublisher publisher)
    {
        var rt = HudUi.CreateCanvas("ROS IP Panel", parent, Position, new Vector2(WidthMm, HeightMm), interactive: true);
        var panel = rt.gameObject.AddComponent<RosIpPanel>();
        panel._publisher = publisher;
        panel.Build(rt);
        return panel;
    }

    void Build(RectTransform root)
    {
        HudUi.Stretch(HudUi.Box(root, "Background", HudUi.PanelColor).rectTransform);

        float font = HudUi.TitleFontSize * HudUi.MmPerMetre;

        _ipLabel = HudUi.Label(root, "IP", "", font, TextAlignmentOptions.Left);
        HudUi.Stretch(_ipLabel.rectTransform, PaddingMm);
        _ipLabel.rectTransform.offsetMax = new Vector2(-(ButtonWidthMm + 2 * PaddingMm), -PaddingMm);

        var button = HudUi.Button(root, "Edit", font, _publisher.OpenIpKeyboard);
        var brt = (RectTransform)button.transform;
        brt.anchorMin = brt.anchorMax = brt.pivot = new Vector2(1f, 0.5f);
        brt.sizeDelta = new Vector2(ButtonWidthMm, HeightMm - 2 * PaddingMm);
        brt.anchoredPosition = new Vector2(-PaddingMm, 0f);
    }

    void Update()
    {
        string state = _publisher.HasConnectionError
            ? "<color=#FF6060>not connected</color>"
            : "<color=#60E080>connected</color>";
        _ipLabel.text = $"ROS IP: {_publisher.DisplayedIp}   {state}";
    }
}
