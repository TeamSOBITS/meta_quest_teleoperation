using TMPro;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit.Inputs;

/// <summary>
/// Small labels above the controllers for the first seconds in a robot screen, so the
/// controller shortcuts are discoverable:
///   left:  "Menu button: back to robots"
///   right: "Trigger: drag blocks (layout mode)"
/// The labels face the head and fade out after <see cref="ShowSeconds"/>.
/// </summary>
public class ControllerHints : MonoBehaviour
{
    public const float ShowSeconds = 10f;
    const float FadeSeconds = 1f;
    const float LabelDistance = 0.5f;               // typical controller distance from the eye (m)
    static readonly Vector3 Offset = new Vector3(0f, 0.07f, 0f);   // above the controller (m)

    Transform _head;
    readonly System.Collections.Generic.List<(Transform anchor, RectTransform label, CanvasGroup group)> _hints = new();
    float _start;

    public static ControllerHints Create(Transform head)
    {
        var modality = FindFirstObjectByType<XRInputModalityManager>();
        if (modality == null) return null;

        var hints = new GameObject("Controller Hints").AddComponent<ControllerHints>();
        hints._head = head;
        hints._start = Time.time;
        hints.Add(modality.leftController, "Menu button: back to robots");
        hints.Add(modality.rightController, "Trigger: drag blocks (layout mode)");
        return hints;
    }

    void Add(GameObject controller, string text)
    {
        if (controller == null) return;
        float body = HudUi.FontAt(HudUi.BodyFontSize, LabelDistance);

        var label = HudUi.Label(null, "Label", text, body);
        label.textWrappingMode = TextWrappingModes.NoWrap;
        var size = label.GetPreferredValues(text);
        var root = HudUi.CreateCanvas("Hint " + controller.name, transform, Vector3.forward,
            new Vector2(size.x + 2f * body, size.y * 1.7f), interactive: false);
        var bg = HudUi.Round(HudUi.Box(root, "Background", HudUi.PanelColor), size.y);
        HudUi.Stretch(bg.rectTransform);
        label.transform.SetParent(root, false);
        HudUi.Stretch(label.rectTransform);
        var group = root.gameObject.AddComponent<CanvasGroup>();
        group.interactable = false;
        group.blocksRaycasts = false;
        _hints.Add((controller.transform, root, group));
    }

    void LateUpdate()
    {
        float t = Time.time - _start;
        float alpha = 1f - Mathf.Clamp01((t - ShowSeconds) / FadeSeconds);
        if (alpha <= 0f) { Destroy(gameObject); return; }

        foreach (var (anchor, label, group) in _hints)
        {
            bool tracked = anchor != null && anchor.gameObject.activeInHierarchy;
            group.alpha = tracked ? alpha : 0f;
            if (!tracked) continue;
            label.position = anchor.position + Offset;
            // Face the eye (canvas front faces -Z, so look away from the head).
            label.rotation = Quaternion.LookRotation(label.position - _head.position, Vector3.up);
        }
    }
}
