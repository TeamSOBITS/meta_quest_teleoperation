using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// First-person layout: a thin outline the size of the head camera's card, hung on the headset at the card's
/// distance, i.e. where the image will be once the robot's head has caught up with yours. It fades in while
/// the headset's direction and the camera frame's direction differ by more than <see cref="ShowAboveDeg"/>
/// and fades out (0.2 s) when they agree again. Created by <see cref="FirstPersonView"/> with the card.
/// </summary>
public class HeadLagOutline : MonoBehaviour
{
    public const float ShowAboveDeg = 3f, FadeSeconds = 0.2f, MaxAlpha = 0.25f;

    Transform _head;
    CameraCard _card;
    Image _ring;
    Canvas _canvas;
    float _alpha;

    // Angle between the headset's and the camera frame's direction (degrees), and the outline's alpha (0 = hidden).
    public float AngleDeg { get; private set; }
    public float Alpha => _alpha;
    public bool Visible => _canvas != null && _canvas.enabled;
    // The canvas the outline is drawn on (its forward is the headset's direction).
    public Transform Plane => _canvas.transform;

    public static HeadLagOutline Create(Transform head, CameraCard card, float distanceM, float radiusMm)
    {
        if (head == null || card == null) return null;
        var rt = HudUi.CreateCanvas("Head Lag Outline", head, new Vector3(0f, 0f, distanceM), new Vector2(100f, 100f), interactive: false);
        rt.GetComponent<Canvas>().sortingOrder = HudTheme.SortingOrder - 11;
        var outline = rt.gameObject.AddComponent<HeadLagOutline>();
        outline._head = head;
        outline._card = card;
        outline._canvas = rt.GetComponent<Canvas>();
        outline._ring = HudUi.Ring(HudUi.Box(rt, "Ring", HudTheme.WithAlpha(HudTheme.Accent, 0f)), radiusMm);
        outline._ring.rectTransform.anchorMin = outline._ring.rectTransform.anchorMax = outline._ring.rectTransform.pivot = new Vector2(0.5f, 0.5f);
        outline._canvas.enabled = false;
        return outline;
    }

    void Update()
    {
        if (_card == null || _head == null) return;
        // The card's canvas looks along the camera frame's forward axis.
        AngleDeg = Vector3.Angle(_head.forward, _card.Rect.forward);
        _alpha = Mathf.MoveTowards(_alpha, AngleDeg > ShowAboveDeg ? 1f : 0f, Time.unscaledDeltaTime / FadeSeconds);
        _canvas.enabled = _alpha > 0f && _card.gameObject.activeInHierarchy;
        if (!_canvas.enabled) return;

        // Same size and place on the canvas as the card.
        var rt = _ring.rectTransform;
        rt.sizeDelta = _card.CardRect.sizeDelta;
        rt.anchoredPosition = _card.CardRect.anchoredPosition;
        _ring.color = HudTheme.WithAlpha(HudTheme.Accent, MaxAlpha * _alpha);
    }
}
