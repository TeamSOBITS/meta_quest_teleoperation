using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// The "Control robot" safety delay. Turning control ON does not publish at once: a 2 s countdown runs,
/// shown as a ring filling around the toggle's box and the label "Control in 2…" / "Control in 1…", and
/// the publisher's controlRobot only becomes true when it completes. Pressing the toggle again while it
/// runs cancels it. Turning control OFF is immediate. Owned by <see cref="HudBar"/> (RequestControl).
/// </summary>
public class ControlCountdown
{
    public const float Seconds = 2f;
    const float RingMarginRatio = 0.18f;   // ring distance from the box, of the box size

    readonly QuestControllerPublisher _publisher;
    readonly Toggle _toggle;
    readonly Image _ring;
    readonly TextMeshProUGUI _label;
    readonly string _idleText;
    float _start = -1f;

    public bool Counting => _start >= 0f;
    // 0..1 while counting.
    public float Progress => Counting ? Mathf.Clamp01((Time.unscaledTime - _start) / Seconds) : 0f;
    public string LabelText => _label != null ? _label.text : "";

    // `toggle` is the "Control robot" toggle made by HudUi.Toggle (its Box and Label children are used).
    public ControlCountdown(QuestControllerPublisher publisher, Toggle toggle)
    {
        _publisher = publisher;
        _toggle = toggle;
        _label = toggle.GetComponentInChildren<TextMeshProUGUI>();
        _idleText = _label != null ? _label.text : "";

        var box = (RectTransform)toggle.transform.Find("Box");
        float margin = box.sizeDelta.x * RingMarginRatio;
        _ring = HudUi.Box(toggle.transform, "Countdown Ring", HudTheme.Accent);
        HudUi.Ring(_ring, margin / HudUi.RingThicknessRatio);
        _ring.type = Image.Type.Filled;   // radial fill needs a filled (not sliced) image
        _ring.fillMethod = Image.FillMethod.Radial360;
        _ring.fillOrigin = (int)Image.Origin360.Top;
        _ring.fillClockwise = true;
        var rt = _ring.rectTransform;
        rt.anchorMin = rt.anchorMax = box.anchorMin;
        rt.pivot = box.pivot;
        rt.sizeDelta = box.sizeDelta + Vector2.one * 2f * margin;
        rt.anchoredPosition = box.anchoredPosition + new Vector2(-margin, 0f);
        _ring.gameObject.SetActive(false);
    }

    // The toggle was switched (or a caller asks): on starts the countdown, off cancels it and stops control at once.
    public void Request(bool on)
    {
        if (on)
        {
            if (_publisher.controlRobot || Counting) return;
            _start = Time.unscaledTime;
            _toggle.SetIsOnWithoutNotify(true);   // a request from code leaves the toggle on, as a press does
            _ring.gameObject.SetActive(true);
            Show();
            DevLog.Log("[Control]", "countdown started");
            return;
        }
        if (Counting)
        {
            End();
            DevLog.Log("[Control]", "countdown cancelled");
        }
        _publisher.controlRobot = false;
    }

    public void Tick()
    {
        if (!Counting) return;
        if (Time.unscaledTime - _start >= Seconds)
        {
            End();
            _publisher.controlRobot = true;
            DevLog.Log("[Control]", "countdown armed");
            return;
        }
        Show();
    }

    void Show()
    {
        _ring.fillAmount = Progress;
        if (_label != null) _label.text = $"Control in {Mathf.CeilToInt(Seconds - (Time.unscaledTime - _start))}…";
    }

    // Back to the idle look; a cancelled countdown also puts the toggle back off.
    void End()
    {
        bool cancelled = Counting && Time.unscaledTime - _start < Seconds;
        _start = -1f;
        _ring.gameObject.SetActive(false);
        if (_label != null) _label.text = _idleText;
        if (cancelled) _toggle.SetIsOnWithoutNotify(false);
    }
}
