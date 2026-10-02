using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Small head-locked status strip shown while the HUD bar (menu) is hidden, in both view modes:
/// at the bottom of the first-person view, or in blocks mode where the bar normally sits. One row: connection, CONTROL ON / LAYOUT, image fps and age, TF rate, round trip,
/// a labelled HEAD box (crosshair, dot at pan x / tilt y, L/R ticks, "pan 29° tilt 17°") and a
/// labelled LIFT bar with its height ("0.40 m"). Created by TeleopHud in both camera layouts
/// (active only while the bar is hidden) and rebuilt when the layout changes.
/// In the blocks layout there is no model reading: HEAD, LIFT and the TF rate are left out ("TF —") and the strip is narrower.
///
/// Head and lift are read from the model's link local poses, which RobotModel sets straight from
/// /tf (FLU -> Unity). A ROS rotation of t about z shows up as -t about Unity's up axis, and a
/// rotation of t about y (pitch, positive = nose down) as +t about Unity's right axis, so
/// pan (left positive) = -twist about up, tilt = -twist about right (the head_tilt joint value, up positive for SOBIT HOME, axis 0 -1 0). This matches
/// the FPV log's head_pan_local_yaw (Unity yaw, i.e. -pan). Lift = link local y minus its zero
/// (the model prefab's local y), shown against the profile's lift range (lift.rangeM). The head gauge ranges are the profile's head limits.
/// </summary>
public class StatusStrip : MonoBehaviour
{
    // Placement relative to the head (metres) and size (mm, canvas units).
    const float Distance = 1.2f, DropM = -0.45f;
    const float WidthMm = 1400f, HeightMm = 130f, RadiusMm = 18f;
    const float NoModelWidthMm = 900f;   // without HEAD and LIFT
    // Blocks mode: the strip hangs where the bar sits (4.3 m), scaled so its text is as large as the
    // bar's body text (the font is sized for Distance and 0.88 x the body size).
    public const float BlocksScale = HudBar.CompactScale * HudUi.ReferenceDistance / (Distance * 0.88f);
    const float Pad = 80f;   // left margin that centres the single row in the wider strip
    const float TextRefreshSeconds = 0.2f;
    const float UnknownRange = 1f;   // gauge range (rad or m) when the profile gives no limit (an asset not yet filled by UrdfModelBuilder)

    QuestControllerPublisher _publisher;
    ImageSubscriber _images;
    RobotModel _model;
    Func<RoundTrip> _rtt;
    int _cameraIndex;

    Transform _pan, _tilt, _lift;
    float _panRange, _tiltMin, _tiltMax, _liftRange;   // from the profile
    float _liftZero;
    bool _liftZeroKnown;

    Image _dot, _pill;
    TextMeshProUGUI _conn, _pillText, _image, _tf, _rttText;
    TextMeshProUGUI _headText, _liftText;
    RectTransform _headDot, _liftFill;
    // HEAD box 160 x 100 mm (dot x = pan, y = tilt), LIFT bar 10 x 72 mm.
    const float HeadBoxW = 160f, HeadBoxH = 100f, LiftBarMm = 72f;
    float _nextText;

    // First person: under the head at the status distance and drop (model != null).
    public static StatusStrip CreateInFirstPerson(Transform head, QuestControllerPublisher publisher, ImageSubscriber images,
                                                  RobotModel model, RobotProfile profile, int cameraIndex, Func<RoundTrip> rtt)
        => Create(head, new Vector3(0f, DropM, Distance), 1f, publisher, images, model, profile, cameraIndex, rtt);

    // `model` may be null (blocks mode): no HEAD / LIFT, "TF —". `scale` multiplies the canvas size.
    public static StatusStrip Create(Transform parent, Vector3 localPosition, float scale, QuestControllerPublisher publisher,
                                     ImageSubscriber images, RobotModel model, RobotProfile profile, int cameraIndex, Func<RoundTrip> rtt)
    {
        if (parent == null) return null;
        var root = HudUi.CreateCanvas("Status Strip", parent, localPosition,
                                      new Vector2(model != null ? WidthMm : NoModelWidthMm, HeightMm), interactive: false);
        root.localScale *= scale;
        root.GetComponent<Canvas>().sortingOrder = HudUi.CanvasSortingOrder + 1;
        var strip = root.gameObject.AddComponent<StatusStrip>();
        strip._publisher = publisher;
        strip._images = images;
        strip._model = model;
        strip._rtt = rtt;
        strip._cameraIndex = cameraIndex;
        strip.FindFrames(profile);
        strip.Build(root);
        return strip;
    }

    public bool HasModel => _model != null;

    void FindFrames(RobotProfile profile)
    {
        if (_model == null) return;
        if (profile == null) return;
        _pan = _model.Frame(profile.head.panFrame);
        _tilt = _model.Frame(profile.head.tiltFrame);
        _lift = _model.Frame(profile.lift.frame);
        _panRange = profile.head.panLimitRad > 0f ? profile.head.panLimitRad : UnknownRange;
        _tiltMax = profile.head.tiltMaxRad > 0f ? profile.head.tiltMaxRad : UnknownRange;
        _tiltMin = profile.head.tiltMinRad < 0f ? -profile.head.tiltMinRad : UnknownRange;   // magnitude of the down limit
        _liftRange = profile.lift.rangeM > 0f ? profile.lift.rangeM : UnknownRange;
        if (_lift != null && profile.modelPrefab != null)
            foreach (var link in profile.modelPrefab.GetComponentsInChildren<RobotLink>(true))
                if (link.frame == profile.lift.frame) { _liftZero = link.transform.localPosition.y; _liftZeroKnown = true; break; }
    }

    TextMeshProUGUI Text(RectTransform root, string name, float left, float width, float font, TextAlignmentOptions align = TextAlignmentOptions.Left)
    {
        var t = HudUi.Label(root, name, "", font, align);
        t.textWrappingMode = TextWrappingModes.NoWrap;
        HudUi.Place(t.rectTransform, left, 0f, width, HeightMm);
        t.verticalAlignment = VerticalAlignmentOptions.Middle;
        return t;
    }

    void Build(RectTransform root)
    {
        float font = HudUi.FontAt(HudUi.BodyFontSize * 0.88f, Distance);
        var bg = HudUi.Round(HudUi.Box(root, "Background", new Color(HudUi.PanelColor.r, HudUi.PanelColor.g, HudUi.PanelColor.b, 0.7f)), RadiusMm);
        HudUi.Stretch(bg.rectTransform);

        // Connection: dot + text.
        _dot = HudUi.Round(HudUi.Box(root, "Connection Dot", HudUi.GoodColor), 7f);
        HudUi.Place(_dot.rectTransform, 14f + Pad, (HeightMm - 14f) / 2f, 14f, 14f);
        _conn = Text(root, "Connection", 34f + Pad, 150f, font);

        // CONTROL ON pill / LAYOUT.
        float pillH = font * 1.5f;
        _pill = HudUi.Round(HudUi.Box(root, "Control Pill", Color.clear), pillH / 2f);
        HudUi.Place(_pill.rectTransform, 192f + Pad, (HeightMm - pillH) / 2f, 135f, pillH);
        _pillText = HudUi.Label(_pill.transform, "Label", "", font);
        _pillText.fontStyle = FontStyles.Bold;
        _pillText.textWrappingMode = TextWrappingModes.NoWrap;
        // Bold "CONTROL ON" is wider than the pill at the strip's font: shrink to fit
        // (it overflowed into the image fps text).
        _pillText.enableAutoSizing = true;
        _pillText.fontSizeMax = font;
        _pillText.fontSizeMin = font * 0.5f;
        _pillText.margin = new Vector4(8f, 0f, 8f, 0f);
        HudUi.Stretch(_pillText.rectTransform);

        _image = Text(root, "Image", 337f + Pad, 170f, font);
        _tf = Text(root, "TF", 515f + Pad, 95f, font);
        _rttText = Text(root, "RTT", 618f + Pad, 100f, font);

        // HEAD: labelled box with a crosshair; the dot is where the head looks (x = pan, left to the
        // left; y = tilt, up is up), with tiny L / R ticks and the angles in muted text beside it.
        float small = font;   // secondary texts: same size as the body font
        float x = 730f + Pad;
        var headLabel = Text(root, "Head Label", x, HeadBoxW, small, TextAlignmentOptions.Center);
        headLabel.text = "HEAD";
        headLabel.color = HudUi.MutedText;
        HudUi.Place(headLabel.rectTransform, x, 2f, HeadBoxW, small * 1.2f);
        var box = HudUi.Round(HudUi.Box(root, "Head Box", HudUi.ControlColor), 8f);
        float boxTop = HeightMm - HeadBoxH - 4f;
        HudUi.Place(box.rectTransform, x, boxTop, HeadBoxW, HeadBoxH);
        foreach (var size in new[] { new Vector2(HeadBoxW - 10f, 2f), new Vector2(2f, HeadBoxH - 10f) })
        {
            var cross = HudUi.Box(box.transform, "Crosshair", new Color(1f, 1f, 1f, 0.3f));
            cross.rectTransform.anchorMin = cross.rectTransform.anchorMax = cross.rectTransform.pivot = new Vector2(0.5f, 0.5f);
            cross.rectTransform.sizeDelta = size;
            cross.rectTransform.anchoredPosition = Vector2.zero;
        }
        _headDot = Marker(box.transform, "Head Dot", new Vector2(14f, 14f));
        foreach (bool leftTick in new[] { true, false })
        {
            var tick = HudUi.Label(box.transform, leftTick ? "L" : "R", leftTick ? "L" : "R", small, leftTick ? TextAlignmentOptions.Left : TextAlignmentOptions.Right);
            tick.color = HudUi.MutedText;
            tick.textWrappingMode = TextWrappingModes.NoWrap;
            tick.rectTransform.anchorMin = tick.rectTransform.anchorMax = tick.rectTransform.pivot = new Vector2(leftTick ? 0f : 1f, 0.5f);
            tick.rectTransform.sizeDelta = new Vector2(small * 1.4f, small * 1.4f);
            tick.rectTransform.anchoredPosition = new Vector2(leftTick ? 4f : -4f, 0f);
        }
        _headText = Text(root, "Head Values", x + HeadBoxW + 8f, 115f, small);
        _headText.color = HudUi.MutedText;
        _headText.textWrappingMode = TextWrappingModes.Normal;   // "pan 29°" over "tilt 17°"
        _headText.verticalAlignment = VerticalAlignmentOptions.Middle;

        // LIFT: labelled vertical bar, fill from the bottom, height beside it.
        float lx = x + HeadBoxW + 8f + 125f;
        var liftLabel = Text(root, "Lift Label", lx, 60f, small, TextAlignmentOptions.Left);
        liftLabel.text = "LIFT";
        liftLabel.color = HudUi.MutedText;
        HudUi.Place(liftLabel.rectTransform, lx, 2f, 80f, small * 1.2f);
        var liftTrack = HudUi.Round(HudUi.Box(root, "Lift Track", HudUi.ControlColor), 4f);
        HudUi.Place(liftTrack.rectTransform, lx + 4f, HeightMm - LiftBarMm - 4f, 10f, LiftBarMm);
        var fill = HudUi.Round(HudUi.Box(liftTrack.transform, "Lift Fill", HudUi.AccentColor), 4f);
        _liftFill = fill.rectTransform;
        _liftFill.anchorMin = Vector2.zero;
        _liftFill.anchorMax = new Vector2(1f, 0f);
        _liftFill.pivot = new Vector2(0.5f, 0f);
        _liftFill.offsetMin = _liftFill.offsetMax = Vector2.zero;
        _liftFill.sizeDelta = new Vector2(0f, 0f);
        _liftText = Text(root, "Lift Value", lx + 22f, 100f, small);
        _liftText.color = HudUi.MutedText;

        bool hasHead = _pan != null || _tilt != null;
        headLabel.gameObject.SetActive(hasHead);
        box.gameObject.SetActive(hasHead);
        _headText.gameObject.SetActive(hasHead);
        liftLabel.gameObject.SetActive(_lift != null);
        liftTrack.gameObject.SetActive(_lift != null);
        _liftText.gameObject.SetActive(_lift != null);

        UpdateTexts();
    }

    static RectTransform Marker(Transform track, string name, Vector2 size)
    {
        var m = HudUi.Round(HudUi.Box(track, name, HudUi.AccentColor), 3f);
        var rt = m.rectTransform;
        rt.anchorMin = rt.anchorMax = rt.pivot = new Vector2(0.5f, 0.5f);
        rt.sizeDelta = size;
        rt.anchoredPosition = Vector2.zero;
        return rt;
    }

    // Twist angle (rad, -pi..pi) of a rotation about a principal axis; 0 = x, 1 = y.
    static float Twist(Quaternion q, int axis)
    {
        float w = q.w, c = axis == 0 ? q.x : q.y;
        if (w < 0f) { w = -w; c = -c; }
        return 2f * Mathf.Atan2(c, w);
    }

    // Head pan (rad, left positive, as in ROS) and tilt (rad = head_tilt joint value, up positive).
    public float PanRad => _pan != null ? -Twist(_pan.localRotation, 1) : 0f;
    public float TiltRad => _tilt != null ? -Twist(_tilt.localRotation, 0) : 0f;
    // Gauge ranges in use: pan +- PanRangeRad, tilt -TiltDownRad..TiltUpRad (rad), lift 0..LiftRangeM (m).
    public float PanRangeRad => _panRange;
    public float TiltUpRad => _tiltMax;
    public float TiltDownRad => _tiltMin;
    public float LiftRangeM => _liftRange;
    // Lift height above its zero (m).
    public float LiftM => _lift != null && _liftZeroKnown ? _lift.localPosition.y - _liftZero : 0f;

    void Update()
    {
        if (_lift != null && !_liftZeroKnown)
        {
            // No prefab value (should not happen): take the first pose seen as zero.
            _liftZero = _lift.localPosition.y;
            _liftZeroKnown = true;
        }
        // Pan left -> dot left (x = -pan), tilt up -> dot up (y = +tilt).
        _headDot.anchoredPosition = new Vector2(
            _pan != null ? Mathf.Clamp(-PanRad / _panRange, -1f, 1f) * (HeadBoxW / 2f - 12f) : 0f,
            _tilt != null ? Mathf.Clamp(TiltRad / (TiltRad >= 0f ? _tiltMax : _tiltMin), -1f, 1f) * (HeadBoxH / 2f - 12f) : 0f);
        if (_lift != null) _liftFill.sizeDelta = new Vector2(0f, Mathf.Clamp01(LiftM / _liftRange) * LiftBarMm);

        if (Time.unscaledTime < _nextText) return;
        _nextText = Time.unscaledTime + TextRefreshSeconds;
        UpdateTexts();
    }

    void UpdateTexts()
    {
        if (_publisher == null) return;
        bool connected = !_publisher.HasConnectionError;
        var c = connected ? HudUi.GoodColor : HudUi.BadColor;
        _dot.color = c;
        _conn.color = c;
        _conn.text = connected ? "connected" : "not connected";

        bool control = _publisher.controlRobot;
        var pc = control ? HudUi.BadColor : HudUi.MutedText;
        _pill.color = control ? new Color(pc.r, pc.g, pc.b, 0.25f) : Color.clear;
        _pillText.color = pc;
        _pillText.text = control ? "CONTROL ON" : "LAYOUT";

        double last = _images != null ? _images.LastFrameTime(_cameraIndex) : -1.0;
        if (_cameraIndex < 0) _image.text = "";
        else if (last < 0.0)
        {
            _image.text = "no image";
            _image.color = HudUi.WarnColor;
        }
        else
        {
            float age = (float)(Time.unscaledTime - last);
            _image.text = $"{_images.Fps(_cameraIndex):F0} fps · {age:F2} s";
            _image.color = age > 1f ? HudUi.WarnColor : Color.white;
        }

        _headText.text = $"pan {PanRad * Mathf.Rad2Deg:F0}\u00B0\ntilt {TiltRad * Mathf.Rad2Deg:F0}\u00B0";
        _liftText.text = $"{LiftM:F2} m";
        _tf.text = _model != null ? $"TF {_model.TfHz:F0} Hz" : "TF —";

        var rtt = _rtt != null ? _rtt() : null;
        bool haveRtt = rtt != null && control && rtt.HasRecent;
        _rttText.text = haveRtt ? $"RTT {rtt.RttMs:F0} ms" : "RTT —";
        _rttText.color = haveRtt ? Color.white : HudUi.MutedText;
    }
}
