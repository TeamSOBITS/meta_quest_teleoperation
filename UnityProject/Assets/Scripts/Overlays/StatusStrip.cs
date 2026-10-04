using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Small head-locked status strip shown while the HUD bar (menu) is hidden, in both view modes:
/// at the bottom of the first-person view, or in blocks mode where the bar normally sits. One row in three
/// groups with thin dividers: LINK (the <see cref="LinkHealth"/> level "link ok / degraded / lost", image fps and age,
/// TF rate, round trip), CONTROL (CONTROL ON / LAYOUT pill) and BODY (a labelled HEAD box: crosshair, dot at
/// pan x / tilt y, L/R ticks, "pan 29° tilt 17°"; a labelled LIFT bar with its height "0.40 m"). Only the item that
/// needs attention is coloured (Warn / Bad), the rest is muted. Created by TeleopHud in both camera layouts
/// (active only while the bar is hidden) and rebuilt when the layout changes.
/// In the blocks layout there is no model reading: the BODY group (HEAD, LIFT) and the TF rate are left out ("TF —") and the strip is narrower.
/// While a recorder publishes its status, a REC group sits after CONTROL (see StatusStrip.Record.cs); the strip grows only then.
///
/// Head and lift are read from the model's link local poses, which RobotModel sets straight from
/// /tf (FLU -> Unity). A ROS rotation of t about z shows up as -t about Unity's up axis, and a
/// rotation of t about y (pitch, positive = nose down) as +t about Unity's right axis, so
/// pan (left positive) = -twist about up, tilt = -twist about right (the head_tilt joint value, up positive for SOBIT HOME, axis 0 -1 0). This matches
/// the FPV log's head_pan_local_yaw (Unity yaw, i.e. -pan). Lift = link local y minus its zero
/// (the model prefab's local y), shown against the profile's lift range (lift.rangeM). The head gauge ranges are the profile's head limits.
/// </summary>
public partial class StatusStrip : MonoBehaviour
{
    // Placement relative to the head (metres) and size (mm, canvas units).
    const float Distance = 1.2f, DropM = -0.45f;
    const float HeightMm = 130f, RadiusMm = 18f;
    const float BackgroundAlpha = 0.7f;   // the strip is more see-through than a panel
    const float DotMm = 14f, HeadBoxRadiusMm = 8f, TrackRadiusMm = 4f, MarkerRadiusMm = 3f;
    static readonly Color CrosshairColor = new Color(1f, 1f, 1f, 0.3f);
    // Text: the body font; one character is about CharWidth x the font wide. The strip is as wide as its
    // groups need (about 1500 mm with the model, less without HEAD and LIFT).
    const float FontScale = 1f, CharWidth = 0.56f;
    const float EdgeMm = 40f, GapMm = 12f, GroupGapMm = 26f, DividerMm = 3f, DividerHeightRatio = 0.6f;
    // Blocks mode: the strip hangs where the bar sits (4.3 m), scaled so its text is as large as the
    // bar's body text (the font is sized for Distance and FontScale x the body size).
    public const float BlocksScale = HudBar.CompactScale * HudTheme.ReferenceDistance / (Distance * FontScale);
    const float TextRefreshSeconds = 0.2f;
    const float UnknownRange = 1f;   // gauge range (rad or m) when the profile gives no limit (an asset not yet filled by UrdfModelBuilder)

    QuestControllerPublisher _publisher;
    ImageSubscriber _images;
    RobotModel _model;
    Func<RoundTrip> _rtt;
    int _cameraIndex;
    LinkHealth _link;

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
    float _font, _controlEnd, _bodyWidth;   // the strip's font; where CONTROL ends and how wide BODY is (mm)
    RectTransform _root, _bodyGroup;

    // First person: under the head at the status distance and drop (model != null).
    public static StatusStrip CreateInFirstPerson(Transform head, QuestControllerPublisher publisher, ImageSubscriber images,
                                                  RobotModel model, RobotProfile profile, int cameraIndex, Func<RoundTrip> rtt,
                                                  Func<RecordStatus> record = null)
        => Create(head, new Vector3(0f, DropM, Distance), 1f, publisher, images, model, profile, cameraIndex, rtt, record);

    // `model` may be null (blocks mode): no HEAD / LIFT, "TF —". `scale` multiplies the canvas size. `record`: the recorder
    // status feed (null = never a REC group).
    public static StatusStrip Create(Transform parent, Vector3 localPosition, float scale, QuestControllerPublisher publisher,
                                     ImageSubscriber images, RobotModel model, RobotProfile profile, int cameraIndex, Func<RoundTrip> rtt,
                                     Func<RecordStatus> record = null)
    {
        if (parent == null) return null;
        var root = HudUi.CreateCanvas("Status Strip", parent, localPosition, new Vector2(1000f, HeightMm), interactive: false);   // width set by Build
        root.localScale *= scale;
        root.GetComponent<Canvas>().sortingOrder = HudTheme.SortingOrder + 1;
        var strip = root.gameObject.AddComponent<StatusStrip>();
        strip._publisher = publisher;
        strip._images = images;
        strip._model = model;
        strip._rtt = rtt;
        strip._record = record;
        strip._cameraIndex = cameraIndex;
        strip._link = new LinkHealth(images, cameraIndex, rtt);
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

    // Three groups with thin dividers: LINK (connection level, image fps / age, TF rate, round trip),
    // CONTROL (state pill) and, with a model, BODY (HEAD box, LIFT bar). Only the item that needs attention is coloured.
    // BODY is built in its own "Body Group" so the REC group can be slotted in before it (Layout).
    void Build(RectTransform root)
    {
        _root = root;
        float font = _font = HudUi.FontAt(HudTheme.BodyFont * FontScale, Distance);
        float cw = font * CharWidth;
        var bg = HudUi.Round(HudUi.Box(root, "Background", HudTheme.WithAlpha(HudTheme.Panel, BackgroundAlpha)), RadiusMm);
        HudUi.Stretch(bg.rectTransform);

        // LINK: level dot + text, image, TF, round trip.
        float x = EdgeMm;
        _dot = HudUi.Round(HudUi.Box(root, "Connection Dot", HudTheme.Good), DotMm / 2f);
        HudUi.Place(_dot.rectTransform, x, (HeightMm - DotMm) / 2f, DotMm, DotMm);
        x += DotMm + GapMm;
        _conn = Text(root, "Connection", x, 8f * cw, font); x += 8f * cw + GapMm;
        _image = Text(root, "Image", x, 15f * cw, font); x += 15f * cw + GapMm;
        _tf = Text(root, "TF", x, 9f * cw, font); x += 9f * cw + GapMm;
        _rttText = Text(root, "RTT", x, 9f * cw, font); x += 9f * cw;

        // CONTROL: CONTROL ON pill / LAYOUT.
        x = Divider(root, x);
        float pillH = font * 1.5f, pillW = 10.5f * cw;
        (_pill, _pillText) = HudUi.Pill(root, "Control Pill", "", font, pillH, Color.clear, Color.white, bold: true);
        HudUi.Place(_pill.rectTransform, x, (HeightMm - pillH) / 2f, pillW, pillH);
        // Bold "CONTROL ON" is wider than the pill at the strip's font: shrink to fit.
        _pillText.enableAutoSizing = true;
        _pillText.fontSizeMax = font;
        _pillText.fontSizeMin = font * 0.5f;
        _pillText.margin = new Vector4(8f, 0f, 8f, 0f);
        x += pillW;
        _controlEnd = x;

        if (_model != null)
        {
            _bodyGroup = Group(root, "Body Group");
            _bodyWidth = BuildBody(_bodyGroup, Divider(_bodyGroup, 0f), font, cw);
        }
        Layout();
        UpdateTexts();
    }

    // An empty full-height container for a group; Layout places it.
    static RectTransform Group(RectTransform root, string name)
    {
        var rt = (RectTransform)new GameObject(name, typeof(RectTransform)).transform;
        rt.SetParent(root, false);
        return rt;
    }

    // Groups after CONTROL from left to right: REC (while shown), BODY; the strip is as wide as they need.
    void Layout()
    {
        float x = _controlEnd;
        if (RecordShown) { HudUi.Place(_recGroup, x, 0f, _recWidth, HeightMm); x += _recWidth; }
        if (_bodyGroup != null) { HudUi.Place(_bodyGroup, x, 0f, _bodyWidth, HeightMm); x += _bodyWidth; }
        _root.sizeDelta = new Vector2(x + EdgeMm, HeightMm);
    }

    // Thin vertical divider between groups; returns the x where the next group starts.
    static float Divider(RectTransform root, float x, string name = "Group Divider")
    {
        x += GroupGapMm;
        var line = HudUi.Box(root, name, HudTheme.Divider);
        float h = HeightMm * DividerHeightRatio;
        HudUi.Place(line.rectTransform, x, (HeightMm - h) / 2f, DividerMm, h);
        return x + DividerMm + GroupGapMm;
    }

    // BODY: HEAD box with a crosshair (the dot is where the head looks, x = pan, left to the left; y = tilt,
    // up is up) with tiny L / R ticks and the angles beside it, then the LIFT bar (fill from the bottom) and its
    // height. Returns the right end.
    float BuildBody(RectTransform root, float x, float font, float cw)
    {
        float small = font;   // secondary texts: same size as the body font
        var headLabel = Text(root, "Head Label", x, HeadBoxW, small, TextAlignmentOptions.Center);
        headLabel.text = "HEAD";
        headLabel.color = HudTheme.Muted;
        HudUi.Place(headLabel.rectTransform, x, 2f, HeadBoxW, small * 1.2f);
        var box = HudUi.Round(HudUi.Box(root, "Head Box", HudTheme.Control), HeadBoxRadiusMm);
        float boxTop = HeightMm - HeadBoxH - 4f;
        HudUi.Place(box.rectTransform, x, boxTop, HeadBoxW, HeadBoxH);
        foreach (var size in new[] { new Vector2(HeadBoxW - 10f, 2f), new Vector2(2f, HeadBoxH - 10f) })
        {
            var cross = HudUi.Box(box.transform, "Crosshair", CrosshairColor);
            cross.rectTransform.anchorMin = cross.rectTransform.anchorMax = cross.rectTransform.pivot = new Vector2(0.5f, 0.5f);
            cross.rectTransform.sizeDelta = size;
            cross.rectTransform.anchoredPosition = Vector2.zero;
        }
        _headDot = Marker(box.transform, "Head Dot", new Vector2(14f, 14f));
        foreach (bool leftTick in new[] { true, false })
        {
            var tick = HudUi.Label(box.transform, leftTick ? "L" : "R", leftTick ? "L" : "R", small, leftTick ? TextAlignmentOptions.Left : TextAlignmentOptions.Right);
            tick.color = HudTheme.Muted;
            tick.textWrappingMode = TextWrappingModes.NoWrap;
            tick.rectTransform.anchorMin = tick.rectTransform.anchorMax = tick.rectTransform.pivot = new Vector2(leftTick ? 0f : 1f, 0.5f);
            tick.rectTransform.sizeDelta = new Vector2(small * 1.4f, small * 1.4f);
            tick.rectTransform.anchoredPosition = new Vector2(leftTick ? 4f : -4f, 0f);
        }
        float valuesW = 8f * cw;
        _headText = Text(root, "Head Values", x + HeadBoxW + 8f, valuesW, small);
        _headText.color = HudTheme.Muted;
        _headText.textWrappingMode = TextWrappingModes.Normal;   // "pan 29°" over "tilt 17°"
        _headText.verticalAlignment = VerticalAlignmentOptions.Middle;
        x += HeadBoxW + 8f + valuesW + GapMm;

        float lx = x;
        var liftLabel = Text(root, "Lift Label", lx, 60f, small, TextAlignmentOptions.Left);
        liftLabel.text = "LIFT";
        liftLabel.color = HudTheme.Muted;
        HudUi.Place(liftLabel.rectTransform, lx, 2f, 4f * cw + 10f, small * 1.2f);
        var liftTrack = HudUi.Round(HudUi.Box(root, "Lift Track", HudTheme.Control), TrackRadiusMm);
        HudUi.Place(liftTrack.rectTransform, lx + 4f, HeightMm - LiftBarMm - 4f, 10f, LiftBarMm);
        var fill = HudUi.Round(HudUi.Box(liftTrack.transform, "Lift Fill", HudTheme.Accent), TrackRadiusMm);
        _liftFill = fill.rectTransform;
        _liftFill.anchorMin = Vector2.zero;
        _liftFill.anchorMax = new Vector2(1f, 0f);
        _liftFill.pivot = new Vector2(0.5f, 0f);
        _liftFill.offsetMin = _liftFill.offsetMax = Vector2.zero;
        _liftFill.sizeDelta = new Vector2(0f, 0f);
        float liftW = 7f * cw;
        _liftText = Text(root, "Lift Value", lx + 22f, liftW, small);
        _liftText.color = HudTheme.Muted;

        bool hasHead = _pan != null || _tilt != null;
        headLabel.gameObject.SetActive(hasHead);
        box.gameObject.SetActive(hasHead);
        _headText.gameObject.SetActive(hasHead);
        liftLabel.gameObject.SetActive(_lift != null);
        liftTrack.gameObject.SetActive(_lift != null);
        _liftText.gameObject.SetActive(_lift != null);
        return lx + 22f + liftW;
    }

    static RectTransform Marker(Transform track, string name, Vector2 size)
    {
        var m = HudUi.Round(HudUi.Box(track, name, HudTheme.Accent), MarkerRadiusMm);
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
        UpdateRecord();
        if (_headDot == null) { Refresh(); return; }   // no model: no gauges
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

        Refresh();
    }

    void Refresh()
    {
        if (Time.unscaledTime < _nextText) return;
        _nextText = Time.unscaledTime + TextRefreshSeconds;
        UpdateTexts();
        UpdateRecordTexts();
    }

    void UpdateTexts()
    {
        if (_publisher == null) return;
        // LINK: the dot always shows the level; the texts are muted unless they are what needs attention.
        var level = _link.Current;
        var levelColour = LinkHealth.Colour(level);
        _dot.color = levelColour;
        _conn.color = level == LinkHealth.Level.Good ? HudTheme.Muted : levelColour;
        _conn.text = LinkHealth.Text(level);

        bool control = _publisher.controlRobot;
        var pc = control ? HudTheme.Bad : HudTheme.Muted;
        _pill.color = control ? HudTheme.PillBackground(pc, strong: true) : Color.clear;
        _pillText.color = pc;
        _pillText.text = control ? "CONTROL ON" : "LAYOUT";

        float age = _link.AgeS;
        double last = _images != null ? _images.LastFrameTime(_cameraIndex) : -1.0;
        if (_cameraIndex < 0) _image.text = "";
        else if (last < 0.0)
        {
            _image.text = "no image";
            _image.color = HudTheme.Warn;
        }
        else
        {
            float shown = (float)(Time.unscaledTime - last);   // also while the camera is off
            _image.text = $"{_images.Fps(_cameraIndex):F0} fps \u00B7 {shown:F2} s";
            _image.color = age >= LinkHealth.LostAgeS ? HudTheme.Bad : age > 1f ? HudTheme.Warn : HudTheme.Muted;
        }

        if (_headText != null) _headText.text = $"pan {PanRad * Mathf.Rad2Deg:F0}\u00B0\ntilt {TiltRad * Mathf.Rad2Deg:F0}\u00B0";
        if (_liftText != null) _liftText.text = $"{LiftM:F2} m";
        _tf.text = _model != null ? $"TF {_model.TfHz:F0} Hz" : "TF \u2014";
        _tf.color = HudTheme.Muted;

        float rttMs = _link.RttMs;
        bool haveRtt = control && rttMs >= 0f;
        _rttText.text = haveRtt ? $"RTT {rttMs:F0} ms" : "RTT \u2014";
        _rttText.color = !haveRtt ? HudTheme.Muted : rttMs >= 400f ? HudTheme.Bad : rttMs >= LinkHealth.GoodRttMs ? HudTheme.Warn : HudTheme.Muted;
    }
}
