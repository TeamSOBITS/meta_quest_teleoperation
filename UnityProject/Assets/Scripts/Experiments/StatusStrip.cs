using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Small head-locked status strip at the bottom of the first-person view (the HUD bar is hidden
/// there). One row: connection, CONTROL ON / LAYOUT, image fps and age, TF rate, round trip,
/// head pan/tilt gauge and lift height bar. Created by TeleopHud in first person while the "status"
/// experiment is on, destroyed when either turns off.
///
/// Head and lift are read from the model's link local poses, which RobotModel sets straight from
/// /tf (FLU -> Unity). A ROS rotation of t about z shows up as -t about Unity's up axis, and a
/// rotation of t about y (pitch, positive = nose down) as +t about Unity's right axis, so
/// pan (left positive) = -twist about up, tilt = -twist about right (the head_tilt joint value, up positive for SOBIT HOME, axis 0 -1 0). This matches
/// the FPV log's head_pan_local_yaw (Unity yaw, i.e. -pan). Lift = link local y minus its zero
/// (the model prefab's local y), shown against the joint range 0..0.69 m.
/// </summary>
public class StatusStrip : MonoBehaviour
{
    // Placement relative to the head (metres) and size (mm, canvas units).
    const float Distance = 1.2f, DropM = -0.45f;
    const float WidthMm = 900f, HeightMm = 80f, RadiusMm = 18f;
    const float TextRefreshSeconds = 0.2f;
    const float GaugeRangeRad = 0.785f, LiftRangeM = 0.69f;
    const string TiltFrame = "head_tilt_link", LiftFrame = "body_lift_link";

    QuestControllerPublisher _publisher;
    ImageSubscriber _images;
    RobotModel _model;
    Func<RoundTrip> _rtt;
    int _cameraIndex;

    Transform _pan, _tilt, _lift;
    float _liftZero;
    bool _liftZeroKnown;

    Image _dot, _pill;
    TextMeshProUGUI _conn, _pillText, _image, _tf, _rttText;
    RectTransform _panMarker, _tiltMarker, _liftFill;
    const float PanHalfMm = 50f, TiltHalfMm = 22f, LiftBarMm = 56f;
    float _nextText;

    public static StatusStrip Create(Transform head, QuestControllerPublisher publisher, ImageSubscriber images,
                                     RobotModel model, RobotProfile profile, int cameraIndex, Func<RoundTrip> rtt)
    {
        if (head == null) return null;
        var root = HudUi.CreateCanvas("Status Strip", head, new Vector3(0f, DropM, Distance),
                                      new Vector2(WidthMm, HeightMm), interactive: false);
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

    void FindFrames(RobotProfile profile)
    {
        if (_model == null) return;
        _pan = _model.Frame(profile != null ? profile.panFrame : null);
        _tilt = _model.Frame(TiltFrame);
        _lift = _model.Frame(LiftFrame);
        if (_lift != null && profile != null && profile.modelPrefab != null)
            foreach (var link in profile.modelPrefab.GetComponentsInChildren<RobotLink>(true))
                if (link.frame == LiftFrame) { _liftZero = link.transform.localPosition.y; _liftZeroKnown = true; break; }
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
        HudUi.Place(_dot.rectTransform, 14f, (HeightMm - 14f) / 2f, 14f, 14f);
        _conn = Text(root, "Connection", 34f, 150f, font);

        // CONTROL ON pill / LAYOUT.
        float pillH = font * 1.5f;
        _pill = HudUi.Round(HudUi.Box(root, "Control Pill", Color.clear), pillH / 2f);
        HudUi.Place(_pill.rectTransform, 192f, (HeightMm - pillH) / 2f, 135f, pillH);
        _pillText = HudUi.Label(_pill.transform, "Label", "", font);
        _pillText.fontStyle = FontStyles.Bold;
        _pillText.textWrappingMode = TextWrappingModes.NoWrap;
        // Bold "CONTROL ON" / "HOLD GRIP" is wider than the pill at the strip's font: shrink to fit
        // (it overflowed into the image fps text).
        _pillText.enableAutoSizing = true;
        _pillText.fontSizeMax = font;
        _pillText.fontSizeMin = font * 0.5f;
        _pillText.margin = new Vector4(8f, 0f, 8f, 0f);
        HudUi.Stretch(_pillText.rectTransform);

        _image = Text(root, "Image", 337f, 170f, font);
        _tf = Text(root, "TF", 515f, 95f, font);
        _rttText = Text(root, "RTT", 618f, 100f, font);

        // Head pan (horizontal track, 120 deg wide in total) and tilt (vertical track).
        var panTrack = HudUi.Round(HudUi.Box(root, "Pan Track", HudUi.ControlColor), 3f);
        HudUi.Place(panTrack.rectTransform, 730f, HeightMm / 2f - 3f, 2f * PanHalfMm, 6f);
        _panMarker = Marker(panTrack.transform, "Pan Marker", new Vector2(6f, 26f));
        var centre = HudUi.Box(panTrack.transform, "Pan Centre", new Color(1f, 1f, 1f, 0.35f));
        centre.rectTransform.anchorMin = centre.rectTransform.anchorMax = new Vector2(0.5f, 0.5f);
        centre.rectTransform.sizeDelta = new Vector2(2f, 14f);
        centre.rectTransform.SetAsFirstSibling();

        var tiltTrack = HudUi.Round(HudUi.Box(root, "Tilt Track", HudUi.ControlColor), 3f);
        HudUi.Place(tiltTrack.rectTransform, 838f, HeightMm / 2f - TiltHalfMm, 6f, 2f * TiltHalfMm);
        _tiltMarker = Marker(tiltTrack.transform, "Tilt Marker", new Vector2(18f, 6f));

        // Lift: thin vertical bar, fill from the bottom.
        var liftTrack = HudUi.Round(HudUi.Box(root, "Lift Track", HudUi.ControlColor), 4f);
        HudUi.Place(liftTrack.rectTransform, 858f, (HeightMm - LiftBarMm) / 2f, 8f, LiftBarMm);
        var fill = HudUi.Round(HudUi.Box(liftTrack.transform, "Lift Fill", HudUi.AccentColor), 4f);
        _liftFill = fill.rectTransform;
        _liftFill.anchorMin = Vector2.zero;
        _liftFill.anchorMax = new Vector2(1f, 0f);
        _liftFill.pivot = new Vector2(0.5f, 0f);
        _liftFill.offsetMin = _liftFill.offsetMax = Vector2.zero;
        _liftFill.sizeDelta = new Vector2(0f, 0f);

        _panMarker.gameObject.SetActive(_pan != null);
        _tiltMarker.gameObject.SetActive(_tilt != null);
        liftTrack.gameObject.SetActive(_lift != null);
        tiltTrack.gameObject.SetActive(_tilt != null);
        panTrack.gameObject.SetActive(_pan != null);

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
        // Pan left -> marker left (x = -pan), tilt up -> marker up (y = +tilt).
        if (_pan != null) _panMarker.anchoredPosition = new Vector2(Mathf.Clamp(-PanRad / GaugeRangeRad, -1f, 1f) * PanHalfMm, 0f);
        if (_tilt != null) _tiltMarker.anchoredPosition = new Vector2(0f, Mathf.Clamp(TiltRad / GaugeRangeRad, -1f, 1f) * TiltHalfMm);
        if (_lift != null) _liftFill.sizeDelta = new Vector2(0f, Mathf.Clamp01(LiftM / LiftRangeM) * LiftBarMm);

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
        bool hold = control && _publisher.deadmanEnabled && !_publisher.DeadmanHeld;
        var pc = hold ? HudUi.WarnColor : control ? HudUi.BadColor : HudUi.MutedText;
        _pill.color = control ? new Color(pc.r, pc.g, pc.b, 0.25f) : Color.clear;
        _pillText.color = pc;
        _pillText.text = hold ? "HOLD GRIP" : control ? "CONTROL ON" : "LAYOUT";

        double last = _images != null ? _images.LastFrameTime(_cameraIndex) : -1.0;
        if (last < 0.0)
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

        _tf.text = _model != null ? $"TF {_model.TfHz:F0} Hz" : "TF —";

        var rtt = _rtt != null ? _rtt() : null;
        bool haveRtt = rtt != null && control && rtt.HasRecent;
        _rttText.text = haveRtt ? $"RTT {rtt.RttMs:F0} ms" : "RTT —";
        _rttText.color = haveRtt ? Color.white : HudUi.MutedText;
    }
}
