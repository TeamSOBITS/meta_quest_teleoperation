using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

// The strip's REC group (recorder status, see RecordStatus / RecordStatusRule): a dot (pulsing while recording) and
// the bold state "REC 00:01:05" in the state colour over a muted "task: pick cup" ("no task" in Warn). Built the
// first time the recorder is heard and shown only while it is (the strip is as wide as before otherwise); it sits after
// CONTROL, before BODY. Its divider is named "Record Divider", not "Group Divider", so the group count stays three
// (first person) / two (blocks) without a recorder. The rest of the strip is in StatusStrip.cs.
public partial class StatusStrip
{
    const float RecordTaskRatio = 0.85f;   // task line font relative to the strip font
    const float LineRatio = 1.2f;          // line height / font

    Func<RecordStatus> _record;
    RectTransform _recGroup;
    float _recWidth;
    Image _recDot;
    TextMeshProUGUI _recLabel, _recTask;

    public bool RecordShown => _recGroup != null && _recGroup.gameObject.activeSelf;
    // Width the REC group adds to the strip while shown (mm, divider included).
    public float RecordWidthMm => _recWidth;

    void BuildRecord()
    {
        _recGroup = Group(_root, "Record Group");
        float x = Divider(_recGroup, 0f, "Record Divider");
        float small = _font * RecordTaskRatio;
        float lineH = _font * LineRatio, smallH = small * LineRatio;
        float top = (HeightMm - lineH - smallH) / 2f;

        _recDot = HudUi.Round(HudUi.Box(_recGroup, "Record Dot", HudTheme.Record), DotMm / 2f);
        HudUi.Place(_recDot.rectTransform, x, top + (lineH - DotMm) / 2f, DotMm, DotMm);
        x += DotMm + GapMm;

        _recLabel = Line(_recGroup, "Record Label", _font);
        _recLabel.fontStyle = FontStyles.Bold;
        // As wide as the longest state (digits have one width): the group keeps its width while the time runs.
        float colW = Mathf.Ceil(Mathf.Max(_recLabel.GetPreferredValues(RecordStatusRule.Label(RecordStatusRule.Look.Paused, 0f)).x,
                                          _recLabel.GetPreferredValues(RecordStatusRule.Label(RecordStatusRule.Look.Recording, 0f)).x)) + 4f;
        HudUi.Place(_recLabel.rectTransform, x, top, colW, lineH);
        _recTask = Line(_recGroup, "Record Task", small);
        _recTask.overflowMode = TextOverflowModes.Ellipsis;   // long task names are cut, the group does not grow
        HudUi.Place(_recTask.rectTransform, x, top + lineH, colW, smallH);
        _recWidth = x + colW;
        _recGroup.gameObject.SetActive(false);
    }

    static TextMeshProUGUI Line(RectTransform parent, string name, float font)
    {
        var t = HudUi.Label(parent, name, "", font, TextAlignmentOptions.Left);
        t.textWrappingMode = TextWrappingModes.NoWrap;
        t.verticalAlignment = VerticalAlignmentOptions.Middle;
        return t;
    }

    // Every frame: show / hide with the feed (the strip resizes), pulse the dot while recording.
    void UpdateRecord()
    {
        var rs = _record?.Invoke();
        bool show = rs != null && rs.Available;
        if (show && _recGroup == null) BuildRecord();
        if (_recGroup == null) return;
        if (show != _recGroup.gameObject.activeSelf)
        {
            _recGroup.gameObject.SetActive(show);
            Layout();
            if (show) UpdateRecordTexts();
        }
        if (!show) return;
        var look = rs.Look;
        _recDot.color = HudTheme.WithAlpha(RecordStatusRule.Colour(look),
                                           look == RecordStatusRule.Look.Recording ? RecordStatusRule.PulseAlpha(Time.unscaledTime) : 1f);
    }

    // At the strip's text rate (and when the group appears).
    void UpdateRecordTexts()
    {
        if (!RecordShown) return;
        var rs = _record?.Invoke();
        if (rs == null) return;
        var look = rs.Look;
        var c = RecordStatusRule.Colour(look);
        _recLabel.text = RecordStatusRule.Label(look, rs.ElapsedS);
        _recLabel.color = c;
        _recTask.text = rs.TaskSet ? "task: " + (rs.TaskName.Length > 0 ? rs.TaskName : "set") : "no task";
        _recTask.color = rs.TaskSet ? HudTheme.Muted : HudTheme.Warn;
    }
}
