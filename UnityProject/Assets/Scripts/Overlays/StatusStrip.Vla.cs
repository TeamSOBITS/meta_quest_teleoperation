using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

// The strip's VLA group (VLA stage status, see VlaStatus / VlaStatusRule), after CONTROL, before BODY:
//   collection   dot (pulsing while recording) | bold state "REC 00:01:05" over a muted "task: pick cup" ("no task" in Warn)
//   deploy       dot (pulsing while the policy drives) | bold "PLAY 00:00:12" over "pick cup · smolvla_fft"
//                | a GRIP chip ("GRIP driving" Good / "GRIP released" Warn, like the CONTROL pill; only while the deadman
//                  is enabled and the policy plays) over a muted "240 steps · 8.1 Hz"
// Every column has a fixed width from its widest text (digits share one width), so the group keeps its width while
// the time runs and the grip toggles; it changes only with the stage. Built the first time a stage node is heard and
// shown only while it is (the strip is as wide as before otherwise). Its divider is named "Vla Divider", not
// "Group Divider", so the group count stays three (first person) / two (blocks) without a stage node.
// The rest of the strip is in StatusStrip.cs.
public partial class StatusStrip
{
    const float VlaTaskRatio = 0.85f;   // task / stats line font relative to the strip font
    const float LineRatio = 1.2f;       // line height / font
    const float VlaChipRatio = 1.3f;    // GRIP chip height / font (CONTROL is 1.5: the chip shares its column with a line)
    const float VlaChipGapMm = 4f;      // between the chip and the stats line
    const int VlaMinPolicyChars = 6;    // the task line cuts the policy's head down to this before the end is cut
    static readonly string VlaTaskSample = VlaStatusRule.TaskLine(VlaStatusRule.StageDeploy, true, "pick cup", "smolvla_fft");
    const string VlaStatsSample = "9999 steps · 99.9 Hz";

    Func<VlaStatus> _vla;
    RectTransform _vlaGroup, _vlaDeploy;
    float _vlaWidth, _vlaX, _vlaColCollection, _vlaColDeploy, _vlaDeployW, _vlaTop, _vlaLineH, _vlaSmallH;
    Image _vlaDot, _vlaChip;
    TextMeshProUGUI _vlaLabel, _vlaTask, _vlaChipText, _vlaStats;
    byte? _vlaStage;
    string _vlaTaskKey;

    public bool VlaShown => _vlaGroup != null && _vlaGroup.gameObject.activeSelf;
    // Width the VLA group adds to the strip while shown (mm, divider included).
    public float VlaWidthMm => _vlaWidth;
    public bool VlaDeployShown => VlaShown && _vlaDeploy.gameObject.activeSelf;

    void BuildVla()
    {
        _vlaGroup = Group(_root, "Vla Group");
        float x = Divider(_vlaGroup, 0f, "Vla Divider");
        float small = _font * VlaTaskRatio;
        _vlaLineH = _font * LineRatio; _vlaSmallH = small * LineRatio;
        _vlaTop = (HeightMm - _vlaLineH - _vlaSmallH) / 2f;

        _vlaDot = HudUi.Round(HudUi.Box(_vlaGroup, "Vla Dot", HudTheme.Record), DotMm / 2f);
        HudUi.Place(_vlaDot.rectTransform, x, _vlaTop + (_vlaLineH - DotMm) / 2f, DotMm, DotMm);
        _vlaX = x += DotMm + GapMm;

        _vlaLabel = Line(_vlaGroup, "Vla Label", _font);
        _vlaLabel.fontStyle = FontStyles.Bold;
        _vlaTask = Line(_vlaGroup, "Vla Task", small);
        _vlaTask.overflowMode = TextOverflowModes.Ellipsis;   // long task names are cut, the group does not grow
        float Bold(VlaStatusRule.Look look) => _vlaLabel.GetPreferredValues(VlaStatusRule.Label(look, 0f)).x;
        // As wide as the longest state of the stage (and, for deploy, a typical "task · policy" line).
        _vlaColCollection = Mathf.Ceil(Mathf.Max(Bold(VlaStatusRule.Look.Paused), Bold(VlaStatusRule.Look.Recording))) + 4f;
        _vlaColDeploy = Mathf.Ceil(Mathf.Max(Mathf.Max(Bold(VlaStatusRule.Look.Playing), Bold(VlaStatusRule.Look.Resetting)),
                                             _vlaTask.GetPreferredValues(VlaTaskSample).x)) + 4f;

        // Deploy column: GRIP chip over the stats line.
        _vlaDeploy = Group(_vlaGroup, "Vla Deploy");
        float chipH = _font * VlaChipRatio;
        (_vlaChip, _vlaChipText) = HudUi.Pill(_vlaDeploy, "Vla Chip", "", _font, chipH, Color.clear, Color.white, bold: true);
        _vlaChipText.margin = new Vector4(8f, 0f, 8f, 0f);
        _vlaChipText.textWrappingMode = TextWrappingModes.NoWrap;
        float chipW = Mathf.Ceil(_vlaChipText.GetPreferredValues("GRIP released").x) + 4f;   // margins included
        _vlaStats = Line(_vlaDeploy, "Vla Stats", small);
        _vlaStats.color = HudTheme.Muted;
        float statsW = Mathf.Ceil(_vlaStats.GetPreferredValues(VlaStatsSample).x) + 4f;
        _vlaDeployW = Mathf.Max(chipW, statsW);
        // The chip and the line as one block, centred like the label and the task line.
        float blockTop = (HeightMm - chipH - VlaChipGapMm - _vlaSmallH) / 2f;
        HudUi.Place(_vlaChip.rectTransform, 0f, blockTop, chipW, chipH);
        HudUi.Place(_vlaStats.rectTransform, 0f, blockTop + chipH + VlaChipGapMm, _vlaDeployW, _vlaSmallH);

        _vlaGroup.gameObject.SetActive(false);
        ApplyVlaStage(VlaStatusRule.StageCollection);
    }

    // Column widths and the deploy column for a stage; the group's width follows (Layout moves BODY).
    void ApplyVlaStage(byte stage)
    {
        _vlaStage = stage;
        bool deploy = stage == VlaStatusRule.StageDeploy;
        float col = deploy ? _vlaColDeploy : _vlaColCollection;
        HudUi.Place(_vlaLabel.rectTransform, _vlaX, _vlaTop, col, _vlaLineH);
        HudUi.Place(_vlaTask.rectTransform, _vlaX, _vlaTop + _vlaLineH, col, _vlaSmallH);
        _vlaDeploy.gameObject.SetActive(deploy);
        float dx = _vlaX + col + GroupGapMm;
        if (deploy) HudUi.Place(_vlaDeploy, dx, 0f, _vlaDeployW, HeightMm);
        _vlaWidth = deploy ? dx + _vlaDeployW : _vlaX + col;
        _vlaTaskKey = null;
    }

    static TextMeshProUGUI Line(RectTransform parent, string name, float font)
    {
        var t = HudUi.Label(parent, name, "", font, TextAlignmentOptions.Left);
        t.textWrappingMode = TextWrappingModes.NoWrap;
        t.verticalAlignment = VerticalAlignmentOptions.Middle;
        return t;
    }

    // Every frame: show / hide with the feed and follow its stage (the strip resizes), pulse the dot and the GRIP chip.
    void UpdateVla()
    {
        var rs = _vla?.Invoke();
        bool show = rs != null && rs.Available;
        if (show && _vlaGroup == null) BuildVla();
        if (_vlaGroup == null) return;
        bool relayout = false;
        if (show && rs.Stage != _vlaStage) { ApplyVlaStage(rs.Stage); relayout = true; }
        if (show != _vlaGroup.gameObject.activeSelf) { _vlaGroup.gameObject.SetActive(show); relayout = true; }
        if (relayout)
        {
            Layout();
            if (show) UpdateVlaTexts();
        }
        if (!show) return;
        var look = rs.Look;
        float pulse = VlaStatusRule.PulseAlpha(Time.unscaledTime);
        _vlaDot.color = HudTheme.WithAlpha(VlaStatusRule.Colour(look),
                                           VlaStatusRule.Pulses(look, rs.DeadmanEngaged || !rs.DeadmanEnabled) ? pulse : 1f);
        if (_vlaChip.gameObject.activeSelf)
        {
            var chip = VlaStatusRule.Deadman(rs.DeadmanEnabled, rs.DeadmanEngaged, rs.State);
            if (chip.HasValue)
                _vlaChip.color = HudTheme.WithAlpha(chip.Value.colour, HudTheme.PillBackground(chip.Value.colour, strong: true).a * (chip.Value.pulse ? pulse : 1f));
        }
    }

    // At the strip's text rate (and when the group appears).
    void UpdateVlaTexts()
    {
        if (!VlaShown) return;
        var rs = _vla?.Invoke();
        if (rs == null) return;
        var look = rs.Look;
        var c = VlaStatusRule.Colour(look);
        _vlaLabel.text = VlaStatusRule.Label(look, rs.ElapsedS);
        _vlaLabel.color = c;
        string key = $"{rs.Stage}|{rs.TaskSet}|{rs.TaskName}|{rs.Policy}";
        if (key != _vlaTaskKey)
        {
            _vlaTaskKey = key;
            _vlaTask.text = FitTaskLine(rs);
        }
        _vlaTask.color = rs.TaskSet ? HudTheme.Muted : HudTheme.Warn;
        if (!rs.IsDeploy) return;

        var chip = VlaStatusRule.Deadman(rs.DeadmanEnabled, rs.DeadmanEngaged, rs.State);
        _vlaChip.gameObject.SetActive(chip.HasValue);
        if (chip.HasValue)
        {
            _vlaChipText.text = chip.Value.text;
            _vlaChipText.color = chip.Value.colour;
            if (!chip.Value.pulse) _vlaChip.color = HudTheme.PillBackground(chip.Value.colour, strong: true);
        }
        _vlaStats.text = VlaStatusRule.Stats(rs.Steps, rs.InferenceHz);
    }

    // The task line; for deploy, when "task · policy" is too wide the policy loses characters at its head ("…la_fft")
    // before the ellipsis cuts the end, so its distinguishing tail stays visible.
    string FitTaskLine(VlaStatus rs)
    {
        string line = VlaStatusRule.TaskLine(rs.Stage, rs.TaskSet, rs.TaskName, rs.Policy);
        if (!rs.IsDeploy) return line;
        string policy = VlaStatusRule.PolicyShort(rs.Policy);
        if (policy.Length == 0) return line;
        string head = line.Substring(0, line.Length - policy.Length);   // "pick cup · "
        float width = _vlaTask.rectTransform.sizeDelta.x;
        string core = policy.TrimStart('…');
        while (_vlaTask.GetPreferredValues(line).x > width && core.Length > VlaMinPolicyChars)
        {
            core = core.Substring(1);
            line = head + "…" + core;
        }
        return line;
    }
}
