using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

/// <summary>
/// Where the camera blocks of an <see cref="ImageSubscriber"/> are and which are shown: the
/// automatic grid (best fit), the saved (dragged) positions and sizes, per-camera visibility, and
/// hiding all blocks while the first-person view is on. Sizes and area come from the façade's
/// serialised layout fields; saved values live in <see cref="Settings"/>.
/// </summary>
public class CameraLayout
{
    readonly ImageSubscriber _owner;
    readonly List<CameraPanel> _panels;
    float _defaultFit;   // best-fit size with every camera visible; sizes are relative to it

    // First-person view: hide / show every camera block without touching the saved
    // visibility or layout. Showing restores exactly the blocks that were visible before.
    readonly List<CameraPanel> _hiddenByMode = new List<CameraPanel>();
    bool _blocksHidden;

    // Raised when a camera is shown or hidden; (index, on).
    public Action<int, bool> VisibilityChanged;

    public CameraLayout(ImageSubscriber owner, List<CameraPanel> panels)
    {
        _owner = owner;
        _panels = panels;
    }

    public bool BlocksShown => !_blocksHidden;

    CameraSettings CamSettings(CameraPanel p) => Settings.For(_owner.Profile).Camera(p.Config);

    // True once the user has dragged any block: the layout is then theirs and is left alone.
    public bool HasCustomLayout
    {
        get
        {
            foreach (var p in _panels)
                if (CamSettings(p).Position != null) return true;
            return false;
        }
    }

    // Initial arrangement once the blocks exist: saved visibility, best fit, saved positions.
    public void Build()
    {
        foreach (var p in _panels)
            if (!CamSettings(p).Visible) SetOn(p, false);
        if (_panels.Count > 0) _defaultFit = BestFit(_panels, out _);
        Arrange();
        ApplySavedPositions();
    }

    // A new block was added (by Find cameras); first person keeps it hidden like the others.
    public void OnPanelAdded(CameraPanel panel)
    {
        if (!_blocksHidden) return;
        _hiddenByMode.Add(panel);
        panel.Visible = false;
    }

    // Blocks were added: the all-cameras reference size changes; a custom layout keeps its blocks and
    // puts the new ones above them.
    public void OnCamerasAdded(int added)
    {
        _defaultFit = BestFit(_panels, out _);
        if (!HasCustomLayout) Arrange();
        else foreach (var p in _panels.Skip(_panels.Count - added)) PlaceNewPanel(p);
    }

    // A frame of another size arrived: size the views again.
    public void OnFrameSizeChanged()
    {
        if (HasCustomLayout) return;
        _defaultFit = BestFit(_panels, out _);
        Arrange();
    }

    // Re-arrange unless the layout is the user's own.
    public void ArrangeIfAutomatic()
    {
        if (!HasCustomLayout) Arrange();
    }

    // Custom (dragged) layout: put a newly found camera just above the others instead of
    // re-arranging blocks the user placed by hand.
    void PlaceNewPanel(CameraPanel p)
    {
        float top = _panels.Where(o => o != p && IsOn(o)).Select(o => ArcY(o.transform.localPosition) + o.Height / 2f)
                           .DefaultIfEmpty(0f).Max();
        Place(p, 0f, top + _owner.rowGap + p.Height / 2f);
    }

    // The grid's offsets (x, y; metres) are arc lengths on a sphere of radius `distance` around the head: they
    // become yaw = x / R and pitch = y / R (rad), the block sits at R * dir(yaw, pitch) and faces the eye. Block
    // sizes and gaps are metres on the tangent plane (angular half extent atan(w / 2R) < w / 2R), so blocks that are
    // a grid step apart never touch. Row and column steps are therefore the same angles at every radius.
    Vector3 OnArc(float x, float y)
    {
        float yaw = x / _owner.distance, pitch = y / _owner.distance;
        float cp = Mathf.Cos(pitch);
        return _owner.distance * new Vector3(Mathf.Sin(yaw) * cp, Mathf.Sin(pitch), Mathf.Cos(yaw) * cp);
    }

    // Grid height (arc length) of a block position, also for saved positions that are not on the sphere.
    float ArcY(Vector3 pos) => _owner.distance * Mathf.Asin(Mathf.Clamp(pos.y / pos.magnitude, -1f, 1f));

    void Place(CameraPanel p, float x, float y)
    {
        var pos = OnArc(x, y);
        p.transform.localPosition = pos;
        p.transform.localRotation = Quaternion.LookRotation(pos);
    }

    // Show/hide a camera. In the automatic layout the remaining cameras are re-arranged
    // and grow into the freed space; a custom (dragged) layout is kept as it is.
    public void SetCameraVisible(CameraPanel panel, bool visible)
    {
        SetOn(panel, visible);
        CamSettings(panel).Visible = visible;
        Settings.Save();
        ArrangeIfAutomatic();
    }

    // Back to the default: every camera shown, automatic layout, dragged positions forgotten.
    public void Reset()
    {
        foreach (var p in _panels)
        {
            CamSettings(p).ResetLayout();
            SetOn(p, true);
        }
        Settings.Save();
        Arrange();
    }

    // Remember where the user dragged or resized a block, per robot and camera. Saved as
    // "x;y;z;size": the block's place relative to the head (on the arc around it) and its view size.
    public void SavePosition(CameraPanel panel)
    {
        var v = panel.transform.localPosition;
        CamSettings(panel).SetPlacement(v, panel.Size);
        Settings.Save();
    }

    void ApplySavedPositions()
    {
        foreach (var p in _panels)
        {
            var saved = CamSettings(p);
            if (saved.Size is float size) p.SetSize(size);   // older saves (x;y;z) keep the automatic size
            if (saved.Position is Vector3 pos)
            {
                p.transform.localPosition = pos;
                p.transform.localRotation = Quaternion.LookRotation(pos);
            }
        }
    }

    // Wanted visibility; while blocks are hidden (first person) it only lives in _hiddenByMode and every panel GameObject stays inactive.
    public bool IsOn(CameraPanel p) => _blocksHidden ? _hiddenByMode.Contains(p) : p.Visible;

    void SetOn(CameraPanel p, bool on)
    {
        bool was = IsOn(p);
        if (!_blocksHidden) p.Visible = on;
        else if (on) { if (!_hiddenByMode.Contains(p)) _hiddenByMode.Add(p); }
        else _hiddenByMode.Remove(p);
        if (was != on) VisibilityChanged?.Invoke(_panels.IndexOf(p), on);
    }

    public void SetBlocksShown(bool shown)
    {
        if (shown == !_blocksHidden) return;
        _blocksHidden = !shown;
        if (!shown)
        {
            foreach (var p in _panels)
                if (p.Visible) { _hiddenByMode.Add(p); p.Visible = false; }
            return;
        }
        foreach (var p in _hiddenByMode)
            if (p != null) p.Visible = true;
        _hiddenByMode.Clear();
    }

    // Automatic layout of the visible cameras: pick the column count that allows the largest
    // views within the layout area, size the views relative to the all-cameras layout (so with
    // every camera shown the layout is the default one), then place the grid centred in front
    // of the head on an arc (see OnArc), each block facing the eye. Rows are sized from the actual blocks, so blocks
    // never overlap; views in a row share a horizontal centre line.
    public void Arrange()
    {
        var visible = _panels.FindAll(IsOn);
        if (visible.Count == 0) return;
        float rowGap = _owner.rowGap, columnGap = _owner.columnGap;

        float fit = BestFit(visible, out int cols);
        float size = Mathf.Min(_owner.maxGrow, fit / _defaultFit);
        foreach (var p in visible) p.SetSize(size);

        Measure(visible, cols, size, out float[] rowAbove, out float[] rowBelow, out float[] rowWidths);
        float totalH = (rowAbove.Length - 1) * rowGap;
        for (int r = 0; r < rowAbove.Length; r++) totalH += rowAbove[r] + rowBelow[r];
        float top = Mathf.Max(totalH / 2f, _owner.minBottom + totalH);

        for (int r = 0, i = 0; r < rowAbove.Length; r++)
        {
            float viewCentreY = top - rowAbove[r];
            float x = -rowWidths[r] / 2f;
            for (int c = 0; c < cols && i < visible.Count; c++, i++)
            {
                var p = visible[i];
                float blockCentreY = viewCentreY + p.AboveViewCentre - p.Height / 2f;
                Place(p, x + p.Width / 2f, blockCentreY);
                x += p.Width + columnGap;
            }
            top -= rowAbove[r] + rowBelow[r] + rowGap;
        }
    }

    // Largest view size factor at which `panels` fit the layout area, over all column counts.
    // A grid with empty slots (e.g. 3+1) only wins if its views are more than 5% larger than
    // those of a full grid, so near-ties go to the tidier layout (4 -> 2x2, 3 -> one row).
    const float EmptySlotPenalty = 0.95f;

    float BestFit(List<CameraPanel> panels, out int bestCols)
    {
        float best = 0f, bestScore = 0f;
        bestCols = panels.Count;
        for (int cols = panels.Count; cols >= 1; cols--)
        {
            // Binary search: block width and height only grow with size.
            float lo = 0.05f, hi = 4f;
            for (int it = 0; it < 20; it++)
            {
                float mid = (lo + hi) / 2f;
                if (Fits(panels, cols, mid)) lo = mid; else hi = mid;
            }
            float score = panels.Count % cols == 0 ? lo : lo * EmptySlotPenalty;
            if (score > bestScore + 1e-3f) { bestScore = score; best = lo; bestCols = cols; }
        }
        return best;
    }

    bool Fits(List<CameraPanel> panels, int cols, float size)
    {
        Measure(panels, cols, size, out float[] above, out float[] below, out float[] widths);
        float h = (above.Length - 1) * _owner.rowGap, w = 0f;
        for (int r = 0; r < above.Length; r++) { h += above[r] + below[r]; w = Mathf.Max(w, widths[r]); }
        return w <= _owner.maxWidth && h <= _owner.maxTop - _owner.minBottom;
    }

    void Measure(List<CameraPanel> panels, int cols, float size,
                 out float[] rowAbove, out float[] rowBelow, out float[] rowWidths)
    {
        int rows = Mathf.CeilToInt(panels.Count / (float)cols);
        rowAbove = new float[rows]; rowBelow = new float[rows]; rowWidths = new float[rows];
        for (int i = 0; i < panels.Count; i++)
        {
            int r = i / cols;
            var m = panels[i].Measure(size);
            rowAbove[r] = Mathf.Max(rowAbove[r], m.AboveViewCentre);
            rowBelow[r] = Mathf.Max(rowBelow[r], m.BelowViewCentre);
            rowWidths[r] += m.Width + (i % cols > 0 ? _owner.columnGap : 0f);
        }
    }
}
