using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// <summary>
/// Builds one <see cref="CameraPanel"/> per camera of the selected robot, lays them
/// out in front of the user (head-locked) and shows each camera's compressed images.
/// </summary>
public class ImageSubscriber : MonoBehaviour
{
    public ROSConnection ros;

    // Used when the scene is opened directly in the Editor (no robot selected).
    public RobotProfile defaultProfile;

    // Panels follow this transform; defaults to the main camera.
    public Transform panelParent;

    [Header("Layout (metres, relative to the head)")]
    public float distance = 4.3f;
    public float columnGap = 0.15f;
    public float rowGap = 0.15f;
    // Area the auto layout may use. minBottom keeps blocks above the HUD bar.
    public float maxWidth = 6.0f;
    public float maxTop = 2.0f;
    public float minBottom = -1.45f;
    // Upper limit for how much views grow when cameras are hidden (1 = default size).
    public float maxGrow = 1.6f;

    readonly List<CameraPanel> _panels = new List<CameraPanel>();
    RobotProfile _profile;
    float _defaultFit;   // best-fit size with every camera visible; sizes are relative to it
    Texture2D[] _textures;
    double[] _lastRenderTime;

    public IReadOnlyList<CameraPanel> Panels => _panels;
    public RobotProfile Profile => _profile;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        var profile = _profile = RobotProfile.Selected != null ? RobotProfile.Selected : defaultProfile;
        if (profile == null)
        {
            Debug.LogError("ImageSubscriber: no robot selected and no default profile set.");
            return;
        }

        if (panelParent == null && Camera.main != null)
            panelParent = Camera.main.transform;

        int n = profile.cameras.Length;
        _textures = new Texture2D[n];
        _lastRenderTime = new double[n];

        for (int i = 0; i < n; i++)
        {
            var cam = profile.cameras[i];
            string topic = profile.FullTopic(cam);
            _panels.Add(CameraPanel.Create(panelParent, cam, topic));
            _textures[i] = new Texture2D(cam.resolution.x, cam.resolution.y, TextureFormat.RGB24, false);

            int index = i;
            ros.Subscribe<CompressedImageMsg>(topic, msg => RenderCompressedTexture(msg, index));
        }

        _defaultFit = BestFit(_panels, out _);
        Layout();
        ApplySavedPositions();
    }

    // True once the user has dragged any block: the layout is then theirs and is left alone.
    public bool HasCustomLayout
    {
        get
        {
            foreach (var p in _panels)
                if (PlayerPrefs.HasKey(PositionKey(p))) return true;
            return false;
        }
    }

    // Show/hide a camera. In the automatic layout the remaining cameras are re-arranged
    // and grow into the freed space; a custom (dragged) layout is kept as it is.
    public void SetCameraVisible(CameraPanel panel, bool visible)
    {
        panel.Visible = visible;
        if (!HasCustomLayout) Layout();
    }

    // Back to the automatic layout (for the cameras currently shown); forget dragged positions.
    public void ResetLayout()
    {
        foreach (var p in _panels)
            PlayerPrefs.DeleteKey(PositionKey(p));
        PlayerPrefs.Save();
        Layout();
    }

    // Remember where the user dragged a block (head-relative), per robot and camera.
    public void SavePosition(CameraPanel panel)
    {
        var v = panel.transform.localPosition;
        PlayerPrefs.SetString(PositionKey(panel),
            string.Format(CultureInfo.InvariantCulture, "{0};{1};{2}", v.x, v.y, v.z));
        PlayerPrefs.Save();
    }

    void ApplySavedPositions()
    {
        foreach (var p in _panels)
        {
            var parts = PlayerPrefs.GetString(PositionKey(p), "").Split(';');
            if (parts.Length == 3 &&
                float.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out float x) &&
                float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float y) &&
                float.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float z))
            {
                var pos = new Vector3(x, y, z);
                p.transform.localPosition = pos;
                p.transform.localRotation = Quaternion.LookRotation(pos);
            }
        }
    }

    string PositionKey(CameraPanel p) => $"PanelPosition/{_profile.name}/{p.Config.displayName}";

    // Automatic layout of the visible cameras: pick the column count that allows the largest
    // views within the layout area, size the views relative to the all-cameras layout (so with
    // every camera shown the layout is the default one), then place the grid centred in front
    // of the head, each block facing the eye. Rows are sized from the actual blocks, so blocks
    // never overlap; views in a row share a horizontal centre line.
    void Layout()
    {
        var visible = _panels.FindAll(p => p.Visible);
        if (visible.Count == 0) return;

        float fit = BestFit(visible, out int cols);
        float size = Mathf.Min(maxGrow, fit / _defaultFit);
        foreach (var p in visible) p.SetSize(size);

        Measure(visible, cols, size, out float[] rowAbove, out float[] rowBelow, out float[] rowWidths);
        float totalH = (rowAbove.Length - 1) * rowGap;
        for (int r = 0; r < rowAbove.Length; r++) totalH += rowAbove[r] + rowBelow[r];
        float top = Mathf.Max(totalH / 2f, minBottom + totalH);

        for (int r = 0, i = 0; r < rowAbove.Length; r++)
        {
            float viewCentreY = top - rowAbove[r];
            float x = -rowWidths[r] / 2f;
            for (int c = 0; c < cols && i < visible.Count; c++, i++)
            {
                var p = visible[i];
                float blockCentreY = viewCentreY + p.AboveViewCentre - p.Height / 2f;
                var pos = new Vector3(x + p.Width / 2f, blockCentreY, distance);
                p.transform.localPosition = pos;
                p.transform.localRotation = Quaternion.LookRotation(pos);
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
        float h = (above.Length - 1) * rowGap, w = 0f;
        for (int r = 0; r < above.Length; r++) { h += above[r] + below[r]; w = Mathf.Max(w, widths[r]); }
        return w <= maxWidth && h <= maxTop - minBottom;
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
            rowWidths[r] += m.Width + (i % cols > 0 ? columnGap : 0f);
        }
    }

    void RenderCompressedTexture(CompressedImageMsg msg, int index)
    {
        if (msg == null || msg.data == null || msg.data.Length == 0)
            return;

        // Hidden cameras skip JPEG decoding entirely.
        if (!_panels[index].Visible)
            return;

        float fps = _panels[index].Config.maxFps;
        if (fps > 0f)
        {
            double now = Time.timeAsDouble;
            if (now - _lastRenderTime[index] < 1.0 / fps)
                return;
            _lastRenderTime[index] = now;
        }

        if (!_textures[index].LoadImage(msg.data))
        {
            Debug.LogWarning($"Failed to decode compressed image on {_panels[index].Topic}. format={msg.format}");
            return;
        }
        _panels[index].SetTexture(_textures[index]);
    }
}
