using System.Collections.Generic;
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
    public int maxColumns = 3;
    public float columnGap = 0.15f;
    public float rowGap = 0.15f;
    // Keep panels above the ROS IP block at the bottom of the view.
    public float minBottom = -1.55f;

    readonly List<CameraPanel> _panels = new List<CameraPanel>();
    Texture2D[] _textures;
    double[] _lastRenderTime;

    public IReadOnlyList<CameraPanel> Panels => _panels;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        var profile = RobotProfile.Selected != null ? RobotProfile.Selected : defaultProfile;
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

        Layout();
    }

    // Grid of up to maxColumns per row, centred in front of the head, each panel facing the eye.
    // Rows are sized from the actual blocks, so a scaled-up camera pushes its neighbours aside
    // instead of overlapping them. Views in a row share a horizontal centre line.
    void Layout()
    {
        int n = _panels.Count;
        if (n == 0) return;

        int cols = Mathf.Min(n, Mathf.Max(1, maxColumns));
        if (n > cols) cols = Mathf.CeilToInt(n / Mathf.Ceil(n / (float)cols));  // balance rows, e.g. 4 -> 2x2
        int rows = Mathf.CeilToInt(n / (float)cols);

        var rowAbove = new float[rows];
        var rowBelow = new float[rows];
        var rowWidths = new float[rows];
        for (int i = 0; i < n; i++)
        {
            int r = i / cols;
            rowAbove[r] = Mathf.Max(rowAbove[r], _panels[i].AboveViewCentre);
            rowBelow[r] = Mathf.Max(rowBelow[r], _panels[i].BelowViewCentre);
            rowWidths[r] += _panels[i].Width + (i % cols > 0 ? columnGap : 0f);
        }

        float totalH = (rows - 1) * rowGap;
        for (int r = 0; r < rows; r++) totalH += rowAbove[r] + rowBelow[r];
        float top = Mathf.Max(totalH / 2f, minBottom + totalH);

        for (int r = 0, i = 0; r < rows; r++)
        {
            float viewCentreY = top - rowAbove[r];
            float x = -rowWidths[r] / 2f;
            for (int c = 0; c < cols && i < n; c++, i++)
            {
                var p = _panels[i];
                float blockCentreY = viewCentreY + p.AboveViewCentre - p.Height / 2f;
                var pos = new Vector3(x + p.Width / 2f, blockCentreY, distance);
                p.transform.localPosition = pos;
                p.transform.localRotation = Quaternion.LookRotation(pos);
                x += p.Width + columnGap;
            }
            top -= rowAbove[r] + rowBelow[r] + rowGap;
        }
    }

    void RenderCompressedTexture(CompressedImageMsg msg, int index)
    {
        if (msg == null || msg.data == null || msg.data.Length == 0)
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
