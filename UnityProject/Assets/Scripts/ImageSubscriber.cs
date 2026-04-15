using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using TMPro;

public class ImageSubscriber : MonoBehaviour
{
    public ROSConnection ros;
    public Renderer[] imageRenderers;

    // Robot namespace (e.g. "sobit_home", "sobit_pro").
    // Leave empty to use absolute topicNames as-is.
    public string robotNamespace = "sobit_home";

    // Camera sub-paths relative to the robot namespace, e.g.
    //   "hand_left_camera/color/compressed"
    //   "hand_right_camera/color/compressed"
    //   "head_camera/color/compressed"
    // At runtime these become: /<robotNamespace>/<cameraTopicSuffix>
    public string[] cameraTopicSuffixes;

    // Per-camera texture resolution. Must match cameraTopicSuffixes length.
    // Default (0,0) falls back to 640×480.
    public Vector2Int[] cameraResolutions;

    // Maximum display rate (frames per second) per camera.
    // Incoming messages that arrive faster than this are silently dropped.
    // Set to 0 to disable throttling for that camera (pass every frame).
    // A single-element array applies the same limit to all cameras.
    public float[] maxFps = new float[] { 15f };

    public TextMeshProUGUI[] textInputs;

    private Texture2D[] textures;
    private Material[] materials;
    private double[] lastRenderTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        int n = cameraTopicSuffixes.Length;

        textures       = new Texture2D[n];
        materials      = new Material[n];
        lastRenderTime = new double[n];

        for (int i = 0; i < n; i++)
        {
            // Build the full topic: /<robotNamespace>/<suffix>
            string ns = string.IsNullOrEmpty(robotNamespace) ? "" : "/" + robotNamespace;
            string fullTopic = ns + "/" + cameraTopicSuffixes[i].TrimStart('/');

            int w = (cameraResolutions != null && i < cameraResolutions.Length && cameraResolutions[i].x > 0)
                    ? cameraResolutions[i].x : 640;
            int h = (cameraResolutions != null && i < cameraResolutions.Length && cameraResolutions[i].y > 0)
                    ? cameraResolutions[i].y : 480;

            materials[i]      = imageRenderers[i].material;
            textures[i]       = new Texture2D(w, h, TextureFormat.RGB24, false);
            lastRenderTime[i] = 0.0;

            int index = i;
            ros.Subscribe<CompressedImageMsg>(fullTopic, msg =>
            {
                RenderCompressedTexture(msg, materials[index], textures[index], index);
            });

            if (textInputs != null && i < textInputs.Length)
                textInputs[i].text = fullTopic;
        }
    }

    private void RenderCompressedTexture(CompressedImageMsg msg, Material mat, Texture2D tex, int index)
    {
        if (msg == null || msg.data == null || msg.data.Length == 0)
            return;

        // Throttle: resolve per-camera maxFps (last element applies to all remaining cameras)
        float fps = (maxFps != null && maxFps.Length > 0)
            ? maxFps[Mathf.Min(index, maxFps.Length - 1)]
            : 0f;

        if (fps > 0f)
        {
            double now = Time.timeAsDouble;
            double minInterval = 1.0 / fps;
            if (now - lastRenderTime[index] < minInterval)
                return;
            lastRenderTime[index] = now;
        }

        bool success = tex.LoadImage(msg.data);
        if (!success)
        {
            Debug.LogWarning($"Failed to decode compressed image. format={msg.format}");
            return;
        }
        mat.mainTexture = tex;
    }

    // Update is called once per frame
    void Update()
    {

    }
}
