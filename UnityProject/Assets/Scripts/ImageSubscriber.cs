using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using TMPro;

public class ImageSubscriber : MonoBehaviour
{
    public ROSConnection ros;
    public Renderer[] imageRenderers;
    public string[] topicNames;
    public TextMeshProUGUI[] textInputs;

    private Texture2D[] textures;
    private Material[] materials;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        ros = ROSConnection.instance;

        int n = topicNames.Length;

        textures = new Texture2D[n];
        materials = new Material[n];

        for (int i = 0; i < n; i++)
        {
            materials[i] = imageRenderers[i].material;
            textures[i] = new Texture2D(640, 480, TextureFormat.RGB24, false);

            int index = i;

            ros.Subscribe<CompressedImageMsg>(topicNames[i], msg =>
            {
                RenderCompressedTexture(msg, materials[index], textures[index]);
            });

            if (textInputs != null && i < textInputs.Length)
                textInputs[i].text = topicNames[i];
        }
    }
    private void RenderCompressedTexture(CompressedImageMsg msg, Material mat, Texture2D tex)
    {
        if (msg == null || msg.data == null || msg.data.Length == 0)
            return;

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
