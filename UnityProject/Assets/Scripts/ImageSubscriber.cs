using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class ImageSubscriber : MonoBehaviour
{
    public ROSConnection ros;
    public Renderer headImageRenderer;
    public Renderer handImageRenderer;
    public Renderer baseFrontImageRenderer;
    public Renderer baseBackImageRenderer;


    public string headTopicName = "/sobit_light/head_camera/color";
    public string handTopicName = "/sobit_light/hand_camera/color";
    public string baseFrontTopicName = "/sobit_light/base_front_camera/color";
    public string baseBackTopicName = "/sobit_light/base_back_camera/color";
    // private Texture2D tex;
    private Texture2D head_tex;
    private Texture2D hand_tex;
    private Texture2D base_front_tex;
    private Texture2D base_back_tex;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        ros = ROSConnection.instance;
        ros.Subscribe<ImageMsg>(headTopicName, HeadImageCallback);
        ros.Subscribe<ImageMsg>(handTopicName, HandImageCallback);
        ros.Subscribe<ImageMsg>(baseFrontTopicName,BaseFrontImageCallback);
        ros.Subscribe<ImageMsg>(baseBackTopicName, BaseBackImageCallback);

        // tex = new Texture2D(640, 480, TextureFormat.RGB24, false);
        head_tex = new Texture2D(640, 480, TextureFormat.RGB24, false);
        hand_tex = new Texture2D(640, 480, TextureFormat.RGB24, false);
        base_front_tex = new Texture2D(640, 480, TextureFormat.RGB24, false);
        base_back_tex = new Texture2D(640, 480, TextureFormat.RGB24, false);
    }

    void HeadImageCallback(ImageMsg msg)
    {
        RenderTexture(msg.data, headImageRenderer, head_tex);
    }

    void HandImageCallback(ImageMsg msg)
    {
        RenderTexture(msg.data, handImageRenderer, hand_tex);
    }

    void BaseFrontImageCallback(ImageMsg msg)
    {
        RenderTexture(msg.data, baseFrontImageRenderer, base_front_tex);
    }

    void BaseBackImageCallback(ImageMsg msg)
    {
        RenderTexture(msg.data, baseBackImageRenderer, base_back_tex);
    }

    private void RenderTexture(byte[] data, Renderer renderer,Texture2D tex)
    {        
        // バイト列からtexture2dを生成
        tex.LoadRawTextureData(data);
        // テクスチャ割当
        renderer.material.mainTexture = tex;
        tex.Apply();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
