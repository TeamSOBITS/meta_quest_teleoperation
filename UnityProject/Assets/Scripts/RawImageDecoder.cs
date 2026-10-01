using RosMessageTypes.Sensor;
using UnityEngine;

/// <summary>
/// Copies an uncompressed sensor_msgs/Image into a Texture2D. Supports rgb8, bgr8, rgba8,
/// bgra8 and mono8; other encodings (e.g. 16-bit depth) are rejected. ROS images start with
/// the top row while textures start with the bottom row, so rows are flipped on the way.
/// </summary>
public static class RawImageDecoder
{
    public static bool IsSupported(string encoding)
        => encoding is "rgb8" or "bgr8" or "rgba8" or "bgra8" or "mono8";

    public static bool TryDecode(ImageMsg msg, ref Texture2D texture, ref byte[] buffer)
    {
        if (msg == null || msg.data == null || !IsSupported(msg.encoding)) return false;

        int w = (int)msg.width, h = (int)msg.height, step = (int)msg.step;
        bool alpha = msg.encoding is "rgba8" or "bgra8";
        int srcBpp = msg.encoding == "mono8" ? 1 : alpha ? 4 : 3;
        int dstBpp = alpha ? 4 : 3;
        if (w <= 0 || h <= 0 || step < w * srcBpp || msg.data.Length < step * h) return false;

        var format = alpha ? TextureFormat.RGBA32 : TextureFormat.RGB24;
        if (texture == null || texture.format != format)
            texture = new Texture2D(w, h, format, false);
        else if (texture.width != w || texture.height != h)
            texture.Reinitialize(w, h);

        int size = w * h * dstBpp;
        if (buffer == null || buffer.Length != size) buffer = new byte[size];

        bool swapRB = msg.encoding is "bgr8" or "bgra8";
        var src = msg.data;
        for (int y = 0; y < h; y++)
        {
            int s = y * step;
            int d = (h - 1 - y) * w * dstBpp;
            for (int x = 0; x < w; x++, s += srcBpp, d += dstBpp)
            {
                if (srcBpp == 1)
                {
                    buffer[d] = buffer[d + 1] = buffer[d + 2] = src[s];
                    continue;
                }
                buffer[d]     = src[swapRB ? s + 2 : s];
                buffer[d + 1] = src[s + 1];
                buffer[d + 2] = src[swapRB ? s : s + 2];
                if (alpha) buffer[d + 3] = src[s + 3];
            }
        }
        texture.LoadRawTextureData(buffer);
        texture.Apply(false);
        return true;
    }
}
