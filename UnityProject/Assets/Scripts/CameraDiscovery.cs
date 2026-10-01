using System.Collections.Generic;
using System.Globalization;
using System.Linq;

/// <summary>
/// Turns the ROS topic list into the cameras of a newly added robot:
///   - sensor_msgs/CompressedImage topics are used;
///   - a raw sensor_msgs/Image topic is used only if it has no ".../compressed" version;
///   - depth images are skipped (they are not viewable colour images).
/// Camera names come from the topic, e.g. /sobit_light/head_camera/color/image_raw/compressed
/// -> "Head Camera". The robot namespace is the first topic segment if all cameras share it.
/// </summary>
public static class CameraDiscovery
{
    public const string CompressedType = "sensor_msgs/CompressedImage";
    public const string RawType = "sensor_msgs/Image";

    // Segments that describe the image stream rather than the camera.
    static readonly HashSet<string> StreamWords = new HashSet<string>
    {
        "compressed", "image", "image_raw", "image_rect", "image_rect_raw", "image_rect_color",
        "image_color", "color", "rgb", "raw",
    };

    public struct Found
    {
        public string topic;
        public bool raw;
        public string displayName;
    }

    public static List<Found> Select(IEnumerable<(string topic, string type)> topics)
    {
        var list = topics.Select(t => (topic: t.topic, type: NormaliseType(t.type))).ToList();
        var compressed = list.Where(t => t.type == CompressedType && !IsDepth(t.topic))
                             .Select(t => t.topic).ToList();
        var compressedSet = new HashSet<string>(compressed);
        var raw = list.Where(t => t.type == RawType && !IsDepth(t.topic)
                                  && !compressedSet.Contains(t.topic.TrimEnd('/') + "/compressed"))
                      .Select(t => t.topic).ToList();

        var found = compressed.Select(t => new Found { topic = t, raw = false })
                              .Concat(raw.Select(t => new Found { topic = t, raw = true }))
                              .OrderBy(f => f.topic)
                              .ToList();

        string ns = CommonNamespace(found.Select(f => f.topic));
        var names = found.Select(f => NameFor(f.topic, ns, 1)).ToList();
        // Make names unique by using more of the topic where two cameras would share a name.
        for (int depth = 2; depth <= 4 && names.Distinct().Count() < names.Count; depth++)
            for (int i = 0; i < found.Count; i++)
                if (names.Count(n => n == names[i]) > 1)
                    names[i] = NameFor(found[i].topic, ns, depth);
        for (int i = 0; i < found.Count; i++)
        {
            var f = found[i];
            f.displayName = names[i];
            found[i] = f;
        }
        return found;
    }

    // First topic segment shared by every camera topic, or "" if they differ.
    public static string CommonNamespace(IEnumerable<string> topics)
    {
        var firsts = topics.Select(t => t.Trim('/').Split('/'))
                           .Select(parts => parts.Length > 1 ? parts[0] : "")
                           .Distinct().ToList();
        return firsts.Count == 1 ? firsts[0] : "";
    }

    // ROS infrastructure topics that say nothing about which robot this is.
    static readonly HashSet<string> SystemTopics = new HashSet<string>
    {
        "/tf", "/tf_static", "/rosout", "/parameter_events", "/clock", "/joy",
    };

    // Namespace for a robot without cameras: the first segment shared by all its namespaced
    // topics (e.g. /my_robot/joint_states, /my_robot/odom -> my_robot), or "" if they differ.
    public static string RobotNamespace(IEnumerable<string> topics)
        => CommonNamespace(topics.Where(t => !SystemTopics.Contains(t) && t.Trim('/').Contains('/')));

    static bool IsDepth(string topic)
        => topic.Split('/').Any(s => s.Contains("depth")) || topic.EndsWith("compressedDepth");

    // "sensor_msgs/msg/CompressedImage" and "sensor_msgs/CompressedImage" are the same type.
    static string NormaliseType(string type) => (type ?? "").Replace("/msg/", "/");

    // Camera name from the topic segments that are not the namespace or stream words;
    // `depth` = how many such segments to use (1 = the closest to the namespace).
    static string NameFor(string topic, string ns, int depth)
    {
        var parts = topic.Trim('/').Split('/').ToList();
        if (ns != "" && parts.Count > 0 && parts[0] == ns) parts.RemoveAt(0);
        var meaningful = parts.Where(p => !StreamWords.Contains(p)).ToList();
        if (meaningful.Count == 0) meaningful = parts;
        var words = meaningful.Take(depth).SelectMany(p => p.Split('_', '-'))
                              .Where(w => w.Length > 0)
                              .Select(w => CultureInfo.InvariantCulture.TextInfo.ToTitleCase(w));
        string name = string.Join(" ", words);
        return name.Length > 0 ? name : topic;
    }
}
