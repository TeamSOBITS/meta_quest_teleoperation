using UnityEngine;

/// <summary>
/// Marks one URDF link inside a robot model prefab (see UrdfModelBuilder). The transform is a direct child of
/// its parent link, so a joint's TF transform can be applied straight to localPosition / localRotation.
/// </summary>
public class RobotLink : MonoBehaviour
{
    /// <summary>URDF link name = TF child frame id (unprefixed).</summary>
    public string frame;
    /// <summary>URDF parent link name, "" for the root.</summary>
    public string parentFrame;
    public bool isRoot;
}
