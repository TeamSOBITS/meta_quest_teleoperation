using UnityEngine;

/// <summary>
/// First-person layout: a big inward-facing sphere around the head that replaces the pure black beyond
/// the floor grid with a very subtle vertical gradient (dark above and below, a slightly lighter band at
/// eye level) and a faint horizon line (the Hud/Surround shader, see <see cref="HudAssets"/>). Opaque but
/// drawn first (queue Background, no depth write), so the floor grid and the model are never hidden. It
/// follows the head's position and never rotates, so the horizon stays level. Created and destroyed with
/// the first-person layout by <see cref="FirstPersonView"/>; tagged Floor so passthrough hides it with the grid.
/// </summary>
public class Surround : MonoBehaviour
{
    public const float RadiusM = 20f;

    public static Surround Create(Transform parent)
    {
        var assets = HudTheme.Assets;
        if (assets == null || assets.surround == null) return null;
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);   // unit sphere of radius 0.5
        go.name = "Surround";
        Destroy(go.GetComponent<Collider>());
        go.transform.SetParent(parent, false);
        go.transform.localScale = Vector3.one * (2f * RadiusM);
        go.tag = PassthroughMode.FloorTag;
        var r = go.GetComponent<MeshRenderer>();
        r.sharedMaterial = assets.surround;
        r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        r.receiveShadows = false;
        go.SetActive(!PassthroughMode.Enabled);
        var surround = go.AddComponent<Surround>();
        surround.Follow();
        return surround;
    }

    void LateUpdate() => Follow();

    void Follow()
    {
        var head = FirstPersonView.Head;
        if (head == null) return;
        transform.SetPositionAndRotation(head.position, Quaternion.identity);
    }
}
