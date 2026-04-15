using UnityEngine;

/// <summary>
/// Generates a curved (cylindrical arc) mesh on a MeshFilter and applies it at runtime.
/// Attach this to any GameObject that already has a MeshFilter + MeshRenderer.
/// The ImageSubscriber (or any script that sets mat.mainTexture) works with this
/// automatically — no changes needed there.
///
/// Inspector fields:
///   arcAngleDeg   — total horizontal arc in degrees (e.g. 120 for a wide curved screen)
///   radius        — cylinder radius in metres (distance from player to screen centre)
///   height        — screen height in metres
///   segmentsH     — horizontal subdivisions (more = smoother curve)
///   segmentsV     — vertical subdivisions (1 is fine for a flat-height screen)
///   flipNormals   — true = normals face inward (player inside cylinder, default)
/// </summary>
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class CurvedScreen : MonoBehaviour
{
    [Header("Curve")]
    [Range(10f, 340f)]
    public float arcAngleDeg  = 120f;
    public float radius       = 3.0f;

    [Header("Size")]
    public float height       = 1.5f;

    [Header("Mesh quality")]
    [Range(4, 64)]
    public int segmentsH      = 32;
    [Range(1, 16)]
    public int segmentsV      = 1;

    [Header("Normals")]
    public bool flipNormals   = true;   // true: player is inside the cylinder

    void Awake()
    {
        BuildMesh();
    }

#if UNITY_EDITOR
    // Rebuild in-editor when inspector values change.
    void OnValidate()
    {
        // OnValidate can fire before Awake; only rebuild if a MeshFilter exists.
        var mf = GetComponent<MeshFilter>();
        if (mf != null) BuildMesh();
    }
#endif

    void BuildMesh()
    {
        int cols = segmentsH + 1;
        int rows = segmentsV + 1;
        int vertCount = cols * rows;

        Vector3[] verts  = new Vector3[vertCount];
        Vector2[] uvs    = new Vector2[vertCount];
        Vector3[] norms  = new Vector3[vertCount];

        float halfArc = arcAngleDeg * 0.5f * Mathf.Deg2Rad;
        float halfH   = height * 0.5f;

        for (int r = 0; r < rows; r++)
        {
            float v   = (float)r / segmentsV;          // 0 .. 1 bottom to top
            float y   = Mathf.Lerp(-halfH, halfH, v);

            for (int c = 0; c < cols; c++)
            {
                float u     = (float)c / segmentsH;    // 0 .. 1 left to right
                float angle = Mathf.Lerp(-halfArc, halfArc, u);

                // Cylinder: X = sin(angle)*r, Z = -cos(angle)*r (screen faces -Z at centre)
                float x = Mathf.Sin(angle) * radius;
                float z = -Mathf.Cos(angle) * radius;

                int idx      = r * cols + c;
                verts[idx]   = new Vector3(x, y, z);
                uvs[idx]     = new Vector2(u, v);
                // Inward-facing normal points toward cylinder axis
                norms[idx]   = flipNormals
                    ? new Vector3(-Mathf.Sin(angle), 0f, Mathf.Cos(angle))
                    : new Vector3( Mathf.Sin(angle), 0f, -Mathf.Cos(angle));
            }
        }

        // Build triangles
        int triCount = segmentsH * segmentsV * 2;
        int[] tris   = new int[triCount * 3];
        int t        = 0;

        for (int r = 0; r < segmentsV; r++)
        {
            for (int c = 0; c < segmentsH; c++)
            {
                int bl = r * cols + c;
                int br = bl + 1;
                int tl = bl + cols;
                int tr = tl + 1;

                if (flipNormals)
                {
                    // Wind clockwise when viewed from inside
                    tris[t++] = bl; tris[t++] = tl; tris[t++] = br;
                    tris[t++] = br; tris[t++] = tl; tris[t++] = tr;
                }
                else
                {
                    tris[t++] = bl; tris[t++] = br; tris[t++] = tl;
                    tris[t++] = br; tris[t++] = tr; tris[t++] = tl;
                }
            }
        }

        Mesh mesh    = new Mesh();
        mesh.name    = "CurvedScreen";
        mesh.vertices  = verts;
        mesh.uv        = uvs;
        mesh.normals   = norms;
        mesh.triangles = tris;
        mesh.RecalculateBounds();

        GetComponent<MeshFilter>().mesh = mesh;
    }
}
