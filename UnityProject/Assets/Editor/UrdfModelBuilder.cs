#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text.RegularExpressions;
using System.Xml;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Builds the SOBIT HOME model prefab from the URDF + decimated STL meshes (tools/decimate_meshes.py).
/// One GameObject per link (direct child of its parent link), fixed joints baked in, no physics.
/// ROS (FLU) -> Unity: position (x,y,z) -> (-y,z,x); rpy -> AngleAxis(-yaw,up)*AngleAxis(pitch,right)*AngleAxis(-roll,forward).
/// Mesh vertices get the same axis change and their triangle winding is flipped (mirror).
/// Run: menu "Robots/Build SOBIT HOME model" or  Unity -batchmode -projectPath P -executeMethod UrdfModelBuilder.BuildSobitHome
/// Normals: all vertices are welded (1e-5 m) and normals recalculated, i.e. smooth shading everywhere (no hard edges).
/// </summary>
public static class UrdfModelBuilder
{
    const string ModelDir = "Assets/Robots/Models/sobit_home";
    const string UrdfPath = ModelDir + "/sobit_home.urdf";
    const string LodDir = ModelDir + "/meshes_lod";
    const string GenDir = ModelDir + "/generated";
    const string PrefabPath = "Assets/Robots/Models/SOBIT_HOME.prefab";
    const string RootName = "SOBIT_HOME";
    const float WeldQuantum = 1e-5f;
    static readonly Color DefaultColor = new Color(0.6f, 0.6f, 0.6f, 1f);

    class JointInfo { public string name, parent, child; public Vector3 pos; public Quaternion rot; }
    class MeshEntry { public Mesh mesh; public int tris; }

    static readonly Dictionary<string, MeshEntry> meshCache = new Dictionary<string, MeshEntry>();
    static readonly Dictionary<string, Material> matCache = new Dictionary<string, Material>();
    static Dictionary<string, Color> urdfMaterials, lodColors;
    static int missing, visualCount, totalTris;

    [MenuItem("Robots/Build SOBIT HOME model")]
    public static void BuildSobitHome()
    {
        bool ok = false;
        try { ok = Build(); }
        catch (Exception e) { Debug.LogError("FPV build: exception " + e); }
        if (Application.isBatchMode) EditorApplication.Exit(ok ? 0 : 1);
    }

    // ---- conversions -------------------------------------------------------------------------------------------
    static float F(string s) { return float.Parse(s, CultureInfo.InvariantCulture); }
    static Vector3 ParseVec(string s, Vector3 def)
    {
        if (string.IsNullOrWhiteSpace(s)) return def;
        var p = s.Split(new[] { ' ', '\t', '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
        return p.Length >= 3 ? new Vector3(F(p[0]), F(p[1]), F(p[2])) : def;
    }
    static Vector3 RosToUnity(Vector3 p) { return new Vector3(-p.y, p.z, p.x); }
    static Quaternion RpyToUnity(Vector3 rpy)
    {
        float roll = rpy.x * Mathf.Rad2Deg, pitch = rpy.y * Mathf.Rad2Deg, yaw = rpy.z * Mathf.Rad2Deg;
        return Quaternion.AngleAxis(-yaw, Vector3.up) * Quaternion.AngleAxis(pitch, Vector3.right) * Quaternion.AngleAxis(-roll, Vector3.forward);
    }
    static void ReadOrigin(XmlNode origin, out Vector3 pos, out Quaternion rot)
    {
        pos = Vector3.zero; rot = Quaternion.identity;
        if (origin == null) return;
        var xyz = ParseVec(origin.Attributes["xyz"]?.Value, Vector3.zero);
        var rpy = ParseVec(origin.Attributes["rpy"]?.Value, Vector3.zero);
        pos = RosToUnity(xyz);
        rot = RpyToUnity(rpy);
    }
    static Color ParseRgba(string s)
    {
        var p = s.Split(new[] { ' ', '\t', '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
        return new Color(F(p[0]), F(p[1]), F(p[2]), p.Length > 3 ? F(p[3]) : 1f);
    }

    // ---- main --------------------------------------------------------------------------------------------------
    static bool Build()
    {
        meshCache.Clear(); matCache.Clear(); missing = 0; visualCount = 0; totalTris = 0;
        if (!File.Exists(UrdfPath)) { Debug.LogError("FPV build: URDF not found " + UrdfPath); return false; }

        var doc = new XmlDocument();
        doc.Load(UrdfPath);
        var robot = doc.DocumentElement;

        urdfMaterials = new Dictionary<string, Color>();
        foreach (XmlNode m in robot.SelectNodes("material"))
        {
            var c = m.SelectSingleNode("color");
            if (c != null && c.Attributes["rgba"] != null) urdfMaterials[m.Attributes["name"].Value] = ParseRgba(c.Attributes["rgba"].Value);
        }
        lodColors = LoadLodColors();

        // clean output folder
        if (AssetDatabase.IsValidFolder(GenDir)) AssetDatabase.DeleteAsset(GenDir);
        AssetDatabase.CreateFolder(ModelDir, "generated");

        var joints = new List<JointInfo>();
        var linkNodes = new Dictionary<string, XmlNode>();
        var linkOrder = new List<string>();
        foreach (XmlNode l in robot.SelectNodes("link")) { string n = l.Attributes["name"].Value; linkNodes[n] = l; linkOrder.Add(n); }
        var childSet = new HashSet<string>();
        foreach (XmlNode j in robot.SelectNodes("joint"))
        {
            var ji = new JointInfo
            {
                name = j.Attributes["name"].Value,
                parent = j.SelectSingleNode("parent").Attributes["link"].Value,
                child = j.SelectSingleNode("child").Attributes["link"].Value
            };
            ReadOrigin(j.SelectSingleNode("origin"), out ji.pos, out ji.rot);
            joints.Add(ji);
            childSet.Add(ji.child);
        }
        string rootLink = null;
        foreach (var n in linkOrder) if (!childSet.Contains(n)) { rootLink = n; break; }
        if (rootLink == null) { Debug.LogError("FPV build: no root link"); return false; }

        var childrenOf = new Dictionary<string, List<JointInfo>>();
        foreach (var j in joints)
        {
            if (!childrenOf.TryGetValue(j.parent, out var lst)) childrenOf[j.parent] = lst = new List<JointInfo>();
            lst.Add(j);
        }

        var gos = new Dictionary<string, GameObject>();
        var root = new GameObject(RootName);
        gos[rootLink] = root;
        AddLink(root, rootLink, "", true);
        var queue = new Queue<string>();
        queue.Enqueue(rootLink);
        while (queue.Count > 0)
        {
            string p = queue.Dequeue();
            if (!childrenOf.TryGetValue(p, out var lst)) continue;
            foreach (var j in lst)
            {
                var go = new GameObject(j.child);
                go.transform.SetParent(gos[p].transform, false);
                go.transform.localPosition = j.pos;
                go.transform.localRotation = j.rot;
                gos[j.child] = go;
                AddLink(go, j.child, p, false);
                queue.Enqueue(j.child);
            }
        }
        foreach (var n in linkOrder) if (!gos.ContainsKey(n)) Debug.LogWarning("FPV build: link not reachable from root, skipped: " + n);

        foreach (var kv in gos)
            if (linkNodes.TryGetValue(kv.Key, out var ln)) BuildVisuals(kv.Value, ln);

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        MakeMeshesNonReadable();

        // transform sanity numbers (root space)
        Bounds? ub = null;
        foreach (var r in root.GetComponentsInChildren<MeshRenderer>())
        {
            var mb = r.GetComponent<MeshFilter>().sharedMesh.bounds;
            var m = root.transform.worldToLocalMatrix * r.transform.localToWorldMatrix;
            for (int i = 0; i < 8; i++)
            {
                var c = mb.center + Vector3.Scale(mb.extents, new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1));
                var w = m.MultiplyPoint3x4(c);
                if (ub == null) ub = new Bounds(w, Vector3.zero); else { var b = ub.Value; b.Encapsulate(w); ub = b; }
            }
        }
        string Pos(string f) { return gos.TryGetValue(f, out var g) ? root.transform.InverseTransformPoint(g.transform.position).ToString("F3") : "n/a"; }
        var bb = ub ?? new Bounds();
        string posSummary = " head_camera_color_frame=" + Pos("head_camera_color_frame") +
                  " arm_left_base_link=" + Pos("arm_left_base_link") + " arm_right_base_link=" + Pos("arm_right_base_link");

        var prefab = PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
        UnityEngine.Object.DestroyImmediate(root);
        if (prefab == null) { Debug.LogError("FPV build: SaveAsPrefabAsset failed"); return false; }
        AssetDatabase.SaveAssets();

        Debug.Log("FPV build: links=" + gos.Count + " visuals=" + visualCount + " missingVisuals=" + missing + " tris=" + totalTris +
                  " meshAssets=" + meshCache.Count + " materials=" + matCache.Count +
                  " boundsSize=" + bb.size.ToString("F3") + " minY=" + bb.min.y.ToString("F3") + " maxY=" + bb.max.y.ToString("F3") +
                  " boundsX=[" + bb.min.x.ToString("F3") + "," + bb.max.x.ToString("F3") + "] boundsZ=[" + bb.min.z.ToString("F3") + "," + bb.max.z.ToString("F3") + "]" +
                  posSummary + " prefab=" + PrefabPath);
        return true;
    }

    static void AddLink(GameObject go, string frame, string parent, bool isRoot)
    {
        var rl = go.AddComponent<RobotLink>();
        rl.frame = frame; rl.parentFrame = parent; rl.isRoot = isRoot;
    }

    // ---- visuals -----------------------------------------------------------------------------------------------
    class Variant { public string file; public string suffix; public string colorKey; }

    static void BuildVisuals(GameObject linkGo, XmlNode link)
    {
        int vi = 0;
        foreach (XmlNode vis in link.SelectNodes("visual"))
        {
            int idx = vi++;
            var meshNode = vis.SelectSingleNode("geometry/mesh");
            if (meshNode == null) { Debug.LogWarning("FPV build: non-mesh visual skipped on " + link.Attributes["name"].Value); continue; }
            string uri = meshNode.Attributes["filename"].Value;
            string scaleStr = meshNode.Attributes["scale"]?.Value;
            Vector3 scale = ParseVec(scaleStr, Vector3.one);
            ReadOrigin(vis.SelectSingleNode("origin"), out var pos, out var rot);

            Color? explicitColor = null;
            var matNode = vis.SelectSingleNode("material");
            if (matNode != null)
            {
                var c = matNode.SelectSingleNode("color");
                if (c != null && c.Attributes["rgba"] != null) explicitColor = ParseRgba(c.Attributes["rgba"].Value);
                else if (matNode.Attributes["name"] != null && urdfMaterials.TryGetValue(matNode.Attributes["name"].Value, out var uc)) explicitColor = uc;
            }

            var variants = ResolveVariants(uri);
            if (variants.Count == 0)
            {
                missing++;
                Debug.LogWarning("FPV build: mesh missing, visual skipped: " + link.Attributes["name"].Value + " -> " + uri);
                continue;
            }
            foreach (var v in variants)
            {
                var entry = GetMesh(v.file, scale, scaleStr);
                var go = new GameObject(variants.Count == 1 ? "visual_" + idx : "visual_" + idx + "_" + v.suffix);
                go.transform.SetParent(linkGo.transform, false);
                go.transform.localPosition = pos;
                go.transform.localRotation = rot;
                var mf = go.AddComponent<MeshFilter>(); mf.sharedMesh = entry.mesh;
                var mr = go.AddComponent<MeshRenderer>();
                Color col = DefaultColor;
                if (v.colorKey != null && lodColors.TryGetValue(v.colorKey, out var lc)) col = lc; // DAE-derived variants keep their own colour
                else if (explicitColor.HasValue) col = explicitColor.Value;
                mr.sharedMaterial = GetMaterial(col);
                mr.shadowCastingMode = ShadowCastingMode.Off;
                mr.receiveShadows = false;
                visualCount++;
                totalTris += entry.tris;
            }
        }
    }

    static string ToLodRelative(string uri)
    {
        var m = Regex.Match(uri, @"^package://([^/]+)/meshes/(.+)$");
        if (m.Success) return m.Groups[1].Value == "sobit_home_description" ? m.Groups[2].Value : "ext/" + m.Groups[1].Value + "/" + m.Groups[2].Value;
        m = Regex.Match(uri, @"/share/([^/]+)/meshes/(.+)$");
        if (m.Success) return m.Groups[1].Value == "sobit_home_description" ? m.Groups[2].Value : "ext/" + m.Groups[1].Value + "/" + m.Groups[2].Value;
        return null;
    }

    static List<Variant> ResolveVariants(string uri)
    {
        var res = new List<Variant>();
        string rel = ToLodRelative(uri);
        if (rel == null) return res;
        string path = LodDir + "/" + rel;
        if (File.Exists(path)) { res.Add(new Variant { file = path, suffix = "", colorKey = null }); return res; }
        string dir = Path.GetDirectoryName(path).Replace('\\', '/');
        string name = Path.GetFileName(path);
        if (!Directory.Exists(dir)) return res;
        var files = Directory.GetFiles(dir, name + ".*.stl");
        Array.Sort(files, StringComparer.Ordinal);
        foreach (var f in files)
        {
            string fn = Path.GetFileName(f);
            string suffix = fn.Substring(name.Length + 1, fn.Length - name.Length - 1 - 4);
            res.Add(new Variant { file = f.Replace('\\', '/'), suffix = suffix, colorKey = rel + "." + suffix });
        }
        return res;
    }

    static Dictionary<string, Color> LoadLodColors()
    {
        var d = new Dictionary<string, Color>();
        string p = LodDir + "/colors.json";
        if (!File.Exists(p)) { Debug.LogWarning("FPV build: colors.json missing"); return d; }
        string num = @"(-?[0-9.eE+\-]+)";
        foreach (Match m in Regex.Matches(File.ReadAllText(p), "\"([^\"]+)\"\\s*:\\s*\\[\\s*" + num + "\\s*,\\s*" + num + "\\s*,\\s*" + num + "(?:\\s*,\\s*" + num + ")?\\s*\\]"))
            d[m.Groups[1].Value] = new Color(F(m.Groups[2].Value), F(m.Groups[3].Value), F(m.Groups[4].Value), m.Groups[5].Success ? F(m.Groups[5].Value) : 1f);
        return d;
    }

    // ---- meshes ------------------------------------------------------------------------------------------------
    static MeshEntry GetMesh(string file, Vector3 scale, string scaleStr)
    {
        string key = file + "|" + scale.x.ToString("R", CultureInfo.InvariantCulture) + "," + scale.y.ToString("R", CultureInfo.InvariantCulture) + "," + scale.z.ToString("R", CultureInfo.InvariantCulture);
        if (meshCache.TryGetValue(key, out var e)) return e;

        ReadBinaryStl(file, out var tri);   // ROS axes, units as stored
        int nTri = tri.Length / 3;
        bool flip = scale.x * scale.y * scale.z < 0f;  // negative scale mirrors again
        var verts = new List<Vector3>(nTri * 3);
        var idx = new List<int>(nTri * 3);
        var weld = new Dictionary<(long, long, long), int>();
        var ids = new int[3];
        for (int t = 0; t < nTri; t++)
        {
            for (int k = 0; k < 3; k++)
            {
                var p = RosToUnity(Vector3.Scale(tri[t * 3 + k], scale));
                var q = (Mathf.RoundToInt(p.x / WeldQuantum), Mathf.RoundToInt(p.y / WeldQuantum), Mathf.RoundToInt(p.z / WeldQuantum));
                var qk = ((long)q.Item1, (long)q.Item2, (long)q.Item3);
                if (!weld.TryGetValue(qk, out int id)) { id = verts.Count; verts.Add(p); weld[qk] = id; }
                ids[k] = id;
            }
            if (ids[0] == ids[1] || ids[1] == ids[2] || ids[0] == ids[2]) continue; // degenerate after welding
            // axis change (-y,z,x) is a mirror -> swap 1,2 ; a negative URDF scale mirrors once more -> keep
            if (!flip) { idx.Add(ids[0]); idx.Add(ids[2]); idx.Add(ids[1]); }
            else { idx.Add(ids[0]); idx.Add(ids[1]); idx.Add(ids[2]); }
        }

        var mesh = new Mesh { name = Path.GetFileNameWithoutExtension(file) + "_" + meshCache.Count };
        if (verts.Count > 65000) mesh.indexFormat = IndexFormat.UInt32;
        mesh.SetVertices(verts);
        mesh.SetTriangles(idx, 0);
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        AssetDatabase.CreateAsset(mesh, GenDir + "/" + SafeName(mesh.name) + ".asset");
        e = new MeshEntry { mesh = mesh, tris = idx.Count / 3 };
        meshCache[key] = e;
        return e;
    }

    static string SafeName(string s) { return Regex.Replace(s, @"[^A-Za-z0-9_\-.]", "_"); }

    static void MakeMeshesNonReadable()
    {
        foreach (var e in meshCache.Values)
        {
            var so = new SerializedObject(e.mesh);
            var p = so.FindProperty("m_IsReadable");
            if (p == null) { Debug.LogWarning("FPV build: m_IsReadable not found, mesh stays readable"); return; }
            p.boolValue = false;
            so.ApplyModifiedPropertiesWithoutUndo();
        }
        AssetDatabase.SaveAssets();
    }

    static void ReadBinaryStl(string path, out Vector3[] tri)
    {
        var b = File.ReadAllBytes(path);
        uint n = BitConverter.ToUInt32(b, 80);
        if (84 + 50L * n > b.Length) throw new InvalidDataException("not a binary STL: " + path);
        tri = new Vector3[n * 3];
        int o = 84;
        for (int t = 0; t < n; t++)
        {
            o += 12; // normal
            for (int k = 0; k < 3; k++)
            {
                tri[t * 3 + k] = new Vector3(BitConverter.ToSingle(b, o), BitConverter.ToSingle(b, o + 4), BitConverter.ToSingle(b, o + 8));
                o += 12;
            }
            o += 2;
        }
    }

    // ---- materials ---------------------------------------------------------------------------------------------
    static Material GetMaterial(Color c)
    {
        // The robot screen's StudioEnvironment uses flat ambient light, so URDF "black" (0.1) renders
        // near-invisible. Keep the hue, raise the brightest channel to at least 0.3.
        float mx = Mathf.Max(c.r, Mathf.Max(c.g, c.b));
        if (mx < 0.3f)
        {
            float k = mx > 1e-4f ? 0.3f / mx : 0f;
            c = mx > 1e-4f ? new Color(c.r * k, c.g * k, c.b * k, c.a) : new Color(0.3f, 0.3f, 0.3f, c.a);
        }
        string key = ColorUtility.ToHtmlStringRGBA(c);
        if (matCache.TryGetValue(key, out var m)) return m;
        var sh = Shader.Find("Universal Render Pipeline/Simple Lit");
        if (sh == null) { Debug.LogError("FPV build: Simple Lit shader not found"); sh = Shader.Find("Universal Render Pipeline/Lit"); }
        m = new Material(sh) { name = "robot_" + key };
        m.SetColor("_BaseColor", c);
        m.color = c;
        if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", 0.25f);
        if (c.a < 0.999f && m.HasProperty("_Surface")) { m.SetFloat("_Surface", 1f); }
        AssetDatabase.CreateAsset(m, GenDir + "/" + m.name + ".mat");
        matCache[key] = m;
        return m;
    }
}
#endif
