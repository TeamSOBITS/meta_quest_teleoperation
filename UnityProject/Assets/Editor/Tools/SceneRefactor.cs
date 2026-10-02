// One-off scene refactor (Part 2 / P4), kept as documentation of what was done to the scenes and prefabs.
// Run on a scratch copy of the project, then copy the resulting .unity/.prefab/.meta files back:
//   Unity -batchmode -projectPath <copy> -executeMethod SceneRefactor.Dump      (hierarchy + missing refs of the rig variant and scenes)
//   Unity -batchmode -projectPath <copy> -executeMethod SceneRefactor.Rig       (step 1: Assets/Prefabs/XR Rig.prefab, scenes use it)
//   Unity -batchmode -projectPath <copy> -executeMethod SceneRefactor.Common    (step 2: Assets/Prefabs/Common.prefab, scenes use it)
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class SceneRefactor
{
    const string Variant = "Assets/VRTemplateAssets/Prefabs/Setup/Complete XR Origin Set Up Hands Variant.prefab";
    static readonly string[] Scenes = { "Assets/Scenes/RobotSelectionScene.unity", "Assets/Scenes/TeleopScene.unity" };

    static string PathOf(Transform t) => t.parent == null ? t.name : PathOf(t.parent) + "/" + t.name;

    static void DumpRoots(GameObject[] roots, StringBuilder sb)
    {
        foreach (var root in roots)
            foreach (var t in root.GetComponentsInChildren<Transform>(true))
            {
                var go = t.gameObject;
                var comps = go.GetComponents<Component>().Select(c => c == null ? "<MISSING SCRIPT>" : c.GetType().Name);
                sb.AppendLine($"{PathOf(t)} [{(go.activeSelf ? "on" : "off")}] tag={go.tag} :: {string.Join(", ", comps)}");
                foreach (var c in go.GetComponents<Component>())
                {
                    if (c == null) continue;
                    var p = new SerializedObject(c).GetIterator();
                    while (p.NextVisible(true))
                        if (p.propertyType == SerializedPropertyType.ObjectReference && p.objectReferenceValue == null && p.objectReferenceInstanceIDValue != 0)
                            sb.AppendLine($"    MISSING REF {c.GetType().Name}.{p.propertyPath} id={p.objectReferenceInstanceIDValue}");
                }
            }
    }

    public static void Dump()
    {
        var sb = new StringBuilder();
        sb.AppendLine("##### " + Variant);
        var c = PrefabUtility.LoadPrefabContents(Variant); DumpRoots(new[] { c }, sb); PrefabUtility.UnloadPrefabContents(c);
        foreach (var s in Scenes)
        {
            sb.AppendLine("##### " + s);
            var sc = EditorSceneManager.OpenScene(s); DumpRoots(sc.GetRootGameObjects(), sb);
            foreach (var r in sc.GetRootGameObjects())
            {
                if (!PrefabUtility.IsAnyPrefabInstanceRoot(r)) continue;
                sb.AppendLine("  OVERRIDES of " + r.name + " (base " + AssetDatabase.GetAssetPath(PrefabUtility.GetCorrespondingObjectFromSource(r)) + ", pos " + r.transform.position + " rot " + r.transform.eulerAngles + ")");
                foreach (var m in PrefabUtility.GetPropertyModifications(r) ?? new PropertyModification[0])
                {
                    var t = m.target;
                    if (m.propertyPath.StartsWith("m_Size") || m.propertyPath.StartsWith("m_Anchor")) continue;
                    sb.AppendLine($"    {(t is Component cc ? PathOf(cc.transform) + ":" + cc.GetType().Name : t is GameObject g ? PathOf(g.transform) : t?.name)} {m.propertyPath} = {m.value} {m.objectReference}");
                }
            }
        }
        var dir = System.Environment.GetEnvironmentVariable("VERIFY_SHOTS") ?? "/tmp";
        File.WriteAllText(Path.Combine(dir, "refactor_dump.txt"), sb.ToString());
        Debug.Log("[Refactor] dump written");
    }

    // ------------------------------------------------------------------ step 1: rig prefab
    const string RigPrefab = "Assets/Prefabs/XR Rig.prefab";
    const string RigName = "XR Origin (XR Rig)";
    const string ActionsAsset = "Assets/Samples/XR Interaction Toolkit/3.1.1/Starter Assets/XRI Default Input Actions.inputactions";

    // Objects removed from the rig (locomotion, teleport, gaze, tutorial callouts - the app has none of these).
    static readonly string[] RemoveObjects =
    {
        "Locomotion", "Camera Offset/Main Camera/TunnelingVignette",
        "Camera Offset/Gaze Interactor", "Camera Offset/Gaze Stabilized",
        "Camera Offset/Left Controller/Teleport Interactor", "Camera Offset/Right Controller/Teleport Interactor",
        "Camera Offset/Left Controller Teleport Stabilized Origin", "Camera Offset/Right Controller Teleport Stabilized Origin",
        "Camera Offset/Left Controller/Affordance Callouts Left", "Camera Offset/Right Controller/Affordance Callouts Right",
    };

    static Dictionary<string, bool> NonNullRefs(GameObject root)
    {
        var d = new Dictionary<string, bool>();
        foreach (var c in root.GetComponentsInChildren<Component>(true))
        {
            if (c == null) continue;
            var p = new SerializedObject(c).GetIterator();
            while (p.NextVisible(true))
                if (p.propertyType == SerializedPropertyType.ObjectReference) d[c.GetInstanceID() + "|" + p.propertyPath] = p.objectReferenceValue != null;
        }
        return d;
    }

    // Removes array entries that pointed to something deleted; logs every other reference that became null.
    static void CleanDeadReferences(GameObject root, Dictionary<string, bool> before)
    {
        foreach (var c in root.GetComponentsInChildren<Component>(true))
        {
            if (c == null) continue;
            var so = new SerializedObject(c);
            bool changed = true;
            while (changed)
            {
                changed = false;
                var p = so.GetIterator();
                while (p.NextVisible(true))
                {
                    if (p.propertyType != SerializedPropertyType.ObjectReference || p.objectReferenceValue != null) continue;
                    if (!before.TryGetValue(c.GetInstanceID() + "|" + p.propertyPath, out var was) || !was) continue;
                    int bracket = p.propertyPath.LastIndexOf(".Array.data[");
                    if (bracket > 0 && p.propertyPath.EndsWith("]"))
                    {
                        var arr = so.FindProperty(p.propertyPath.Substring(0, bracket));
                        int idx = int.Parse(p.propertyPath.Substring(bracket + 12).TrimEnd(']'));
                        Debug.Log($"[Refactor] removing dead entry {PathOf(c.transform)}:{c.GetType().Name}.{p.propertyPath}");
                        arr.DeleteArrayElementAtIndex(idx);
                        // Re-key the following entries lazily: restart the scan with the keys re-read.
                        so.ApplyModifiedPropertiesWithoutUndo(); changed = true;
                        foreach (var k in before.Keys.Where(k => k.StartsWith(c.GetInstanceID() + "|" + arr.propertyPath + ".Array.data[")).ToList()) before.Remove(k);
                        var p2 = so.FindProperty(arr.propertyPath);
                        for (int i = 0; i < p2.arraySize; i++) before[c.GetInstanceID() + "|" + p2.propertyPath + ".Array.data[" + i + "]"] = p2.GetArrayElementAtIndex(i).objectReferenceValue != null;
                        break;
                    }
                    Debug.Log($"[Refactor] reference now null {PathOf(c.transform)}:{c.GetType().Name}.{p.propertyPath}");
                }
            }
        }
    }

    public static void Rig()
    {
        Directory.CreateDirectory("Assets/Prefabs");
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        var root = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(Variant));
        PrefabUtility.UnpackPrefabInstance(root, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
        root.name = RigName;
        var before = NonNullRefs(root);

        foreach (var rel in RemoveObjects)
        {
            var t = root.transform.Find(rel);
            if (t == null) { Debug.LogError("[Refactor] not found: " + rel); continue; }
            Object.DestroyImmediate(t.gameObject);
        }
        // Root + controller components that only served locomotion / gaze / the callouts.
        RemoveByName(root, "XRGazeAssistance"); RemoveByName(root, "CharacterController");
        foreach (var side in new[] { "Left", "Right" })
            RemoveByName(root.transform.Find("Camera Offset/" + side + " Controller").gameObject, "CalloutGazeController");

        CleanDeadReferences(root, before);

        // The controller input manager keeps its UI-scroll action; everything locomotion related is cleared.
        foreach (var side in new[] { "Left", "Right" })
        {
            var cam = root.transform.Find("Camera Offset/" + side + " Controller").GetComponent("ControllerInputActionManager");
            var so = new SerializedObject(cam);
            foreach (var f in new[] { "m_TeleportInteractor", "m_TeleportMode", "m_TeleportModeCancel", "m_Turn", "m_SnapTurn", "m_Move" })
                so.FindProperty(f).objectReferenceValue = null;
            foreach (var f in new[] { "m_SmoothMotionEnabled", "m_SmoothTurnEnabled" }) so.FindProperty(f).boolValue = false;
            so.ApplyModifiedPropertiesWithoutUndo();
        }
        var iam = new SerializedObject(root.GetComponent("InputActionManager")); var assets = iam.FindProperty("m_ActionAssets");
        var asset = AssetDatabase.LoadAssetAtPath<Object>(ActionsAsset);
        for (int i = 0; i < assets.arraySize; i++)
            Debug.Log("[Refactor] InputActionManager asset before: " + (assets.GetArrayElementAtIndex(i).objectReferenceValue ? AssetDatabase.GetAssetPath(assets.GetArrayElementAtIndex(i).objectReferenceValue) : "null"));
        assets.arraySize = 1; assets.GetArrayElementAtIndex(0).objectReferenceValue = asset; iam.ApplyModifiedPropertiesWithoutUndo();

        var sb = new StringBuilder(); DumpRoots(new[] { root }, sb);
        var dir = System.Environment.GetEnvironmentVariable("VERIFY_SHOTS") ?? "/tmp";
        File.WriteAllText(Path.Combine(dir, "rig_dump.txt"), sb.ToString());
        PrefabUtility.SaveAsPrefabAsset(root, RigPrefab);
        Object.DestroyImmediate(root);
        AssetDatabase.SaveAssets();

        // Both scenes: the rig instance becomes the new prefab (same name, same transform, same sibling index).
        foreach (var s in Scenes) ReplaceRig(s);
        Debug.Log("[Refactor] rig done");
    }

    static void RemoveByName(GameObject go, string typeName)
    {
        foreach (var c in go.GetComponents<Component>())
            if (c != null && c.GetType().Name == typeName) Object.DestroyImmediate(c);
    }

    static GameObject FindRig(Scene sc) => sc.GetRootGameObjects().First(g => AssetDatabase.GetAssetPath(PrefabUtility.GetCorrespondingObjectFromSource(g)) == Variant);

    static void ReplaceRig(string scenePath)
    {
        var sc = EditorSceneManager.OpenScene(scenePath);
        var old = FindRig(sc);
        var pos = old.transform.position; var rot = old.transform.rotation; int sib = old.transform.GetSiblingIndex();
        var oldComps = old.GetComponentsInChildren<Component>(true).Where(c => c != null).ToList();
        // Remember scene components that point into the old rig, to re-point them at the same component in the new one.
        var links = new List<(SerializedObject so, string prop, string path, System.Type type, int nth)>();
        foreach (var c in sc.GetRootGameObjects().Where(g => g != old).SelectMany(g => g.GetComponentsInChildren<Component>(true)))
        {
            if (c == null) continue;
            var so = new SerializedObject(c); var p = so.GetIterator();
            while (p.NextVisible(true))
            {
                if (p.propertyType != SerializedPropertyType.ObjectReference || p.objectReferenceValue == null) continue;
                var o = p.objectReferenceValue; Transform t = o is Component cc ? cc.transform : o is GameObject g ? g.transform : null;
                if (t == null || !t.IsChildOf(old.transform)) continue;
                string rel = PathOf(t).Substring(old.name.Length);
                var same = o is Component c2 ? t.GetComponents(c2.GetType()).ToList().IndexOf(c2) : -1;
                links.Add((so, p.propertyPath, rel, o.GetType(), same));
                Debug.Log($"[Refactor] {scenePath}: {c.GetType().Name}.{p.propertyPath} -> rig{rel} ({o.GetType().Name})");
            }
        }
        Object.DestroyImmediate(old);
        var inst = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(RigPrefab), sc);
        inst.name = RigName; inst.transform.SetPositionAndRotation(pos, rot); inst.transform.SetSiblingIndex(sib);
        foreach (var l in links)
        {
            var t = l.path.Length == 0 ? inst.transform : inst.transform.Find(l.path.TrimStart('/'));
            Object target = null;
            if (t != null) target = typeof(Component).IsAssignableFrom(l.type) ? (Object)t.GetComponents(l.type)[Mathf.Max(0, l.nth)] : t.gameObject;
            if (target == null) { Debug.LogError("[Refactor] could not re-point " + l.prop + " -> " + l.path); continue; }
            l.so.FindProperty(l.prop).objectReferenceValue = target; l.so.ApplyModifiedPropertiesWithoutUndo();
        }
        EditorSceneManager.MarkSceneDirty(sc); EditorSceneManager.SaveScene(sc);
    }

    // ------------------------------------------------------------------ step 2: Common prefab
    const string CommonPrefab = "Assets/Prefabs/Common.prefab";

    // TeleopScene's EventSystem (Starter Assets XRI Default Input Actions), Directional Light (realtime) and the Grid floor
    // (tag Floor) become one prefab; the selection scene's own EventSystem / light / "Ground" are replaced by an instance.
    public static void Common()
    {
        var sc = EditorSceneManager.OpenScene(Scenes[1]);
        var roots = sc.GetRootGameObjects();
        var es = roots.First(g => g.name == "EventSystem");
        var lightRoot = roots.First(g => g.name == "Lighting"); var light = lightRoot.transform.Find("Directional Light");
        var envRoot = roots.First(g => g.name == "Environment"); var grid = envRoot.transform.Find("Grid");
        var common = new GameObject("Common");
        es.transform.SetParent(common.transform, true);
        light.SetParent(common.transform, true);
        grid.SetParent(common.transform, true); grid.name = "Floor";
        Object.DestroyImmediate(lightRoot); Object.DestroyImmediate(envRoot);
        PrefabUtility.SaveAsPrefabAssetAndConnect(common, CommonPrefab, InteractionMode.AutomatedAction);
        EditorSceneManager.SaveScene(sc);

        var sel = EditorSceneManager.OpenScene(Scenes[0]);
        foreach (var n in new[] { "EventSystem", "Directional Light", "Ground" })
        {
            var g = sel.GetRootGameObjects().FirstOrDefault(x => x.name == n);
            if (g == null) Debug.LogError("[Refactor] selection scene: no " + n); else Object.DestroyImmediate(g);
        }
        PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(CommonPrefab), sel);
        EditorSceneManager.SaveScene(sel);
        Debug.Log("[Refactor] common done");
    }
}
