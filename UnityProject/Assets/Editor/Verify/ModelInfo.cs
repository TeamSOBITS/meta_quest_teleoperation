// Verify harness: Prints bounds and mesh info of a model; diagnostic, not a pass/fail suite.
// Run: tools/verify.sh --suite ModelInfo   (or Unity -batchmode -projectPath <copy> -executeMethod ModelInfo.Run)
using UnityEditor; using UnityEngine; using System.Linq;
public static class ModelInfo {
  public static void Run() {
    var p = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Robots/Models/SOBIT_HOME.prefab");
    var g = (GameObject)PrefabUtility.InstantiatePrefab(p);
    foreach (var r in g.GetComponentsInChildren<MeshRenderer>()) {
      var b = r.bounds; var m = r.GetComponent<MeshFilter>().sharedMesh;
      Debug.Log("MI " + r.transform.parent.name + "/" + r.name + " " + m.name + " min=" + b.min.ToString("F2") + " max=" + b.max.ToString("F2"));
    }
    foreach (var l in g.GetComponentsInChildren<RobotLink>()) Debug.Log("LK " + l.frame + " " + g.transform.InverseTransformPoint(l.transform.position).ToString("F3"));
    EditorApplication.Exit(0);
  }
}
