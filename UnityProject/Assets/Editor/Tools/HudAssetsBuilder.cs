using UnityEditor;
using UnityEngine;

// Creates the HUD's shader materials and the HudAssets asset that references them (Resources/HudAssets).
// Idempotent; also run headlessly: Unity -batchmode -executeMethod HudAssetsBuilder.Run
public static class HudAssetsBuilder
{
    const string SurroundShader = "Assets/Shaders/HudSurround.shader", UndistortShader = "Assets/Shaders/UndistortImage.shader";
    const string SurroundMaterial = "Assets/Materials/HudSurround.mat", UndistortMaterial = "Assets/Materials/UndistortImage.mat";
    const string AssetsPath = "Assets/Resources/HudAssets.asset";

    [MenuItem("Tools/HUD/Create assets")]
    public static void Run()
    {
        AssetDatabase.Refresh();
        var assets = AssetDatabase.LoadAssetAtPath<HudAssets>(AssetsPath);
        if (assets == null)
        {
            assets = ScriptableObject.CreateInstance<HudAssets>();
            AssetDatabase.CreateAsset(assets, AssetsPath);
        }
        assets.surround = Material(SurroundMaterial, SurroundShader);
        assets.undistort = Material(UndistortMaterial, UndistortShader);
        EditorUtility.SetDirty(assets);
        AssetDatabase.SaveAssets();
        Debug.Log($"HudAssetsBuilder: {AssetsPath} -> {assets.surround.shader.name}, {assets.undistort.shader.name}");
    }

    static Material Material(string path, string shaderPath)
    {
        var shader = AssetDatabase.LoadAssetAtPath<Shader>(shaderPath);
        if (shader == null) throw new System.Exception("shader missing: " + shaderPath);
        var m = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (m == null)
        {
            m = new Material(shader);
            AssetDatabase.CreateAsset(m, path);
        }
        m.shader = shader;
        if (path == SurroundMaterial) m.renderQueue = 1000;   // Background
        EditorUtility.SetDirty(m);
        return m;
    }
}
