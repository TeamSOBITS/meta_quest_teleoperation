using UnityEngine;
using UnityEngine.XR.ARFoundation;

/// <summary>
/// Optional mixed-reality view: shows the real room (Quest passthrough) instead of the dark
/// studio background. On Quest this is driven by AR Foundation: an ARSession plus an
/// ARCameraManager on the main camera, enabled while passthrough is on, with a camera that
/// clears to a fully transparent colour (the passthrough video is composited behind it).
/// The grid floor is hidden while it is on, since a floor drawn over the real one only gets
/// in the way.
///
/// The choice is remembered across sessions (PlayerPrefs). Every call is null-guarded and
/// the AR components are harmless in the Editor, where there is no AR runtime: there it only
/// changes the camera clear colour and the floor.
/// </summary>
public static class PassthroughMode
{
    const string PrefKey = "Passthrough";
    // The floor is called "Grid" in the teleop scene and "Ground" in the robot selection scene.
    static readonly string[] FloorNames = { "Grid", "Ground" };

    public static bool Enabled
    {
        get => PlayerPrefs.GetInt(PrefKey, 0) == 1;
        set
        {
            PlayerPrefs.SetInt(PrefKey, value ? 1 : 0);
            PlayerPrefs.Save();
        }
    }

    public static void Apply(Camera cam, bool on)
    {
        if (cam != null)
        {
            if (on)
            {
                EnsureSession();
                var manager = cam.GetComponent<ARCameraManager>();
                if (manager == null) manager = cam.gameObject.AddComponent<ARCameraManager>();
                // Start passthrough only once the AR session is ready; enabling it earlier makes
                // the runtime reject the start (XR_ERROR_UNEXPECTED_STATE_PASSTHROUGH_FB) and retry.
                manager.enabled = SessionReady;
                if (!SessionReady) WaitForSession();

                // Passthrough is layered behind the rendered image, so the clear colour must
                // be transparent; keep the studio rgb so toggling off restores the same look.
                var clear = StudioEnvironment.Background;
                clear.a = 0f;
                cam.clearFlags = CameraClearFlags.SolidColor;
                cam.backgroundColor = clear;
                RenderSettings.skybox = null;
            }
            else
            {
                // Disable rather than destroy: switching back and forth stays cheap.
                var manager = cam.GetComponent<ARCameraManager>();
                if (manager != null) manager.enabled = false;

                cam.clearFlags = CameraClearFlags.SolidColor;
                cam.backgroundColor = StudioEnvironment.Background;
            }
        }
        SetFloorVisible(!on);
    }

    static bool SessionReady => ARSession.state >= ARSessionState.Ready;
    static bool _waiting;

    static void WaitForSession()
    {
        if (_waiting) return;
        _waiting = true;
        ARSession.stateChanged += OnSessionStateChanged;
    }

    static void OnSessionStateChanged(ARSessionStateChangedEventArgs args)
    {
        if (args.state < ARSessionState.Ready) return;
        ARSession.stateChanged -= OnSessionStateChanged;
        _waiting = false;
        var cam = Camera.main;
        var manager = cam != null ? cam.GetComponent<ARCameraManager>() : null;
        if (manager != null) manager.enabled = Enabled;
    }

    static void EnsureSession()
    {
        if (Object.FindFirstObjectByType<ARSession>(FindObjectsInactive.Include) != null) return;
        new GameObject("AR Session").AddComponent<ARSession>();
    }

    // The floor is deactivated while passthrough is on, so GameObject.Find would not see it
    // again when switching back; search including inactive scene objects instead.
    static void SetFloorVisible(bool visible)
    {
        foreach (var t in Resources.FindObjectsOfTypeAll<Transform>())
        {
            if (t is RectTransform || !t.gameObject.scene.IsValid() || t.hideFlags != HideFlags.None) continue;
            if (System.Array.IndexOf(FloorNames, t.name) >= 0)
                t.gameObject.SetActive(visible);
        }
    }
}
