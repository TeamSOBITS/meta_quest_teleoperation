using UnityEngine;

/// <summary>
/// Sets up the robot screen around the camera blocks: studio surroundings, the HUD bar
/// (ROS IP + controls), the camera block dragger, controller hints and the optional
/// "lazy follow" mode. Runs after ImageSubscriber so it can use the blocks it created.
/// </summary>
[DefaultExecutionOrder(100)]
public class TeleopHud : MonoBehaviour
{
    public QuestControllerPublisher publisher;
    public ImageSubscriber images;

    // Panels follow this transform; defaults to the main camera.
    public Transform hudParent;

    const string LazyFollowKey = "LazyFollow";

    Transform _bar;
    HeadFollower _follower;

    public bool LazyFollow { get; private set; }

    void Start()
    {
        StudioEnvironment.Apply(Camera.main);

        if (hudParent == null && Camera.main != null)
            hudParent = Camera.main.transform;

        _bar = HudBar.Create(hudParent, publisher, images, this).transform;

        var dragger = gameObject.AddComponent<PanelDragger>();
        dragger.publisher = publisher;
        dragger.images = images;

        ControllerHints.Create(hudParent);

        if (PlayerPrefs.GetInt(LazyFollowKey, 0) == 1)
            SetLazyFollow(true);
    }

    // Off (default): the HUD is rigidly attached to the head. On: it follows the head with a
    // dead zone and a short delay (see HeadFollower). Panel positions relative to the head
    // are kept either way, so switching never moves a panel within the view.
    public void SetLazyFollow(bool on)
    {
        if (on == LazyFollow) return;
        LazyFollow = on;
        PlayerPrefs.SetInt(LazyFollowKey, on ? 1 : 0);
        PlayerPrefs.Save();

        if (on)
        {
            if (_follower == null) _follower = HeadFollower.Create(hudParent);
            _follower.Snap();
        }
        Transform parent = on ? _follower.transform : hudParent;

        foreach (var panel in images.Panels)
            panel.transform.SetParent(parent, false);
        _bar.SetParent(parent, false);
        images.panelParent = parent;
    }
}
