using UnityEngine;

/// <summary>
/// Creates the head-locked HUD bar (ROS IP + controls) and the camera block dragger.
/// Runs after ImageSubscriber so both can use the camera blocks it created.
/// </summary>
[DefaultExecutionOrder(100)]
public class TeleopHud : MonoBehaviour
{
    public QuestControllerPublisher publisher;
    public ImageSubscriber images;

    // Panels follow this transform; defaults to the main camera.
    public Transform hudParent;

    void Start()
    {
        if (hudParent == null && Camera.main != null)
            hudParent = Camera.main.transform;

        HudBar.Create(hudParent, publisher, images);

        var dragger = gameObject.AddComponent<PanelDragger>();
        dragger.publisher = publisher;
        dragger.images = images;
    }
}
