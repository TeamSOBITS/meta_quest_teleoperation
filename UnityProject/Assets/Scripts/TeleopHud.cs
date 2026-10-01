using UnityEngine;

/// <summary>
/// Creates the head-locked HUD bar (ROS IP + controls). Runs after ImageSubscriber
/// so the bar can list the camera blocks it created.
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
    }
}
