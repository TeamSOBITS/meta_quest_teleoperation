using UnityEngine;

/// <summary>
/// Creates the head-locked ROS IP block and control panel. Runs after ImageSubscriber
/// so the control panel can list the camera blocks it created.
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

        RosIpPanel.Create(hudParent, publisher);
        ControlPanel.Create(hudParent, publisher, images);
    }
}
