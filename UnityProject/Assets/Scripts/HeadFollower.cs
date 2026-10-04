using UnityEngine;

/// <summary>
/// "Lazy follow" anchor for the HUD: stays at the head's position, but only turns with the
/// head once the head has turned more than <see cref="DeadZoneDegrees"/>, then catches up
/// smoothly. UI rigidly attached to the head is a known cause of VR discomfort; this keeps
/// the panels steady during small head movements while never losing them.
/// </summary>
public class HeadFollower : MonoBehaviour
{
    public const float DeadZoneDegrees = 12f;
    const float SettleDegrees = 1f;
    const float CatchUpSpeed = 4f;   // 1/s, exponential approach

    Transform _head;
    bool _following;

    public static HeadFollower Create(Transform head)
    {
        var follower = new GameObject("HUD Anchor (lazy follow)").AddComponent<HeadFollower>();
        follower._head = head;
        follower.Snap();
        return follower;
    }

    // Jump to the head's current pose.
    public void Snap()
    {
        transform.SetPositionAndRotation(_head.position, _head.rotation);
        _following = false;
    }

    void LateUpdate()
    {
        transform.position = _head.position;

        float angle = Quaternion.Angle(transform.rotation, _head.rotation);
        if (angle > DeadZoneDegrees) _following = true;
        if (!_following) return;

        float k = 1f - Mathf.Exp(-CatchUpSpeed * Time.deltaTime);
        transform.rotation = Quaternion.Slerp(transform.rotation, _head.rotation, k);
        if (angle < SettleDegrees) _following = false;
    }
}
