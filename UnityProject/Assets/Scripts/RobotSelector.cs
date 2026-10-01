using UnityEngine;
using UnityEngine.SceneManagement;

public class RobotSelector : MonoBehaviour
{
    public void Select(RobotProfile profile)
    {
        RobotProfile.Selected = profile;
        SceneManager.LoadScene(profile.sceneName);
    }
}
