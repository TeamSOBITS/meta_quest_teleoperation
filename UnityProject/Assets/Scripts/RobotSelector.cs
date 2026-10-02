using UnityEngine;
using UnityEngine.SceneManagement;

public class RobotSelector : MonoBehaviour
{
    public void LoadScene(string sceneName)
    {
        SceneManager.LoadScene(sceneName);
    }
}
