using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class Menu : MonoBehaviour
{
    public void SandBox()
    {
        SceneManager.LoadScene("SandBox");
    }

    public void Washer()
    {
        SceneManager.LoadScene("Washer");
    }

    public void QuitGame()
    {
        Application.Quit();
    }
}
