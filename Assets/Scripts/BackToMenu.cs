using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class BackToMenu : MonoBehaviour
{
    public DragNShoot dragNShoot;
    public Spawner spawner;

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            if (dragNShoot != null)
            {
                dragNShoot.ResetScene();
            }
            if (spawner != null)
            {
                spawner.ResetScene();
            }
            SceneManager.LoadScene("MenuScene");
        }
    }
}
