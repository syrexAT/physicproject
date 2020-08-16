using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Spawner : MonoBehaviour
{
    public GameObject circle;
    public GameObject rectangle;
    public List<GameObject> spawnedCircles = new List<GameObject>();

    public Vector2 force;
    public PhysicsBody pB;

    public float radius;

    public GameObject[] circlesToDelete;
    public float maxSpawnedCircles;

    public Slider[] sliders;


    public void SpawnCircle(Vector3 position)
    {
        //PhysicsBodySphere circleClone = circle.GetComponent<PhysicsBodySphere>();
        //circleClone.radius = radius;
        //circleClone.AdjustRadius(circleClone.radius);
        Instantiate(circle).transform.position = position;
    }
    
    public void SpawnRectangle(Vector3 position)
    {
        Instantiate(rectangle).transform.position = position;
    }


    // Start is called before the first frame update
    void Start()
    {
        pB = FindObjectOfType<PhysicsBody>();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Mouse0) && spawnedCircles.Count <= maxSpawnedCircles && MouseOverUICheck.BlockedByUI == false)
        {
            Vector3 worldPoint = Camera.main.ScreenToWorldPoint(Input.mousePosition, Camera.MonoOrStereoscopicEye.Mono);

            Vector3 adjustZ = new Vector3(worldPoint.x, worldPoint.y, circle.transform.position.z);

            SpawnCircle(adjustZ);
            spawnedCircles.Add(this.circle);
        }

        if (Input.GetKeyDown(KeyCode.Mouse1))
        {
            Vector3 worldPoint = Camera.main.ScreenToWorldPoint(Input.mousePosition, Camera.MonoOrStereoscopicEye.Mono);

            Vector3 adjustZ = new Vector3(worldPoint.x, worldPoint.y, circle.transform.position.z);

            SpawnRectangle(adjustZ);
        }
    }

    public void ResetScene()
    {
        circlesToDelete = GameObject.FindGameObjectsWithTag("Circle");
        foreach (var circle in circlesToDelete)
        {
            Destroy(circle);
        }
        foreach (var slider in sliders)
        {
            slider.value = 0;
        }
        spawnedCircles.Clear();
        //PhysicsBodySphere circlePB = circle.GetComponent<PhysicsBodySphere>();
        //circlePB.radius = 2f;
        //circlePB.gravityScale = 0f;
        //circlePB.bounciness = 1f;
        //circlePB.density = 1f;
    }
}
