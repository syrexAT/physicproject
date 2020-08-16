using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class DragNShoot : MonoBehaviour
{
    public float power = 10f;

    public Vector2 minPower;
    public Vector2 maxPower;

    LineTrajectory lt;

    Camera cam;
    Vector2 force;
    Vector3 startPoint;
    Vector3 endPoint;

    public GameObject physicsObjectCircle;
    public GameObject physicsObjectRectangle;
    GameObject clone;
    public Vector3 velocity;

    public bool noUIcontrolsInUse;

    public GameObject circle;
    public GameObject rectangle;
    public List<GameObject> spawnedCircles = new List<GameObject>();

    public PhysicsBody pB;

    public float radius;

    public GameObject[] circlesToDelete;
    public float maxSpawnedCircles;

    public Slider[] sliders;
    public float sliderDefaultValue;
    public Toggle[] toggles;


    private void Start()
    {
        cam = Camera.main;
        lt = GetComponent<LineTrajectory>();

    }

    private void Update()
    {
        if (MouseOverUICheck.BlockedByUI == false)
        {
            if (Input.GetMouseButtonDown(0))
            {
                startPoint = cam.ScreenToWorldPoint(Input.mousePosition);
                startPoint.z = 15;
            }

            if (Input.GetMouseButton(0))
            {
                Vector3 currentPoint = cam.ScreenToWorldPoint(Input.mousePosition);
                currentPoint.z = 15;
                lt.RenderLine(startPoint, currentPoint);

            }

            if (Input.GetMouseButtonUp(0))
            {
                endPoint = cam.ScreenToWorldPoint(Input.mousePosition);
                endPoint.z = 15;

                force = new Vector2(Mathf.Clamp(startPoint.x - endPoint.x, minPower.x, maxPower.x), Mathf.Clamp(startPoint.y - endPoint.y, minPower.y, maxPower.y));
                float startEndPointX = startPoint.x - endPoint.x;
                float startEndPointY = startPoint.y - endPoint.y;
                velocity = new Vector3(Mathf.Clamp(startEndPointX, minPower.x, maxPower.x), Mathf.Clamp(startEndPointY, minPower.y, maxPower.y), 0f) * power;

                clone = Instantiate(physicsObjectCircle, startPoint, Quaternion.identity);
                clone.GetComponent<PhysicsBody>().velocity = velocity;
                PhysicsManager.instance.CheckforSelection(); //not fully implemented //To deselect every other clone before spawning the new one, so the newest spawned circle is always isSelected = true;
                clone.GetComponent<PhysicsBody>().isSelected = true;
                
                lt.EndLine();
            }

            //RECTANGLE
            //if (Input.GetMouseButtonDown(1))
            //{
            //    startPoint = cam.ScreenToWorldPoint(Input.mousePosition);
            //    startPoint.z = 15;
            //}

            //if (Input.GetMouseButton(1))
            //{
            //    Vector3 currentPoint = cam.ScreenToWorldPoint(Input.mousePosition);
            //    currentPoint.z = 15;
            //    lt.RenderLine(startPoint, currentPoint);

            //}

            //if (Input.GetMouseButtonUp(1))
            //{
            //    endPoint = cam.ScreenToWorldPoint(Input.mousePosition);
            //    endPoint.z = 15;

            //    force = new Vector2(Mathf.Clamp(startPoint.x - endPoint.x, minPower.x, maxPower.x), Mathf.Clamp(startPoint.y - endPoint.y, minPower.y, maxPower.y));
            //    float startEndPointX = startPoint.x - endPoint.x;
            //    float startEndPointY = startPoint.y - endPoint.y;
            //    velocity = new Vector3(Mathf.Clamp(startEndPointX, minPower.x, maxPower.x), Mathf.Clamp(startEndPointY, minPower.y, maxPower.y), 0f) * power;

            //    clone = Instantiate(physicsObjectRectangle, startPoint, Quaternion.identity);
            //    clone.GetComponent<PhysicsBody>().velocity = velocity;

            //    lt.EndLine();
            //}
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
            //sliderDefaultValue = slider.value;
        }
        foreach (var toggle in toggles)
        {
            toggle.isOn = false;
        }
        spawnedCircles.Clear();
    }

}
