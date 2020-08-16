using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class LineTrajectory : MonoBehaviour
{
    public LineRenderer lr;

    private void Awake()
    {
        lr = GetComponent<LineRenderer>();
    }

    public void RenderLine(Vector3 startPoint, Vector3 endPoint)
    {
        lr.positionCount = 2;
        Vector3[] points = new Vector3[2]; //new vector3 array which is gonna have 2 points available in it
        points[0] = startPoint;
        points[1] = endPoint;

        lr.SetPositions(points);
    }


    public void EndLine()
    {
        lr.positionCount = 0;
    }

}
