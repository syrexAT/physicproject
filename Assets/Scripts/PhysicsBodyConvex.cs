using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsBodyConvex : PhysicsBody
{
    public List<Vector2> points = new List<Vector2>();

    public Vector2 checkedPoint;

    public bool CheckIfPointIsInside(Vector2 point)
    {
        if (point.x <= boundsMax.x && point.x >= boundsMin.x && point.y <= boundsMax.y && point.y >= boundsMin.y)
        {
            float currentAngle = transform.eulerAngles.z;
            Vector2 pc = (Vector2)transform.position + RotateVector2(offset, currentAngle); //um das zentrum drehen sihc die 4 punkte (Point Center von der shape mit dem offset)

            //Gizmos.color = Color.green;
            if (points.Count > 1)
            { //pOld ist imer der letzte punkt eines for schleifen durchlaufes, man zeichnet von letzten punkt zum neuen punkt usw. 
                Vector2 pOld = pc + RotateVector2(points[points.Count - 1], currentAngle);
                for (int i = 0; i < points.Count; ++i)
                {
                    Vector2 p = pc + RotateVector2(points[i], currentAngle);

                    Vector2 dir = (p - pOld).normalized;

                    Vector2 normal = new Vector2(-dir.y, dir.x); //90 grad rotieren (umdrehen für um oder gegen den uhrzeiersinnn)

                    Vector2 delta = point - pOld; //Vektor von alten punkt zu dem berechneten punkt (ob er draussen oder drin liegt)

                    float projectedPointDepth = Vector2.Dot(normal, delta);

                    if (projectedPointDepth > 0) //ob es draußen oder drinnen liegt, wenn nur bei einem check scon draussen liegt dann ist es draussen und false
                    {
                        return false;
                    }

                    pOld = p;
                }
                return true;
            }
        }

        return false;
    }

    public override void UpdateBoundingBoxes()
    {

        float currentAngle = transform.eulerAngles.z;
        Vector2 pc = (Vector2)transform.position + RotateVector2(offset, currentAngle);

        if (points.Count > 0)
        {
            boundsMin = new Vector2(Mathf.Infinity, Mathf.Infinity); //so groß wie möglich
            boundsMax = new Vector2(Mathf.NegativeInfinity, Mathf.NegativeInfinity);


            for (int i = 0; i < points.Count; ++i)
            {
                Vector2 p = pc + RotateVector2(points[i], currentAngle);
                if (p.x < boundsMin.x)
                {
                    boundsMin.x = p.x;
                }
                if (p.x > boundsMax.x)
                {
                    boundsMax.x = p.x;
                }

                if (p.y < boundsMin.y)
                {
                    boundsMin.y = p.y;
                }
                if (p.y > boundsMax.y)
                {
                    boundsMax.y = p.y;
                }
            }
        }
        else
        {
            boundsMin = pc;
            boundsMax = pc;
        }


    }

    protected override void OnDrawGizmos()
    {
        float currentAngle = transform.eulerAngles.z;
        Vector2 pc = (Vector2)transform.position + RotateVector2(offset, currentAngle); //um das zentrum drehen sihc die 4 punkte (Point Center von der shape mit dem offset)

        Gizmos.color = Color.green;
        if (points.Count > 1)
        { //pOld ist imer der letzte punkt eines for schleifen durchlaufes, man zeichnet von letzten punkt zum neuen punkt usw. 
            Vector2 pOld = pc + RotateVector2(points[points.Count - 1], currentAngle);
            for (int i = 0; i < points.Count; ++i)
            {
                Vector2 p = pc + RotateVector2(points[i], currentAngle);
                Gizmos.DrawLine(pOld, p);
                pOld = p;
            }
        }
        Gizmos.color = CheckIfPointIsInside(pc + RotateVector2(checkedPoint, currentAngle)) ? Color.green : Color.red;
        Gizmos.DrawWireSphere(pc + RotateVector2(checkedPoint, currentAngle), 0.25f);

        base.OnDrawGizmos();
    }
}
