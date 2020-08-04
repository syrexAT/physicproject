using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RectangleBody : PhysicsBody
{
    //in extent --> Außmase? die halbe größe, wie weit das rectangel vom urpsrung zur seite und nach oben geht, und links und unten 
    public Vector2 extent; //sind half

    //theoretisch noch bool ob das objekt kinematic ist --> dann unendlich mass
    public override void CalculateStaticParameters()
    {
        mass = extent.x * extent.y * 4 * density; //* 4 weil a = 2x, b = 2y ; 2*x * 2*y = 4 *x *y
        //momentOfInertia = mass * extent.x * 2 * extent.x * 2
        momentOfInertia = mass * (extent.x * extent.x * 4 + extent.y * extent.y * 4) / 12; //Formel steht in folie), * 4 weil extents nur half sind
    }

    public override void UpdateBoundingBoxes()
    {

        float currentAngle = transform.eulerAngles.z;
        Vector2 pc = (Vector2)transform.position + RotateVector2(offset, transform.eulerAngles.z); //um das zentrum drehen sihc die 4 punkte Point Center

        Vector2 p0 = pc + RotateVector2(new Vector2( extent.x, extent.y), currentAngle); //Rechts oberer punkt
        Vector2 p1 = pc + RotateVector2(new Vector2( extent.x, -extent.y), currentAngle);
        Vector2 p2 = pc + RotateVector2(new Vector2(-extent.x, -extent.y), currentAngle);
        Vector2 p3 = pc + RotateVector2(new Vector2(-extent.x, extent.y), currentAngle);

        //die aboslut kleinsten werte
        boundsMin = new Vector2(Mathf.Min(p0.x, p1.x, p2.x, p3.x), Mathf.Min(p0.y, p1.y, p2.y, p3.y));
        boundsMax = new Vector2(Mathf.Max(p0.x, p1.x, p2.x, p3.x), Mathf.Max(p0.y, p1.y, p2.y, p3.y));

        //boundsMin = (Vector2)transform.position + offset - extent;
        //boundsMax = (Vector2)transform.position + offset + extent;
    }

    protected override void OnDrawGizmos()
    {
        Gizmos.color = Color.green;

        float currentAngle = transform.eulerAngles.z;
        Vector2 pc = (Vector2)transform.position + RotateVector2(offset, transform.eulerAngles.z); //um das zentrum drehen sihc die 4 punkte Point Center

        Vector2 p0 = pc + RotateVector2(new Vector2(extent.x, extent.y), currentAngle); //Rechts oberer punkt
        Vector2 p1 = pc + RotateVector2(new Vector2(extent.x, - extent.y), currentAngle);
        Vector2 p2 = pc + RotateVector2(new Vector2(- extent.x, - extent.y), currentAngle);
        Vector2 p3 = pc + RotateVector2(new Vector2(- extent.x, extent.y), currentAngle);

        Gizmos.DrawLine(p0, p1);
        Gizmos.DrawLine(p1, p2);
        Gizmos.DrawLine(p2, p3);
        Gizmos.DrawLine(p3, p0);

        //Gizmos.DrawLine(boundsMin, new Vector2(boundsMin.x, boundsMax.y));
        //Gizmos.DrawLine(new Vector2(boundsMin.x, boundsMax.y), boundsMax);
        //Gizmos.DrawLine(boundsMax, new Vector2(boundsMax.x, boundsMin.y));
        //Gizmos.DrawLine(new Vector2(boundsMax.x, boundsMin.y), boundsMin);
        //Gizmos.DrawWireCube(transform.position + (Vector3)offset, extent * 2);


        base.OnDrawGizmos();
    }

    public void GetWorldSpaceVertices(ref Vector2[] vertices)
    {
        Vector2 center = GetCenter();

        //würde auch gehen aber mit 2 is effizieneter, die anderen 2 werden negiert
        //Vector2 p0 = pc + RotateVector2(new Vector2(extent.x, extent.y), currentAngle); //Rechts oberer punkt
        //Vector2 p1 = pc + RotateVector2(new Vector2(extent.x, -extent.y), currentAngle);
        //Vector2 p2 = pc + RotateVector2(new Vector2(-extent.x, -extent.y), currentAngle);
        //Vector2 p3 = pc + RotateVector2(new Vector2(-extent.x, extent.y), currentAngle);

        //delta von center zum rechts oberen punkt
        Vector2 corner0 = RotateVector2(extent, rotation);
        //links oberer punkt
        Vector2 corner1 = RotateVector2(new Vector2(-extent.x, extent.y), rotation);

        vertices[0] = center + corner0;
        vertices[1] = center + corner1;
        vertices[2] = center - corner0;
        vertices[3] = center - corner1;




    }
}
