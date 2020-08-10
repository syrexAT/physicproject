using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsBodySphere : PhysicsBody
{
    //radius muss positiv sein
    public float radius;

    public override void CalculateStaticParameters()
    {
        mass = radius * radius * Mathf.PI * density;
        momentOfInertia = mass * radius * radius / 2; //Formel steht in folie
    }

    public override void UpdateBoundingBoxes()
    {
        //float currentAngle = transform.eulerAngles.z * Mathf.Deg2Rad;
        //float s = Mathf.Sin(currentAngle);
        //float c = Mathf.Cos(currentAngle);
        Vector2 rotatedOffset = RotateVector2(offset, transform.eulerAngles.z);

        //berechnen der bounding boxes für eine sphere
        //hier ist alles linear also nicht so schlimm, alles hier reinpacken am besten
        boundsMin = (Vector2)transform.position + rotatedOffset - Vector2.one * Mathf.Abs(radius); //damit ist es nicht negativ;
        boundsMax = (Vector2)transform.position + rotatedOffset + Vector2.one * Mathf.Abs(radius);
        //Debug.Log(boundsMin.x + "/" + boundsMin.y + "///" + boundsMax.x + "/" + boundsMax.y);


    }

    [ExecuteInEditMode]
    protected override void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Vector2 rotatedOffset = RotateVector2(offset, transform.eulerAngles.z);
        Gizmos.DrawWireSphere(transform.position + (Vector3)rotatedOffset, radius);

        base.OnDrawGizmos();
    }
}
