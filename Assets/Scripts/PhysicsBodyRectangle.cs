using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsBodyRectangle : PhysicsBody
{
    public float x;
    public float y;
    public float width;
    public float height;

    //public float minY;
    //public float minX;
    //public float maxY;
    //public float maxX;


    public override void UpdateBoundingBoxes()
    {
        boundsMin = (Vector2)transform.position + new Vector2(x, y);
        boundsMax = (Vector2)transform.position + new Vector2(x + width, y + height);

        //boundsMin < boundsMax

    }

}
