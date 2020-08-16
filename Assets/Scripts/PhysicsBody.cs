using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;


public class PhysicsBody : MonoBehaviour
{
    //Settings
    public Vector2 boundsMin; //mindestwerte X u Y
    public Vector2 boundsMax;
    public Vector2 offset;
    public float density = 1f; //wurde im editor nicht auf 1 gesetzt?
    [Range(0,1)] //wenn sie größer ist als 1 dann wird aus dem nix energie erzeugt
    public float bounciness; //restitution

    public bool kinematic = false;
    public bool isTrigger = false;
    public bool isTriggered = false;

    public float friction; //reibung
    public float gravityScale = 1f;

    //Runtime values
    public Vector2 velocity; //lineare velocity
    public float angularVelocity; //angular velocity, gibt nur 1 wert im 2D --> Z achse, IMMER um den MASSE Mittelpunkt(nicht objektmittelpunkt), z.B. beim convex collider wäre der massemittelpunkt irgendwo, nicht immer mitte, hier bei uns is es baer eh der GetCenter weil simple objects

    public float mass; 
    public float momentOfInertia; //im 3d bräuchte man für alle 3 achsen ein eigenes MOI
    //bei primitives Masse mittelpunkt ist eh im zentrum, bei Convex nicht

    public SpriteRenderer sr;
    public bool colorChanged = false;
    public float drag;
    public float angularDrag;

    public bool isSelected;

    public Vector2 acceleration = new Vector2(0.1f, 0.1f);

    public float rotation
    {
        get
        {
            return transform.eulerAngles.z;
        }
    }


    public Vector2 GetVelocityOfPoint(Vector2 p)
    {
        Vector2 delta = p - GetCenter(); //vom collisionspunkt zum center
        //entwede den normalisierten vektor um 90 grad drehen und multiplizieren mit der distanz zum punkt, aber die variante hier unten is effizienter
        //90 grad gegen den uhrzeiger drehen
        return velocity + new Vector2(-delta.y, delta.x) * angularVelocity;
    }

    public static Vector2 RotateVector2(Vector2 v, float degree)
    {
        //float currentAngle = transform.eulerAngles.z * Mathf.Deg2Rad;
        float s = Mathf.Sin(degree * Mathf.Deg2Rad);
        float c = Mathf.Cos(degree * Mathf.Deg2Rad);
        return new Vector2(v.x * c - v.y * s, v.y * c + v.x * s);
    }

    public Vector2 GetCenter()
    {
        return (Vector2)transform.position + RotateVector2(offset, transform.eulerAngles.z);
    }


    public virtual void CalculateStaticParameters() //hier werden alle statischen werte ausgerechnet, wird 1 mal beim start ausgerechnet
    {

    }

    //function damit jede shape sich seine bounding box berechnen kann, sphere physics body und rectangle physics body
    public virtual void UpdateBoundingBoxes()
    {

    }



    private void Start()
    {
        //masse automatisch berechnen
        if (kinematic)
        {
            density = 10000000000000000000000000000f;
            gravityScale = 0;
        }
        if (isTrigger)
        {
            gravityScale = 0;
        }

        sr = GetComponentInChildren<SpriteRenderer>();
        CalculateStaticParameters();
    }


    private void OnEnable()
	{
        PhysicsManager.instance.physicsBodies.Add(this);
	}

	private void OnDisable()
	{
		if(PhysicsManager.hasValidInstance)
		{
            PhysicsManager.instance.physicsBodies.Remove(this);
		}
	}

    public void ChangeColorTrigger()
    {
        if (colorChanged == false)
        {
            //sr.color = Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
            sr.color = new Color(Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f));
            colorChanged = true;
        }
    }

    //private void Update()
    //{
    //    if (isTriggered && colorChanged == false)
    //    {
    //        sr.color = Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
    //        colorChanged = true;
    //    }
    //}

    protected virtual void OnDrawGizmos()
    {

        UpdateBoundingBoxes();
        Gizmos.color = Color.white;
        //von links unte nach links oben
        Gizmos.DrawLine(boundsMin, new Vector2(boundsMin.x, boundsMax.y));
        Gizmos.DrawLine(new Vector2(boundsMin.x, boundsMax.y), boundsMax);
        Gizmos.DrawLine(boundsMax, new Vector2(boundsMax.x, boundsMin.y));
        Gizmos.DrawLine(new Vector2(boundsMax.x, boundsMin.y), boundsMin);
    }

    //wir gehen davon aus das das alles im FixedUpdate abläuft das heißt fixeddeltaTime
    public void AddForce(Vector2 force)
    {
        //Vector2 a = force / mass; //acceleration

        //Vector2 velocityDelta = a * Time.fixedDeltaTime; //wieivel velocity auf meine fixeddelta auswirkt

        //velocity += velocityDelta;

        //in one lime
        velocity += force * Time.fixedDeltaTime / mass;


    }

    public void AddImpulse(Vector2 impulse)
    {
        velocity += impulse / mass;
    }

    public void AddTorque(float torque) 
    {
        angularVelocity += torque * Time.fixedDeltaTime / momentOfInertia;
    }

    public void AddAngularImpulse(float angularImpulse) //DrehImpuls
    {
        angularVelocity += angularImpulse / momentOfInertia;
    }

    public void AddForceAtPosition(Vector2 force, Vector2 position)
    {
        velocity += force * Time.fixedDeltaTime / mass; //bestimmt änderung der linearen velocity

        //Force in einen drehmoment umrechnen
        Vector2 delta = position - GetCenter(); //strecke ausrechnen vom center zur position wo force applied wird

        //Tatsächliche distanz also delta auf tangente projektizieren ..> Distanz wie weit die kraft vom massemitelpunkt weg ist
        float dist = Vector2.Dot(new Vector2(-force.y, force.x).normalized, delta);

        //Drehmoment ausrechenn
        float M = -force.magnitude * dist; //- um die drehrichtung zu ändern

        angularVelocity += M * Time.fixedDeltaTime / momentOfInertia;
    }

    public void AddImpulseAtPosition(Vector2 impulse, Vector2 position)
    {
        velocity += impulse / mass;

        Vector2 delta = position - GetCenter();

        float dist = Vector2.Dot(new Vector2(-impulse.y, impulse.x).normalized, delta);

        float M = -impulse.magnitude * dist;

        angularVelocity += M / momentOfInertia;
    }

    //public void OnPointerClick(PointerEventData eventData)
    //{
    //    Debug.Log(name + "GameObject clicked");
    //}


}
