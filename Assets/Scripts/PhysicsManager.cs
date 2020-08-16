using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PhysicsManager : MonoBehaviour
{
    private static PhysicsManager _instance;
    public static PhysicsManager instance
    {
        get
        {
            if (_instance)
            {
                return _instance;
            }
            else
            {
                var go = new GameObject();
                go.name = "PhysicsManager";
                _instance = go.AddComponent<PhysicsManager>();
                return _instance;
            }
        }
    }
    public static bool hasValidInstance
    {
        get
        {
            return _instance != null;
        }
    }

    public static float gravitationalConstant = 30f;

    public List<PhysicsBody> physicsBodies = new List<PhysicsBody>(); 

    public float gravityConstant = -9.8f;
    private Vector2 gravityVector = Vector2.down.normalized;

    public GameObject positionReference;
    public InputField[] inputFields;

    private void Start()
    {
        positionReference = GameObject.Find("PositionReference");
    }


    private void FixedUpdate()
    {
        //foreach(var physicsBody in physicsBodies)
        //{

        //	//f = (m0 * m1) * g / r^2
        //	//a = f / m
        //	//f = a * m 

        //	//m0 * g / r^2 = a
        //	//
        //	//
        //	// p(x) = p0 + v * t

        //	//v(x) = v0 + a0 * t


        //	Vector2 delta = physicsBody.transform.position;
        //	Vector2 f = -delta.normalized * physicsBody.mass * 3f * gravitationalConstant / physicsBody.transform.position.sqrMagnitude;
        //	Vector2 a = f / physicsBody.mass;

        //	physicsBody.velocity += a * Time.fixedDeltaTime;

        //	physicsBody.transform.position += (Vector3)physicsBody.velocity * Time.fixedDeltaTime;
        //}




        //hier jedes objekt mit jedem anderen objekt 1 mal zusammen durchgehen



        //Solver, Solve Velocities
        foreach (var body in physicsBodies)
        {
            Transform t = body.transform;
            SpriteRenderer sprite = body.GetComponentInChildren<SpriteRenderer>();
            if (body.gravityScale > 0)
            {
                body.velocity.y += gravityConstant * body.gravityScale * Time.fixedDeltaTime;
            }

            body.velocity *= 1 - Time.fixedDeltaTime * body.drag;
            body.angularVelocity *= 1 - Time.fixedDeltaTime * body.angularDrag;

            /* t.position += (Vector3)body.velocity + (Vector3)gravityVector * gravityConstant * Time.fixedDeltaTime; *///obvious problem: durch das +gravityVector, haben objekte wenn sie sich moven nach links und rechts eine extrem starke velocity
            //if (body.velocity.x >= Mathf.Abs(0.01f) && body.velocity.y >= Mathf.Abs(0.01f))
            //{
                t.position += (Vector3)body.velocity * Time.fixedDeltaTime;
            //}

                /*t.position += (Vector3)gravityVector.normalized * (gravityConstant * body.gravityScale)* Time.fixedDeltaTime;*/ //body mass
                //body.velocity *= 1 - Time.fixedDeltaTime * body.drag;
            //t.position += (Vector3)body.velocity * Time.fixedDeltaTime;
            /*t.position += (Vector3)body.velocity + (Vector3)gravityVector * gravityConstant * Time.fixedDeltaTime;*/ //obvious problem: durch das +gravityVector, haben objekte wenn sie sich moven nach links und rechts eine extrem starke velocity

            if (body.kinematic == true)
            {
                t.position += (Vector3)body.velocity * Time.fixedDeltaTime;
            }
            

            t.RotateAround(body.GetCenter(), Vector3.forward, body.angularVelocity * Mathf.Rad2Deg * Time.fixedDeltaTime);
        }

        foreach (var body in physicsBodies)
        {
            Transform t = body.transform;
            if (body.isSelected)
            {
                if (Input.GetKeyDown(KeyCode.F))
                {
                    body.AddForce(new Vector2(5550, 5));
                }
                if (Input.GetKeyDown(KeyCode.J))
                {
                    body.AddImpulse(new Vector2(5, 0));
                }
                if (Input.GetKeyDown(KeyCode.T))
                {
                    body.AddTorque(5000);
                }
                if (Input.GetKeyDown(KeyCode.G))
                {
                    body.AddAngularImpulse(-500);
                }
                if (Input.GetKeyDown(KeyCode.L))
                {
                    body.AddForceAtPosition(new Vector2(0, -1500), body.transform.position + new Vector3(2, 2));
                    Instantiate(positionReference, body.transform.position + new Vector3(2, 2), Quaternion.identity);
                }
                if (Input.GetKeyDown(KeyCode.I))
                {
                    body.AddImpulseAtPosition(new Vector2(0, -150), body.transform.position + new Vector3(2, 2));
                    Instantiate(positionReference, body.transform.position + new Vector3(2, 2), Quaternion.identity);
                }
            }
        }


        //IST DAS SELBE WIE DIE FOREACH OBEN
        //for (int i = 0; i < physicsBodies.Count; i++)
        //{
        //    physicsBodies[i].transform.position += (Vector3)physicsBodies[i].velocity * Time.fixedDeltaTime;
        //    physicsBodies[i].transform.RotateAround(physicsBodies[i].GetCenter(), Vector3.forward, physicsBodies[i].angularVelocity * Mathf.Rad2Deg * Time.fixedDeltaTime);

        //}

        //UPDATE BOUNDING BOXES
        for (int i = 0; i < physicsBodies.Count; i++)
        {
            physicsBodies[i].UpdateBoundingBoxes();
        }


        //RESOLVE COLLISIONS
        Vector2 contactPoint = Vector2.zero;
        Vector2 contactNormal = Vector2.zero;
        float penetrationDepth = 0;
        bool validCollision = false;
        //bool triggerDetected = false;


        for (int i = 0; i < physicsBodies.Count; i++)
        {
            PhysicsBody a = physicsBodies[i];

            for (int ii = i + 1; ii < physicsBodies.Count; ii++)
            {
                PhysicsBody b = physicsBodies[ii];
                //X achse (max muss größer als min sein von einem objekt)
                //Broad phase collision detection (Bounding box)
                if (a.boundsMax.x > b.boundsMin.x &&
                    a.boundsMin.x < b.boundsMax.x)
                {
                    if (a.boundsMax.y > b.boundsMin.y &&
                        a.boundsMin.y < b.boundsMax.y)
                    {
                        validCollision = false;

                        if (a is PhysicsBodySphere && b is PhysicsBodySphere)
                        {
                            validCollision = CalculateCollisionData((PhysicsBodySphere)a, (PhysicsBodySphere)b, ref contactPoint, ref contactNormal, ref penetrationDepth);


                            //PhysicsBodySphere sphereA = (PhysicsBodySphere)a;
                            //PhysicsBodySphere sphereB = (PhysicsBodySphere)b;

                            //entweder contact point auf sphere a, b, oder in der mitte von beiden (mittelwert)
                            //Narrow Phase Collision Detection, ob sie wrkl kollidieren
                            //man braucht von beiden spheres das zentrum der sphere

                            //Vector2 centerA = sphereA.GetCenter();
                            //Vector2 centerB = sphereB.GetCenter();
                            //Vector2 centerDelta = centerB - centerA;
                            //float combinedRadius = sphereA.radius + sphereB.radius;

                            ////check if sie kollidieren
                            //if (centerDelta.sqrMagnitude <= combinedRadius * combinedRadius) // quadrieren weil centera-centerb auch squared ist
                            //{
                            //    //man braucht Contact normal, conctact point, penetration depth
                            //    //Contact normal
                            //    Vector2 contactNormal = centerDelta.normalized; //gibt die richtung an in denen sie zusammenstecken

                            //    //Penetration Depth
                            //    //center delta, davon die magnitude, dann weiss man wie weit die zentren von einander entfernt sind, und man weiss auch den radius von beiden also weiss man wie wiet sie auseinander sein könnten bevor sie kollidieren
                            //    float penetrationDepth = combinedRadius - centerDelta.magnitude;

                            //    //Vektor richtung andere sphere * den radius - der penetrationdepth durch 2 
                            //    Vector2 contactPoint = centerA + contactNormal * (sphereA.radius - penetrationDepth * 0.5f); //dann genau der punkt der in der mitte liegt von der überlappung

                            //    Debug.DrawRay(contactPoint, contactNormal, Color.magenta);



                            //}
                        }
                        else if (a is PhysicsBodySphere && b is RectangleBody)
                        {
                            validCollision = CalculateCollisionData((PhysicsBodySphere)a, (RectangleBody)b, ref contactPoint, ref contactNormal, ref penetrationDepth);
                        }
                        else if (a is RectangleBody && b is PhysicsBodySphere)
                        {
                            validCollision = CalculateCollisionData((PhysicsBodySphere)b, (RectangleBody)a, ref contactPoint, ref contactNormal, ref penetrationDepth);
                            contactNormal = -contactNormal; //weil die immer vom surface von a wegberechnet wird, hier drehen wir aber unsere objekte um --> zuerst b dann a, damit wir nicht noch eine funktion schrieben müssen
                        }
                        else if (a is RectangleBody && b is RectangleBody)
                        {
                            validCollision = CalculateCollisionData((RectangleBody)a, (RectangleBody)b, ref contactPoint, ref contactNormal, ref penetrationDepth);
                            //contactNormal = -contactNormal; //this glitches it out
                        }

                        if (validCollision && !a.isTrigger && !b.isTrigger)
                        {
                            Debug.DrawRay(contactPoint, contactNormal, Color.magenta);
                            //Collision Resolution, formel steht in folie
                            //DEPENETRATION / Penetration resolution
                            a.transform.position -= (Vector3)(contactNormal * penetrationDepth * b.mass / (a.mass + b.mass));
                            b.transform.position += (Vector3)(contactNormal * penetrationDepth * a.mass / (a.mass + b.mass));
                            a.UpdateBoundingBoxes();
                            b.UpdateBoundingBoxes();

                            //Collision resolution, Impulserhaltung
                            Vector2 contactVelocityA = a.GetVelocityOfPoint(contactPoint);
                            Vector2 contactVelocityB = b.GetVelocityOfPoint(contactPoint);

                            //umdrehen
                            Vector2 tangent = new Vector2(-contactNormal.y, contactNormal.x);

                            //das delta von beiden holen
                            Vector2 contactDeltaVelocity = contactVelocityB - contactVelocityA;

                            //ContactDeltavelocity auf contact normal projezieren
                            //contactDeltaVelocity = Vector3.Project(contactDeltaVelocity, contactNormal);



                            //das selbe wie contactDeltaVelocity nur besser ? 
                            float projectedContactDeltaVelocity = Vector2.Dot(contactDeltaVelocity, contactNormal);

                            if (projectedContactDeltaVelocity < 0) //sonst buggts bei der edge von einem rectangle z.B. mit einer sphere weil ein impuls ins objekt kommt
                            {
                                //Abstand von Contact zum massemittelpunkt von A und B entlang der contactNormal
                                //ProjectOnPlane --> Plane ist eine normal vektor, und der vektor der im Raum liegt und man den auf die Plane projeziert --> googeln
                                //masse mittelpunkt zu contact point
                                float normalDistanceA = Vector2.Dot(tangent, contactPoint - a.GetCenter());
                                float normalDistanceB = Vector2.Dot(tangent, contactPoint - b.GetCenter());
                                //float normalDistanceA = Vector3.ProjectOnPlane((contactPoint - a.GetCenter()), contactNormal).magnitude;
                                //float normalDistanceB = Vector3.ProjectOnPlane((contactPoint - b.GetCenter()), contactNormal).magnitude;

                                //(Kombinierter Impulserhaltungssatz)Impuls der rauskommt --> F= formel in der folie ; p ist linearer impuls (Folie)
                                float p = projectedContactDeltaVelocity / ((1 / a.mass) + (1 / b.mass) + (normalDistanceA * normalDistanceA / a.momentOfInertia) + (normalDistanceB * normalDistanceB / b.momentOfInertia));

                                float combinedBounciness = a.bounciness * b.bounciness;


                                p *= (1 + combinedBounciness); // wenn es 0 ist ist es p, wenn combinedBounciness 1 ist dann haben wir * 2

                                //Den ausgerechneten Impuls auf lineare und angular velocity bringen
                                a.velocity += contactNormal * p / a.mass; //steht in der folie --> v1 = v1-n * p/m
                                b.velocity -= contactNormal * p / b.mass;

                                //minus weil der drehimpuls ja umgekehrt gehört, weil etwas an der normal anstößt und sich dann in die richtung dreht, statt daran gezogen (an der normal)
                                a.angularVelocity -= p * normalDistanceA / a.momentOfInertia;
                                b.angularVelocity += p * normalDistanceB / b.momentOfInertia;

                                //FRICTION
                                float projectedContactDeltaVelocityFriction = Vector2.Dot(contactDeltaVelocity, tangent);
                                float normalDistanceAFriction = Vector2.Dot(contactNormal, contactPoint - a.GetCenter());
                                float normalDistanceBFriction = Vector2.Dot(contactNormal, contactPoint - b.GetCenter());

                                float pFriction = projectedContactDeltaVelocityFriction / ((1 / a.mass) + (1 / b.mass) + (normalDistanceAFriction * normalDistanceAFriction / a.momentOfInertia) + (normalDistanceBFriction * normalDistanceBFriction / b.momentOfInertia));
                                //pFriction *= (1 - combinedBounciness);


                                float combinedFriction = a.friction * b.friction;
                                //wieviel impuls kann dazu oder abgenommen werden abhängig von friction? maximal mögliche negierbar
                                float pMaxFriction = Mathf.Abs(p * combinedFriction); //soviel kann maximal aufgenommen werden, absolut werte

                                //Debug.Log("pFriction: " + pFriction);

                                pFriction = Mathf.Clamp(pFriction, -pMaxFriction, pMaxFriction);

                                a.velocity += tangent * pFriction / a.mass;
                                b.velocity -= tangent * pFriction / b.mass;

                                a.angularVelocity += pFriction * normalDistanceAFriction / a.momentOfInertia;
                                b.angularVelocity -= pFriction * normalDistanceBFriction / b.momentOfInertia;
                            }



                            



                        }
                        //TRIGGER
                        if (validCollision && a.isTrigger || b.isTrigger)
                        {
                            if (a.isTrigger && !b.isTrigger)
                            {
                                a.isTriggered = true;
                                b.ChangeColorTrigger();
                                Debug.Log("A is triggered");
                            }
                            else if (b.isTrigger && !a.isTrigger)
                            {
                                b.isTriggered = true;
                                a.ChangeColorTrigger();
                                Debug.Log("B is triggered");
                            }
                        }
                        if (validCollision && a.isTrigger && b.isTrigger) //wenn beide isTrigger on sind passiert nichts
                        {
                            return;
                        }
                    }
                }
            }
        }
    }

    //bool weil schauen ob collision überhaupt da ist
    public bool CalculateCollisionData(PhysicsBodySphere bodyA, PhysicsBodySphere bodyB, ref Vector2 contactPoint, ref Vector2 contactNormal, ref float penetrationDepth)
    {
        Vector2 centerA = bodyA.GetCenter();
        Vector2 centerB = bodyB.GetCenter();
        Vector2 centerDelta = centerB - centerA;
        float combinedRadius = bodyA.radius + bodyB.radius;

        //check if sie kollidieren
        if (centerDelta.sqrMagnitude <= combinedRadius * combinedRadius) // quadrieren weil centera-centerb auch squared ist
        {
            //man braucht Contact normal, conctact point, penetration depth
            //Contact normal
            contactNormal = centerDelta.normalized; //gibt die richtung an in denen sie zusammenstecken

            //Penetration Depth
            //center delta, davon die magnitude, dann weiss man wie weit die zentren von einander entfernt sind, und man weiss auch den radius von beiden also weiss man wie wiet sie auseinander sein könnten bevor sie kollidieren
            penetrationDepth = combinedRadius - centerDelta.magnitude;

            //Vektor richtung andere sphere * den radius - der penetrationdepth durch 2 
            contactPoint = centerA + contactNormal * (bodyA.radius - penetrationDepth * 0.5f); //dann genau der punkt der in der mitte liegt von der überlappung

            //Debug.DrawRay(contactPoint, contactNormal, Color.magenta);
            //Debug.Log("IS COLLIDING");
            return true;
        }

        return false;
    }

    public bool CalculateCollisionData(PhysicsBodySphere bodyA, RectangleBody bodyB, ref Vector2 contactPoint, ref Vector2 contactNormal, ref float penetrationDepth)
    {
        //center a in den local space from rectangle bekommen, viel leichter zu berechnen wenn man sichs im lokal space vorstellt, weil das rectangle dann axis aligned ist, un dman spart sich viele vektor berechenungen, skalarprodukte etc.
        //die rotation zurücksetzen und dann wieder zurückdrehen, deswegen mal -bodyB.rotation
        Vector2 centerA = PhysicsBody.RotateVector2(bodyA.GetCenter() - bodyB.GetCenter(), -bodyB.rotation); //wie weit center von sphere vom center vom rectangle entfernt ist

        //vergrößerte bounding box (extents) um zu checken obs innerhalb oder außerhalb liegt, bei den faces workeds, aber bei den ecken dann nichtmehr, da braucht man noch einen check
        Vector2 inflatedExtents = bodyB.extent + Vector2.one * bodyA.radius;

        //jetzt checken ob centerA (den sphere mittelpunkt) innerhalb von diesen inflatedExtents liegt, es kann aber immer noch in den ecken liegen!
        if (centerA.x <= inflatedExtents.x /*zumindest nicht rechts von der box*/ && centerA.x >= -inflatedExtents.x /*nicht links von der box*/ &&
            centerA.y <= inflatedExtents.y && centerA.y >= -inflatedExtents.y)
        {
            //Check ob wir in den ecken liegen
            //absolut wert von x und y
            Vector2 absolutCenterA = new Vector2(Mathf.Abs(centerA.x), Mathf.Abs(centerA.y));

            if (absolutCenterA.x >= bodyB.extent.x && absolutCenterA.y >= bodyB.extent.y) //wenn wir in dem rechteck (ecke) liegen
            {
                if ((absolutCenterA - bodyB.extent).sqrMagnitude <= bodyA.radius * bodyA.radius) //ob wir wirklich mit der ecke collidieren
                {
                    Vector2 mirrorDir = new Vector2(Mathf.Sign(centerA.x), Mathf.Sign(centerA.y));//mathf.Sign liefert +1, -1, oder 0 zurück, kommt drauf an was der input war z.B. 10 = 1, -5 = -1 usw. 
                    //nur für ++ space also oberes viertel, deswegen y achse -1 multiplizieren zum spiegeln
                    contactPoint = bodyB.extent;
                    contactNormal = (contactPoint - absolutCenterA).normalized * mirrorDir; //* mirrorDir weil es druaf ankommt in welcher ecke es liegt
                    contactPoint *= mirrorDir;
                    penetrationDepth = bodyA.radius - (absolutCenterA - bodyB.extent)/* hier ist man rechts oben*/.magnitude; //magnitude weil man will die tatsächliche distanz haben

                    //radius - die distanz von center zum point --> da bekommt man die pendepth

                    //jetzt wieder in den World space !!!
                    contactPoint = bodyB.GetCenter() + PhysicsBody.RotateVector2(contactPoint, bodyB.rotation); /*local contactpoint muss rotiert werden */
                    //normalized Vector muss rotiert werden
                    contactNormal = PhysicsBody.RotateVector2(contactNormal, bodyB.rotation);
                    return true;
                }
                else
                {
                    return false;
                }
            }
            //Berechnung ob es an den flächen links rechts oben unten liegt, und nicht in den ecken
            //RIGHT
            else if (centerA.x > bodyB.extent.x)
            {
                //hier wieder in local space, ContactPoint bie face to face is in der mitte (kann auf der edge sein, oder der punkt der am weitesten drin is
                contactPoint = new Vector2((bodyB.extent.x + centerA.x - bodyA.radius) * 0.5f /*mittelwert*/, centerA.y);
                contactNormal = Vector2.left; //weil wir rechts liegen
                penetrationDepth = Mathf.Abs(bodyB.extent.x - (centerA.x - bodyA.radius));

                //weider in world space
                contactPoint = bodyB.GetCenter() + PhysicsBody.RotateVector2(contactPoint, bodyB.rotation);
                contactNormal = PhysicsBody.RotateVector2(contactNormal, bodyB.rotation);
                return true;

            }
            //Left
            else if (centerA.x < -bodyB.extent.x)
            {
                contactPoint = new Vector2((-bodyB.extent.x + centerA.x + bodyA.radius) * 0.5f /*mittelwert*/, centerA.y);
                contactNormal = Vector2.right;
                penetrationDepth = Mathf.Abs(-bodyB.extent.x - (centerA.x + bodyA.radius));

                contactPoint = bodyB.GetCenter() + PhysicsBody.RotateVector2(contactPoint, bodyB.rotation);
                contactNormal = PhysicsBody.RotateVector2(contactNormal, bodyB.rotation);
                return true;
            }
            //UP
            else if (centerA.y > bodyB.extent.y)
            {
                contactPoint = new Vector2(centerA.x, (bodyB.extent.y + centerA.y - bodyA.radius) * 0.5f);
                contactNormal = Vector2.down;
                penetrationDepth = Mathf.Abs(bodyB.extent.y - (centerA.y - bodyA.radius));

                contactPoint = bodyB.GetCenter() + PhysicsBody.RotateVector2(contactPoint, bodyB.rotation);
                contactNormal = PhysicsBody.RotateVector2(contactNormal, bodyB.rotation);
                return true;
            }
            //Down
            else if (centerA.y < -bodyB.extent.y)
            {
                contactPoint = new Vector2(centerA.x, (-bodyB.extent.y + centerA.y + bodyA.radius) * 0.5f);
                contactNormal = Vector2.up;
                penetrationDepth = Mathf.Abs(-bodyB.extent.y - (centerA.y + bodyA.radius));

                contactPoint = bodyB.GetCenter() + PhysicsBody.RotateVector2(contactPoint, bodyB.rotation);
                contactNormal = PhysicsBody.RotateVector2(contactNormal, bodyB.rotation);
                return true;
            }
            else
            {
                return false;
            }
        }
        return false;
    }

    public bool CalculateCollisionData(RectangleBody bodyA, RectangleBody bodyB, ref Vector2 contactPoint, ref Vector2 contactNormal, ref float penetrationDepth)
    {
        Vector2[] verticesA = new Vector2[4];
        Vector2[] verticesB = new Vector2[4];

        bodyA.GetWorldSpaceVertices(ref verticesA);
        bodyB.GetWorldSpaceVertices(ref verticesB);

        //liste aus collisiondata, da steht drin um welchen punkt es geht -> index vom vertex, penetration depth(die mindest heruasgefundene), und penetration normal
        //List<CollisionData> collisionDatasA = new List<CollisionData>(4);
        //List<CollisionData> collisionDatasB = new List<CollisionData>(4);
        List<CollisionData> collisionDatasA = new List<CollisionData>
        {
            new CollisionData(verticesA[0]),
            new CollisionData(verticesA[1]),
            new CollisionData(verticesA[2]),
            new CollisionData(verticesA[3])
        };
        List<CollisionData> collisionDatasB = new List<CollisionData>
        {
            new CollisionData(verticesB[0]),
            new CollisionData(verticesB[1]),
            new CollisionData(verticesB[2]),
            new CollisionData(verticesB[3])
        };

        //VERTICES VON B IN A
        //alle achsen von a durchgehen(kombination punkt 0 auf 1, 1 auf 2 usw., jede achse wird mit den punkten von b verglichen, 
        //man speichert den index für den anfangspunkt
        {
            int p0Index = 3;
            for (int i = 0; i < 4; i++)
            {            //vertex 3 zu 0, 0 zu 1 usw.

                Vector2 axis = (verticesA[i] - verticesA[p0Index]).normalized;
                Vector2 norm = new Vector2(axis.y, -axis.x);
                for (int ii = 0; ii < collisionDatasB.Count; ii++)
                {
                    float depth = -Vector2.Dot(norm, collisionDatasB[ii].vertexPosition - verticesA[p0Index]);
                    if (depth <= 0) //liegt ausserhalb
                    {
                        collisionDatasB.RemoveAt(ii);
                        ii--;
                    }
                    else
                    {
                        if (depth < collisionDatasB[ii].penetrationDepth)
                        {
                            CollisionData collisionData = collisionDatasB[ii];
                            collisionData.penetrationDepth = depth;
                            collisionData.penetrationNormal = norm;
                            collisionDatasB[ii] = collisionData;
                        }

                    }
                }
                //int p1Index = i;
                //p0Index = p1Index;
                p0Index = i;

            }
        }

        {
            //VERTICES VON A IN B
            int p0Index = 3;
            for (int i = 0; i < 4; i++)
            {            //vertex 3 zu 0, 0 zu 1 usw.

                Vector2 axis = (verticesB[i] - verticesB[p0Index]).normalized;
                Vector2 norm = new Vector2(axis.y, -axis.x);
                for (int ii = 0; ii < collisionDatasA.Count; ii++)
                {
                    float depth = -Vector2.Dot(norm, collisionDatasA[ii].vertexPosition - verticesB[p0Index]);
                    if (depth <= 0) //liegt ausserhalb
                    {
                        collisionDatasA.RemoveAt(ii);
                        ii--;
                    }
                    else
                    {
                        if (depth < collisionDatasA[ii].penetrationDepth)
                        {
                            CollisionData collisionData = collisionDatasA[ii];
                            collisionData.penetrationDepth = depth;
                            collisionData.penetrationNormal = norm;
                            collisionDatasA[ii] = collisionData;
                        }

                    }
                }
                //int p1Index = i;
                //p0Index = p1Index;
                p0Index = i;

            }
        }

        //get most penetration depth
        penetrationDepth = Mathf.NegativeInfinity; //wir wollen die kleinstmögliche penetration kleinster wert

        foreach (CollisionData collisionData in collisionDatasB)
        {
            if (collisionData.penetrationDepth > penetrationDepth)
            {
                penetrationDepth = collisionData.penetrationDepth;
                contactNormal = collisionData.penetrationNormal;
                contactPoint = collisionData.vertexPosition;
            }
        }

        foreach (CollisionData collisionData in collisionDatasA)
        {
            if (collisionData.penetrationDepth > penetrationDepth)
            {
                penetrationDepth = collisionData.penetrationDepth;
                contactNormal = -collisionData.penetrationNormal; //pen normal negieren
                contactPoint = collisionData.vertexPosition;
            }
        }

        return penetrationDepth != Mathf.NegativeInfinity;

    }

    private struct CollisionData //bessere performance als eine klasse, weil am stack und nicht am heap
    {
        public Vector2 vertexPosition;
        //die größte penetrationDepth haben, der wert der am wenigsten penetratet
        public float penetrationDepth;
        public Vector2 penetrationNormal;

        public CollisionData(Vector2 vertexPosition)
        {
            this.vertexPosition = vertexPosition;
            penetrationDepth = Mathf.Infinity;
            penetrationNormal = Vector2.zero;
        }
    }


    //Penetration depth von punkt auf achse
    public static bool GetPointToPointPenetrationDepth(Vector2 p0, Vector2 p1, Vector2 point, ref float depth)
    {
        // im uhrzeigersinn (alles was rechts vom vektor liegt außerhalb, alles links innerhalb)
        Vector2 axis = (p1 - p0).normalized;
        Vector2 norm = new Vector2(axis.y, axis.x);
        depth = Vector2.Dot(axis, point - p0);
        return depth < 0;
    }

    public void CheckforSelection()
    {
        foreach (var body in physicsBodies)
        {
            if (body.isSelected)
            {
                body.isSelected = false;
            }
        }
    }

}


