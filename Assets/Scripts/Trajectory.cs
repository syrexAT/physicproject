using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Trajectory : MonoBehaviour
{

    public Vector2 initialVelocity;
    public Vector2 constantAcceleration; //muss konstant bleiben damit die formel wirklich integrierbar ist

    public float endTime = 10;
    public float timeStepSize = 0.1f; //umso größer umso weniger wird die accerlartion einberechnet (wird extrem ungenau)

    //gibt uns den neuen punkt zur bestimmten zeit zurück für integrate method
    private Vector2 CalculatePosition(float t)
    {
        //start position +  initial velocity * zeit + accerlation * t^2 / 2
        //formeln stehen in der folie
        return (Vector2)transform.position + initialVelocity * t + constantAcceleration * t * t / 2;
    }


    private void OnDrawGizmos()
    {

        if (timeStepSize > 0)
        {
            //Euler methode braucht immer das ergebnis vom step davor, also eine variable wo es dazugeaddet wird
            //EULER METHODE
            //for schleife mit fixer step zeit in sekunde und eine endzeit bis wohin wir drawen
            Vector2 pos = transform.position;
            Vector2 vel = initialVelocity;
            Gizmos.color = Color.yellow;
            for (float t = 0; t < endTime; t += timeStepSize)
            {
                Vector2 oldPos = pos; //alte pos cachen

                //diese 2 zeilen kann man auch umdrehen, entweder man hat zuviel gravity oder zu wenig gravity
                pos += (vel + constantAcceleration * timeStepSize / 2) * timeStepSize; // hier wird es halbiert(mittelwert), damit das zuviel/zuwenig gravity nicht passiert, ERROR WIRD KOMPLETT BESEITIGT
                /*pos += vel * timeStepSize;*/ //normale formel /wie vie zeit zwischen den steps
                vel += constantAcceleration * timeStepSize; // hier is tes egal weil die vel constant ist und am ende des frames ist es genau dieser wert, die pos wird aber während des ganzen zeitraums des frames beeinflusst

                Gizmos.DrawLine(oldPos, pos);
            }

            //INTEGRATE METHODE
            //gibt die fläche unterhalb der funktion aus, nicht so wie bei f(x) = y da gibt es dir den wert, beim integrieren bekommt man den flächeninhalt von x= 0 bis zu dem wert den man will (time)
            //wir brauchen keine lokalen variablen, man braucht nicht die information vom vorherigen frame
            //der error, ist abhängig von der stepsize, umso größer die stepsize umso mehr error
            //vorteil hier ist das man es zu jedem belibeigen zeitpunkt berechnen kann ohne davor was sichern zu  müssen
            Gizmos.color = Color.magenta;
            for (float t = 0; t < endTime; t += timeStepSize)
            {
                Vector2 pos0 = CalculatePosition(t); //aktuelle zeit
                Vector2 pos1 = CalculatePosition(t + timeStepSize); //aktuelle zeit + stepsize, im nächsten frame ist pos 1 die pos 0 also sollte ma nes cachen wenn mans gscheit macht
                Gizmos.DrawLine(pos0, pos1);
            }
        }


    }
}
