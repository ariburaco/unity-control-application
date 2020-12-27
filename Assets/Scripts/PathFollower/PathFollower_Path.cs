using PathCreation;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class PathFollower_Path : MonoBehaviour
{
   
    public GameObject player;
    public GameObject wayPoint;
    public PathCreator pathCreation;
    public float Kp = 0.0001f;
    public float Ki = 0.0001f;
    public float Kd = 0.0001f;
    public float Speed = 5f;
    public float ErrorThreshold = 0.1f;

    private Vector3[] vertices;

    float error = 0f;
    float error_z1 = 0f;
    float error_z2 = 0f;
    float PWM = 0f;
    float PWM_z1 = 0f;



    void Start()
    {
        GetVertices();
        CreateWayPoints();


    }

    void LateUpdate()
    {
        MovePlayer(PWM);
    }

    void FixedUpdate()
    {


        VertexDistance error_vector = CalculateDistance();

        error_z2 = error_z1;
        error_z1 = error;
        error = error_vector.Direction.x;
        PIDController(Kp, Ki, Kd);

        if (Math.Abs(error) < ErrorThreshold)
        {
            PWM = 0f;
        }

        

        Debug.Log(error);
        // Debug.Log("e: " + error + " e_z1: " + error_z1 + " e_z2: " + error_z2);

    }

    void MovePlayer(float x)
    {
        //float speed = 10;
        float moveX = Speed * Input.GetAxis("Horizontal") * Time.deltaTime;
        float moveZ = Speed * Input.GetAxis("Vertical") * Time.deltaTime;
        Vector3 translate = new Vector3(moveX, 0, moveZ );
        player.transform.Translate(translate);
       
        translate = new Vector3(x, 0, 0);
        player.transform.Translate(translate);

    }

    void GetVertices()
    {
        vertices = pathCreation.path.vertices;

    }

    void CreateWayPoints()
    {
        foreach (var pos in vertices)
        {
            Instantiate(wayPoint, pos, Quaternion.identity);
        }

    }

    VertexDistance CalculateDistance()
    {
        List<VertexDistance> vertexDistances = new List<VertexDistance>();
        vertexDistances.Clear();
        foreach (var point in vertices)
        {

            float distance = Vector3.Distance(player.transform.position, point);
            Vector3 direction = (player.transform.position - point).normalized;

            VertexDistance vertexDistance = new VertexDistance();
            vertexDistance.Point = point;
            vertexDistance.Distance = distance;
            vertexDistance.Direction = direction;

            vertexDistances.Add(vertexDistance);
        }

        VertexDistance minDistancePoint = vertexDistances.Min();
        return minDistancePoint;

    }


    void PIDController(float a, float b, float c)
    {
        float a0 = 1;
        float a1 = a + b + c;
        float a2 = (a * b + c * (b + a));
        float a3 = a * b * c;

        float Kd = -a3;
        float Kp = 2 * Kd - a2;
        float Ki = -(a1 + Kp + Kd);


        PWM = PWM_z1 + Kp * (error - error_z1) + Ki * error + Kd * (error - 2 * error_z1 + error_z2);
        PWM_z1 = PWM;
    }


}

public class VertexDistance : IComparable<VertexDistance>
{
    public Vector3 Point { get; set; }
    public float Distance { get; set; }
    public Vector3 Direction { get; set; }
    int IComparable<VertexDistance>.CompareTo(VertexDistance other)
    {
        if (other.Distance > this.Distance)
            return -1;
        else if (other.Distance == this.Distance)
            return 0;
        else
            return 1;
    }
}


/*
 function PWM = fcn(PWM_z1, error, error_z1, error_z2, a, b, c )

% PWM = error * Kp;
a0 = 1;
a1 = a + b + c;
a2 = (a*b + c*(b+a));
a3 = a*b*c;

Kd = -a3;
Kp = 2*Kd - a2;
Ki = -(a1 + Kp + Kd);


PWM = PWM_z1 + Kp*(error - error_z1) + Ki * error + Kd * (error - 2*error_z1 + error_z2);
 */
