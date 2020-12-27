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
    public bool ActivateController = false;
    public bool ResetError = false;
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

    List<VertexDistance> vertexDistances = new List<VertexDistance>();


    void Start()
    {
        GetVertices();
        CreateWayPoints();


    }

    void LateUpdate()
    {
       
    }

    void FixedUpdate()
    {
        if (ActivateController)
        {
            VertexDistance error_vector = CalculateDistance();

           // Debug.Log("x: " + error_vector.Direction.x + " z: " + error_vector.Direction.z);

            error_z2 = error_z1;
            error_z1 = error;
            error = error_vector.Distance * error_vector.Direction;
            PIDController(Kp, Ki, Kd);
            MovePlayer(PWM);

            if (ResetError)
            {
                error = 0;
                error_z2 = 0;
                error_z1 = 0;
                PWM = 0;
                PWM_z1 = 0;
            }

            if (Math.Abs(error) < ErrorThreshold)
            {
                PWM = 0f;
            }
        }



        Debug.Log("e: " + error + " PWM: " + PWM + " PWM_z1: " + PWM_z1);

    }

    void MovePlayer(float x)
    {
        //float speed = 10;
        float moveX = Speed * Input.GetAxis("Horizontal") * Time.deltaTime;
        float moveZ = Speed  * Time.deltaTime; // Input.GetAxis("Vertical")
        Vector3 translate = new Vector3(moveX, 0, moveZ);
        player.transform.Translate(translate);

        if (ActivateController)
        {
            translate = new Vector3(x * Time.deltaTime, 0, 0);
            player.transform.Translate(translate);
        }

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

        vertexDistances.Clear();
        foreach (var point in vertices)
        {

            float distance = Vector3.Distance(point , player.transform.position);
            Vector3 direction = (point - player.transform.position).normalized;

            VertexDistance vertexDistance = new VertexDistance();
            vertexDistance.Point = point;
            vertexDistance.Distance = distance;
            if (direction.x >= 0)
            {
                vertexDistance.Direction = 1;

            }
            else
            {
                vertexDistance.Direction = -1;
            }

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

        PWM_z1 = PWM;
        PWM = PWM_z1 + Kp * (error - error_z1) + Ki * error + Kd * (error - 2 * error_z1 + error_z2);
       
    }


}

public class VertexDistance : IComparable<VertexDistance>
{
    public Vector3 Point { get; set; }
    public float Distance { get; set; }
    public float Direction { get; set; }
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
