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
    public float ErrorThreshold = 0.1f;
    public float Kp = 0.1f;
    public float Ki = 0.1f;
    public float Kd = 0.1f;
    public float Speed = 10f;

    List<VertexDistance> vertexDistances = new List<VertexDistance>();
    private Vector3[] vertices;
    VertexDistance error_vector;

    PID_Controller horizontalPID;
    PID_Controller verticalPID;

    void Start()
    {
        horizontalPID = new PID_Controller();
        verticalPID = new PID_Controller();
        GetVertices();
        //CreateWayPoints();

    }

    void LateUpdate()
    {

    }

    void FixedUpdate()
    {
        if (ActivateController)
        {


            error_vector = CalculateDistance();
            float errorX = error_vector.Distance * error_vector.DirectionX;
            float errorZ = error_vector.Distance * error_vector.DirectionZ;


            if (ResetError)
            {
                errorX = 0;
                errorZ = 0;
                horizontalPID.ResetController();
            }

            if (Math.Abs(errorX) < ErrorThreshold)
            {
                horizontalPID.ResetController();
            }

            if (Math.Abs(errorZ) < ErrorThreshold)
            {
                verticalPID.ResetController();
            }

            horizontalPID.Kp = Kp;
            horizontalPID.Ki = Ki;
            horizontalPID.Kd = Kd;

            verticalPID.Kp = Kp;
            verticalPID.Ki = Ki;
            verticalPID.Kd = Kd;

            float x = horizontalPID.PID_Compute(errorX);
            float z = verticalPID.PID_Compute(errorZ);

            MovePlayer(x,z);




            //Debug.Log("e: " + horizontalPID.Error + " PWM: " + horizontalPID.Output);
        }





    }

    void MovePlayer(float x, float z)
    {
       
        float moveX = Speed * Input.GetAxis("Horizontal") * Time.deltaTime;
        float moveZ = Speed * Input.GetAxis("Vertical") *  Time.deltaTime;
        
        
        Vector3 translate;


        if (ActivateController)
        {
            translate = new Vector3(x * Time.deltaTime + moveX, 0, z * Time.deltaTime + moveZ);
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

        for (int i = 0; i < vertices.Length; i++)
        {

            Vector3 nextPoint = 
            float distance = Vector3.Distance(point, player.transform.position);
            Vector3 direction = (point - player.transform.position).normalized;

            VertexDistance vertexDistance = new VertexDistance();
            vertexDistance.Point = point;

            if (direction.x >= 0)
            {
                vertexDistance.DirectionX = 1;
            }
            else
            {
                vertexDistance.DirectionX = -1;
            }

            if (direction.y >= 0)
            {
                vertexDistance.DirectionY = 1;
            }
            else
            {
                vertexDistance.DirectionY = -1;
            }

            if (direction.z >= 0)
            {
                vertexDistance.DirectionZ = 1;
            }
            else
            {
                vertexDistance.DirectionZ = -1;
            }

            vertexDistance.Distance = distance;

            vertexDistances.Add(vertexDistance);

        }


       
        foreach (var point in vertices)
        {

        }

        VertexDistance minDistancePoint = vertexDistances.Min();
        return minDistancePoint;

    }
}

public class VertexDistance : IComparable<VertexDistance>
{
    public Vector3 Point { get; set; }
    public float Distance { get; set; }
    public float DirectionX { get; set; }
    public float DirectionY { get; set; }
    public float DirectionZ { get; set; }
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




public class PID_Controller
{

    private float kp;

    public float Kp
    {
        get { return kp; }
        set { kp = value; }
    }


    private float ki;

    public float Ki
    {
        get { return ki; }
        set { ki = value; }
    }

    private float kd;

    public float Kd
    {
        get { return kd; }
        set { kd = value; }
    }



    private float error;

    public float Error
    {
        get { return error; }
        set { error = value; }
    }


    private float output;

    public float Output
    {
        get { return output; }
        set { output = value; }
    }



    private float output_z1 = 0f;
    private float error_z1 = 0f;
    private float error_z2 = 0f;


    public PID_Controller()
    {
        error_z2 = 0;
        error_z1 = 0;
        error = 0;
        output = 0;
        output_z1 = 0;
    }


    /// <summary>
    /// Compute the PID output each cycle.
    /// </summary>
    /// <returns>Returns the PID Output</returns>
    public float PID_Compute(float _error)
    {

        error_z2 = error_z1;
        error_z1 = error;
        error = _error;

        output = output_z1 + kp * (error - error_z1) + ki * error + kd * (error - 2 * error_z1 + error_z2);
        output_z1 = output;

       // Debug.Log(error + " " + error_z1 + " " + error_z2);

        return output;
    }

    public void ResetController()
    {
        error = 0;
        error_z1 = 0;
        error_z2 = 0;
        output = 0;
        output_z1 = 0;
        
    }

}