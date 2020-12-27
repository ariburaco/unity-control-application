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
    public bool HorizontalController = false;
    public bool VerticalController = false;
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
            float errorX = error_vector.DistanceX * error_vector.DirectionX;
            float errorZ = error_vector.DistanceZ * error_vector.DirectionZ;


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

            float x = 0;
            float z = 0;


            if (HorizontalController)
            {
                x = horizontalPID.PID_Compute(errorX);
            }

            
            if (VerticalController)
            {
                z = verticalPID.PID_Compute(errorZ);
            }

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

           
            float distance = Vector3.Distance(vertices[i], player.transform.position);
            float distanceX = Mathf.Abs(vertices[i].x - player.transform.position.x);
            float distanceY = Mathf.Abs(vertices[i].y - player.transform.position.y);
            float distanceZ = Mathf.Abs(vertices[i].z - player.transform.position.z);


            Vector3 direction = (vertices[i] - player.transform.position).normalized;

            VertexDistance vertexDistance = new VertexDistance();
            vertexDistance.Point = vertices[i];

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
            vertexDistance.DistanceX = distanceX;
            vertexDistance.DistanceY = distanceY;
            vertexDistance.DistanceZ = distanceZ;

            vertexDistances.Add(vertexDistance);

        }
        var minDistanceItem = vertexDistances.Select((item, index) => (item, index)).Min();
        int minIndex = minDistanceItem.index;
        VertexDistance minDistancePoint = minDistanceItem.item;
        


        VertexDistance nextPoint = vertexDistances[minIndex + 1];
        VertexDistance targetPoint = new VertexDistance();

        targetPoint.Distance = minDistancePoint.Distance + ( nextPoint.Distance - minDistancePoint.Distance  ) * 0.75f;
        targetPoint.DistanceX = minDistancePoint.DistanceX + ( nextPoint.DistanceX - minDistancePoint.DistanceX) * 0.75f;
        targetPoint.DistanceY = minDistancePoint.DistanceY + ( nextPoint.DistanceY - minDistancePoint.DistanceY) * 0.75f;
        targetPoint.DistanceZ = minDistancePoint.DistanceZ + ( nextPoint.DistanceX - minDistancePoint.DistanceX) * 2f;

        Vector3 targetDirection = (minDistancePoint.Point - player.transform.position).normalized;

        if (targetDirection.x >= 0)
        {
            targetPoint.DirectionX = 1;
        }
        else
        {
            targetPoint.DirectionX = -1;
        }

        if (targetDirection.y >= 0)
        {
            targetPoint.DirectionY = 1;
        }
        else
        {
            targetPoint.DirectionY = -1;
        }

        if (targetDirection.z >= 0)
        {
            targetPoint.DirectionZ = 1;
        }
        else
        {
            targetPoint.DirectionZ = -1;
        }

        Debug.Log("min target: " + minDistancePoint.DistanceZ + " next target: " + nextPoint.DistanceZ);
        //var minDistancedSorted = vertexDistances.OrderBy(m => m.Distance).ToList();
        minDistancePoint.DistanceZ = 2* nextPoint.DistanceZ;
        //VertexDistance minDistancePoint = minDistancedSorted[1];
        return minDistancePoint;

    }
}

public class VertexDistance : IComparable<VertexDistance>
{
    public Vector3 Point { get; set; }
    public float Distance { get; set; }
    public float DistanceX { get; set; }
    public float DistanceY { get; set; }
    public float DistanceZ { get; set; }
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