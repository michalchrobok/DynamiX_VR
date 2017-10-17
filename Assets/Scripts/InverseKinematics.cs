using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class InverseKinematics : MonoBehaviour {

    // The rotation in each joint of the IRB2600 robot
    float[] theta = new float[6];
    // Dimensions of the kinematic structure of robot
    float L0,L1, L1sin, L1cos, L2, L3, L3sin, L3cos, L4, L5, L6, L35;
    // Joints where rotations are applied  
    Transform rot1, rot2, rot3, rot4, rot5, rot6, end;
    // Coordinates for postion kinematic structure should obtain
    public static Transform virtualPos;
    // Variable used to eliminate the shift caused by robot structure(not all axis are in line with the joints)
    float transDeg;
    // This variable describes linear distance that robot's gripper can cover within one frame
    public static float step;
    // Range of robot
    public static float range;
    // Default position of the robot
    Transform defaultPos;
    // Coortinates and rotation of the object robot should grab
    public static Vector3 objectPosition;
    public static Quaternion objectRotation;
    // Is the robot close enough to the object
    bool steady = false;
    
    void Start()
    {
        // assiging values and objects to variables
        rot1 = transform.Find("rot1");
        rot2 = rot1.transform.Find("rot2");
        rot3 = rot2.transform.Find("rot3");
        rot4 = rot3.transform.Find("rot4");
        rot5 = rot4.transform.Find("rot5");
        rot6 = rot5.transform.Find("rot6");
        end = rot6.transform.Find("end");
        
        L0 = rot1.position.y / transform.lossyScale.y;
        L1 = Vector3.Distance(rot1.position, rot2.position) / transform.lossyScale.x;
        L1sin = rot2.localPosition.y;
        L1cos = rot2.localPosition.x;
        L2 = Vector3.Distance(rot2.position, rot3.position) / transform.lossyScale.x;
        L3 = Vector3.Distance(rot3.position, rot4.position) / transform.lossyScale.x;
        L3cos = rot4.localPosition.x;
        L3sin = rot4.localPosition.y;
        L4 = Vector3.Distance(rot4.position, rot5.position) / transform.lossyScale.x;
        L5 = Vector3.Distance(rot5.position, rot6.position) / transform.lossyScale.x;
        L6 = Vector3.Distance(rot6.position, end.position)/ transform.lossyScale.x;
        L35 = Vector3.Distance(rot3.position, rot5.position) / transform.lossyScale.x;
        transDeg = Mathf.Atan2(rot5.localPosition.y + rot4.localPosition.y,rot5.localPosition.x + rot4.localPosition.x)*180f/Mathf.PI;

        virtualPos = transform.Find("virtualPos");
        defaultPos = transform.Find("defaultPos");
        objectPosition = defaultPos.position;
        objectRotation = defaultPos.rotation; 
        SphereCollider robotSpace = (SphereCollider)GameObject.Find("RobotSpace").GetComponent<Collider>();
        range = robotSpace.radius;
            
        step = 0.1f;
    }
    void Update()
    {
        // Applying calculated rotations to joints
        rot1.localEulerAngles = new Vector3(0, theta[0], 0);
        rot2.localEulerAngles = new Vector3(0, 0, theta[1]);
        rot3.localEulerAngles = new Vector3(0, 0, theta[2]);
        rot4.localEulerAngles = new Vector3(theta[3], 0, 0);
        rot5.localEulerAngles = new Vector3(0, 0, theta[4]);
        rot6.localEulerAngles = new Vector3(theta[5], 0, 0);
    }

    void LateUpdate()
    {
        if ((Vector3.Distance(objectPosition, virtualPos.position) < 0.0025f || steady) && RobotSpace.playerIn)
        {
            steady = true;
        }
        if (!RobotSpace.playerIn)
        {
            steady = false;
        }
        
        // Check if the object is within the range and control the rapidity of robot
        if (Vector3.Distance(objectPosition, rot1.position) < range && objectPosition.y > 0)
        {
            if (steady)
            {
                virtualPos.position = objectPosition;
                virtualPos.rotation = objectRotation;
            }
            else
            {
                
                virtualPos.position = Vector3.Lerp(virtualPos.position, objectPosition, step / Vector3.Distance(virtualPos.position, objectPosition) );
                virtualPos.rotation = Quaternion.Slerp(virtualPos.rotation, objectRotation, 2 * step / (Quaternion.Angle(virtualPos.rotation, objectRotation)*Mathf.PI/180));
            }
        }
        else
        {
            virtualPos.position = Vector3.Lerp(virtualPos.position, defaultPos.position, Time.deltaTime * 2);
            virtualPos.rotation = Quaternion.Slerp(virtualPos.rotation, defaultPos.rotation, Time.deltaTime * 2);
        }
        
        inverseKinematics(virtualPos);
    }


    // This method calculates inverse kinematics for ABB IRB2600 robot as well as any other 6DOF manipulator provided it has 
    // similar transform structure in Unity scene.
    void inverseKinematics(Transform obj)
    {
        float p2x, p2y, p2z;
        float C3;
        
        // Position of the object relative to robot coordinate system
        Vector3 localObj = transform.InverseTransformPoint(obj.position);
        
        // Point of kinematic decoupling, idealy the point where joint5 should be positioned
        Vector3 p5 = localObj + Quaternion.Inverse(transform.rotation) * obj.rotation * new Vector3(0, 0, (L5 + L6));
        
        // World coordinates of p5
        Vector3 worldP5 = obj.position + obj.rotation * new Vector3(0, 0, (L5 + L6));

        //Calculating theta1
        theta[0] = Mathf.Atan2(p5.z, p5.x);

        // Coordinates of joint2 after rotating base by theta1
        p2x = L1cos * Mathf.Cos(theta[0]);
        p2y = L1cos * Mathf.Sin(theta[0]);
        p2z = L1sin + L0;

        Vector3 p2 = new Vector3(p2x, p2z, p2y);

        // Using law of cosines to calculate theta3
        C3 = ((Mathf.Pow(p5.x - p2.x, 2) + Mathf.Pow(p5.z - p2.z, 2) + Mathf.Pow(p5.y - p2.y, 2) - Mathf.Pow(L2, 2) - Mathf.Pow(L35, 2))
            / (2 * (L2) * (L35)));

        theta[2] = Mathf.Atan2(Mathf.Sqrt(1 - Mathf.Pow(C3, 2)), C3);

        // Using similarities between triangles to calculate theta1
        float M = L2 + L35 * (C3);
        float N = L35 * Mathf.Sin(theta[2]);
        float A = Mathf.Sqrt(Mathf.Pow(p5.x - p2.x, 2) + Mathf.Pow(p5.z - p2.z, 2));
        float B = p5.y - p2.y;
        theta[1] = Mathf.Atan2(M * A - N * B, N * A + M * B);
        
        // Using virtual plane to calculate the values od theta 4,5,6
        Plane plane = new Plane();
        plane.Set3Points(rot4.transform.position, worldP5, obj.position);
        DrawPlane(rot5.transform.position, plane.normal);
        theta[3] = -relativeRotation(rot3, plane.normal).x;
        theta[4] = relativeRotation(rot4, -obj.forward).z;
        theta[5] = -relativeRotation(rot5, obj.right ).x;

        theta[0] = -theta[0] * 180f / Mathf.PI;
        theta[1] = -theta[1] * 180f / Mathf.PI;
        theta[2] = 90 - theta[2] * 180f / Mathf.PI - transDeg;
        
    }

    // Debug function for drawing planes
    void DrawPlane(Vector3 position, Vector3 normal)
    {

        Vector3 v3;

        if (normal.normalized != Vector3.forward)
            v3 = Vector3.Cross(normal, Vector3.forward).normalized * normal.magnitude;
        else
            v3 = Vector3.Cross(normal, Vector3.up).normalized * normal.magnitude;

        var corner0 = position + v3;
        var corner2 = position - v3;
        var q = Quaternion.AngleAxis(90.0f, normal);
        v3 = q * v3;
        var corner1 = position + v3;
        var corner3 = position - v3;

        Debug.DrawLine(corner0, corner2, Color.green);
        Debug.DrawLine(corner1, corner3, Color.green);
        Debug.DrawLine(corner0, corner1, Color.green);
        Debug.DrawLine(corner1, corner2, Color.green);
        Debug.DrawLine(corner2, corner3, Color.green);
        Debug.DrawLine(corner3, corner0, Color.green);
        Debug.DrawRay(position, normal, Color.red);
    }

    // Function that calculates rotation in euler degrees of given "vector" in given "transform" coordinate system
    Vector3 relativeRotation(Transform transform, Vector3 vector)
    {
        Vector3 a = transform.InverseTransformVector(vector);

        float xRotation = Mathf.Atan2(a.y, a.z) * 180 / Mathf.PI;
        float yRotation = Mathf.Atan2(a.x, a.z) * 180 / Mathf.PI;
        float zRotation = Mathf.Atan2(a.y, a.x) * 180 / Mathf.PI;

        Vector3 degrees = new Vector3(xRotation, yRotation, zRotation);

        return degrees;
    }
}