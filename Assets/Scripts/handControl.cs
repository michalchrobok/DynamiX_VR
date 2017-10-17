using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class handControl : MonoBehaviour {
    
    // Arrays holding transforms of both hands 
    Transform[] rightArm,leftArm;
    // Variable used to define whether given position is within hand's reach 
    public static float range;
    // Transforms used to control hand movement with other scripts
    public static Transform virtualPosLeft, virtualPosRight;
    // Default position and rotation for hands
    Vector3 defaultVecR, defaultVecL;
    Quaternion defaultQuatR, defaultQuatL;
    
    // Assigning objects and values
    void Start () {

        rightArm = new Transform[5];
        leftArm = new Transform[5];

        rightArm[0] = transform.Find("upper_arm_R");
        rightArm[1] = rightArm[0].Find("forearm_R");
        rightArm[2] = rightArm[1].Find("hand_R");
        rightArm[3] = rightArm[2].Find("palm");
        rightArm[4] = transform.Find("virtualPos_R");

        leftArm[0] = transform.Find("upper_arm_L");
        leftArm[1] = leftArm[0].Find("forearm_L");
        leftArm[2] = leftArm[1].Find("hand_L");
        leftArm[3] = leftArm[2].Find("palm");
        leftArm[4] = transform.Find("virtualPos_L");

        virtualPosLeft = transform.Find("virtualPos_L");
        virtualPosRight = transform.Find("virtualPos_R");

        defaultVecL = virtualPosLeft.localPosition;
        defaultVecR = virtualPosRight.localPosition;

        defaultQuatL = virtualPosLeft.localRotation;
        defaultQuatR = virtualPosRight.localRotation;
    }

    // Using the LateUpdate here because coordinates for hands are calculated in other script
    void LateUpdate() {

        if (!FindObjectOfType<PlayerVR>().enabled)
        {
            virtualPosLeft.localPosition = defaultVecL;
            virtualPosRight.localPosition = defaultVecR;
            virtualPosLeft.localRotation = defaultQuatL;
            virtualPosRight.localRotation = defaultQuatR;
        }
        armInverseKinematics(rightArm);
        armInverseKinematics(leftArm);
    }
    
    // Function that performs calcualtion of inverse kinematics for human arm.
    // To obtain direct solution one of dof's was removed.
    void armInverseKinematics(Transform[] arm)
    {
        float px, py, pz;
        float C2, beta, gamma, c;
        Vector3 Wrist;
        Quaternion quat;
        float[] theta = new float[3];
        float L1, L2, L3;
        Transform shoulder, forearm, hand, palm;
        Transform obj;
        float scale = transform.lossyScale.x;

        shoulder = arm[0];
        forearm = arm[1];
        hand = arm[2];
        palm = arm[3];
        obj = arm[4];

        L1 = Vector3.Distance(shoulder.position, forearm.position)/ scale;
        L2 = Vector3.Distance(forearm.position, hand.position)/ scale;
        L3 = Vector3.Distance(hand.position, palm.position)/ scale;

        range = (L1 + L2 + L3) * scale;

        px = obj.localPosition.x;
        py = obj.localPosition.z;
        pz = obj.localPosition.y;

        Wrist = new Vector3(px,pz,py) + obj.localRotation * new Vector3(0,0,-L3);
        
        theta[0] = Mathf.Atan2(Wrist.x - shoulder.localPosition.x, Wrist.z - shoulder.localPosition.z) *180/Mathf.PI;
        
        c = Vector3.Distance(shoulder.localPosition, Wrist);
        
        C2 = ((Mathf.Pow(c, 2) - Mathf.Pow(L2, 2) - Mathf.Pow(L1, 2))
            / (2 * (L2) * (L1)));

        theta[2] = Mathf.Atan2(Mathf.Sqrt(1 - Mathf.Pow(C2, 2)), C2);
        

        beta = Mathf.Asin((shoulder.localPosition.y - Wrist.y)/ c);
        gamma = Mathf.Asin(L2 * Mathf.Sin(theta[2]) / c);

        theta[1] = 90 - beta * 180 / Mathf.PI - gamma * 180 / Mathf.PI;
        
        quat = Quaternion.LookRotation(obj.up,-obj.right);

        if (Vector3.Distance(palm.position ,obj.position) > 0.1f)
        {
            shoulder.localRotation = Quaternion.Slerp(shoulder.localRotation, Quaternion.Euler(-theta[1], theta[0], 90),Time.deltaTime*2);
            forearm.localRotation = Quaternion.Slerp(forearm.localRotation, Quaternion.Euler(-obj.localEulerAngles.z, theta[2] * 180 / Mathf.PI, 0), Time.deltaTime * 2);
            if (hand.gameObject.name == "hand_L")
                hand.rotation = Quaternion.Slerp(hand.rotation, quat * Quaternion.Euler(90, 0, 0), Time.deltaTime*2);
            else
                hand.rotation = Quaternion.Slerp(hand.rotation, quat * Quaternion.Euler(-90, 0, 0), Time.deltaTime * 2);
        }
        else
        {
            shoulder.localRotation = Quaternion.Euler(-theta[1], theta[0], 90);
            forearm.localRotation = Quaternion.Euler(-obj.localEulerAngles.z, theta[2] * 180 / Mathf.PI, 0);
            if (hand.gameObject.name == "hand_L")
                hand.rotation = quat * Quaternion.Euler(90, 0, 0);
            else
                hand.rotation = quat * Quaternion.Euler(-90, 0, 0);
        }
    }
}