using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotSpace : MonoBehaviour {

    public static bool playerIn = false;

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
	}

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("rightHand"))
            playerIn = true;
    }

    private void OnTriggerStay(Collider other)
    {
        if (other.CompareTag("rightHand"))
        {
            if (Input.GetKeyDown(KeyCode.Space))
            {
                playerIn = !playerIn;
                if (!playerIn)
                    Gripper.close = false;
            }
        }
        if (other.CompareTag("Player"))
        {
            InverseKinematics.step = 0.025f;
            Gripper.t = 0.005f;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("rightHand"))
        {
            playerIn = false;
            //InverseKinematics.objectPosition = new Vector3();
            //InverseKinematics.objectRotation = new Quaternion();
            //Gripper.close = false;
        }
    }
}
