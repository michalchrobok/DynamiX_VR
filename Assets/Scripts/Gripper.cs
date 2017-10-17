using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Gripper : MonoBehaviour {

    float min, max, currentPos;
    bool  contact;
    public static bool close, objAttached, gripperActive;
    public static float pos1, pos2, shift, t;
    Transform obj;

    // Use this for initialization
    void Start () {
        max = 0.1237f;
        min = 0.038f;
        t = 0.005f;
        currentPos = Mathf.Abs(transform.localPosition.z);
        contact = false;
        close = false;
        gripperActive = false;
        objAttached = false;
    }
	
	// Update is called once per frame
	void Update () {

        if (!close)
        {
            contact = false;
        }
        
        if (close && !contact)
        {
            if(!(currentPos <= min))
                currentPos -= t; 
        }
        if (!close && currentPos < max)
        {
            currentPos += t;
        }

        currentPos = Mathf.Clamp(currentPos, min, max);

        if (transform.name == "arm1")
        {
            transform.localPosition = new Vector3(0.031f, 0, currentPos);
            pos1 = currentPos;
        }
        if (transform.name == "arm2")
        {
            transform.localPosition = new Vector3(0.031f, 0, -currentPos);
            pos2 = currentPos;
        }

       /* if (close && gripperActive)
        {
            shift = pos1 - pos2;

            if (shift > 0.001f)
            {
                if (transform.name == "arm1")
                    currentPos -= 0.0005f;
                if (transform.name == "arm2")
                    currentPos += 0.0005f;
            }

            if (shift < -0.001f)
            {
                if (transform.name == "arm1")
                    currentPos += 0.0005f;
                if (transform.name == "arm2")
                    currentPos -= 0.0005f;
            }
        }*/
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.tag == "PlayableObject")
        {
            contact = true;
            obj = other.transform;
            obj.GetComponent<Rigidbody>().isKinematic = true;
        }
        if(other.tag == "PlayableObject")
        {
            contact = true;
            obj = other.transform;
            obj.GetComponent<Rigidbody>().isKinematic = true;
        }

    }

}
