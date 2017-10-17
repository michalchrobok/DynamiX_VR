using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Finger : MonoBehaviour {

    // Transforms of fingerparts
    Transform fingerJoint1, fingerJoint2, fingerJoint3;
    // Variable used to control the angle of each fingerjoint
    float angle = 0;
    // Default values for angles in fingerjoints
    Vector3 default1, default2, default3;
    // Should the hand grab the object?
    public static bool grab = false;
    // 
    public static bool touched = false;
    // Transfrom of the object player is holding
    public static Transform playableObject;
    // Has the finger touched the object already?
    bool touch = false;
    // Assigning needed values and objects
    void Start () {
        fingerJoint1 = transform;
        default1 = fingerJoint1.localEulerAngles;
        fingerJoint2 = fingerJoint1.parent;
        default2 = fingerJoint2.localEulerAngles;
        fingerJoint3 = fingerJoint2.parent;
        default3 = fingerJoint3.localEulerAngles;
    }

    // Script is attached to every finger, on grab command fingers start to clench until they form a fist.
    // Should they encounter playable object before, they will stop and pass object's transform up to PlayerVR 
	void Update () {
        
        if (!grab)
        {
            touch = false;
            touched = false;
            playableObject = null;
        }

        if (grab && angle<100 && !touch)
            angle += 2f;
        if (!grab && angle > 0)
            angle -= 2f;

        
        fingerJoint1.localEulerAngles = default1 + new Vector3(0, angle / 2, 0);
        if (!(transform.name == "thumb_03_L" || transform.name == "thumb_03_R"))
        {
            fingerJoint3.localEulerAngles = default3 + new Vector3(0, angle, 0);
            fingerJoint2.localEulerAngles = default2 + new Vector3(0, angle / 1.5f, 0);
        }
        if (transform.name == "thumb_03_L")
        {
            fingerJoint3.localEulerAngles = default3 + new Vector3(angle / 3, Mathf.Clamp(angle / 1.5f, 0, 25), -angle / 5);
            fingerJoint2.localEulerAngles = default2 + new Vector3(0, angle / 2f, 0);
        }
        if (transform.name == "thumb_03_R")
        {
            fingerJoint3.localEulerAngles = default3 + new Vector3(-angle / 3f, Mathf.Clamp(angle / 1.5f, 0, 25), angle / 5);
            fingerJoint2.localEulerAngles = default2 + new Vector3(0, angle / 2f, 0);
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (grab && other.transform.tag == "PlayableObject")
        {
            touch = true;
            touched = true;
            playableObject = other.gameObject.transform;
            print(playableObject.name);
        }
    }

    private void OnTriggerExit(Collider other)
    {
    }
}
