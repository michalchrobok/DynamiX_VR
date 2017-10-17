using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RubiksCube : MonoBehaviour
{
    // Current axis of rotation
    Vector3 axis;
    // Speed of mixing
    public int speed;
    // Variables used to randomize mixing of the cube
    int position, rotation, a;
    // Dynamic Gameobject used to turn particular side of rubik's cube 
    Transform side;
    bool rotate = false;
    // Boolean used to acces and turn on/off automixing
    public static bool mix;
    // Time given for each turn
    float time;

    // Basic class for rubik's cube object, for now it handles mixing the rubik's cube
    void Start()
    {
        
        side = new GameObject("side").transform;
        side.SetParent(transform);
        side.localPosition = Vector3.zero;
        side.localRotation = Quaternion.Euler(0, 0, 0);
        side.localScale = new Vector3(1, 1, 1);
    }

    // Update is called once per frame
    void Update()
    {
        // Randomizing input for side method

        if (mix && !rotate)
        { 
            int b = Random.Range(-100, 100);

            if (b >= -100 && b < -33)
                position = 1;
            if (b >= -33 && b < 33)
                position = 0;
            if (b <= 100 && b >= 33)
                position = -1;

            rotation = Random.Range(1, 2);

            int a = Random.Range(1, 3);
            switch (a)
            {
                case 1:
                    axis = Vector3.right;
                    break;
                case 2:
                    axis = Vector3.up;
                    break;
                case 3:
                    axis = Vector3.forward;
                    break;
            }

            Side(axis, position);
            time = 0;
            rotate = true;
        }

        // Rotating the side by given rotation
        if (rotate)
        {
            time += speed;
            if (time <= 100)
                side.transform.localEulerAngles = Vector3.Lerp(Vector3.zero, axis * rotation * 90, time / 100);
            else
                rotate = false;
        }
        }
    

    // This method parents 9 elements of the cube to side gameobject base on the input axis and position
     void Side(Vector3 axis, int pos)
    {
        Transform[] sidePieces = new Transform[side.childCount];
        for (int n = 0; n < side.childCount; n++)
            sidePieces[n] = side.GetChild(n);
        
        for (int n = 0; n < sidePieces.Length; n++)
            sidePieces[n].SetParent(transform);
        
        side.localPosition = Vector3.zero;
        side.localRotation = Quaternion.Euler(0, 0, 0);
        
        Transform[] pieces = new Transform[transform.childCount];
        for (int n = 0; n < transform.childCount; n++)
            pieces[n] = transform.GetChild(n);
        for (int i = 0; i < pieces.Length; i++)
        {
            if (axis == Vector3.right)
                if (Mathf.Abs(pieces[i].localPosition.x - pos) < 0.1f)
                    pieces[i].SetParent(side);

            if (axis == Vector3.up)
                if (Mathf.Abs(pieces[i].localPosition.y - pos) < 0.1f)
                    pieces[i].SetParent(side);

            if (axis == Vector3.forward)
                if (Mathf.Abs(pieces[i].localPosition.z - pos) < 0.1f)
                    pieces[i].SetParent(side);
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if(other.tag == "magicSphere" && transform.parent == null)
        {
            mix = true;
            GetComponent<Rigidbody>().isKinematic = true;
        }
    }
}