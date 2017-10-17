using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// This is the main class for player controlling, it handles movement of the player, both in VR and with keybord control,
/// interactions with playable objects around the scene and calculates the positions of playable objects as an input for other classes. 
/// </summary>
public class PlayerVR : MonoBehaviour
{
    // 
    public bool computerInput;
    int mouseSensitivity = 2;
    float walkingSpeed = 0.02f;
    float multiplier = 1;
    float cameraX = 0;
    float transformY = 0;
    Vector3 pos;
    float x, y, z;
    bool grab = false;
    bool steerRobot = false;
    public static bool objAttached = false;
    Vector3 translation, objTranslation, handTranslation, rotation;
    Quaternion objRotation, normalRot, handRotation;
    Transform hand;
    CharacterController character;
    GameObject Marker;
    Transform playableObject;
    Canvas menu;
    bool gotIt = false;
    float step;
    Vector3 previousPos, currentPos, futurePos;
    GameObject ball;

    // Use this for initialization
    void Start()
    {
        character = transform.GetComponent<CharacterController>();
        menu = FindObjectOfType<Canvas>();
        menu.enabled = true;
        hand = handControl.virtualPosRight;
    }

    // Update is called once per frame
    void Update()
    {
        
        if (Input.GetKeyDown(KeyCode.Escape))
            menu.enabled = !menu.enabled;
        if (menu.enabled)
        {
            Time.timeScale = 0;
            Input.ResetInputAxes();
            Cursor.visible = true;
        }
        else
        {
            Time.timeScale = 1;
            Cursor.visible = false;
            if (Input.GetKeyDown(KeyCode.G))
            {
                grab = !grab;
                Finger.grab = !Finger.grab;
            }

            if (Input.GetKeyDown(KeyCode.Space))
            {
                steerRobot = !steerRobot;
            }

            if (computerInput)
                computerControl();

            Teleporting();
        }

        if (grab && Finger.playableObject != null && !objAttached)
        {
            if (Finger.touched)
            {
                Finger.playableObject.GetComponent<Rigidbody>().isKinematic = true;
                Finger.playableObject.SetParent(hand);
                playableObject = Finger.playableObject;
                objAttached = true;
                if (Finger.playableObject.name == "Rubik's Cube")
                    RubiksCube.mix = false;
            }
        }

        if (RobotSpace.playerIn && objAttached)
            steeringRobot();
        else if(!objAttached)
            Shoot();

        if(objAttached && hand.childCount == 0)
        {
            objAttached = false;
            grab = !grab;
            Finger.grab = !Finger.grab;
        }

        // Detaching the object
        if (objAttached && !grab)
            {
            foreach (Transform t in hand)
                if (t.tag == "PlayableObject")
                {
                    t.parent = null;
                    objAttached = false;
                    Rigidbody rb = t.GetComponent<Rigidbody>();
                    rb.isKinematic = false;
                }
            }
    }

    void computerControl()
    {

        if (Input.GetKey(KeyCode.LeftShift))
            multiplier = 4;
        else
            multiplier = 1;

        character.Move(pos);
        transform.Find("Main Camera").transform.localRotation = Quaternion.Euler(cameraX, 0, 0);
        transform.rotation = Quaternion.Euler(0, transformY, 0);
        if (!(Input.GetKey(KeyCode.Mouse0) || Input.GetKey(KeyCode.Mouse1)))
        {
            transformY += Input.GetAxisRaw("Mouse X") * mouseSensitivity;
            cameraX = Mathf.Clamp(cameraX -= Input.GetAxisRaw("Mouse Y") * mouseSensitivity, -60, 50);
        }


        pos = Quaternion.Euler(0, transformY, 0) * new Vector3(Input.GetAxisRaw("Horizontal") * walkingSpeed * multiplier, 0, Input.GetAxisRaw("Vertical") * walkingSpeed * multiplier);
        hand.localPosition = hand.localPosition + handTranslation;
        hand.localRotation = hand.localRotation * handRotation;

        if (Input.GetKey(KeyCode.Mouse1))
        {
            x = Input.GetAxisRaw("Mouse Y") * mouseSensitivity;
            if (Input.GetKey(KeyCode.Mouse2))
            {
                z = -Input.GetAxisRaw("Mouse X") * mouseSensitivity;
                y = 0;
            }
            else
            {
                y = -Input.GetAxisRaw("Mouse X") * mouseSensitivity;
                z = 0;
            }

            handRotation = Quaternion.Euler(new Vector3(x, y, z));
        }
        else
        {
            handRotation = Quaternion.Euler(Vector3.zero);
        }

        if (Input.GetKey(KeyCode.Mouse0))
        {
            if (!Input.GetKey(KeyCode.Mouse2))
                handTranslation = new Vector3(Input.GetAxisRaw("Mouse X") * mouseSensitivity, Input.GetAxisRaw("Mouse Y") * mouseSensitivity) / 250 * Time.deltaTime;
            else
                handTranslation = new Vector3(Input.GetAxisRaw("Mouse X") * mouseSensitivity, 0, Input.GetAxisRaw("Mouse Y") * mouseSensitivity) / 250 * Time.deltaTime;
        }
        else
            handTranslation = Vector3.zero;

    }



    public void Restart()
    {
        SceneManager.LoadScene("vrHandControl");
    }

    public void ExitGame()
    {
        Application.Quit();
    }

    public void ComputerControl()
    {
        computerInput = !computerInput;
    }

    void Teleporting()
    {
        RaycastHit markerPos;
        if (Input.GetKeyDown(KeyCode.J))
        {
            Marker = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            Marker.transform.localScale = new Vector3(0.25f, 0.01f, 0.25f);
            Marker.GetComponent<Collider>().enabled = false;
            Marker.AddComponent<LineRenderer>();

            GameObject haloRed = Instantiate(Resources.Load("HaloRed") as GameObject);
            haloRed.transform.SetParent(Marker.transform);
            haloRed.transform.localPosition.Set(0, 0, 0);

            GameObject haloCyan = Instantiate(Resources.Load("HaloCyan") as GameObject);
            haloCyan.transform.SetParent(Marker.transform);
            haloCyan.transform.localPosition.Set(0, 0, 0);
        }

        if (Input.GetKey(KeyCode.J))
        {
            Vector3[] vertex = Trajectory.trajectory(hand.position, 10 * hand.forward, 0.05f, out markerPos);
            Trajectory.trajectoryLine(Marker.transform, vertex, markerPos);
        }

        if (Input.GetKeyUp(KeyCode.J))
        {
            if (Marker.transform.FindChild("HaloCyan(Clone)").gameObject.activeSelf)
                transform.position = new Vector3(Marker.transform.position.x, 0, Marker.transform.position.z);
            Destroy(Marker);
        }


        if (!Input.GetKey(KeyCode.J) && Marker != null)
            Destroy(Marker);
    }

    void steeringRobot()
    {
        foreach (Transform t in hand)
            if (t.tag == "PlayableObject")
            {
                Debug.DrawRay(t.position, t.position - hand.position, Color.red);
                Ray ray = new Ray(t.position, t.position - hand.position);
                Vector3 rayVector = ray.direction;
                Vector3 shift = new Vector3();
                RaycastHit hit;
                if (Physics.Raycast(ray, out hit))
                {
                    Vector3 vectorXY = Vector3.ProjectOnPlane(rayVector, t.forward);
                    Vector3 vectorXZ = Vector3.ProjectOnPlane(rayVector, t.up);
                    Vector3 vectorYZ = Vector3.ProjectOnPlane(rayVector, t.right);

                    float angleXY = Vector3.Angle(rayVector, vectorXY);
                    float angleXZ = Vector3.Angle(rayVector, vectorXZ);
                    float angleYZ = Vector3.Angle(rayVector, vectorYZ);

                    if (angleXY < angleXZ && angleXY < angleYZ)
                    {
                        objRotation = Quaternion.LookRotation(vectorXY, t.forward);
                        shift = vectorXY;
                    }
                    if (angleXZ < angleXY && angleXZ < angleYZ)
                    {
                        objRotation = Quaternion.LookRotation(vectorXZ, t.up);
                        shift = vectorXZ;
                    }
                    if (angleYZ < angleXZ && angleYZ < angleXY)
                    {
                        objRotation = Quaternion.LookRotation(vectorYZ, t.right);
                        shift = vectorYZ;
                    }
                    Debug.DrawLine(t.position, t.position + shift * 0.05f, Color.blue);
                }

                Vector3 objPos = t.position + transform.TransformVector(handTranslation);
                Vector3 coriolis = Quaternion.Inverse(Quaternion.Euler(0, transformY, 0)) * (Quaternion.Euler(0, transformY, 0) * (objPos - transform.position) - transform.rotation * (objPos - transform.position));

                Vector3 futureObjPos = objPos + shift * 0.05f + coriolis + pos;
                Quaternion futureObjRot = objRotation * Quaternion.Euler(0, 0, 90) * handRotation;

                InverseKinematics.objectPosition = futureObjPos;
                InverseKinematics.objectRotation = futureObjRot;
                if (Vector3.Distance(InverseKinematics.virtualPos.position, futureObjPos) < 0.05f)
                    Gripper.close = true;
            }
    }

    void Shoot()
    {
        if (Input.GetKeyDown(KeyCode.F))
        {
            InverseKinematics.step = 0.1f;
            Gripper.t = 0.05f;
            Gripper.close = false;
            if (ball != null)
                Destroy(ball);
            ball = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            ball.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            ball.tag = "PlayableObject";
            ball.GetComponent<Collider>().isTrigger = true;
            ball.AddComponent<Rigidbody>();
            ball.GetComponent<Rigidbody>().mass = 0.1f;
            ball.GetComponent<Rigidbody>().isKinematic = true;
            ball.GetComponent<Rigidbody>().useGravity = true;
            ball.GetComponent<MeshRenderer>().material.mainTexture = Resources.Load("baseballTexture") as Texture;
        }

        if (Input.GetKey(KeyCode.F))
        {
            Transform cam = transform.Find("Main Camera").transform;
            ball.transform.position = cam.position + 0.5f * cam.forward - 0.1f * cam.up;
            ball.transform.rotation = cam.rotation * Quaternion.Euler(-30,0,0);
        }

        if (Input.GetKeyUp(KeyCode.F))
        {
            ball.GetComponent<Rigidbody>().isKinematic = false;
            Vector3 v = 5 * ball.transform.forward;
            ball.GetComponent<Rigidbody>().velocity = v;
            Vector3[] velocity;
            float[] time;
            Vector3[] point = Trajectory.trajectory(ball.transform.position, v, 0.05f, out velocity, out time);
            for (int n = 0; n < point.Length; n++)
                if (Vector3.Distance(GameObject.Find("IRB 2600").transform.position + new Vector3(0, 0.3495002f, 0), point[n]) < InverseKinematics.range)
                {
                    InverseKinematics.objectPosition = point[n];
                    InverseKinematics.objectRotation = Quaternion.FromToRotation(Vector3.forward, velocity[n]);
                    float dist = 0.1237f - 0.05f;
                    Gripper.t = dist / (time[n] / Time.deltaTime);
                    Gripper.close = true;
                    break;
                }
        }
        if (ball != null)
        {
            if (Vector3.Distance(ball.transform.position, InverseKinematics.virtualPos.position) < 0.1f)
            {
                ball.GetComponent<Rigidbody>().isKinematic = true;
            }
            if (Vector3.Distance(GameObject.Find("IRB 2600").transform.position, ball.transform.position) < InverseKinematics.range)
            {
                currentPos = ball.transform.position;
                if (previousPos != Vector3.zero)
                {
                    step = Vector3.Distance(previousPos, currentPos);
                    Vector3 objPos = ball.transform.position;
                    Vector3 velocity = ball.transform.GetComponent<Rigidbody>().velocity;
                    float tetha, phi;
                    Trajectory.sphericalCoordinates(velocity, out tetha, out phi);

                    float x =  step * Mathf.Cos(-phi * Mathf.PI / 180) + objPos.x;
                    float y = Trajectory.height( step, velocity.magnitude, tetha, objPos.y);
                    float z =  step * Mathf.Sin(-phi * Mathf.PI / 180) + objPos.z;

                    InverseKinematics.objectPosition = new Vector3(x, y, z);
                    if(velocity.magnitude != 0)
                    InverseKinematics.objectRotation = Quaternion.FromToRotation(Vector3.forward, velocity);
                }
                previousPos = currentPos;
            }
        }
    }
}