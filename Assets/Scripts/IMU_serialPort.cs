using UnityEngine;
using System;
using System.IO.Ports;
using System.Collections;
using System.Threading;
using System.Text;


public class IMU_serialPort : MonoBehaviour {

    // Seperate thread for IMU's reading 
    Thread Thread;
    // Serial port we're reading from. Currently using USB, but will probably change for Bluetooth and/or MQTT in the future
    SerialPort serialPort;

    // Ingame camera steered by Playstation joystick 
    //public Camera Camera;

    // Array of gameobjects that real-life IMU's are assigned to. For now they're assigned inside editor,
    // later We will use app's UI to assign IMU's to humanoid rig joints.
    public GameObject[] IMU;
    // Array of gameobjects that should move accordingly to Users body. These are mostly joints of humanoid rig.
    public GameObject[] obj;

    // Values for highPass function which eliminates large noises in case of IMU's faulty sensors, mostly for debuging purposes.
    public float accelHighPass;
    public float gyroHighPass;
    public float magHighPass;

    // Quaternions array We use for setting rotation of each of humanoid rig joints. We use quaterninons rather than Euler angles,
    // since they are a common way of describing rotations in computer graphics and are immune to gimbal lock.  
    Quaternion[] Quat;


    // Number of sensors we are using.
    private int Sensors;
    // Serial port's name.
    public string portName;
    // Rate at which Arduino sends the packages.
    public int baudRate;
    // Amount of time for which thread should wait for next package.
    public int readTimeout;
    // Amplification of Madgwick's filter.
    public float KP;
    private bool stop = false;
    private bool connected = false;
    // Float array for storing IMU's readings.
    private float[] matrix;
    // Float array for storing Quaternions coordinates.
    private float[] quat;
    
    // Arrays of IMU's readings: Accelration, Gyroscope and Magnetometer in XYZ coordinate system.
    private float[] ax;
    private float[] ay;
    private float[] az;
    private float[] gx;
    private float[] gy;
    private float[] gz;
    private float[] mx;
    private float[] my;
    private float[] mz;
    
    // Axis and button readings of Playstation joystic currently used as a movement controller. 
    float jkX;
    float jkY;
    bool jkbutton;

    // Since we are using two seperate IMU types with slightly diffrent values read(i.e. inverted axis), we need to know how many of each o them are 
    // attached to Arduino's board. This numbers are generated in arduino and passed at the beggining od the package.
    private int LSM9DS0 = 0;
    private int MPU9250 = 3;

    // This bool variable is used for making OnButtonPressed function for Joystick Button.
    private bool click = false;
    // This bool variable is used to for calibration.
    private bool calibrated = false;

    // This part is used for initialization
    public void Start()
    {

        Sensors = IMU.Length;
    
        // Defining array size and setting initial values for the Madgwick's filter.
        quat = new float[Sensors*4];
        for (int n = 0; n<Sensors;n++)
        {
            quat[0 + n * 4] = 1f;
            quat[1 + n * 4] = 0f;
            quat[2 + n * 4] = 0f;
            quat[3 + n * 4] = 0f;

        }
        // Starting Thread.
        startThread();
        
        // In case if there are more objects of Obj array;
        if (obj.Length > Sensors)
        {
            print("Too few IMU's for this kinematic structure.");
        }

        Quat = new Quaternion[Sensors];

        ax = new float[Sensors];
        ay = new float[Sensors];
        az = new float[Sensors];
        gx = new float[Sensors];
        gy = new float[Sensors];
        gz = new float[Sensors];
        mx = new float[Sensors];
        my = new float[Sensors];
        mz = new float[Sensors];
    }
    
    private void Update()
    {
        // Using Playstation joystick to steer the camera.
        //Camera.transform.Translate(Vector3.forward * jkY/2000);
        //Camera.transform.Rotate(-Vector3.up * jkX/500);

        // Calculating Quaternions for both IMU types;

        if (LSM9DS0 != 0)
        {
            for (int n = 0; n < LSM9DS0; n++)
            {
                Quat[n] = Madgwick(n, gx[n] * Mathf.PI / 180f, gy[n] * Mathf.PI / 180f, gz[n] * Mathf.PI / 180f, ax[n], ay[n], az[n], mx[n], my[n], -mz[n], Time.deltaTime, KP);
            }
        }
        if (MPU9250 != 0)
        {
            for (int n = LSM9DS0; n < LSM9DS0 + MPU9250; n++)
            {
                Quat[n] = Madgwick(n, gx[n], gy[n], gz[n], ax[n], ay[n], az[n], mx[n], my[n], mz[n], Time.deltaTime, KP);
            }
        }
        // This piece is a simple walk-around to avoid using complicated quaternion mathematics. We want to let the user mimic current humanoid rig pose and
        // with the press of joystick button the rig starts to move with the user. This is much more user-friendly approach than forcing him to get into
        // T-pose position each time we want to calibrate the IMU's. On joystick button press we are setting IMU's gameobject as parent of Obj gameobject.
        // This means that from now on this objects will have exactly the same rotations. 
        for (int n = 0; n < Sensors; n++)
        {
            IMU[n].transform.rotation = Quat[n];
        }
        
        if (Input.GetKeyDown(KeyCode.Q))
        {
            if (!calibrated)
            {
                for (int n = 0; n < Sensors; n++)
                {
                    obj[n].transform.SetParent(IMU[n].transform);
                }
                calibrated = true;
            }
            else if (calibrated)
            {
                for (int n = 0; n < Sensors; n++)
                {
                    IMU[n].transform.DetachChildren();
                }
                calibrated = false;
            }
        }
    }

    // simple GUI for test purposes
    void OnGUI()
    {
        if (connected) {
                Rect rectObj = new Rect(40, 10, 200, 400);
                GUIStyle style = new GUIStyle();
                style.alignment = TextAnchor.UpperLeft;
                GUI.Box(rectObj, "Serial port connection: " + portName + "\n"
                            + "\nAcceleration(mg): \n"
                            + "X:" + ax[0] + " Y:" + ay[0] + " Z:" + az[0] + "\n"
                            + "X:" + ax[1] + " Y:" + ay[1] + " Z:" + az[1] + "\n"
                            + "X:" + ax[2] + " Y:" + ay[2] + " Z:" + az[2] + "\n"
                            + "X:" + ax[3] + " Y:" + ay[3] + " Z:" + az[3] + "\n"
                            + "X:" + ax[4] + " Y:" + ay[4] + " Z:" + az[4] + "\n"
                            + "X:" + ax[5] + " Y:" + ay[5] + " Z:" + az[5] + "\n"
                            + "X:" + ax[6] + " Y:" + ay[6] + " Z:" + az[6] + "\n"
                            + "\nGyro(deg/s): \n"
                            + "X:" + gx[0] + " Y:" + gy[0] + " Z:" + gz[0] + "\n"
                            + "X:" + gx[1] + " Y:" + gy[1] + " Z:" + gz[1] + "\n"
                            + "X:" + gx[2] + " Y:" + gy[2] + " Z:" + gz[2] + "\n"
                            + "X:" + gx[3] + " Y:" + gy[3] + " Z:" + gz[3] + "\n"
                            + "X:" + gx[4] + " Y:" + gy[4] + " Z:" + gz[4] + "\n"
                            + "X:" + gx[5] + " Y:" + gy[5] + " Z:" + gz[5] + "\n"
                            + "X:" + gx[6] + " Y:" + gy[6] + " Z:" + gz[6] + "\n"
                            + "\nMagnetic Field(mg): \n"
                            + "X:" + mx[0] + " Y:" + my[0] + " Z:" + mz[0] + "\n"
                            + "X:" + mx[1] + " Y:" + my[1] + " Z:" + mz[1] + "\n"
                            + "X:" + mx[2] + " Y:" + my[2] + " Z:" + mz[2] + "\n"
                            +" X:" + mx[3] + " Y:" + my[3] + " Z:" + mz[3] + "\n"
                            + "X:" + mx[4] + " Y:" + my[4] + " Z:" + mz[4] + "\n"
                            + "X:" + mx[5] + " Y:" + my[5] + " Z:" + mz[5] + "\n"
                            + "X:" + mx[6] + " Y:" + my[6] + " Z:" + mz[6] + "\n",
                             style);
            
        }
    }

    // initializing receiveData thread
    private void startThread()
    {
        Thread = new Thread(new ThreadStart(receiveData));
        Thread.IsBackground = true;
        Thread.Start();
    }

    // thread script
    private void receiveData()
    {
        print("Establishing connection");
        //assigning port name and baud rate(bits/sec) to our serial port object
        serialPort = new SerialPort(portName, baudRate);
        //read timeout defines how many miliseconds script should wait for next message
        serialPort.ReadTimeout = readTimeout;
        // Data terminal is ready
        serialPort.DtrEnable = true;
        serialPort.RtsEnable = true;
        try
        {
            //opening port
            serialPort.Open();
            print("Connection established");
            connected = true;
        }
        catch ( Exception error)
        {
            print("Could not establish connection");
            print(error);
        }
        
        // this loop runs until stop function is called
        while (stop != true)
        {
            try
            {
                string data = serialPort.ReadLine();
               
                // Assigning coordinates to array and converting them to float.
                char delimeter = ',';
                matrix = Array.ConvertAll(data.Split(delimeter), float.Parse);
                // Reading amount of each IMU type.
                //LSM9DS0 = (int)matrix[0];
                //MPU9250 = (int)matrix[1];
                for (int n = 0; n < Sensors; n++)
                {
                    ax[n] = highPass(matrix[0 + n * 9], accelHighPass);
                    ay[n] = highPass(matrix[1 + n * 9], accelHighPass);
                    az[n] = highPass(matrix[2 + n * 9], accelHighPass);
                    gx[n] = highPass(matrix[3 + n * 9], gyroHighPass);
                    gy[n] = highPass(matrix[4 + n * 9], gyroHighPass);
                    gz[n] = highPass(matrix[5 + n * 9], gyroHighPass);
                    mx[n] = highPass(matrix[6 + n * 9], magHighPass);
                    my[n] = highPass(matrix[7 + n * 9], magHighPass);
                    mz[n] = highPass(matrix[8 + n * 9], magHighPass);
                }
                // Playstation joystick readings are always at the end of the package.
                int l = matrix.Length;
                print(l);
                //jkbutton = joystickButton(matrix[l-3]);
                //jkX = highPass(matrix[l-2] - 526, 5f);
                //jkY = highPass(matrix[l-1]-495,5f);
                
            }
            // this exception happens whenever ReadTimeout is exceeded
            catch (TimeoutException)
            {
                print("Waiting for message");
            }
            catch(Exception error)
            {
                print(error.ToString());
            }
        }
    }
    // this function ends receiveData loop
    private void Stop()
    {
        lock (this)
        {
            stop = true; 
        }
    }
 
    // this function is called when user quits application (or quits playmode in editor)
    void OnApplicationQuit()
    {
        if (serialPort != null)
        {
            try
            {
                // closing port
                serialPort.Close();
                
                print("Connection closed");
            }
            catch (Exception error)
            {
                print(error);
            }
         
            if (Thread != null)
            {
                // finishing loop
                Stop();
                Thread = null;
                print("Thread closed");
            }
        }
    }


    // This is customized version of Open Source Madgwick's AHRS and IMU algorithms which can be found here: http://x-io.co.uk/open-source-ahrs-with-x-imu/ . They are changed
    // into a function that can calculate Quaternion coordinates for each IMU applied. I'm using float array for Quaternion coordinates because Unity's Quaternion struct 
    // rounds up each of them at the 2nd place after decimal point, therefore its value is always [1 0 0 0] and the filter gets stuck. 
    private Quaternion Madgwick(int n,float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float SamplePeriod, float Beta)
    {
        float q1 = quat[0 + n*4], q2 = quat[1 + n * 4], q3 = quat[2 + n * 4], q4 = quat[3 + n * 4];   // short name local variable for readability
        
        float norm;
        float hx, hy, _2bx, _2bz;
        float s1, s2, s3, s4;
        float qDot1, qDot2, qDot3, qDot4;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q1mx;
        float _2q1my;
        float _2q1mz;
        float _2q2mx;
        float _4bx;
        float _4bz;
        float _2q1 = 2f * q1;
        float _2q2 = 2f * q2;
        float _2q3 = 2f * q3;
        float _2q4 = 2f * q4;
        float _2q1q3 = 2f * q1 * q3;
        float _2q3q4 = 2f * q3 * q4;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;
        float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;
        float q4q4 = q4 * q4;

        // Normalise accelerometer measurement
        norm = Mathf.Sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0f) return new Quaternion(0,0,0,0); // handle NaN
        norm = 1 / norm;        // use reciprocal for division
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = Mathf.Sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0f) return new Quaternion(0,0,0,0); // handle NaN
        norm = 1 / norm;        // use reciprocal for division
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        _2q1mx = 2f * q1 * mx;
        _2q1my = 2f * q1 * my;
        _2q1mz = 2f * q1 * mz;
        _2q2mx = 2f * q2 * mx;
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
        _2bx = Mathf.Sqrt(hx * hx + hy * hy);
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
        _4bx = 2f * _2bx;
        _4bz = 2f * _2bz;

        // Gradient decent algorithm corrective step
        s1 = -_2q3 * (2f * q2q4 - _2q1q3 - ax) + _2q2 * (2f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2f * q2q4 - _2q1q3 - ax) + _2q1 * (2f * q1q2 + _2q3q4 - ay) - 4f * q2 * (1 - 2f * q2q2 - 2f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2f * q2q4 - _2q1q3 - ax) + _2q4 * (2f * q1q2 + _2q3q4 - ay) - 4f * q3 * (1 - 2f * q2q2 - 2f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2f * q2q4 - _2q1q3 - ax) + _2q3 * (2f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        norm = 1f /Mathf.Sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

        // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

        // Integrate to yield quaternion
        q1 += qDot1 * SamplePeriod;
        q2 += qDot2 * SamplePeriod;
        q3 += qDot3 * SamplePeriod;
        q4 += qDot4 * SamplePeriod;
        norm = 1f /Mathf.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
        quat[0 + n * 4] = q1 * norm;
        quat[1 + n * 4] = q2 * norm;
        quat[2 + n * 4] = q3 * norm;
        quat[3 + n * 4] = q4 * norm;
        // The quaternions coordinates are oriented in this way to match the actual IMU's coordinate system.
        return new Quaternion(-quat[1 + n * 4], quat[3 + n * 4], -quat[2+ n*4], -quat[0 + n * 4]);
        
    }

    // Function that eliminates large constant noises in these IMU's which have faulty sensors(~ 3xbigger than normal noises). The Madgwick's filter 
    // can't cope with them and they're causing drift. 
    float highPass(float var,float treshold)
    {
        if (Mathf.Abs(var) > treshold)
        {
            return var;
        }
        else
        {
            return 0;
        }
    }

    // Function that changes Playstation joystic button variable from int to bool.
    bool joystickButton(float button)
    {
        if (button == 1)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    // Function that returns true during the frame the Playstation joystick button is pressed.
    bool onButtonPressed(bool button)
    {
        if (button && !click)
        {
            click = true;
            return true;
        }
        if (!button && click)
        {
            click = false;
            return false;
        }
        else
        {
            return false;
        }
    }

}