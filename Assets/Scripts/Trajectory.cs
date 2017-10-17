using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Trajectory class provides a set of methods for calculating and rendering a trajectory line with a few overloads for diffretn outputs.
// It also contains a set of methods to calculate a certain parameters of trajectory
public class Trajectory {
    

    public static Vector3[] trajectory(Vector3 objPos, Vector3 velocity, float step, out RaycastHit Hit)
    {
        RaycastHit hit = new RaycastHit();
        float tetha, phi;
        float v = velocity.magnitude;

        sphericalCoordinates(velocity, out tetha, out phi);

        float dist = distance(v, tetha, objPos.y);

        int vertices = Mathf.CeilToInt(dist / step);

        List<Vector3> vertexList = new List<Vector3>();
        List<Vector3> velocityList = new List<Vector3>();

        for (int n = 0; n <= vertices; n++)
        {
            float x = n * step * Mathf.Cos(-phi * Mathf.PI / 180) + objPos.x;
            float y = height(n * step, v, tetha, objPos.y);
            float z = n * step * Mathf.Sin(-phi * Mathf.PI / 180) + objPos.z;
            float vx = v * Mathf.Cos(tetha * Mathf.PI / 180) * Mathf.Cos(-phi * Mathf.PI / 180);
            float vy = v * Mathf.Sin(tetha * Mathf.PI / 180) - ((9.81f * n * step) / v * Mathf.Cos(tetha * Mathf.PI / 180));
            float vz = v * Mathf.Cos(tetha * Mathf.PI / 180) * Mathf.Sin(-phi * Mathf.PI / 180);
            velocityList.Add(new Vector3(vx, vy, vz));
            vertexList.Add(new Vector3(x, y, z));

            if (Physics.Raycast(vertexList[n], velocityList[n], out hit, 1.05f * step))
            {
                vertexList.Add(hit.point);
                break;
            }
            else if (n == vertices)
            {
                hit.point = new Vector3(dist * Mathf.Cos(-phi * Mathf.PI / 180) + objPos.x, 0, dist * Mathf.Sin(-phi * Mathf.PI / 180) + objPos.z);
                hit.normal = Vector3.up;
                vertexList.Add(hit.point);
            }
        }

        Hit = hit;

        Vector3[] vertex = new Vector3[vertexList.Count];
        for (int i = 0; i < vertexList.Count; i++)
            vertex[i] = vertexList[i];

        return vertex;
    }

    public static Vector3[] trajectory(Vector3 objPos, Vector3 velocity, float step, out Vector3[] instantaneousVelocity, out float[] time)
    {
        float tetha, phi;
        float v = velocity.magnitude;

        sphericalCoordinates(velocity, out tetha, out phi);

        float dist = distance(v, tetha, objPos.y);

        int vertices = Mathf.CeilToInt(dist / step);

        List<Vector3> vertexList = new List<Vector3>();
        List<Vector3> velocityList = new List<Vector3>();
        List<float> timeList = new List<float>();

        for (int n = 0; n <= vertices; n++)
        {
            float x = n * step * Mathf.Cos(-phi * Mathf.PI / 180) + objPos.x;
            float y = height(n * step, v, tetha, objPos.y);
            float z = n * step * Mathf.Sin(-phi * Mathf.PI / 180) + objPos.z;
            float vx = v * Mathf.Cos(tetha * Mathf.PI / 180) * Mathf.Cos(-phi * Mathf.PI / 180);
            float vy = v * Mathf.Sin(tetha * Mathf.PI / 180) - ((9.81f * n * step) / v * Mathf.Cos(tetha * Mathf.PI / 180));
            float vz = v * Mathf.Cos(tetha * Mathf.PI / 180) * Mathf.Sin(-phi * Mathf.PI / 180);
            float t = Trajectory.time(n * step, v, tetha);
            velocityList.Add(new Vector3(vx, vy, vz));
            vertexList.Add(new Vector3(x, y, z));
            timeList.Add(t);
        }

        Vector3[] vertex = new Vector3[vertexList.Count];
        for (int i = 0; i < vertexList.Count; i++)
            vertex[i] = vertexList[i];

        instantaneousVelocity = new Vector3[velocityList.Count];
        for (int i = 0; i < velocityList.Count; i++)
            instantaneousVelocity[i] = velocityList[i];

        time = new float[timeList.Count];
        for (int i = 0; i < timeList.Count; i++)
            time[i] = timeList[i];

        return vertex;
    }

    public static Vector3[] trajectory(Vector3 objPos, Vector3 velocity, float step)
    {
        RaycastHit hit = new RaycastHit();
        float tetha, phi;
        float v = velocity.magnitude;

        sphericalCoordinates(velocity, out tetha, out phi);

        float dist = distance(v, tetha, objPos.y);

        int vertices = Mathf.CeilToInt(dist / step);

        List<Vector3> vertexList = new List<Vector3>();
        List<Vector3> velocityList = new List<Vector3>();

        for (int n = 0; n <= vertices; n++)
        {
            float x = n * step * Mathf.Cos(-phi * Mathf.PI / 180) + objPos.x;
            float y = height(n * step, v, tetha, objPos.y);
            float z = n * step * Mathf.Sin(-phi * Mathf.PI / 180) + objPos.z;
            float vx = v * Mathf.Cos(tetha * Mathf.PI / 180) * Mathf.Cos(-phi * Mathf.PI / 180);
            float vy = v * Mathf.Sin(tetha * Mathf.PI / 180) - ((9.81f * n * step) / v * Mathf.Cos(tetha * Mathf.PI / 180));
            float vz = v * Mathf.Cos(tetha * Mathf.PI / 180) * Mathf.Sin(-phi * Mathf.PI / 180);
            velocityList.Add(new Vector3(vx, vy, vz));
            vertexList.Add(new Vector3(x, y, z));

            if (Physics.Raycast(vertexList[n], velocityList[n], out hit, 1.05f * step))
            {
                vertexList.Add(hit.point);
                break;
            }
            else if (n == vertices)
            {
                hit.point = new Vector3(dist * Mathf.Cos(-phi * Mathf.PI / 180) + objPos.x, 0, dist * Mathf.Sin(-phi * Mathf.PI / 180) + objPos.z);
                hit.normal = Vector3.up;
                vertexList.Add(hit.point);
            }
        }

        Vector3[] vertex = new Vector3[vertexList.Count];
        for (int i = 0; i < vertexList.Count; i++)
            vertex[i] = vertexList[i];

        return vertex;
    }

    public static void trajectoryLine(Transform marker, Vector3[] vertex, RaycastHit hit)
    {
        LineRenderer trajectory = marker.GetComponent<LineRenderer>();
        Material markerCyan = Resources.Load("marker") as Material;
        Material markerRed = Resources.Load("markerRed") as Material;
        trajectory.startWidth = 0.025f;
        if (hit.normal == Vector3.up)
        {
            trajectory.material = markerCyan;
            marker.GetComponent<MeshRenderer>().material = markerCyan;
            marker.FindChild("HaloCyan(Clone)").gameObject.SetActive(true);
            marker.FindChild("HaloRed(Clone)").gameObject.SetActive(false);
        }
        else
        {
            trajectory.material = markerRed;
            marker.GetComponent<MeshRenderer>().material = markerRed;
            marker.FindChild("HaloCyan(Clone)").gameObject.SetActive(false);
            marker.FindChild("HaloRed(Clone)").gameObject.SetActive(true);
        }
        
        
        trajectory.numPositions = vertex.Length;
        trajectory.SetPositions(vertex);
        marker.transform.up = hit.normal;
        marker.transform.position = hit.point;
    }

    // Mathematical equation for height of projectile trajectory.
    public static float height(float distance, float velocity, float tetha, float y0)
    {
        float height;
        if (velocity != 0)
            height = y0 + distance * Mathf.Tan(tetha * Mathf.PI / 180) - ((9.81f * Mathf.Pow(distance, 2)) / (2 * Mathf.Pow(velocity * Mathf.Cos(tetha * Mathf.PI / 180), 2)));
        else
            height = y0 + distance * Mathf.Tan(tetha * Mathf.PI / 180);
            return height;
    }

    // Mathematical equation for distance of projectile trajectory.
    public static float distance(float velocity, float tetha, float y0)
    {
        float dist = ((velocity * Mathf.Cos(tetha * Mathf.PI / 180)) / (9.81f)) * (velocity * Mathf.Sin(tetha * Mathf.PI / 180) + Mathf.Sqrt(Mathf.Pow(velocity * Mathf.Sin(tetha * Mathf.PI / 180), 2) + 2 * 9.81f * y0));
        return dist;
    }

    // Mathematical equation for time when projectile reaches certain horizontal distance
    public static float time(float distance, float velocity, float tetha)
    {
        float time = distance / (velocity * Mathf.Cos(tetha * Mathf.PI / 180));
        return time;
    }

    // Simple function for calculating Euler angles.
    public static void sphericalCoordinates(Vector3 direction, out float theta, out float phi)
    {
        theta = new float();
        phi = new float();
        if (direction.y >= 0)
        {
            theta = Vector3.Angle(new Vector3(direction.x, 0, direction.z), direction);
        }
        else if (direction.y < 0)
        {
            theta = -Vector3.Angle(new Vector3(direction.x, 0, direction.z), direction);
        }
        if (direction.z >= 0)
        {
            phi = -Vector3.Angle(new Vector3(direction.x, 0, direction.z), Vector3.right);
        }
        else if (direction.z < 0)
        {
            phi = Vector3.Angle(new Vector3(direction.x, 0, direction.z), Vector3.right);
        }
    }
}