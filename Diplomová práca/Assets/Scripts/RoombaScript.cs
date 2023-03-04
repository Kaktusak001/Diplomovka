using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEditor.TerrainTools;
using UnityEngine;
using UnityEngine.Serialization;

public class RoombaScript : MonoBehaviour
{
    [SerializeField] private WheelCollider rightWheel;
    [SerializeField] private WheelCollider leftWheel;
    private float wheelDistance;
    [SerializeField] private float rightTorque;
    [SerializeField] private float leftTorque;
    public float rightPower;
    public float leftPower;

    [SerializeField] private CapsuleCollider leftBumperSensor;
    [SerializeField] private CapsuleCollider rightBumperSensor;
    private bool _leftBumperSensorData;
    private bool _rightBumperSensorData;

    [SerializeField] private float sensorMaxDistance;
    [SerializeField] private List<Transform> distanceSensors;
    private List<float> _distanceSensorData = new List<float>();

    [SerializeField] private float cliffSensorMaxDistance;
    [SerializeField] private float cliffSensorActivationDistance;
    [SerializeField] private List<Transform> cliffSensors = new List<Transform>();
    private List<float> _cliffSensorData = new List<float>();

    [SerializeField] private Transform baseStation;
    [SerializeField] private float baseDetectDistance;

    [SerializeField] private bool debugData;
    [SerializeField] private bool debugGizmos;
    [SerializeField] private NavigationScript navigation;
    
    [SerializeField] private double distanceCalibration;

    [SerializeField] private Vector3 estimatedPosition;
    [SerializeField] private float estimatedRotation;
    
    private List<Vector4> previousTransforms = new List<Vector4>();
    private int sample;
    [SerializeField] private int samplingRate;
    [SerializeField] private int maxPreviousTransforms;

    private int calibratingTransforms = -1;
    [SerializeField] private float calibrationTime;

    private LayerMask _raycastLayerMask;

    [SerializeField] private bool CALIBRATE; // TODO REMOVE WHEN DONE

    private void GetCapsuleData(CapsuleCollider capsule, out Vector3 start, out Vector3 end, out float radius)
    {
        Vector3 offset = Vector3.zero;
        offset[capsule.direction] = (capsule.height - capsule.radius) * 0.5f;
        
        start = capsule.transform.TransformPoint(capsule.center + offset);
        Transform capsuleTransform;
        end = (capsuleTransform = capsule.transform).TransformPoint(capsule.center - offset);

        Vector3 mapping = capsuleTransform.lossyScale;
        mapping[capsule.direction] = 0f;
        radius = Mathf.Max(mapping[0], mapping[1], mapping[2]) * capsule.radius;
    }

    private void Awake()
    {
        distanceSensors.ForEach(_ => _distanceSensorData.Add(-1f));
        cliffSensors.ForEach(_ => _cliffSensorData.Add(-1));
        _raycastLayerMask = LayerMask.GetMask("Default");
        wheelDistance = (rightWheel.transform.position - leftWheel.transform.position).magnitude;
    }

    public Vector3 GetBaseStationOffset() => baseStation.position;
    public bool GetDrawGizmos() => debugGizmos;

    private void EstimatePosition()
    {
        Vector4 baseTransform = BaseStationSensor();
        if (float.IsNaN(baseTransform.x))
        {
            float rightWheelDistance = Mathf.Clamp(rightPower, -1f, 1f) * (float) (Time.fixedDeltaTime / distanceCalibration);
            float leftWheelDistance = Mathf.Clamp(leftPower, -1f, 1f) * (float) (Time.fixedDeltaTime / distanceCalibration);
            float angle = (leftWheelDistance - rightWheelDistance) / wheelDistance;

            if (angle != 0f)
            {
                float radius = (rightWheelDistance + leftWheelDistance) / (angle * 2f);
                Vector3 localPositionChange = new Vector3(radius * (Mathf.Cos(angle) - 1f), 0f, radius * Mathf.Sin(angle));
                estimatedPosition += Quaternion.Euler(0f, estimatedRotation, 0f) * localPositionChange;
                estimatedRotation = (estimatedRotation + angle * Mathf.Rad2Deg) % 360f;
            }
            else
                estimatedPosition += Quaternion.Euler(0f, estimatedRotation, 0f) * Vector3.forward * rightWheelDistance;
        }
        else
        {
            estimatedPosition = baseTransform;
            estimatedRotation = baseTransform.w;
        }

        if (maxPreviousTransforms > -1 && sample == 0)
        {
            previousTransforms.Add(new Vector4(estimatedPosition.x, estimatedPosition.y, estimatedPosition.z, estimatedRotation));
            if (maxPreviousTransforms > 0)
                for (int i = 0; i < previousTransforms.Count - maxPreviousTransforms; i++)
                    previousTransforms.RemoveAt(0);
        }
        
        sample = (sample + 1) % samplingRate;
    }

    private void ProcessMotors()
    {
        rightWheel.motorTorque = Mathf.Clamp(rightPower, -1f, 1f) * rightTorque;
        leftWheel.motorTorque = Mathf.Clamp(leftPower, -1f, 1f) * leftTorque;

        rightWheel.brakeTorque = rightPower == 0f ? rightTorque : 0f;
        leftWheel.brakeTorque = leftPower == 0f ? leftTorque : 0f;
    }

    private void ProcessSensors()
    {
        for (int i = 0; i < distanceSensors.Count; i++)
            _distanceSensorData[i] = Physics.Raycast(distanceSensors[i].position, distanceSensors[i].forward, out RaycastHit raycastHit,
                sensorMaxDistance, _raycastLayerMask, QueryTriggerInteraction.Ignore) ? raycastHit.distance : -1f;
        
        for (int i = 0; i < cliffSensors.Count; i++)
            _cliffSensorData[i] = Physics.Raycast(cliffSensors[i].position, -cliffSensors[i].up, out RaycastHit raycastHit,
                cliffSensorMaxDistance, _raycastLayerMask, QueryTriggerInteraction.Ignore) ? raycastHit.distance : -1f;

        Vector3 capsuleStart;
        Vector3 capsuleEnd;
        float capsuleRadius;
        
        GetCapsuleData(leftBumperSensor, out capsuleStart, out capsuleEnd, out capsuleRadius);
        _leftBumperSensorData = Physics.CheckCapsule(capsuleStart, capsuleEnd, capsuleRadius, _raycastLayerMask, QueryTriggerInteraction.Ignore);
        
        GetCapsuleData(rightBumperSensor, out capsuleStart, out capsuleEnd, out capsuleRadius);
        _rightBumperSensorData = Physics.CheckCapsule(capsuleStart, capsuleEnd, capsuleRadius, _raycastLayerMask, QueryTriggerInteraction.Ignore);

        if (debugData)
        {
            Debug.Log("Distance sensors:");
            GetSensorData().ForEach(d => Debug.Log(d.Distance));
            Debug.Log("Cliff sensors:");
            GetCliffSensorData().ForEach(d => Debug.Log(d.Distance));
            Debug.Log("LeftBumper: " + GetLeftBumperData());
            Debug.Log("RightBumper: " + GetRightBumperData());
        }
    }

    private void OnDrawGizmos()
    {
        if (debugGizmos)
        {
            for (int i = 0; i < GetSensorCount(); i++)
            {
                Sensor s = GetSensorData(i);
                
                Gizmos.color = Color.red;
                if (s.Distance < 0f)
                    Gizmos.DrawRay(distanceSensors[i].position, distanceSensors[i].forward * sensorMaxDistance);
                else
                {
                    Gizmos.DrawRay(distanceSensors[i].position + distanceSensors[i].forward * s.Distance, distanceSensors[i].forward * (sensorMaxDistance - s.Distance));
                    Gizmos.color = Color.green;
                    Gizmos.DrawRay(distanceSensors[i].position, distanceSensors[i].forward * s.Distance);
                }
            }

            for (int i = 0; i < GetCliffSensorCount(); i++)
            {
                Sensor s = GetCliffSensorData(i);
                
                Gizmos.color = s.Distance > 0f && s.Distance < cliffSensorActivationDistance ? Color.red : Color.green;
                Gizmos.DrawRay(cliffSensors[i].position, cliffSensors[i].up * 0.1f);
            }

            Gizmos.color = GetRightBumperData() ? Color.green : Color.red;
            Gizmos.DrawRay(rightBumperSensor.transform.position, rightBumperSensor.transform.forward * 0.1f);
            
            Gizmos.color = GetLeftBumperData() ? Color.green : Color.red;
            Gizmos.DrawRay(leftBumperSensor.transform.position, leftBumperSensor.transform.forward * 0.1f);
            
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(rightWheel.transform.position, rightWheel.transform.up * 0.2f);
            Gizmos.DrawRay(leftWheel.transform.position, leftWheel.transform.up * 0.2f);
            
            Gizmos.color = Color.magenta;
            Gizmos.DrawRay(transform.position, transform.up * 0.4f);
            Gizmos.DrawRay(transform.position, transform.forward * 0.4f);
            
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(estimatedPosition, Vector3.up * 0.4f);
            Gizmos.DrawRay(estimatedPosition, Quaternion.Euler(0f, estimatedRotation, 0f) * Vector3.forward * 0.4f);
            
            Gizmos.color = Color.red;
            Gizmos.DrawLine(transform.position + transform.up * 0.4f, estimatedPosition + Vector3.up * 0.4f);
            Gizmos.DrawLine(transform.position + transform.forward * 0.4f, estimatedPosition + Quaternion.Euler(0f, estimatedRotation, 0f) * Vector3.forward * 0.4f);

            for (int i = 0; i < previousTransforms.Count; i++)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawRay(previousTransforms[i], Vector3.up * 0.4f);
                Gizmos.color = Color.cyan;
                Gizmos.DrawRay((Vector3)previousTransforms[i] + Quaternion.Euler(0f, previousTransforms[i].w, 0f) * Vector3.right * wheelDistance * 0.5f, Vector3.up * 0.2f);
                Gizmos.DrawRay((Vector3)previousTransforms[i] - Quaternion.Euler(0f, previousTransforms[i].w, 0f) * Vector3.right * wheelDistance * 0.5f, Vector3.up * 0.2f);

                if (i != 0)
                {
                    Gizmos.DrawLine((Vector3)previousTransforms[i] + Quaternion.Euler(0f, previousTransforms[i].w, 0f) * Vector3.right * wheelDistance * 0.5f + Vector3.up * 0.01f,
                        (Vector3)previousTransforms[i - 1] + Quaternion.Euler(0f, previousTransforms[i - 1].w, 0f) * Vector3.right * wheelDistance * 0.5f + Vector3.up * 0.01f);
                    Gizmos.DrawLine((Vector3)previousTransforms[i] - Quaternion.Euler(0f, previousTransforms[i].w, 0f) * Vector3.right * wheelDistance * 0.5f + Vector3.up * 0.01f,
                        (Vector3)previousTransforms[i - 1] - Quaternion.Euler(0f, previousTransforms[i - 1].w, 0f) * Vector3.right * wheelDistance * 0.5f + Vector3.up * 0.01f);
                }
            }
        }
    }

    private void FixedUpdate()
    {
        if (CALIBRATE) // TODO REMOVE WHEN DONE
        {
            CALIBRATE = false;
            CalibrateDistance();
        }
        
        EstimatePosition();
        ProcessSensors();
        navigation.UpdateNavigation();
        ProcessMotors();
    }

    public bool IsCalibrating() => calibratingTransforms == -1;

    public void CalibrateDistance()
    {
        if (calibratingTransforms != -1)
        {
            float distance = ((Vector3) previousTransforms[^1] - transform.position).magnitude;
            previousTransforms.RemoveAt(previousTransforms.Count - 1);
            distanceCalibration = calibrationTime / distance;
            maxPreviousTransforms = calibratingTransforms;
            calibratingTransforms = -1;
        }
        else
        {
            calibratingTransforms = maxPreviousTransforms;
            maxPreviousTransforms = -1;
            previousTransforms.Add(new Vector4(transform.position.x, transform.position.y, transform.position.z, transform.rotation.eulerAngles.y));
            rightPower = 1f;
            leftPower = 1f;
            Invoke(nameof(CalibrateDistance), calibrationTime);
        }
    }

    public void GoToBaseStation()
    {
        // TODO WHEN NAVIGATION IS FINISHED and transfer it to navigation
    }

    public Vector4 BaseStationSensor()
    {
        if ((baseStation.position - transform.position).magnitude < baseDetectDistance)
        {
            Vector4 ret = transform.position - baseStation.position;
            ret.w = transform.rotation.eulerAngles.y;
            return ret;
        }
        
        return new Vector4(float.NaN, float.NaN, float.NaN, float.NaN);
    }

    public void ResetRoomba()
    {
        rightPower = 0;
        leftPower = 0;
        transform.position = baseStation.position;
        transform.rotation = baseStation.rotation;
        estimatedPosition = Vector3.zero;
        estimatedRotation = 0f;
    }

    public List<Sensor> GetSensorData()
    {
        List<Sensor> sensors = new List<Sensor>();
        for (int i = 0; i < GetSensorCount(); i++)
            sensors.Add(new Sensor(distanceSensors[i].localPosition, distanceSensors[i].localRotation, _distanceSensorData[i]));
        return sensors;
    }
    
    public List<Sensor> GetCliffSensorData()
    {
        List<Sensor> sensors = new List<Sensor>();
        for (int i = 0; i < GetCliffSensorCount(); i++)
            sensors.Add(new Sensor(cliffSensors[i].localPosition, cliffSensors[i].localRotation, _cliffSensorData[i]));
        return sensors;
    }

    public bool DetectCliff(out List<bool> sensorActivation)
    {
        sensorActivation = new List<bool>();
        var list = sensorActivation;
        GetSensorData().ForEach(s => list.Add(s.Distance > cliffSensorActivationDistance || s.Distance < 0f));
        return sensorActivation.Any(s => s);
    }

    public bool DetectCliff() => GetCliffSensorData().Any(s => s.Distance > cliffSensorActivationDistance || s.Distance < 0f);
    
    public Sensor GetSensorData(int index) => new Sensor(distanceSensors[index].localPosition, distanceSensors[index].localRotation, _distanceSensorData[index]);

    public int GetSensorCount() => Math.Min(distanceSensors.Count, _distanceSensorData.Count);

    public Sensor GetCliffSensorData(int index) => new Sensor(cliffSensors[index].localPosition, cliffSensors[index].localRotation, _cliffSensorData[index]);

    public int GetCliffSensorCount() => Math.Min(cliffSensors.Count, _cliffSensorData.Count);

    public bool GetLeftBumperData() => _leftBumperSensorData;
    public bool GetRightBumperData() => _rightBumperSensorData;

    public Vector3 GetEstimatedPosition() => estimatedPosition;
    public float GetEstimatedRotation() => estimatedRotation;
}

[Serializable]
public class CellX
{
    public CellX(int index, List<CellY> celly)
    {
        this.index = index;
        this.celly = celly;
    }

    public int index;
    public List<CellY> celly;
}

[Serializable]
public class CellY
{
    public CellY(int index, bool wall, bool visited)
    {
        this.index = index;
        this.wall = wall;
        this.visited = visited;
    }

    public int index;
    public bool wall;
    public bool visited;
}

public struct Sensor
{
    private Vector3 _location;
    private Quaternion _rotation;
    private float _distance;

    public Sensor(Vector3 location, Quaternion rotation, float distance)
    {
        _location = location;
        _distance = distance;
        _rotation = rotation;
    }

    public Vector3 Location => _location;
    public Quaternion Rotation => _rotation;
    public float Distance => _distance;
}
