using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

public class RoombaScript : MonoBehaviour
{
    [SerializeField] private WheelCollider rightWheel;
    [SerializeField] private WheelCollider leftWheel;
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
    [SerializeField] private List<Transform> cliffSensors;
    private List<float> _cliffSensorData = new List<float>();

    [SerializeField] private bool debugData;

    private LayerMask _raycastLayerMask;

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
    }

    private void FixedUpdate()
    {
        // Motors
        rightWheel.motorTorque = Mathf.Clamp(rightPower, -1f, 1f) * rightTorque;
        leftWheel.motorTorque = Mathf.Clamp(leftPower, -1f, 1f) * leftTorque;

        rightWheel.brakeTorque = rightPower == 0f ? rightTorque : 0f;
        leftWheel.brakeTorque = leftPower == 0f ? leftTorque : 0f;
        
        // Sensors
        for (int i = 0; i < distanceSensors.Count; i++)
            _distanceSensorData[i] = Physics.Raycast(distanceSensors[i].position, distanceSensors[i].forward, out RaycastHit raycastHit, sensorMaxDistance, _raycastLayerMask, QueryTriggerInteraction.Ignore) ? raycastHit.distance : -1f;
        
        for (int i = 0; i < cliffSensors.Count; i++)
            _cliffSensorData[i] = Physics.Raycast(cliffSensors[i].position, -cliffSensors[i].up, out RaycastHit raycastHit, cliffSensorMaxDistance, _raycastLayerMask, QueryTriggerInteraction.Ignore) ? raycastHit.distance : -1f;

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

    public void ResetRoomba()
    {
        rightPower = 0;
        leftPower = 0;
        transform.position = Vector3.zero;
        transform.rotation = Quaternion.identity;
    }

    public List<Sensor> GetSensorData()
    {
        List<Sensor> sensors = new List<Sensor>();
        for (int i = 0; i < distanceSensors.Count; i++)
            sensors.Add(new Sensor(distanceSensors[i], _distanceSensorData[i]));
        return sensors;
    }
    
    public List<Sensor> GetCliffSensorData()
    {
        List<Sensor> sensors = new List<Sensor>();
        for (int i = 0; i < cliffSensors.Count; i++)
            sensors.Add(new Sensor(cliffSensors[i], _cliffSensorData[i]));
        return sensors;
    }
    
    public Sensor GetSensorData(int index) => new Sensor(distanceSensors[index], _distanceSensorData[index]);

    public int GetSensorCount() => distanceSensors.Count;

    public Sensor GetCliffSensorData(int index) => new Sensor(cliffSensors[index], _cliffSensorData[index]);

    public int GetCliffSensorCount() => cliffSensors.Count;

    public bool GetLeftBumperData() => _leftBumperSensorData;
    public bool GetRightBumperData() => _rightBumperSensorData;
}

public struct Sensor
{
    private Transform _location;
    private float _distance;

    public Sensor(Transform location, float distance)
    {
        _location = location;
        _distance = distance;
    }

    public Transform Location => _location;
    public float Distance => _distance;
}
