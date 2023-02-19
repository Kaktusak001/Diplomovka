using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NavigationScript : MonoBehaviour
{
    [SerializeField] private RoombaScript roomba;
    [SerializeField] private ConsoleScript console;
    
    [SerializeField] private float distanceSensitivity;
    private float _rotationSensitivity = 1f / 90f;
    
    [SerializeField] private double distanceCalibration; // the length of time the motors need on full power to travel 1 unit
    [SerializeField] private double rotationCalibration; // the length of time the motors need on full power to rotate 90 degrees

    private Vector3 _calibrationStartPosition;
    private Quaternion _calibrationStartRotation;
    private double _calibrationStartTime = double.NaN;

    private List<Command> _commands = new List<Command>();
    private double _timer = Double.NaN;

    private void FixedUpdate()
    {
        if (!double.IsNaN(_calibrationStartTime))
        {
            if (float.IsNegativeInfinity(_calibrationStartPosition.x))
            {
                if (Quaternion.Angle(_calibrationStartRotation, roomba.transform.rotation) < 90f) return;
                
                rotationCalibration = Time.timeAsDouble - _calibrationStartTime;
                _calibrationStartTime = Double.NaN;
                roomba.ResetRoomba();
                console.PrintLine("Calibration complete.");
            }
            else if ((_calibrationStartPosition - roomba.transform.position).magnitude >= 1f)
            {
                distanceCalibration = Time.timeAsDouble - _calibrationStartTime;
                roomba.rightPower = -1f;
                _calibrationStartPosition = Vector3.negativeInfinity;
                _calibrationStartRotation = roomba.transform.rotation;
                _calibrationStartTime = Time.timeAsDouble;
                console.PrintLine("Calibrating rotation...");
            }
        }
        else if (_commands.Count > 0)
        {
            if (double.IsNaN(_timer))
            {
                _timer = Time.timeAsDouble;
                roomba.rightPower = _commands[0].RightPower;
                roomba.leftPower = _commands[0].LeftPower;
                return;
            }

            if (Time.timeAsDouble - _timer < _commands[0].Time) return;
            
            _commands.RemoveAt(0);
            _timer = double.NaN;
            roomba.rightPower = 0f;
            roomba.leftPower = 0f;
        }
    }

    public void Forward(float distance, float speed) => _commands.Add(new Command(speed, speed, (distance * distanceSensitivity * distanceCalibration) / speed));
    public void Backward(float distance, float speed) => _commands.Add(new Command(-speed, -speed, (distance * distanceSensitivity * distanceCalibration) / speed));
    public void Right(float angle, float speed) => _commands.Add(new Command(-speed, speed, (angle * _rotationSensitivity * rotationCalibration) / speed));
    public void Left(float angle, float speed) => _commands.Add(new Command(speed, -speed, (angle * _rotationSensitivity * rotationCalibration) / speed));

    public void Stop()
    {
        _timer = double.NaN;
        _calibrationStartTime = double.NaN;
        roomba.rightPower = 0f;
        roomba.leftPower = 0f;
        _commands.Clear();
    }

    public void Calibrate()
    {
        roomba.ResetRoomba();
        roomba.rightPower = 1f;
        roomba.leftPower = 1f;
        _calibrationStartPosition = roomba.transform.position;
        _calibrationStartTime = Time.timeAsDouble;
        console.PrintLine("Calibrating position...");
    }

    private bool IsCalibrating() => !double.IsNaN(_calibrationStartTime);
    
    private struct Command
    {
        public float RightPower;
        public float LeftPower;
        public double Time;

        public Command(float rightPower, float leftPower, double time)
        {
            this.RightPower = rightPower;
            this.LeftPower = leftPower;
            this.Time = time;
        }
    }
}