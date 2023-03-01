using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;

public class NavigationScript : MonoBehaviour
{
    [SerializeField] private RoombaScript roomba;
    [SerializeField] private ConsoleScript console;
    
    [SerializeField] private float distanceSensitivity;
    private float _rotationSensitivity = 1f / 90f;

    [SerializeField] private float wallFollowDistance;
    [SerializeField] private float wallFollowTolerance;
    [SerializeField] private float correctionAngle;
    [SerializeField] private float sweepWidth;
    
    [SerializeField] private double distanceCalibration; // the length of time the motors need on full power to travel 1 unit
    [SerializeField] private double rotationCalibration; // the length of time the motors need on full power to rotate 90 degrees

    private Vector3 _calibrationStartPosition;
    private Quaternion _calibrationStartRotation;
    private double _calibrationStartTime = double.NaN;

    public List<Command> commands = new List<Command>();
    private double _timer = Double.NaN;
    
    [SerializeField] private Vector3 estimatedPosition;
    [SerializeField] private float estimatedRotation;

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
            }
        }
        else if (commands.Count > 0)
        {
            if (double.IsNaN(_timer))
            {
                _timer = Time.timeAsDouble;
                roomba.rightPower = commands[0].RightPower;
                roomba.leftPower = commands[0].LeftPower;
                return;
            }
            
            if (commands[0].WallFollowDistance > 0f && roomba.GetSensorData(0).Distance >= 0f)
            {
                float wallDistance = roomba.GetSensorData(0).Distance;
                if (wallDistance - commands[0].WallFollowDistance > wallFollowTolerance)
                {
                    Command newCommand = commands[0];
                    newCommand.Time -= Time.timeAsDouble - _timer;
                    Stop(false, false);
                    commands.Insert(0, newCommand);
                    commands.Insert(0, Forward(wallFollowTolerance / distanceSensitivity, 1f));

                    if (wallDistance - newCommand.WallFollowDistance > 2f * wallFollowTolerance)
                    {
                        commands.Insert(0, Left(90, 1f));
                        commands.Insert(0, Forward((wallDistance - newCommand.WallFollowDistance) / distanceSensitivity, 1f));
                        commands.Insert(0, Right(90, 1f));
                    }
                    else
                        commands.Insert(0, Left(correctionAngle, 1f));
                }
                else if (commands[0].WallFollowDistance - wallDistance > wallFollowTolerance)
                {
                    Command newCommand = commands[0];
                    newCommand.Time -= Time.timeAsDouble - _timer;
                    Stop(false, false);
                    commands.Insert(0, newCommand);
                    commands.Insert(0, Forward(wallFollowTolerance / distanceSensitivity, 1f));
                    
                    if (newCommand.WallFollowDistance - wallDistance > 2f * wallFollowTolerance)
                    {
                        commands.Insert(0, Right(90, 1f));
                        commands.Insert(0, Forward((newCommand.WallFollowDistance - wallDistance) / distanceSensitivity, 1f));
                        commands.Insert(0, Left(90, 1f));
                    }
                    else
                        commands.Insert(0, Right(correctionAngle, 1f));
                }
            }

            if (Time.timeAsDouble - _timer >= commands[0].Time || (commands[0].RightPower > 0f && commands[0].LeftPower > 0f && (roomba.GetLeftBumperData() || roomba.GetRightBumperData() || roomba.DetectCliff())))
                Stop(false, true);
        }
    }

    public void NavigateSpace()
    {
        // Go forward until you hit a wall or a ledge
        // follow the wall or ledge and repeat 1 until you closed the space
        // if the space is open go 90deg from the wall until you find a new wall or ledge
        // repeat until you find a closed space
        // do loops around the space until wall sensors can no longer see the walls
        // if you hit any new walls map them
        // if you are about to hit a known wall, go around it
        // calculate area that wasnt covered
        // if there is any area left, find the longest edge and center it to a grid with cells smaller than roomba
        // one by one cover every cell that doesnt overlap with the covered area
    }
    
    public Command FollowWall(float distance, float speed) => new(speed, speed, (distance * distanceSensitivity * distanceCalibration) / speed, wallFollowDistance);

    public Command Forward(float distance, float speed) => new(speed, speed, (distance * distanceSensitivity * distanceCalibration) / speed);
    
    public Command Backward(float distance, float speed) => new(-speed, -speed, (distance * distanceSensitivity * distanceCalibration) / speed);
    public Command Right(float angle, float speed) => new(-speed, speed, (angle * _rotationSensitivity * rotationCalibration) / speed);
    public Command Left(float angle, float speed) => new(speed, -speed, (angle * _rotationSensitivity * rotationCalibration) / speed);

    public void ResetEstimatedPosition() => estimatedPosition = Vector3.zero;
    public void ResetEstimatedRotation() => estimatedRotation = 0f;

    public void Stop(bool clearCommands, bool invokeFunction)
    {
        if (Math.Sign(roomba.rightPower) == Math.Sign(roomba.leftPower))
            estimatedPosition += Quaternion.Euler(0f, estimatedRotation, 0f) * Vector3.forward * (roomba.rightPower * (float)((Time.timeAsDouble - _timer) / distanceCalibration));
        else
        {
            estimatedRotation += roomba.leftPower * (float)((Time.timeAsDouble - _timer) / rotationCalibration) / _rotationSensitivity;
            if (estimatedRotation is > 180f or < -180f) estimatedRotation = 180f - Math.Abs(estimatedRotation) + -180f * Math.Sign(estimatedRotation);
        }
        
        _timer = double.NaN;
        _calibrationStartTime = double.NaN;
        roomba.rightPower = 0f;
        roomba.leftPower = 0f;

        if (commands.Count == 0)
            return;

        if (invokeFunction)
            commands[0].PassAction?.Invoke();
        
        commands.RemoveAt(0);
            
        if (clearCommands)
            commands.Clear();
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
    
    [Serializable]
    public struct Command
    {
        public float RightPower;
        public float LeftPower;
        public double Time;
        public Action PassAction;
        public float WallFollowDistance;

        public Command(float rightPower, float leftPower, double time, float wallFollowDistance, Action passAction)
        {
            RightPower = rightPower;
            LeftPower = leftPower;
            Time = time;
            PassAction = passAction;
            WallFollowDistance = wallFollowDistance;
        }
        
        public Command(float rightPower, float leftPower, double time, float wallFollowDistance)
        {
            RightPower = rightPower;
            LeftPower = leftPower;
            Time = time;
            PassAction = null;
            WallFollowDistance = wallFollowDistance;
        }
        
        public Command(float rightPower, float leftPower, double time)
        {
            RightPower = rightPower;
            LeftPower = leftPower;
            Time = time;
            PassAction = null;
            WallFollowDistance = -1f;
        }
    }
}