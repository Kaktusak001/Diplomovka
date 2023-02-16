using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NavigationScript : MonoBehaviour
{
    [SerializeField] private float distanceSensitivity;
    [SerializeField] private float distanceCalibration; // the length of time the motors need on full power to travel 1 unit
    [SerializeField] private float rotationCalibration; // the length of time the motors need on full power to rotate 90 degrees

    private void FixedUpdate()
    {
        
    }
}
