using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LimitationController : MonoBehaviour
{
    [Space(10)] [Header("Delay")] [Range(0.0f, 500.0f)] [SerializeField]
    private float trackingDelay = 0.0f;

    [Space(10)] [Header("Velocity")] [Range(0.0f, 10.0f)] [SerializeField]
    private float maxVelocity = 10.0f;
    [SerializeField]
    private bool updateVelocity = false;

    [Space(10)] [Header("Joint Limits")] [SerializeField]
    private bool switchRobot = false;

    private bool _switchInProgress = false;
    
    public float TrackingDelay
    {
        get => trackingDelay;
        set => trackingDelay = value;
    }

    public float MaxVelocity
    {
        get => maxVelocity;
        set => maxVelocity = value;
    }

    public bool UpdateVelocity
    {
        get => updateVelocity;
        set => updateVelocity = value;
    }

    public bool SwitchRobot
    {
        get => switchRobot;
        set => switchRobot = value;
    }

    public bool SwitchInProgress
    {
        get => _switchInProgress;
        set => _switchInProgress = value;
    }
}
