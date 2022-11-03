using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InputManager : MonoBehaviour
{
    [Header("References")]
    PlayerControls playerControls;

    [Header("Input Variables")]
    public static float accelerationInput;
    public static float steeringInput;

    void Awake()
    {
        playerControls = new PlayerControls();
    }

    void Update()
    {
        accelerationInput = playerControls.Car.Acceleration.ReadValue<float>();
        steeringInput = playerControls.Car.Steering.ReadValue<float>();
    }

    void OnEnable()
    {
        playerControls.Enable();
    }

    void OnDisable()
    {
        playerControls.Disable();
    }
}
