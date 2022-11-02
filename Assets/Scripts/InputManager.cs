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
        Vector2 locomotionInput = playerControls.Car.Locomotion.ReadValue<Vector2>();
        accelerationInput = locomotionInput.y;
        steeringInput = locomotionInput.x;
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
