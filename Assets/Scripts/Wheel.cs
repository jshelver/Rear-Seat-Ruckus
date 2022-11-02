using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class Wheel : MonoBehaviour
{
    [Header("References")]
    [SerializeField] Rigidbody rb;

    [Header("Suspension Settings")]
    [SerializeField] float wheelRadius = 1f;
    [SerializeField] float suspensionRestDistance = 0.7f;
    [SerializeField] float springTravelDistance = 0.2f;
    [SerializeField] float springStrength = 10000f;
    [SerializeField] float springDamper = 1400f;
    float minLength, maxLength;

    [Header("Friction Settings")]
    [Range(0, 1)][SerializeField] float lateralTireGripPercentage = 0.9f;
    [Range(0, 1)][SerializeField] float longitudinalTireGripPercentage = 0.7f;
    [Tooltip("Weight in Kilograms")][SerializeField] float tireMass = 10f;

    [Header("Aceleration/Braking Settings")]
    [SerializeField] float speedModifier = 2000f;
    [SerializeField] float carMaxSpeed = 10f;
    [SerializeField] AnimationCurve torqueCurve;

    void Start()
    {
        minLength = suspensionRestDistance - springTravelDistance;
        maxLength = suspensionRestDistance + springTravelDistance;
    }

    void FixedUpdate()
    {
        if (Physics.Raycast(transform.position, -transform.up, out RaycastHit hit, wheelRadius + maxLength))
        {
            SuspensionForce(hit);
            LateralFrictionForce();
            LongitudinalFrictionForce();
            AccelerationForce(InputManager.accelerationInput);
        }

    }

    private void SuspensionForce(RaycastHit _hit)
    {
        // World-space direction of spring force
        Vector3 springForceDirection = transform.up;

        // World-space velocity of tire
        Vector3 tireWorldVelocity = rb.GetPointVelocity(transform.position);

        // Calculate offset from raycast
        float offset = suspensionRestDistance - _hit.distance;

        // Calculate velocity along spring force direction
        float velocity = Vector3.Dot(springForceDirection, tireWorldVelocity);

        // Calculate magnitude of the spring force
        float force = (offset * springStrength) - (velocity * springDamper);

        // Apply the force at tire position (multiplied by 100 to make variables more readable)
        rb.AddForceAtPosition(springForceDirection * force * 100, transform.position);
    }

    private void LateralFrictionForce()
    {
        // World-space direction of friction force
        Vector3 frictionDirection = transform.right;

        // World-space velocity of tire
        Vector3 tireWorldVelocity = rb.GetPointVelocity(transform.position);

        // Calculate the tire's velocity in the friction direction
        float frictionDirectionVelocity = Vector3.Dot(frictionDirection, tireWorldVelocity);

        // By inverting the friciton direction velocity, tire grip can be controlled by adjusting the tireGripPercentage
        float desiredVelocityChange = -frictionDirectionVelocity * lateralTireGripPercentage;

        // Acceleration = change in velocity / time
        float desiredAcceleration = desiredVelocityChange / Time.fixedDeltaTime;

        rb.AddForceAtPosition(frictionDirection * tireMass * desiredAcceleration, transform.position);
    }

    private void LongitudinalFrictionForce()
    {
        // World-space direction of friction force
        Vector3 frictionDirection = transform.forward;

        // World-space velocity of tire
        Vector3 tireWorldVelocity = rb.GetPointVelocity(transform.position);

        // Calculate the tire's velocity in the friction direction
        float frictionDirectionVelocity = Vector3.Dot(frictionDirection, tireWorldVelocity);

        // By inverting the friciton direction velocity, tire grip can be controlled by adjusting the tireGripPercentage
        float desiredVelocityChange = -frictionDirectionVelocity * longitudinalTireGripPercentage;

        // Acceleration = change in velocity / time
        float desiredAcceleration = desiredVelocityChange / Time.fixedDeltaTime;

        rb.AddForceAtPosition(frictionDirection * tireMass * desiredAcceleration, transform.position);
    }

    private void AccelerationForce(float accelerationInput)
    {
        // World-space direction of acceleration/braking force
        Vector3 accelerationDirection = transform.forward;

        // Acceleration torque
        if (accelerationInput >= 0.01f || accelerationInput <= -0.01f)
        {
            // Forward speed of the car (in direction of driving)
            float carSpeed = Vector3.Dot(rb.transform.forward, rb.velocity);

            // Normalized car speed
            float normalizedCarSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carMaxSpeed);

            // Get the torque amount by checking car's current speed
            float torqueAmount = torqueCurve.Evaluate(normalizedCarSpeed) * accelerationInput;

            rb.AddForceAtPosition(accelerationDirection * torqueAmount * speedModifier, transform.position);
        }
    }
}
