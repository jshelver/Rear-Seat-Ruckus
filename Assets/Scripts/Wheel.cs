using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class Wheel : MonoBehaviour
{
    [Header("References")]
    [SerializeField] Rigidbody rb;

    [Header("Suspension Settings")]
    [SerializeField] float suspensionRestDistance = 0.2f;
    [SerializeField] float springTravelDistance = 0.1f;
    [SerializeField] float springStrength = 100f;
    [SerializeField] float springDamper = 10f;
    float minLength, maxLength;

    [Header("Wheel Settings")]
    [SerializeField] float wheelRadius = 0.5f;

    void Start()
    {
        minLength = suspensionRestDistance - springTravelDistance;
        maxLength = suspensionRestDistance + springTravelDistance;
    }

    void FixedUpdate()
    {
        SuspensionForce();
    }

    private void SuspensionForce()
    {
        if (Physics.Raycast(transform.position, -transform.up, out RaycastHit hit, wheelRadius + maxLength))
        {
            // World-space direction of spring force
            Vector3 springForceDirection = transform.up;

            // Calculate offset from raycast
            float offset = suspensionRestDistance - hit.distance;

            // World-space velocity of tire
            Vector3 tireWorldVelocity = rb.GetPointVelocity(transform.position);

            // Calculate velocity along spring force direction
            float velocity = Vector3.Dot(springForceDirection, tireWorldVelocity);

            // Calculate magnitude of the spring force
            float force = (offset * springStrength) - (velocity * springDamper);

            // Apply the force at tire position
            rb.AddForceAtPosition(springForceDirection * force, transform.position);
        }
    }
}
