using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ProjectileShooter : MonoBehaviour
{
    [SerializeField] GameObject projectilePrefab;

    void Start()
    {
        InvokeRepeating("ShootProjectile", 4f, 5f);
    }

    private void ShootProjectile()
    {
        Rigidbody rb = GameObject.Instantiate(projectilePrefab, transform.position, GetRandomQuaternion()).GetComponent<Rigidbody>();
        Destroy(rb.gameObject, 3f);

        rb.AddForce(transform.forward * 5000f, ForceMode.Impulse);
    }

    private Quaternion GetRandomQuaternion()
    {
        Vector3 eulerAngles = new Vector3(Random.Range(0f, 360f), Random.Range(0f, 360f), Random.Range(0f, 360f));
        return Quaternion.Euler(eulerAngles);
    }
}
