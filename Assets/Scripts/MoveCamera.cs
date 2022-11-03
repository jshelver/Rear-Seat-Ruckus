using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveCamera : MonoBehaviour
{
    [SerializeField] Transform mainCamera;
    [SerializeField] Transform cameraHolder;
    [SerializeField] float cameraHeight = 2f;
    [SerializeField] float positionChangeSmoothTime = 0.15f;
    Vector3 positionVelocity;

    void Start()
    {

    }

    void LateUpdate()
    {
        mainCamera.position = Vector3.SmoothDamp(mainCamera.position, new Vector3(cameraHolder.position.x, transform.position.y + cameraHeight, cameraHolder.position.z), ref positionVelocity, positionChangeSmoothTime);
        Vector3 newEulerAngles = new Vector3(mainCamera.eulerAngles.x, transform.eulerAngles.y, mainCamera.eulerAngles.z);
        mainCamera.rotation = Quaternion.Euler(newEulerAngles);
    }
}