using UnityEngine;

public class SampleCameraFollowing : MonoBehaviour
{
    public Transform cameraTransform;
    public float distance = 2f;
    public Vector3 offset = new Vector3(0, 0, 0);

    void LateUpdate()
    {
        transform.position =
            cameraTransform.position +
            cameraTransform.forward * distance +
            offset;

        transform.rotation =
            Quaternion.LookRotation(
                transform.position - cameraTransform.position
            );
    }
}