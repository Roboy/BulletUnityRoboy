using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class DisablePositionTracking : MonoBehaviour
{
    [SerializeField] private Camera mainCamera;

    // Start is called before the first frame update
    void Start()
    {
        //XRDevice.DisableAutoXRCameraTracking(mainCamera, true);
        //UnityEngine.XR.InputTracking.disablePositionalTracking = true;
    }

    // Update is called once per frame
    void Update()
    {
        mainCamera.transform.position = Vector3.zero;
    }
}