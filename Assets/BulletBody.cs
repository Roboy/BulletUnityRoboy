using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEditor.Animations;
using System.Linq;

public class BulletBody : MonoBehaviour
{

    public string urdfPath;
    public bool resetPose;
    public Transform baseFrame;
    public Vector3 translation;
    public Vector3 rotation;
    public bool useFixedBase;

    private IntPtr pybullet;
    private BulletBridge bb;
    private int b3BodyId;
    private bool isInitialized;

    void Awake()
    {
        isInitialized = false;
        var parentBodies = GetComponentsInParent<BulletBody>();
        
        if (parentBodies.Length>1)
        {
            if (!parentBodies.Last().isInitialized)
                enabled = false;
        }
    }
    // Start is called before the first frame update
    void Start()
    {
        bb = GameObject.Find("BulletBridge").GetComponent<BulletBridge>();
        while (!bb.isInitialized)
        {
            StartCoroutine(bb.WaitForConnection());
            //bb = GameObject.Find("BulletBridge").GetComponent<BulletBridge>();
        }
        pybullet = bb.GetPhysicsServerPtr();
        Debug.Log("Connected " + name + " to bullet server.");

        BulletBridge.MakeKinematic(GetComponentsInChildren<Rigidbody>());
        if (resetPose)
        {
            var newPos = baseFrame.position + baseFrame.InverseTransformPoint(translation);
            var newOrn = rotation + baseFrame.rotation.eulerAngles;
            transform.SetPositionAndRotation(newPos, Quaternion.Euler(newOrn));
            b3BodyId = bb.LoadURDF(Application.dataPath + urdfPath, newPos, Quaternion.Euler(newOrn), useFixedBase ? 1 : 0);
        }
        else
        {
            b3BodyId = bb.LoadURDF(Application.dataPath + urdfPath, transform.position, transform.rotation, useFixedBase ? 1 : 0);
        }
        
        bb.AddGameObject(gameObject, b3BodyId);

        var childBodies = GetComponentsInChildren<BulletBody>();
        isInitialized = true;
        foreach (var child in childBodies)
        {
            child.enabled = true;
        }
    }

    // Update is called once per frame
    void Update()
    {
        bb.SyncPoseBullet2Unity(gameObject);
    }
}
