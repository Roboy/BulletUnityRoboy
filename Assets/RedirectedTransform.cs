using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RedirectedTransform : MonoBehaviour
{
    [SerializeField]
    private float redirectFactor = 1.0f;

    private GameObject _helperObject;

    public Transform Transform => _helperObject.transform;

    private void Start()
    {
        _helperObject = new GameObject();
    }

    private void Update()
    {
        var position = gameObject.transform.position;
        _helperObject.transform.position = new Vector3(position.x * redirectFactor, position.y * redirectFactor, position.z);
        _helperObject.transform.rotation = gameObject.transform.rotation;
    }
}
