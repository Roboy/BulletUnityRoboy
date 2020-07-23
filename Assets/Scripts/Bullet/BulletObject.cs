﻿using System;
using System.Collections;
using Bullet.Helper;
using RosSharp;
using UnityEngine;
using Utils;

namespace Bullet
{
    /// <summary>
    /// A [BulletStaticObject] is an object, that never changes (moves) after instantiation.
    /// </summary>
    public class BulletObject : MonoBehaviour
    {
        [SerializeField] private string urdfPath;

        [SerializeField] private float instantiateDelay = 1.0f;

        [SerializeField] private bool isStatic = true;

        private BulletBridge _bulletBridge;
        private readonly BulletBodyInformation _bulletBodyInformation = new BulletBodyInformation();

        public BulletBodyInformation BulletBodyInformation => _bulletBodyInformation;

        private void Start()
        {
            _bulletBridge = GameObject.FindGameObjectWithTag("BulletConnectionController").GetComponent<BulletBridge>();
            if (!_bulletBridge.isInitialized)
            {
                Debug.LogError("BulletBridge is not ready");
                Application.Quit();
            }

            BulletBridge.MakeKinematic(GetComponentsInChildren<Rigidbody>());

            StartCoroutine(InstantiateObject());
        }

        private IEnumerator InstantiateObject()
        {
            Debug.Log("[Object] Spawning " + urdfPath + " in " + instantiateDelay + " seconds...");
            yield return new WaitForSeconds(instantiateDelay);

            _bulletBodyInformation.UrdfPadth = urdfPath;
            _bulletBodyInformation.Position = transform.position;
            _bulletBodyInformation.Rotation = transform.rotation;
            _bulletBodyInformation.Scaling = transform.localScale.x;
            _bulletBodyInformation.IsStatic = isStatic;
            _bulletBodyInformation.Instantiated = false;

            yield return true;
        }

        private void Update()
        {
            if (!_bulletBodyInformation.Instantiated) return;
            if (_bulletBodyInformation.IsStatic) return;

            transform.position = _bulletBodyInformation.Position;
            if (Math3d.AreQuaternionsClose(transform.rotation, _bulletBodyInformation.Rotation))
            {
                transform.rotation = _bulletBodyInformation.Rotation;
            }
        }
    }
}