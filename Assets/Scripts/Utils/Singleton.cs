using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Utils
{
    public class Singleton<T> : MonoBehaviour where T : MonoBehaviour
    {
        private static bool _mShuttingDown = false;
        private static object _mLock = new object();
        private static T _mInstance;

        public static T Instance
        {
            get
            {
                if (_mShuttingDown)
                {
                    Debug.LogWarning("[Singleton] Instance '" + typeof(T) +
                                     "' already destroyed. Returning null.");
                    return null;
                }

                lock (_mLock)
                {
                    if (_mInstance == null)
                    {
                        T[] instances;

                        // Search for existing instance.
                        instances = (T[])FindObjectsOfType(typeof(T));

                        if (instances.Length == 0)
                            Debug.LogError("No instance of " + typeof(T) + " found in scene");
                        else if (instances.Length > 1)
                            Debug.LogError("More than one instance of " + typeof(T) + " found in scene");
                        else
                            _mInstance = instances[0];
                    }

                    return _mInstance;
                }
            }
        }

        private void OnApplicationQuit()
        {
            _mShuttingDown = true;
        }


        private void OnDestroy()
        {
            _mShuttingDown = true;
        }
    }
}