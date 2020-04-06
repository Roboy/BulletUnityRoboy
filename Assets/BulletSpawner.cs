using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BulletSpawner : MonoBehaviour
{
    [Header("Attributes")]
    public Vector3 Size = new Vector3(0.5f, 0.5f, 0.5f);
    public float Mass = 1.0f;
    public float Gravity = -9.81f;

    [Header("References")]
    public Transform Origin;
    public Transform Target;

    [SerializeField]
    private List<GameObject> Bullets = new List<GameObject>();

    void Start()
    {
        Physics.gravity = new Vector3(0.0f, Gravity, 0.0f);   
    }


    void Update()
    {
        if (Input.GetKeyDown(KeyCode.I)) 
        {
            SpawnBullet();
        }

    }

    private void OnValidate()
    {
        Physics.gravity = new Vector3(0.0f, Gravity, 0.0f);
    }

    private void SpawnBullet() 
    {
        //Calculate vector to target object
        Vector3 direction = Target.position - Origin.position;
        direction = direction.normalized;
        //Spawn bullet and initialize it
        GameObject go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        go.transform.localScale = Size;
        go.transform.position = Origin.position;
        go.transform.forward = direction;
        //Add rigibody component and set it's mass
        Rigidbody rb = go.AddComponent<Rigidbody>();
        rb.mass = Mass;
        //Apply force in direction to target object
        rb.AddForce(direction, ForceMode.Impulse);
        Bullets.Add(go);
    }

    private void UpdateBullets() 
    {
        if (Bullets.Count > 0) 
        {
        foreach (GameObject go in Bullets) 
            {
                //CHECK FOR DESTROY AND THEN KILL IT
            }
        }
    
    }
}
