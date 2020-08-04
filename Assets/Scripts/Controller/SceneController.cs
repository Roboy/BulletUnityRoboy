using UnityEngine;
using Valve.VR.InteractionSystem;

namespace Controller
{
    public class SceneController : MonoBehaviour
    {
        [SerializeField]
        private GameObject[] roboyHeadObjects;

        // Start is called before the first frame update
        void Start()
        {
            roboyHeadObjects.ForEach((o => o.GetComponent<MeshRenderer>().enabled = false));
        }

        // Update is called once per frame
        void Update()
        {
        
        }
    }
}
