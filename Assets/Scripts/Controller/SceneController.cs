using UnityEngine;

namespace Controller
{
    public class SceneController : MonoBehaviour
    {
        [SerializeField]
        private GameObject roboyHead;

        // Start is called before the first frame update
        void Start()
        {
            roboyHead.GetComponent<MeshRenderer>().enabled = false;
        }

        // Update is called once per frame
        void Update()
        {
        
        }
    }
}
