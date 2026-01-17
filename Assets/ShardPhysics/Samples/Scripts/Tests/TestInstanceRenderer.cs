using UnityEngine;

namespace Shard.Samples
{
    public class TestInstanceRenderer : MonoBehaviour
    {
        [SerializeField] Mesh proxyMesh;
        [SerializeField] UnityEngine.Material sphereInstanceMat;
        SdfSphereInstancedRenderer sphereInstanceRenderer;
        private void Awake()
        {
            sphereInstanceRenderer = new SdfSphereInstancedRenderer(sphereInstanceMat, proxyMesh);
        }
        // Update is called once per frame
        void Update()
        {
            sphereInstanceRenderer.Render(new Vector3[] { Vector3.zero }, new Quaternion[] { Quaternion.identity }, new float[] { 1 }, new Color[] { Color.cyan }, new float[] { 0 }, new float[] { 0 });
        }
    }
}
