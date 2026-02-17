using UnityEngine;

namespace Shard.Dev.Tests
{
    public class BoxBoxSatTest : MonoBehaviour
    {
        [SerializeField] Transform box1;
        [SerializeField] Transform box2;


        BoxBoxSolver.BoxBoxContactPoints cps;
        bool collision;

        void Update()
        {
            collision = BoxBoxSolver.Solve(new BoxBoxSolver.Box(box1.position, box1.rotation, box1.localScale / 2), new BoxBoxSolver.Box(box2.position, box2.rotation, box2.localScale / 2), out cps);

            if (collision)
            {
                Debug.Log($"Collision! {Time.time}");
            }
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (!collision) return;
            for (int i = 0; i < cps.numContactPoints; i++)
            {
                DrawContactPoint(i);
            }
        }

        void DrawContactPoint(int i)
        {
            var cp = cps[i];

            Gizmos.matrix = Matrix4x4.identity;
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(cp.point, 0.02f);
            Gizmos.DrawLine(cp.point, cp.point + cp.normal * cp.depth);
        }
#endif
    }
}
