using UnityEditor;
using UnityEngine;

namespace Shard.Dev.Tests
{
    public class CylinderBoxTest : MonoBehaviour
    {
        [SerializeField] Transform boxTrans;
        [SerializeField] Transform cylinderTrans;

        CylinderBoxSolver.CylinderBoxContactPoints cps;
        bool hit;

        void Update()
        {
            hit = CylinderBoxSolver.Solve(new CylinderBoxSolver.Box(boxTrans.position, boxTrans.rotation, boxTrans.localScale / 2),
                                    new CylinderBoxSolver.Cylinder(cylinderTrans.position, cylinderTrans.rotation, cylinderTrans.localScale.y / 2, cylinderTrans.localScale.x / 2),
                                    out cps);        

            if (hit)
            {
                Debug.Log("Cylinder box hit! " + Time.time);
            }
            print(hit);
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {

            Handles.matrix = Matrix4x4.identity;
            Handles.color = Color.green;
            Handles.DrawWireDisc(cylinderTrans.position + cylinderTrans.up * cylinderTrans.localScale.y / 2, cylinderTrans.up, cylinderTrans.localScale.x / 2);

            if (!hit) return;
            for (int i = 0; i < cps.numContactPoints; i++)
            {
                DrawContactPoint(i);
            }
        }

        void DrawContactPoint(int i)
        {
            var cp = cps[i];

            Gizmos.matrix = Matrix4x4.identity;
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(cp.point, 0.02f);
            Gizmos.DrawLine(cp.point, cp.point + cp.normal * cp.depth);
        }
#endif
    }
}
