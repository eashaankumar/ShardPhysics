using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

namespace Shard.Dev.Tests
{
    public class CylinderCylinderTest : MonoBehaviour
    {
        [SerializeField] Transform cyl1Trans;
        [SerializeField] Transform cyl2Trans;

        bool hit;
        CylinderCylinderSolver.CylinderCylinderContactPoints cps;
        private void Update()
        {
            hit = CylinderCylinderSolver.Solve(GetCylinder(cyl1Trans), GetCylinder(cyl2Trans), out cps);
        }

        CylinderCylinderSolver.Cylinder GetCylinder(Transform cyl)
        {
            return new CylinderCylinderSolver.Cylinder(cyl.transform.position, cyl.transform.rotation, cyl.transform.localScale.y / 2, cyl.transform.localScale.x / 2);
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (hit)
            {
                for (int i = 0; i < cps.numContactPoints; i++)
                {
                    DrawContactPoint(i);
                }
            }

            DrawCylinder(GetCylinder(cyl1Trans));
            DrawCylinder(GetCylinder(cyl2Trans));
        }

        void DrawCylinder(CylinderCylinderSolver.Cylinder cyl)
        {
            Handles.matrix = Matrix4x4.identity;
            Handles.color = Color.yellow;
            float3 up = math.rotate(cyl.rot, math.up());
            Handles.DrawWireDisc(cyl.center + up * cyl.halfHeight, up, cyl.radius);
        }

        void DrawContactPoint(int i)
        {
            var cp = cps[i];

            Gizmos.matrix = Matrix4x4.identity;
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(cp.point, 0.02f);
            Gizmos.DrawLine(cp.point, cp.point + cp.normal * cp.depth);
        }
#endif
    }
}
