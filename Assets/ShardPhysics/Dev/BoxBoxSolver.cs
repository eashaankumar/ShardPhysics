using Unity.Mathematics;
using UnityEngine;

public struct BoxBoxSolver
{
    public struct ContactPoint
    {
        public float3 point;
        public float3 normal;
        public float3 tangent;
    }

    public static void Solve(float3 center1, float3 size1, quaternion rot1, float3 center2, float3 size2, quaternion rot2, ContactPoint[] cps)
    {

    }
}
