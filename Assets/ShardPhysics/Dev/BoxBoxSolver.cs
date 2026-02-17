using System.Runtime.CompilerServices;
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

    public struct BoxBoxContactPoints
    {
        public ContactPoint p1;
        public ContactPoint p2;
        public ContactPoint p3;
        public ContactPoint p4;
    }

    public struct Box
    {
        public float3 center;
        public float3 size;
        public quaternion rot;
    }

    public struct BoxAxis
    {
        public float3 right;
        public float3 up;
        public float3 forward;

        public float3 this[int index]
        {
            get
            {
                if (index == 0) return right;
                if (index == 1) return up;
                if (index == 2) return forward;
                return float3.zero;
            }
            set
            {
                if (index == 0) right = value;
                if (index == 1) up = value;
                if (index == 2) forward = value;

            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Solve(Box box1, Box box2, out BoxBoxContactPoints bbcps)
    {
        bbcps = default;

        BoxAxis box1Axis = GetBoxAxis(box1);
        BoxAxis box2Axis = GetBoxAxis(box2);

        return false;
    }

    public static BoxAxis GetBoxAxis(Box box)
    {
        BoxAxis boxAxis = default;
        boxAxis.right = math.rotate(box.rot, math.right());
        boxAxis.up = math.rotate(box.rot, math.up());
        boxAxis.forward = math.rotate(box.rot, math.forward());
        return boxAxis;
    }
}
