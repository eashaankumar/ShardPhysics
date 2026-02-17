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
        public readonly float3 center;
        public readonly float3 extents;
        public readonly float3 halfExtents;
        public readonly quaternion rot;

        public Box(float3 center, quaternion rot, float3 halfExtents)
        {
            this.center = center;
            this.rot = rot;
            this.halfExtents = halfExtents;
            this.extents = halfExtents * 2;
        }
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

    public struct SeparatingAxises
    {
        // --- Face axes (always valid; should already be unit) ---
        public float3 a1, a2, a3; // A0,A1,A2
        public float3 b1, b2, b3; // B0,B1,B2

        // --- Cross axes (need validity + safe normalize) ---
        public float3 a1xb1, a1xb2, a1xb3;
        public float3 a2xb1, a2xb2, a2xb3;
        public float3 a3xb1, a3xb2, a3xb3;

        public bool a1xb1Valid, a1xb2Valid, a1xb3Valid;
        public bool a2xb1Valid, a2xb2Valid, a2xb3Valid;
        public bool a3xb1Valid, a3xb2Valid, a3xb3Valid;

        public static readonly int NUM_SEPARATING_AXIS = 15;

        // Axis lookup (0..14)
        public float3 this[int index]
        {
            get
            {
                if (index == 0) return a1;
                if (index == 1) return a2;
                if (index == 2) return a3;

                if (index == 3) return b1;
                if (index == 4) return b2;
                if (index == 5) return b3;

                if (index == 6) return a1xb1;
                if (index == 7) return a1xb2;
                if (index == 8) return a1xb3;

                if (index == 9) return a2xb1;
                if (index == 10) return a2xb2;
                if (index == 11) return a2xb3;

                if (index == 12) return a3xb1;
                if (index == 13) return a3xb2;
                if (index == 14) return a3xb3;

                return float3.zero;
            }
        }

        // Validity lookup (only meaningful for indices 6..14)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsCrossAxisValid(int index)
        {
            // You can call this only for index 6..14 if you want.
            if (index == 6) return a1xb1Valid;
            if (index == 7) return a1xb2Valid;
            if (index == 8) return a1xb3Valid;

            if (index == 9) return a2xb1Valid;
            if (index == 10) return a2xb2Valid;
            if (index == 11) return a2xb3Valid;

            if (index == 12) return a3xb1Valid;
            if (index == 13) return a3xb2Valid;
            if (index == 14) return a3xb3Valid;

            // Face axes (0..5) are always valid
            return true;
        }

        /// <summary>
        /// Safely normalize each cross axis and set its valid flag.
        /// If near-zero length, mark invalid and set axis = 0.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void NormalizeCrossAxes(float epsLenSq = 1e-8f)
        {
            NormalizeCross(ref a1xb1, out a1xb1Valid, epsLenSq);
            NormalizeCross(ref a1xb2, out a1xb2Valid, epsLenSq);
            NormalizeCross(ref a1xb3, out a1xb3Valid, epsLenSq);

            NormalizeCross(ref a2xb1, out a2xb1Valid, epsLenSq);
            NormalizeCross(ref a2xb2, out a2xb2Valid, epsLenSq);
            NormalizeCross(ref a2xb3, out a2xb3Valid, epsLenSq);

            NormalizeCross(ref a3xb1, out a3xb1Valid, epsLenSq);
            NormalizeCross(ref a3xb2, out a3xb2Valid, epsLenSq);
            NormalizeCross(ref a3xb3, out a3xb3Valid, epsLenSq);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void NormalizeCross(ref float3 axis, out bool valid, float epsLenSq)
        {
            float lenSq = math.lengthsq(axis);
            if (lenSq <= epsLenSq)
            {
                axis = float3.zero;
                valid = false;
                return;
            }

            axis *= math.rsqrt(lenSq); // normalize without sqrt
            valid = true;
        }
    }
    public struct ComputedQuantities
    {
        public float3 box1ToBox2CenterVec;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Solve(Box box1, Box box2, out BoxBoxContactPoints bbcps)
    {
        bbcps = default;

        BoxAxis box1Axis = GetBoxAxis(box1);
        BoxAxis box2Axis = GetBoxAxis(box2);

        SeparatingAxises sepAxs = ComputeSATAxisGroups(box1Axis, box2Axis);

        ComputedQuantities computedQuants = default;
        computedQuants.box1ToBox2CenterVec = box2.center - box1.center;

        float minOverlap = float.MaxValue;
        int minOverlapAxis = -1;

        // sat overlap test
        for(int axisI = 0; axisI < SeparatingAxises.NUM_SEPARATING_AXIS; axisI++)
        {
            // Skip invalid cross axes
            if (axisI >= 6 && !sepAxs.IsCrossAxisValid(axisI))
                continue;
            var n = sepAxs[axisI];
            var rA = ProjectRadii(box1, n, box1Axis);
            var rB = ProjectRadii(box2, n, box2Axis);
            var dist = math.abs(math.dot(computedQuants.box1ToBox2CenterVec, n));
            var overlap = (rA + rB) - dist;

            if (overlap < 0f) return false;

            if (minOverlapAxis < 0)
            {
                minOverlapAxis = axisI;
                minOverlap = overlap;
            }
            else if (overlap < minOverlap)
            {
                minOverlapAxis = axisI;
                minOverlap = overlap;
            }
        }

        // produce normal pointing from box1 to box2
        float3 penAxis = sepAxs[minOverlapAxis];
        if (math.dot(penAxis, computedQuants.box1ToBox2CenterVec) < 0f)
            penAxis = -penAxis;

        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float ProjectRadii(in Box box, float3 n, in BoxAxis boxAxis)
    {
        return box.halfExtents.x * math.abs(math.dot(n, boxAxis[0])) +
               box.halfExtents.y * math.abs(math.dot(n, boxAxis[1])) +
               box.halfExtents.z * math.abs(math.dot(n, boxAxis[2]));
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static BoxAxis GetBoxAxis(Box box)
    {
        BoxAxis boxAxis = default;
        boxAxis.right = math.rotate(box.rot, math.right());
        boxAxis.up = math.rotate(box.rot, math.up());
        boxAxis.forward = math.rotate(box.rot, math.forward());
        return boxAxis;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static SeparatingAxises ComputeSATAxisGroups(in BoxAxis a, in BoxAxis b)
    {
        SeparatingAxises groups = default;

        groups.a1 = a[0];
        groups.a2 = a[1];
        groups.a3 = a[2];

        groups.b1 = b[0];
        groups.b2 = b[1];
        groups.b3 = b[2];

        groups.a1xb1 = math.cross(a[0], b[0]);
        groups.a1xb2 = math.cross(a[0], b[1]);
        groups.a1xb3 = math.cross(a[0], b[2]);

        groups.a2xb1 = math.cross(a[1], b[0]);
        groups.a2xb2 = math.cross(a[1], b[1]);
        groups.a2xb3 = math.cross(a[1], b[2]);

        groups.a3xb1 = math.cross(a[2], b[0]);
        groups.a3xb2 = math.cross(a[2], b[1]);
        groups.a3xb3 = math.cross(a[2], b[2]);

        groups.NormalizeCrossAxes();

        return groups;
    }
}
