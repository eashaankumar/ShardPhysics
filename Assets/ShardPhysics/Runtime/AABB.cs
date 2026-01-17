using System;
using Unity.Mathematics;

namespace Shard
{
    public struct Aabb
    {
        public float3 Min;
        public float3 Max;

        public float3 Extents => (Max - Min) * 0.5f;
        public float3 Center => (Max + Min) * 0.5f;

        public static Aabb Empty()
        {
            return new Aabb
            {
                Min = new float3(float.PositiveInfinity),
                Max = new float3(float.NegativeInfinity)
            };
        }

        public void Encapsulate(Aabb other)
        {
            Min = math.min(Min, other.Min);
            Max = math.max(Max, other.Max);
        }

        public static Aabb FromCenterExtents(float3 c, float3 e)
            => new Aabb { Min = c - e, Max = c + e };


        public static Aabb AabbForOrientedExtents(float3 center, quaternion rot, float3 extents)
        {
            // Conservative AABB extents for an oriented box with half-extents `extents`.
            float3x3 R = new float3x3(rot);
            float3 ax = math.abs(R.c0) * extents.x;
            float3 ay = math.abs(R.c1) * extents.y;
            float3 az = math.abs(R.c2) * extents.z;
            float3 aabbExt = ax + ay + az;
            return Aabb.FromCenterExtents(center, aabbExt);
        }

        public static Aabb ComputeConeLocalAabb(in ConeCollider c)
        {
            float r = math.max(0f, c.BaseRadius) + math.max(0f, c.RoundingRadius);
            float hy = math.max(0f, c.HalfHeight) + math.max(0f, c.RoundingRadius);

            // extents in the cone’s local axis-aligned frame (Y up)
            float3 ext = new float3(r, hy, r);

            // apply orientation
            return AabbForOrientedExtents(c.Center, c.Orientation, ext);
        }

        public static Aabb ComputeCylinderLocalAabb(in CylinderCollider c)
        {
            float r = math.max(0f, c.Radius) + math.max(0f, c.RoundingRadius);
            float hy = math.max(0f, c.HalfHeight) + math.max(0f, c.RoundingRadius);

            float3 ext = new float3(r, hy, r);
            return AabbForOrientedExtents(c.Center, c.Orientation, ext);
        }


    }

    public struct TransformQv
    {
        public float3 Position;
        public quaternion Rotation;

        public static TransformQv Identity => new TransformQv { Position = default, Rotation = quaternion.identity };
    }
}
