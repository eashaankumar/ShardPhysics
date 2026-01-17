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

    }

    public struct TransformQv
    {
        public float3 Position;
        public quaternion Rotation;

        public static TransformQv Identity => new TransformQv { Position = default, Rotation = quaternion.identity };
    }
}
