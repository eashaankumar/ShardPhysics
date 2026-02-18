using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Mathematics;
using UnityEngine;

namespace Shard.Runtime
{

    [BurstCompile]
    public struct ForceAccumulator
    {
        public float3 forceAccumulator;
        public float3 torqueAccumulator;

        public void ClearAccumulators()
        {
            forceAccumulator = 0;
            torqueAccumulator = 0;
        }
    }

    [BurstCompile]
    public struct Velocity
    {
        public float3 linearVelocity;
        public float3 angularVelocity;
    }

    [BurstCompile]
    public struct Mass
    {
        public float mass;
        public float invMass;
    }

    [BurstCompile]
    public struct Inertia
    {
        public float3x3 inertia;
        public float3x3 invInertia;
    }

    [BurstCompile]
    public struct ShardBody
    {
        public int slot;
    }
}
