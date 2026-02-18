using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Shard.Runtime
{
    public class ShardPhysicsSystem : IDisposable
    {

        NativeList<ForceAccumulator> forceAccumulators;
        NativeList<Velocity> velocities;
        NativeList<Mass> masses;
        NativeList<Inertia> inertias;
        NativeList<ShardBody> bodies;

        DenseSlotMap slotMap;

        public float3 gravity;

        public ShardPhysicsSystem()
        {
            forceAccumulators = new NativeList<ForceAccumulator>(Allocator.Persistent);
            velocities = new NativeList<Velocity>(Allocator.Persistent);
            masses = new NativeList<Mass>(Allocator.Persistent);
            inertias = new NativeList<Inertia>(Allocator.Persistent);
            bodies = new NativeList<ShardBody>(Allocator.Persistent);
            slotMap = new DenseSlotMap(initialCapacity: 16, allocator: Allocator.Persistent);
        }

        public void Dispose()
        {
            if (forceAccumulators.IsCreated) forceAccumulators.Dispose();
            if (velocities.IsCreated) velocities.Dispose();
            if (masses.IsCreated) masses.Dispose();
            if (inertias.IsCreated) inertias.Dispose();
            if (bodies.IsCreated) bodies.Dispose();
            slotMap.Dispose();
        }

        public ShardBody CreateBody(float mass, float3x3 inertia, Velocity initialVel = default)
        {
            slotMap.Allocate(out int slot, out int dense);

            // Append dense data at index = dense
            bodies.Add(new ShardBody { slot = slot });

            forceAccumulators.Add(new ForceAccumulator());
            velocities.Add(initialVel);

            float m = math.max(mass, 1e-8f);
            masses.Add(new Mass { mass = m, invMass = 1.0f / m });

            // You can replace this with SafeInverse later
            inertias.Add(new Inertia { inertia = inertia, invInertia = math.inverse(inertia) });

            return new ShardBody { slot = slot };
        }

        public void DestroyBody(ShardBody h)
        {
            if (!slotMap.TryResolveDense(h.slot, out int dense))
                return;

            int last = bodies.Length - 1;

            if (dense != last)
            {
                // Swap-remove in all dense arrays
                bodies[dense] = bodies[last];
                forceAccumulators[dense] = forceAccumulators[last];
                velocities[dense] = velocities[last];
                masses[dense] = masses[last];
                inertias[dense] = inertias[last];

                // Tell slot map we moved last -> dense
                slotMap.OnDenseElementMoved(fromLastDense: last, toDense: dense);
            }

            // Pop last from all dense arrays
            bodies.RemoveAt(last);
            forceAccumulators.RemoveAt(last);
            velocities.RemoveAt(last);
            masses.RemoveAt(last);
            inertias.RemoveAt(last);

            // Free slot + pop dense mapping
            slotMap.Free(h.slot);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Simulate(float dt)
        { 
        }
    }
}
