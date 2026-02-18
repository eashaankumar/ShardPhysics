using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Shard.Runtime
{
    public class ShardPhysicsSystem : IDisposable
    {

        NativeList<ForceAccumulator> forceAccumulators;
        NativeList<Velocity> velocities;
        NativeList<Mass> masses;
        NativeList<Inertia> inertias;
        NativeList<Pose> poses;

        DenseSlotMap slotMap;

        public float3 gravity;

        public ShardPhysicsSystem()
        {
            forceAccumulators = new NativeList<ForceAccumulator>(Allocator.Persistent);
            velocities = new NativeList<Velocity>(Allocator.Persistent);
            masses = new NativeList<Mass>(Allocator.Persistent);
            inertias = new NativeList<Inertia>(Allocator.Persistent);
            poses = new NativeList<Pose> (Allocator.Persistent);
            slotMap = new DenseSlotMap(initialCapacity: 16, allocator: Allocator.Persistent);
        }

        public void Dispose()
        {
            if (forceAccumulators.IsCreated) forceAccumulators.Dispose();
            if (velocities.IsCreated) velocities.Dispose();
            if (masses.IsCreated) masses.Dispose();
            if (inertias.IsCreated) inertias.Dispose();
            if (poses.IsCreated) poses.Dispose();
            slotMap.Dispose();
        }

        public ShardBodyHandle CreateBody(Pose initialPose, float mass, float3x3 inertia, Velocity initialVel = default)
        {
            slotMap.Allocate(out int slot, out int dense);

            // Append dense data at index = dense

            forceAccumulators.Add(new ForceAccumulator());
            velocities.Add(initialVel);

            float m = math.max(mass, 1e-8f);
            masses.Add(new Mass { mass = m, invMass = 1.0f / m });

            // You can replace this with SafeInverse later
            inertias.Add(new Inertia { inertia = inertia, invInertia = math.inverse(inertia) });

            poses.Add(initialPose);

            return new ShardBodyHandle(slot);
        }

        public void DestroyBody(ShardBodyHandle h)
        {
            if (!slotMap.TryResolveDense(h.handle, out int dense))
                return;

            int last = poses.Length - 1; // or velocities.Length; must match all arrays

            if (dense != last)
            {
                // Swap-remove in all dense arrays
                poses[dense] = poses[last];
                forceAccumulators[dense] = forceAccumulators[last];
                velocities[dense] = velocities[last];
                masses[dense] = masses[last];
                inertias[dense] = inertias[last];

                // Update mapping for the moved element (last -> dense)
                slotMap.OnDenseElementMoved(fromLastDense: last, toDense: dense);
            }

            // Pop last from all dense arrays
            poses.RemoveAt(last);
            forceAccumulators.RemoveAt(last);
            velocities.RemoveAt(last);
            masses.RemoveAt(last);
            inertias.RemoveAt(last);

            // Free slot + pop dense mapping
            slotMap.Free(h.handle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Simulate(float dt)
        {
            for (int i = 0; i < poses.Length; i++)
            {
                float invM = masses[i].invMass;

                // Clear forces even for static bodies
                // (or you can skip if you don't want them to accumulate)
                if (invM == 0f)
                {
                    forceAccumulators[i].ClearAccumulators();
                    continue;
                }

                // Linear
                float3 a = forceAccumulators[i].forceAccumulator * invM + gravity;
                Velocity v = velocities[i];
                v.linearVelocity += a * dt;

                Pose p = poses[i];
                p.position += v.linearVelocity * dt;

                // Angular (local inertia -> world inertia)
                float3 torque = forceAccumulators[i].torqueAccumulator;
                float3x3 invIlocal = inertias[i].invInertia;
                float3x3 R = new float3x3(p.rotation);
                float3x3 invIworld = math.mul(R, math.mul(invIlocal, math.transpose(R)));

                v.angularVelocity += math.mul(invIworld, torque) * dt;

                // Integrate rotation from angular velocity (simple)
                p.rotation = IntegrateRotation(p.rotation, v.angularVelocity, dt);

                velocities[i] = v;
                poses[i] = p;

                forceAccumulators[i].ClearAccumulators();
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static quaternion IntegrateRotation(quaternion q, float3 w, float dt)
        {
            // dq/dt = 0.5 * omega(q) * q
            quaternion omega = new quaternion(w.x, w.y, w.z, 0f);
            quaternion dq = math.mul(omega, q);
            q.value += 0.5f * dq.value * dt;

            // normalize to prevent drift
            return math.normalize(q);
        }
    }
}
