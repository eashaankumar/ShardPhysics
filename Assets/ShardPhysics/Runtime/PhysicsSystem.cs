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

        #region ---------- Handle -> dense resolution ----------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool TryGetDense(ShardBodyHandle h, out int dense)
        {
            return slotMap.TryResolveDense(h.handle, out dense);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsAlive(ShardBodyHandle h)
        {
            return slotMap.IsAlive(h.handle);
        }
        #endregion

        #region ---------- Pose ----------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetPose(ShardBodyHandle h, out Pose pose)
        {
            if (!TryGetDense(h, out int d))
            {
                pose = default;
                return false;
            }

            pose = poses[d];
            return true;
        }

        /// <summary>Teleport/set pose directly.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool SetPose(ShardBodyHandle h, in Pose pose)
        {
            if (!TryGetDense(h, out int d))
                return false;

            poses[d] = pose;
            return true;
        }
        #endregion

        #region ---------- Velocity ----------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetVelocity(ShardBodyHandle h, out Velocity vel)
        {
            if (!TryGetDense(h, out int d))
            {
                vel = default;
                return false;
            }

            vel = velocities[d];
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool SetVelocity(ShardBodyHandle h, in Velocity vel)
        {
            if (!TryGetDense(h, out int d))
                return false;

            velocities[d] = vel;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool SetLinearVelocity(ShardBodyHandle h, float3 v)
        {
            if (!TryGetDense(h, out int d))
                return false;

            var vel = velocities[d];
            vel.linearVelocity = v;
            velocities[d] = vel;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool SetAngularVelocity(ShardBodyHandle h, float3 w)
        {
            if (!TryGetDense(h, out int d))
                return false;

            var vel = velocities[d];
            vel.angularVelocity = w;
            velocities[d] = vel;
            return true;
        }
        #endregion

        #region ---------- Mass / Inertia ----------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetMass(ShardBodyHandle h, out Mass mass)
        {
            if (!TryGetDense(h, out int d))
            {
                mass = default;
                return false;
            }

            mass = masses[d];
            return true;
        }

        /// <summary>
        /// Sets mass and derived invMass. (Does NOT auto-recompute inertia distribution yet.)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool SetMass(ShardBodyHandle h, float mass)
        {
            if (!TryGetDense(h, out int d))
                return false;

            float m = math.max(mass, 1e-8f);
            masses[d] = new Mass { mass = m, invMass = 1.0f / m };

            // TODO (later): if you store unit inertia, scale inertia/invInertia with mass here.
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetInertia(ShardBodyHandle h, out Inertia inertia)
        {
            if (!TryGetDense(h, out int d))
            {
                inertia = default;
                return false;
            }

            inertia = inertias[d];
            return true;
        }

        /// <summary>
        /// Sets inertia tensors directly. Caller must provide valid (invertible) inertia.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool SetInertia(ShardBodyHandle h, in float3x3 inertia)
        {
            if (!TryGetDense(h, out int d))
                return false;

            // TODO: replace with SafeInverse3x3 later to prevent NaNs.
            inertias[d] = new Inertia { inertia = inertia, invInertia = math.inverse(inertia) };
            return true;
        }
        #endregion

        #region ---------- Forces ----------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AddForce(ShardBodyHandle h, float3 force)
        {
            if (!TryGetDense(h, out int d))
                return false;

            var fa = forceAccumulators[d];
            fa.forceAccumulator += force;
            forceAccumulators[d] = fa;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AddTorque(ShardBodyHandle h, float3 torque)
        {
            if (!TryGetDense(h, out int d))
                return false;

            var fa = forceAccumulators[d];
            fa.torqueAccumulator += torque;
            forceAccumulators[d] = fa;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ClearForces(ShardBodyHandle h)
        {
            if (!TryGetDense(h, out int d))
                return false;

            var fa = forceAccumulators[d];
            fa.ClearAccumulators();
            forceAccumulators[d] = fa;
            return true;
        }
        #endregion

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
