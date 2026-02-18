using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Shard.Runtime
{
    public class ShardPhysicsWorld : IDisposable
    {

        NativeList<ForceAccumulator> forceAccumulators;
        NativeList<Velocity> velocities;
        NativeList<Mass> masses;
        NativeList<Inertia> inertias;
        NativeList<Pose> poses;
        NativeList<BodyType> bodyTypes;

        DenseSlotMap slotMap;
        ShardColliderStore colliderStore;

        public float3 gravity;

        public ShardPhysicsWorld()
        {
            forceAccumulators = new NativeList<ForceAccumulator>(Allocator.Persistent);
            velocities = new NativeList<Velocity>(Allocator.Persistent);
            masses = new NativeList<Mass>(Allocator.Persistent);
            inertias = new NativeList<Inertia>(Allocator.Persistent);
            poses = new NativeList<Pose> (Allocator.Persistent);
            bodyTypes = new NativeList<BodyType>(Allocator.Persistent);

            colliderStore = new ShardColliderStore(initialNodeCapacity: 32, initialBodyCapacity: 16, allocator: Allocator.Persistent);
            slotMap = new DenseSlotMap(initialCapacity: 16, allocator: Allocator.Persistent);
        }

        public void Dispose()
        {
            if (forceAccumulators.IsCreated) forceAccumulators.Dispose();
            if (velocities.IsCreated) velocities.Dispose();
            if (masses.IsCreated) masses.Dispose();
            if (inertias.IsCreated) inertias.Dispose();
            if (poses.IsCreated) poses.Dispose();
            if (bodyTypes.IsCreated) bodyTypes.Dispose();
            slotMap.Dispose();
            colliderStore.Dispose();
        }

        #region --------- Create Body ----------
        /// <summary>
        /// Create a rigid body from a set of colliders, computing inertia automatically.
        /// - 'mass' is the final total body mass (unless mass <= 0 => static body).
        /// - collider.density is used only for *relative weighting* between colliders.
        ///   The final inertia is scaled so total mass == 'mass'.
        /// </summary>
        public ShardBodyHandle CreateBody(Pose initialPose, BodyType type, float mass, NativeArray<ShardCollider> colliders, Velocity initialVel = default)
        {
            // Static body convention: mass <= 0 => infinite mass + inertia.
            if (mass <= 0f)
            {
                var hStatic = CreateBodyInternal(initialPose, type, 0f, float3x3.zero, initialVel);
                for (int i = 0; i < colliders.Length; i++)
                    AddCollider(hStatic, colliders[i]);
                return hStatic;
            }

            ShardCreateBody.ComputeMassPropertiesFromColliders(
                colliders,
                desiredTotalMass: mass,
                out float3x3 inertiaBody);

            var h = CreateBodyInternal(initialPose, type, mass, inertiaBody, initialVel);

            for (int i = 0; i < colliders.Length; i++)
                AddCollider(h, colliders[i]);

            return h;
        }

        /// <summary>
        /// Managed convenience overload (no allocations). Useful from non-Burst callsites.
        /// </summary>
        public ShardBodyHandle CreateBody(Pose initialPose, BodyType type, float mass, ReadOnlySpan<ShardCollider> colliders, Velocity initialVel = default)
        {
            if (mass <= 0f)
            {
                var hStatic = CreateBodyInternal(initialPose, type, 0f, float3x3.zero, initialVel);
                for (int i = 0; i < colliders.Length; i++)
                    AddCollider(hStatic, colliders[i]);
                return hStatic;
            }

            ShardCreateBody.ComputeMassPropertiesFromColliders(
                colliders,
                desiredTotalMass: mass,
                out float3x3 inertiaBody);

            var h = CreateBodyInternal(initialPose, type, mass, inertiaBody, initialVel);

            for (int i = 0; i < colliders.Length; i++)
                AddCollider(h, colliders[i]);

            return h;
        }

        // ---- Internals ----

        // Keep your original allocator/slotMap logic here, but factor it so the new overload can reuse it.
        private ShardBodyHandle CreateBodyInternal(Pose initialPose, BodyType type, float mass, float3x3 inertia, Velocity initialVel)
        {
            slotMap.Allocate(out int slot, out int dense);

            forceAccumulators.Add(new ForceAccumulator());
            velocities.Add(type == BodyType.Dynamic ? initialVel : default);
            poses.Add(initialPose);
            bodyTypes.Add(type);

            if (type == BodyType.Dynamic)
            {
                float m = math.max(mass, 1e-8f);
                masses.Add(new Mass { mass = m, invMass = 1f / m });
                inertias.Add(new Inertia { inertia = inertia, invInertia = math.inverse(inertia) }); // TODO SafeInverse later
            }
            else
            {
                masses.Add(new Mass { mass = 0f, invMass = 0f });
                inertias.Add(new Inertia { inertia = float3x3.zero, invInertia = float3x3.zero });

            }

            colliderStore.OnBodyAdded();
            return new ShardBodyHandle(slot);
        }

        public bool RecomputeMassProperties(ShardBodyHandle h, float desiredMassForDynamic)
        {
            if (!TryGetDense(h, out int d))
                return false;

            if (bodyTypes[d] != BodyType.Dynamic)
                return false;

            float m = math.max(desiredMassForDynamic, 1e-8f);
            float3x3 inertiaBody = ComputeInertiaFromBodyColliders(d, m);

            masses[d] = new Mass { mass = m, invMass = 1f / m };
            inertias[d] = new Inertia { inertia = inertiaBody, invInertia = math.inverse(inertiaBody) };

            velocities[d] = default;
            forceAccumulators[d].ClearAccumulators();

            return true;
        }
        #endregion

        public void DestroyBody(ShardBodyHandle h)
        {
            if (!slotMap.TryResolveDense(h.handle, out int dense))
                return;

            int last = poses.Length - 1;

            // Free colliders owned by the body being destroyed (at dense)
            colliderStore.OnBodyRemoving(dense);

            if (dense != last)
            {
                // Move last body data into dense
                poses[dense] = poses[last];
                forceAccumulators[dense] = forceAccumulators[last];
                velocities[dense] = velocities[last];
                masses[dense] = masses[last];
                inertias[dense] = inertias[last];
                bodyTypes[dense] = bodyTypes[last];

                // Move last body collider list into dense + fix owners
                colliderStore.OnBodySwapMoved(fromDense: last, toDense: dense);

                // Update mapping for the moved body (last -> dense)
                slotMap.OnDenseElementMoved(fromLastDense: last, toDense: dense);
            }

            // Pop dense arrays
            poses.RemoveAt(last);
            forceAccumulators.RemoveAt(last);
            velocities.RemoveAt(last);
            masses.RemoveAt(last);
            inertias.RemoveAt(last);
            bodyTypes.RemoveAt(last);

            // Pop collider list row (always corresponds to last)
            colliderStore.OnBodyPoppedBack();

            // Free slot + pop dense mapping
            slotMap.Free(h.handle);
        }

        #region ---------- Body Type ----------
        public bool TryGetBodyType(ShardBodyHandle h, out BodyType type)
        {
            if (!TryGetDense(h, out int d))
            {
                type = default;
                return false;
            }
            type = bodyTypes[d];
            return true;
        }

        /// <summary>
        /// Sets body type and auto-updates mass/inertia.
        /// - For Dynamic: recomputes inertia from the body's colliders and sets mass.
        /// - For Static/Kinematic: sets invMass=0 and invInertia=0.
        /// </summary>
        public bool SetBodyType(ShardBodyHandle h, BodyType newType, float desiredMassForDynamic = 1f)
        {
            if (!TryGetDense(h, out int d))
                return false;

            bodyTypes[d] = newType;

            if (newType == BodyType.Dynamic)
            {
                float m = math.max(desiredMassForDynamic, 1e-8f);

                // Gather colliders owned by this body and compute inertia
                float3x3 inertiaBody = ComputeInertiaFromBodyColliders(d, m);

                masses[d] = new Mass { mass = m, invMass = 1f / m };
                inertias[d] = new Inertia { inertia = inertiaBody, invInertia = math.inverse(inertiaBody) }; // TODO SafeInverse later
            }
            else
            {
                // Static or Kinematic: infinite mass/inertia
                masses[d] = new Mass { mass = 0f, invMass = 0f };
                inertias[d] = new Inertia { inertia = float3x3.zero, invInertia = float3x3.zero };

                velocities[d] = default;
                forceAccumulators[d].ClearAccumulators();
            }

            return true;
        }

        private float3x3 ComputeInertiaFromBodyColliders(int dense, float desiredTotalMass)
        {
            // Pull colliders from linked list into a temp NativeList (Allocator.Temp = ok for infrequent edits)
            var tmp = new NativeList<ShardCollider>(math.max(1, colliderStore.GetColliderCount(dense)), Allocator.Temp);

            int n = colliderStore.GetHead(dense);
            while (n != -1)
            {
                if (!colliderStore.TryGetNode(n, out ShardCollider c, out int next))
                    break;

                tmp.Add(c);
                n = next;
            }

            // Reuse your existing computation (this function should scale to desiredTotalMass)
            ShardCreateBody.ComputeMassPropertiesFromColliders(tmp.AsArray(), desiredTotalMass, out float3x3 inertiaBody);

            tmp.Dispose();
            colliderStore.ClearDirty(dense);

            return inertiaBody;
        }
        #endregion

        #region ---------- Colliders ----------
        public ShardColliderHandle AddCollider(ShardBodyHandle b, in ShardCollider collider)
        {
            if (!TryGetDense(b, out int d))
                return default;

            return colliderStore.AddCollider(d, collider);
        }

        public bool RemoveCollider(ShardBodyHandle b, ShardColliderHandle c)
        {
            if (!TryGetDense(b, out int d))
                return false;

            return colliderStore.RemoveCollider(d, c);
        }

        public void ClearColliders(ShardBodyHandle b)
        {
            if (!TryGetDense(b, out int d))
                return;

            colliderStore.ClearColliders(d);
        }

        public bool TryGetCollider(ShardColliderHandle c, out ShardCollider collider)
        {
            return colliderStore.TryGetCollider(c, out collider, out _);
        }

        public bool SetCollider(ShardColliderHandle c, in ShardCollider collider)
        {
            return colliderStore.SetCollider(c, collider);
        }

        public bool SetColliderLocalPose(ShardColliderHandle c, in Pose localPose)
        {
            return colliderStore.SetColliderLocalPose(c, localPose);
        }

        public int GetColliderCount(ShardBodyHandle b)
        {
            if (!TryGetDense(b, out int d))
                return 0;
            return colliderStore.GetColliderCount(d);
        }
        #endregion

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

        #region ---------- Simulation/Integration/Solve ----------

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Simulate(float dt)
        {
            Integrate(dt);
            SolveCollisions(dt);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Integrate(float dt)
        {
            for (int i = 0; i < poses.Length; i++)
            {
                BodyType type = bodyTypes[i];

                if (type == BodyType.Static)
                {
                    forceAccumulators[i].ClearAccumulators();
                    continue;
                }

                Pose p = poses[i];
                Velocity v = velocities[i];

                if (type == BodyType.Dynamic)
                {
                    float invM = masses[i].invMass;

                    // Linear
                    float3 a = forceAccumulators[i].forceAccumulator * invM + gravity;
                    v.linearVelocity += a * dt;
                    p.position += v.linearVelocity * dt;

                    // Angular
                    float3 torque = forceAccumulators[i].torqueAccumulator;

                    float3x3 invIlocal = inertias[i].invInertia;
                    float3x3 R = new float3x3(p.rotation);
                    float3x3 invIworld = math.mul(R, math.mul(invIlocal, math.transpose(R)));

                    v.angularVelocity += math.mul(invIworld, torque) * dt;
                    p.rotation = IntegrateRotation(p.rotation, v.angularVelocity, dt);
                }
                else // Kinematic
                {
                    // velocity-driven kinematic (your current behavior)
                    p.position += v.linearVelocity * dt;
                    p.rotation = IntegrateRotation(p.rotation, v.angularVelocity, dt);
                }

                poses[i] = p;
                velocities[i] = v;

                forceAccumulators[i].ClearAccumulators();
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SolveCollisions(float dt)
        {
            const float kRestitution = 0f;
            const float kImpulseSlop = 1e-6f;
            const float kDepthSlop = 1e-5f;

            for (int a = 0; a < poses.Length; a++)
            {
                for (int b = a + 1; b < poses.Length; b++)
                {
                    bool aDyn = bodyTypes[a] == BodyType.Dynamic;
                    bool bDyn = bodyTypes[b] == BodyType.Dynamic;
                    if (!aDyn && !bDyn)
                        continue;

                    if (!TryGetBodyBoxHalfExtents(a, out float3 aHalf) ||
                        !TryGetBodyBoxHalfExtents(b, out float3 bHalf))
                        continue;

                    Pose pa = poses[a];
                    Pose pb = poses[b];

                    var boxA = new BoxBoxSolver.Box(pa.position, pa.rotation, aHalf);
                    var boxB = new BoxBoxSolver.Box(pb.position, pb.rotation, bHalf);

                    if (!BoxBoxSolver.Solve(boxA, boxB, out BoxBoxSolver.BoxBoxContactPoints cps))
                        continue;

                    float3 globalN = cps.globalPenAxis;   // A -> B
                    float depth = cps.globalPenDepth;
                    if (depth <= kDepthSlop)
                        continue;

                    // ---- (1) Global penetration correction ONCE ----
                    if (aDyn && bDyn)
                    {
                        float3 corr = globalN * (depth * 0.5f);
                        pa.position -= corr;
                        pb.position += corr;
                        poses[a] = pa;
                        poses[b] = pb;
                    }
                    else if (aDyn && !bDyn)
                    {
                        pa.position -= globalN * depth;
                        poses[a] = pa;
                    }
                    else if (!aDyn && bDyn)
                    {
                        pb.position += globalN * depth;
                        poses[b] = pb;
                    }

                    // Refresh poses after correction
                    pa = poses[a];
                    pb = poses[b];

                    // ---- (2) Per-contact impulses ----
                    int count = cps.numContactPoints;
                    if (count <= 0)
                        continue;

                    Velocity va = velocities[a];
                    Velocity vb = velocities[b];

                    float invMassA = aDyn ? masses[a].invMass : 0f;
                    float invMassB = bDyn ? masses[b].invMass : 0f;

                    float3x3 invIworldA = float3x3.zero;
                    float3x3 invIworldB = float3x3.zero;

                    if (aDyn)
                    {
                        float3x3 R = new float3x3(pa.rotation);
                        invIworldA = math.mul(R, math.mul(inertias[a].invInertia, math.transpose(R)));
                    }
                    if (bDyn)
                    {
                        float3x3 R = new float3x3(pb.rotation);
                        invIworldB = math.mul(R, math.mul(inertias[b].invInertia, math.transpose(R)));
                    }

                    for (int ci = 0; ci < count; ci++)
                    {
                        var cp = cps[ci];
                        float3 p = cp.point;
                        float3 n = cp.normal;

                        float3 rA = p - pa.position;
                        float3 rB = p - pb.position;

                        float3 vA = va.linearVelocity + math.cross(va.angularVelocity, rA);
                        float3 vB = vb.linearVelocity + math.cross(vb.angularVelocity, rB);
                        float3 vRel = vB - vA;

                        float vn = math.dot(vRel, n);

                        // only if closing
                        if (vn > -kImpulseSlop)
                            continue;

                        float3 rAxN = math.cross(rA, n);
                        float3 rBxN = math.cross(rB, n);

                        float3 angA = aDyn ? math.cross(math.mul(invIworldA, rAxN), rA) : float3.zero;
                        float3 angB = bDyn ? math.cross(math.mul(invIworldB, rBxN), rB) : float3.zero;

                        float denom = invMassA + invMassB + math.dot(n, angA + angB);
                        if (denom <= 1e-12f)
                            continue;

                        float j = -(1f + kRestitution) * vn / denom;
                        float3 impulse = n * j;

                        if (aDyn)
                        {
                            va.linearVelocity -= impulse * invMassA;
                            va.angularVelocity -= math.mul(invIworldA, rAxN * j);
                        }
                        if (bDyn)
                        {
                            vb.linearVelocity += impulse * invMassB;
                            vb.angularVelocity += math.mul(invIworldB, rBxN * j);
                        }
                    }

                    velocities[a] = va;
                    velocities[b] = vb;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool TryGetBodyBoxHalfExtents(int dense, out float3 halfExtents)
        {
            int n = colliderStore.GetHead(dense);
            while (n != -1)
            {
                if (!colliderStore.TryGetNode(n, out ShardCollider c, out int next))
                    break;

                if (c.type == ShardColliderType.Box)
                {
                    halfExtents = c.halfExtents;
                    return true;
                }

                n = next;
            }

            halfExtents = default;
            return false;
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
        #endregion
    }
}
