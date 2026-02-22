using Shard.Runtime.Solvers;
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
            velocities.Add(type == BodyType.Static ? default : initialVel);
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

                if (newType == BodyType.Static)
                {
                    velocities[d] = default;              // static: kill motion
                }
                forceAccumulators[d].ClearAccumulators();  // both: clear forces
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

        public bool SetKinematicPose(ShardBodyHandle h, in Pose newPose, float dt)
        {
            if (!TryGetDense(h, out int d)) return false;
            if (bodyTypes[d] != BodyType.Kinematic) return false;

            Pose old = poses[d];

            float3 linVel = (newPose.position - old.position) / math.max(dt, 1e-6f);
            // angular vel estimate is optional; you can keep angularVelocity as user-authored for now

            poses[d] = newPose;

            var v = velocities[d];
            v.linearVelocity = linVel;
            velocities[d] = v;

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

            var type = bodyTypes[d];

            if (type == BodyType.Static)
            {
                velocities[d] = default;   // enforce invariant
                return true;               // or return false if you prefer strict behavior
            }

            velocities[d] = vel;          // Dynamic + Kinematic allowed
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool SetLinearVelocity(ShardBodyHandle h, float3 v)
        {
            if (!TryGetDense(h, out int d))
                return false;

            var type = bodyTypes[d];
            if (type == BodyType.Static)
            {
                velocities[d] = default;
                return true;
            }

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

            var type = bodyTypes[d];
            if (type == BodyType.Static)
            {
                velocities[d] = default;
                return true;
            }

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

            if (bodyTypes[d] != BodyType.Dynamic)
                return true; // ignore for Static + Kinematic

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

            if (bodyTypes[d] != BodyType.Dynamic)
                return true; // ignore for Static + Kinematic

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
        public void Simulate(float dt, int substeps=4, int collisionIterations = 6)
        {
            EnforceTypeInvariants();

            float h = dt / substeps;

            for (int s = 0; s < substeps; s++)
            {
                Integrate(h);
                SolveCollisions(h, collisionIterations);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void EnforceTypeInvariants()
        {
            for (int i = 0; i < poses.Length; i++)
            {
                BodyType t = bodyTypes[i];

                if (t == BodyType.Static)
                {
                    // Static: no motion, no forces.
                    velocities[i] = default;
                    forceAccumulators[i].ClearAccumulators();
                }
                else if (t == BodyType.Kinematic)
                {
                    // Kinematic: keep velocity (user-driven), but never accumulate forces.
                    forceAccumulators[i].ClearAccumulators();
                }
                // Dynamic: do nothing
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Integrate(float dt)
        {
            for (int i = 0; i < poses.Length; i++)
            {
                BodyType type = bodyTypes[i];

                if (type == BodyType.Static)
                {
                    velocities[i] = default;              // enforce invariant
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
        private void SolveCollisions(float dt, int iterations = 6)
        {
            for (int iter = 0; iter < iterations; iter++)
            {
                SolveCollisionsSinglePass(dt);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool GetContactManifold(int a, int b, Pose pa, Pose pb, out ContactPointManifold cps, out float restitution, out float muS, out float muD, out float muR)
        {
            cps = default;
            restitution = default;
            muS = default;
            muD = default;
            muR = default;
            // ---------- Box - Box ----------
            if (TryGetBodyBox(a, out float3 aHalf, out ShardColliderMaterial matA_box) &&
                TryGetBodyBox(b, out float3 bHalf, out ShardColliderMaterial matB_box))
            {
                CombineMaterials(matA_box, matB_box, out restitution, out muS, out muD, out muR);

                var boxA = new BoxBoxSolver.Box(pa.position, pa.rotation, aHalf);
                var boxB = new BoxBoxSolver.Box(pb.position, pb.rotation, bHalf);

                if (!BoxBoxSolver.Solve(boxA, boxB, out cps))
                    return false;

                return true;
            }

            // ---------- Box - Cylinder (A = Box) ----------
            else if (TryGetBodyBox(a, out float3 boxHalf, out ShardColliderMaterial matBox) &&
                     TryGetBodyCylinder(b, out float cylHH, out float cylR, out ShardColliderMaterial matCyl))
            {
                CombineMaterials(matBox, matCyl, out restitution, out muS, out muD, out muR);

                var box = new CylinderBoxSolver.Box(pa.position, pa.rotation, boxHalf);
                var cyl = new CylinderBoxSolver.Cylinder(pb.position, pb.rotation, cylHH, cylR);

                if (!CylinderBoxSolver.Solve(box, cyl, out cps))
                {
                    return false;
                }

                // solver already returns box -> cyl (A -> B)
                return true;
            }

            // ---------- Cylinder - Box (A = Cylinder) ----------
            else if (TryGetBodyCylinder(a, out cylHH, out cylR, out matCyl) &&
                     TryGetBodyBox(b, out boxHalf, out matBox))
            {
                CombineMaterials(matCyl, matBox, out restitution, out muS, out muD, out muR);

                var box = new CylinderBoxSolver.Box(pb.position, pb.rotation, boxHalf);
                var cyl = new CylinderBoxSolver.Cylinder(pa.position, pa.rotation, cylHH, cylR);

                if (!CylinderBoxSolver.Solve(box, cyl, out cps))
                {
                    return false;
                }

                // flip manifold (solver gives box->cyl, we need A->B)
                cps.globalPenAxis = -cps.globalPenAxis;

                for (int i = 0; i < cps.numContactPoints; i++)
                    cps[i] = new ContactPoint { point = cps[i].point, normal = -cps[i].normal, depth = cps[i].depth };

                return true;
            }

            // ---------- Cylinder - Cylinder ----------
            else if (TryGetBodyCylinder(a, out cylHH, out cylR, out matCyl) &&
                     TryGetBodyCylinder(b, out var cylHH2, out var cylR2, out var matCyl2))
            {
                CombineMaterials(matCyl, matCyl2, out restitution, out muS, out muD, out muR);

                var cyl1 = new CylinderCylinderSolver.Cylinder(pa.position, pa.rotation, cylHH, cylR);
                var cyl2 = new CylinderCylinderSolver.Cylinder(pb.position, pb.rotation, cylHH2, cylR2);

                if (!CylinderCylinderSolver.Solve(cyl1, cyl2, out cps))
                {
                    return false;
                }

                // flip manifold (solver gives box->cyl, we need A->B)
                //cps.globalPenAxis = -cps.globalPenAxis;

                //for (int i = 0; i < cps.numContactPoints; i++)
                //    cps[i] = new ContactPoint { point = cps[i].point, normal = -cps[i].normal, depth = cps[i].depth };

                return true;
            }
            else
                return false;
            

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SolveCollisionsSinglePass(float dt)
        {
            const float kImpulseSlop = 1e-6f;
            const float kDepthSlop = 1e-5f;
            const float kStaticVelEps = 0.05f;  // m/s threshold to treat as “trying to stick”
            const float kRollEps = 1e-8f;

            for (int a = 0; a < poses.Length; a++)
            {
                for (int b = a + 1; b < poses.Length; b++)
                {
                    bool aDyn = bodyTypes[a] == BodyType.Dynamic;
                    bool bDyn = bodyTypes[b] == BodyType.Dynamic;
                    if (!aDyn && !bDyn)
                        continue;

                    Pose pa = poses[a];
                    Pose pb = poses[b];

                    bool gotContact = GetContactManifold(a, b, pa, pb, out var cps, out var restitution, out var muS, out var muD, out var muR);

                    if (!gotContact)
                        continue;

                    float3 globalN = cps.globalPenAxis; // A -> B
                    float depth = cps.globalPenDepth;
                    if (depth <= kDepthSlop)
                        continue;

                    // ---- (1) Global penetration correction (positional only) ----
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

                    // refresh after correction
                    pa = poses[a];
                    pb = poses[b];

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
                        float3x3 RA = new float3x3(pa.rotation);
                        invIworldA = math.mul(RA, math.mul(inertias[a].invInertia, math.transpose(RA)));
                    }
                    if (bDyn)
                    {
                        float3x3 RB = new float3x3(pb.rotation);
                        invIworldB = math.mul(RB, math.mul(inertias[b].invInertia, math.transpose(RB)));
                    }

                    // Solve per-contact impulses (normal + friction) + rolling “drag”
                    for (int ci = 0; ci < count; ci++)
                    {
                        var cp = cps[ci];

                        float3 p = cp.point;

                        // Use the contact’s own normal (better than globalPenAxis for impulses)
                        float3 n = cp.normal;
                        float nLenSq = math.lengthsq(n);
                        if (nLenSq < 1e-12f)
                            continue;
                        n *= math.rsqrt(nLenSq);

                        float3 rA = p - pa.position;
                        float3 rB = p - pb.position;

                        float3 vPointA = va.linearVelocity + math.cross(va.angularVelocity, rA);
                        float3 vPointB = vb.linearVelocity + math.cross(vb.angularVelocity, rB);
                        float3 vRel = vPointB - vPointA;

                        float vn = math.dot(vRel, n);

                        // ---------------- Bias (Baumgarte stabilization) ----------------
                        const float beta = 0.2f;         // 0.1–0.3 is typical
                        const float allowedPen = 1e-4f;

                        float pen = math.max(0f, depth - allowedPen);
                        float biasVel = (pen > 0f)
                            ? (beta * pen / math.max(dt, 1e-6f))
                            : 0f;
                        // ---------------------------------------------------------------


                        // --- Normal impulse ---
                        float3 rAxN = math.cross(rA, n);
                        float3 rBxN = math.cross(rB, n);

                        float3 angA_n = aDyn ? math.cross(math.mul(invIworldA, rAxN), rA) : float3.zero;
                        float3 angB_n = bDyn ? math.cross(math.mul(invIworldB, rBxN), rB) : float3.zero;

                        float denomN = invMassA + invMassB + math.dot(n, angA_n + angB_n);
                        if (denomN <= 1e-12f)
                            continue;


                        // Restitution only for real impacts
                        float e = restitution;
                        if (-vn < 1.0f) e = 0f;


                        // Solve impulse with bias
                        float jn = (-(1f + e) * vn + biasVel) / denomN;


                        // Clamp (no pulling forces)
                        if (jn < 0f) jn = 0f;


                        float3 impulseN = n * jn;

                        if (aDyn)
                        {
                            va.linearVelocity -= impulseN * invMassA;
                            va.angularVelocity -= math.mul(invIworldA, rAxN * jn);
                        }
                        if (bDyn)
                        {
                            vb.linearVelocity += impulseN * invMassB;
                            vb.angularVelocity += math.mul(invIworldB, rBxN * jn);
                        }

                        // --- Tangential (static/dynamic) friction impulse ---
                        // recompute relative velocity after normal impulse (more stable)
                        vPointA = va.linearVelocity + math.cross(va.angularVelocity, rA);
                        vPointB = vb.linearVelocity + math.cross(vb.angularVelocity, rB);
                        vRel = vPointB - vPointA;

                        float3 vt = vRel - n * math.dot(vRel, n);
                        float vtLen = math.length(vt);

                        if (vtLen > 1e-8f)
                        {
                            float3 t = vt / vtLen; // tangent dir

                            float3 rAxT = math.cross(rA, t);
                            float3 rBxT = math.cross(rB, t);

                            float3 angA_t = aDyn ? math.cross(math.mul(invIworldA, rAxT), rA) : float3.zero;
                            float3 angB_t = bDyn ? math.cross(math.mul(invIworldB, rBxT), rB) : float3.zero;

                            float denomT = invMassA + invMassB + math.dot(t, angA_t + angB_t);
                            if (denomT > 1e-12f)
                            {
                                float jt = -math.dot(vRel, t) / denomT;

                                // Static vs dynamic: if we're “almost resting”, allow bigger (static) cap,
                                // otherwise use dynamic cap.
                                float maxStatic = muS * jn;
                                float maxDynamic = muD * jn;

                                float cap = (vtLen < kStaticVelEps) ? maxStatic : maxDynamic;

                                jt = math.clamp(jt, -cap, cap);

                                float3 impulseT = t * jt;

                                if (aDyn)
                                {
                                    va.linearVelocity -= impulseT * invMassA;
                                    va.angularVelocity -= math.mul(invIworldA, rAxT * jt);
                                }
                                if (bDyn)
                                {
                                    vb.linearVelocity += impulseT * invMassB;
                                    vb.angularVelocity += math.mul(invIworldB, rBxT * jt);
                                }
                            }
                        }

                        // --- Rolling friction (cheap, contact-based angular damping) ---
                        // Treat muR as “angular drag per unit normal impulse”.
                        // This is not a perfect model, but feels right and is stable.
                        if (muR > 0f && jn > 0f)
                        {
                            float k = muR * jn; // scale with contact strength
                            if (aDyn)
                            {
                                float wLen = math.length(va.angularVelocity);
                                if (wLen > kRollEps)
                                {
                                    float3 wDir = va.angularVelocity / wLen;
                                    float wNew = math.max(0f, wLen - k);
                                    va.angularVelocity = wDir * wNew;
                                }
                            }
                            if (bDyn)
                            {
                                float wLen = math.length(vb.angularVelocity);
                                if (wLen > kRollEps)
                                {
                                    float3 wDir = vb.angularVelocity / wLen;
                                    float wNew = math.max(0f, wLen - k);
                                    vb.angularVelocity = wDir * wNew;
                                }
                            }
                        }
                    }

                    velocities[a] = va;
                    velocities[b] = vb;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool TryGetBodyBox(int dense, out float3 halfExtents, out ShardColliderMaterial mat)
        {
            int n = colliderStore.GetHead(dense);
            while (n != -1)
            {
                if (!colliderStore.TryGetNode(n, out ShardCollider c, out int next))
                    break;

                if (c.type == ShardColliderType.Box)
                {
                    halfExtents = c.halfExtents;
                    mat = c.material; // <--- your new field
                    return true;
                }

                n = next;
            }

            halfExtents = default;
            mat = default;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool TryGetBodyCylinder(int dense, out float halfHeight, out float radius, out ShardColliderMaterial mat)
        {
            int n = colliderStore.GetHead(dense);
            while (n != -1)
            {
                if (!colliderStore.TryGetNode(n, out ShardCollider c, out int next))
                    break;

                if (c.type == ShardColliderType.Cylinder)
                {
                    halfHeight = c.height/2;
                    radius = c.radius;
                    mat = c.material;
                    return true;
                }

                n = next;
            }

            halfHeight = 0f;
            radius = 0f;
            mat = default;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void CombineMaterials(
            in ShardColliderMaterial a,
            in ShardColliderMaterial b,
            out float restitution,
            out float muS,
            out float muD,
            out float muR)
        {
            restitution = math.max(a.bounciness, b.bounciness);

            // geometric mean (nice “middle ground”)
            muS = math.sqrt(math.max(0f, a.frictionStatic) * math.max(0f, b.frictionStatic));
            muD = math.sqrt(math.max(0f, a.frictionDynamic) * math.max(0f, b.frictionDynamic));
            muR = math.sqrt(math.max(0f, a.frictionRolling) * math.max(0f, b.frictionRolling));

            // enforce muD <= muS (helps stability)
            muD = math.min(muD, muS);
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
