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
        ShardTriangleMeshStore triangleMeshStore;
        Broadphase broadphase;
        NativeList<CachedContact> cachedContacts;

        private struct CachedContact
        {
            public int a;
            public int b;
            public ContactPointManifold cps;
            public float restitution;
            public float muS;
            public float muD;
            public float muR;
        }

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
            triangleMeshStore = new ShardTriangleMeshStore(
                initialVertexCapacity: 1024,
                initialIndexCapacity: 2048,
                initialMeshCapacity: 32,
                allocator: Allocator.Persistent);
            
            slotMap = new DenseSlotMap(initialCapacity: 16, allocator: Allocator.Persistent);
            broadphase = new Broadphase(initialBodyCapacity: 16, allocator: Allocator.Persistent);
            cachedContacts = new NativeList<CachedContact>(initialCapacity: 64, allocator: Allocator.Persistent);
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
            triangleMeshStore.Dispose();
            broadphase.Dispose();
            if (cachedContacts.IsCreated) cachedContacts.Dispose();
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
        
        #region ---------- Triangle Meshes ----------

        public ShardTriangleMeshHandle AddTriangleMesh(
            NativeArray<float3> vertices,
            NativeArray<int> indices)
        {
            return triangleMeshStore.AddMesh(vertices, indices);
        }

        public ShardTriangleMeshHandle AddTriangleMesh(
            ReadOnlySpan<float3> vertices,
            ReadOnlySpan<int> indices)
        {
            return triangleMeshStore.AddMesh(vertices, indices);
        }

        public bool RemoveTriangleMesh(ShardTriangleMeshHandle mesh)
        {
            return triangleMeshStore.RemoveMesh(mesh);
        }

        public bool TryGetTriangleMeshInfo(
            ShardTriangleMeshHandle mesh,
            out ShardTriangleMeshInfo info)
        {
            return triangleMeshStore.TryGetMeshInfo(mesh, out info);
        }

        public bool TryGetTriangleMeshInfo(
            int meshIndex,
            out ShardTriangleMeshInfo info)
        {
            return triangleMeshStore.TryGetMeshInfo(meshIndex, out info);
        }

        public bool TryGetTriangle(
            ShardTriangleMeshHandle mesh,
            int triangleIndex,
            out ShardTriangle triangle)
        {
            return triangleMeshStore.TryGetTriangle(mesh, triangleIndex, out triangle);
        }

        public bool TryGetTriangle(
            int meshIndex,
            int triangleIndex,
            out ShardTriangle triangle)
        {
            return triangleMeshStore.TryGetTriangle(meshIndex, triangleIndex, out triangle);
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
            // Expensive collision detection now happens once per substep.
            // The solver iterations only reuse the cached contact manifolds.
            BuildCachedContactsForSubstep();

            for (int iter = 0; iter < iterations; iter++)
            {
                SolveCachedContacts(dt, applyBias: iter == 0);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool GetContactManifold(
            int a,
            int b,
            Pose pa,
            Pose pb,
            out ContactPointManifold cps,
            out float restitution,
            out float muS,
            out float muD,
            out float muR)
        {
            cps = default;
            restitution = default;
            muS = default;
            muD = default;
            muR = default;

            // ---------- Sphere - Sphere ----------
            if (TryGetBodySphereShape(a, pa, out SimpleShapeSolvers.Sphere sphereA, out ShardColliderMaterial matSphereA) &&
                TryGetBodySphereShape(b, pb, out SimpleShapeSolvers.Sphere sphereB, out ShardColliderMaterial matSphereB))
            {
                CombineMaterials(matSphereA, matSphereB, out restitution, out muS, out muD, out muR);
                return SimpleShapeSolvers.SolveSphereSphere(sphereA, sphereB, out cps);
            }

            // ---------- Sphere - Box ----------
            if (TryGetBodySphereShape(a, pa, out sphereA, out matSphereA) &&
                TryGetBodyBoxShape(b, pb, out SimpleShapeSolvers.Box boxB, out ShardColliderMaterial matBoxB))
            {
                CombineMaterials(matSphereA, matBoxB, out restitution, out muS, out muD, out muR);
                return SimpleShapeSolvers.SolveSphereBox(sphereA, boxB, out cps);
            }

            // ---------- Box - Sphere ----------
            if (TryGetBodyBoxShape(a, pa, out SimpleShapeSolvers.Box boxA, out ShardColliderMaterial matBoxA) &&
                TryGetBodySphereShape(b, pb, out sphereB, out matSphereB))
            {
                CombineMaterials(matBoxA, matSphereB, out restitution, out muS, out muD, out muR);
                return SimpleShapeSolvers.SolveBoxSphere(boxA, sphereB, out cps);
            }

            // ---------- Sphere - Capsule ----------
            if (TryGetBodySphereShape(a, pa, out sphereA, out matSphereA) &&
                TryGetBodyCapsuleShape(b, pb, out SimpleShapeSolvers.Capsule capsuleB, out ShardColliderMaterial matCapsuleB))
            {
                CombineMaterials(matSphereA, matCapsuleB, out restitution, out muS, out muD, out muR);
                return SimpleShapeSolvers.SolveSphereCapsule(sphereA, capsuleB, out cps);
            }

            // ---------- Capsule - Sphere ----------
            if (TryGetBodyCapsuleShape(a, pa, out SimpleShapeSolvers.Capsule capsuleA, out ShardColliderMaterial matCapsuleA) &&
                TryGetBodySphereShape(b, pb, out sphereB, out matSphereB))
            {
                CombineMaterials(matCapsuleA, matSphereB, out restitution, out muS, out muD, out muR);
                return SimpleShapeSolvers.SolveCapsuleSphere(capsuleA, sphereB, out cps);
            }

            // ---------- Capsule - Capsule ----------
            if (TryGetBodyCapsuleShape(a, pa, out capsuleA, out matCapsuleA) &&
                TryGetBodyCapsuleShape(b, pb, out capsuleB, out matCapsuleB))
            {
                CombineMaterials(matCapsuleA, matCapsuleB, out restitution, out muS, out muD, out muR);
                return SimpleShapeSolvers.SolveCapsuleCapsule(capsuleA, capsuleB, out cps);
            }

            // ---------- Capsule - Box ----------
            if (TryGetBodyCapsuleShape(a, pa, out capsuleA, out matCapsuleA) &&
                TryGetBodyBoxShape(b, pb, out boxB, out matBoxB))
            {
                CombineMaterials(matCapsuleA, matBoxB, out restitution, out muS, out muD, out muR);
                return SimpleShapeSolvers.SolveCapsuleBox(capsuleA, boxB, out cps);
            }

            // ---------- Box - Capsule ----------
            if (TryGetBodyBoxShape(a, pa, out boxA, out matBoxA) &&
                TryGetBodyCapsuleShape(b, pb, out capsuleB, out matCapsuleB))
            {
                CombineMaterials(matBoxA, matCapsuleB, out restitution, out muS, out muD, out muR);
                return SimpleShapeSolvers.SolveBoxCapsule(boxA, capsuleB, out cps);
            }

            // ---------- Sphere - Triangle ----------
            if (TryGetBodySphereShape(a, pa, out sphereA, out matSphereA) &&
                TryGetBodyTriangle(b, pb, out SimpleShapeSolvers.Triangle triangleB, out ShardColliderMaterial matTriangleB))
            {
                CombineMaterials(matSphereA, matTriangleB, out restitution, out muS, out muD, out muR);
                return SimpleShapeSolvers.SolveSphereTriangle(sphereA, triangleB, out cps);
            }

            // ---------- Triangle - Sphere ----------
            if (TryGetBodyTriangle(a, pa, out SimpleShapeSolvers.Triangle triangleA, out ShardColliderMaterial matTriangleA) &&
                TryGetBodySphereShape(b, pb, out sphereB, out matSphereB))
            {
                CombineMaterials(matTriangleA, matSphereB, out restitution, out muS, out muD, out muR);

                if (!SimpleShapeSolvers.SolveSphereTriangle(sphereB, triangleA, out cps))
                    return false;

                SimpleShapeSolvers.FlipManifold(ref cps);
                return true;
            }

            // ---------- Capsule - Triangle ----------
            if (TryGetBodyCapsuleShape(a, pa, out capsuleA, out matCapsuleA) &&
                TryGetBodyTriangle(b, pb, out triangleB, out matTriangleB))
            {
                CombineMaterials(matCapsuleA, matTriangleB, out restitution, out muS, out muD, out muR);
                return SimpleShapeSolvers.SolveCapsuleTriangle(capsuleA, triangleB, out cps);
            }

            // ---------- Triangle - Capsule ----------
            if (TryGetBodyTriangle(a, pa, out triangleA, out matTriangleA) &&
                TryGetBodyCapsuleShape(b, pb, out capsuleB, out matCapsuleB))
            {
                CombineMaterials(matTriangleA, matCapsuleB, out restitution, out muS, out muD, out muR);

                if (!SimpleShapeSolvers.SolveCapsuleTriangle(capsuleB, triangleA, out cps))
                    return false;

                SimpleShapeSolvers.FlipManifold(ref cps);
                return true;
            }

            // ---------- Box - Triangle ----------
            if (TryGetBodyBoxShape(a, pa, out boxA, out matBoxA) &&
                TryGetBodyTriangle(b, pb, out triangleB, out matTriangleB))
            {
                CombineMaterials(matBoxA, matTriangleB, out restitution, out muS, out muD, out muR);
                return SimpleShapeSolvers.SolveBoxTriangle(boxA, triangleB, out cps);
            }

            // ---------- Triangle - Box ----------
            if (TryGetBodyTriangle(a, pa, out triangleA, out matTriangleA) &&
                TryGetBodyBoxShape(b, pb, out boxB, out matBoxB))
            {
                CombineMaterials(matTriangleA, matBoxB, out restitution, out muS, out muD, out muR);

                if (!SimpleShapeSolvers.SolveBoxTriangle(boxB, triangleA, out cps))
                    return false;

                SimpleShapeSolvers.FlipManifold(ref cps);
                return true;
            }

            // ---------- Sphere - TriangleMesh ----------
            if (TryGetBodySphereShape(a, pa, out sphereA, out matSphereA) &&
                TryGetBodyTriangleMesh(b, pb, out ShardCollider meshColliderB, out Pose meshPoseB, out ShardTriangleMeshInfo meshInfoB, out ShardColliderMaterial matMeshB))
            {
                CombineMaterials(matSphereA, matMeshB, out restitution, out muS, out muD, out muR);

                return SolveSphereTriangleMesh(
                    sphereA,
                    meshColliderB.meshIndex,
                    meshInfoB,
                    meshPoseB,
                    out cps);
            }

            // ---------- TriangleMesh - Sphere ----------
            if (TryGetBodyTriangleMesh(a, pa, out ShardCollider meshColliderA, out Pose meshPoseA, out ShardTriangleMeshInfo meshInfoA, out ShardColliderMaterial matMeshA) &&
                TryGetBodySphereShape(b, pb, out sphereB, out matSphereB))
            {
                CombineMaterials(matMeshA, matSphereB, out restitution, out muS, out muD, out muR);

                if (!SolveSphereTriangleMesh(
                        sphereB,
                        meshColliderA.meshIndex,
                        meshInfoA,
                        meshPoseA,
                        out cps))
                    return false;

                SimpleShapeSolvers.FlipManifold(ref cps);
                return true;
            }

            // ---------- Capsule - TriangleMesh ----------
            if (TryGetBodyCapsuleShape(a, pa, out capsuleA, out matCapsuleA) &&
                TryGetBodyTriangleMesh(b, pb, out meshColliderB, out meshPoseB, out meshInfoB, out matMeshB))
            {
                CombineMaterials(matCapsuleA, matMeshB, out restitution, out muS, out muD, out muR);

                return SolveCapsuleTriangleMesh(
                    capsuleA,
                    meshColliderB.meshIndex,
                    meshInfoB,
                    meshPoseB,
                    out cps);
            }

            // ---------- TriangleMesh - Capsule ----------
            if (TryGetBodyTriangleMesh(a, pa, out meshColliderA, out meshPoseA, out meshInfoA, out matMeshA) &&
                TryGetBodyCapsuleShape(b, pb, out capsuleB, out matCapsuleB))
            {
                CombineMaterials(matMeshA, matCapsuleB, out restitution, out muS, out muD, out muR);

                if (!SolveCapsuleTriangleMesh(
                        capsuleB,
                        meshColliderA.meshIndex,
                        meshInfoA,
                        meshPoseA,
                        out cps))
                    return false;

                SimpleShapeSolvers.FlipManifold(ref cps);
                return true;
            }

            // ---------- Box - TriangleMesh ----------
            if (TryGetBodyBoxShape(a, pa, out boxA, out matBoxA) &&
                TryGetBodyTriangleMesh(b, pb, out meshColliderB, out meshPoseB, out meshInfoB, out matMeshB))
            {
                CombineMaterials(matBoxA, matMeshB, out restitution, out muS, out muD, out muR);

                return SolveBoxTriangleMesh(
                    boxA,
                    meshColliderB.meshIndex,
                    meshInfoB,
                    meshPoseB,
                    out cps);
            }

            // ---------- TriangleMesh - Box ----------
            if (TryGetBodyTriangleMesh(a, pa, out meshColliderA, out meshPoseA, out meshInfoA, out matMeshA) &&
                TryGetBodyBoxShape(b, pb, out boxB, out matBoxB))
            {
                CombineMaterials(matMeshA, matBoxB, out restitution, out muS, out muD, out muR);

                if (!SolveBoxTriangleMesh(
                        boxB,
                        meshColliderA.meshIndex,
                        meshInfoA,
                        meshPoseA,
                        out cps))
                    return false;

                SimpleShapeSolvers.FlipManifold(ref cps);
                return true;
            }

            // ---------- Box - Box ----------
            if (TryGetBodyBoxShape(a, pa, out boxA, out matBoxA) &&
                TryGetBodyBoxShape(b, pb, out boxB, out matBoxB))
            {
                CombineMaterials(matBoxA, matBoxB, out restitution, out muS, out muD, out muR);

                var solverBoxA = new BoxBoxSolver.Box(boxA.center, boxA.rotation, boxA.halfExtents);
                var solverBoxB = new BoxBoxSolver.Box(boxB.center, boxB.rotation, boxB.halfExtents);

                return BoxBoxSolver.Solve(solverBoxA, solverBoxB, out cps);
            }

            // ---------- Box - Cylinder ----------
            // Cylinders are currently ignored for localPose correctness work.
            if (TryGetBodyBox(a, out float3 boxAHalf, out matBoxA) &&
                TryGetBodyCylinder(b, out float cylBHalfHeight, out float cylBRadius, out ShardColliderMaterial matCylB))
            {
                CombineMaterials(matBoxA, matCylB, out restitution, out muS, out muD, out muR);

                var box = new CylinderBoxSolver.Box(pa.position, pa.rotation, boxAHalf);
                var cyl = new CylinderBoxSolver.Cylinder(pb.position, pb.rotation, cylBHalfHeight, cylBRadius);

                return CylinderBoxSolver.Solve(box, cyl, out cps);
            }

            // ---------- Cylinder - Box ----------
            // Cylinders are currently ignored for localPose correctness work.
            if (TryGetBodyCylinder(a, out float cylAHalfHeight, out float cylARadius, out ShardColliderMaterial matCylA) &&
                TryGetBodyBox(b, out float3 boxBHalf, out matBoxB))
            {
                CombineMaterials(matCylA, matBoxB, out restitution, out muS, out muD, out muR);

                var box = new CylinderBoxSolver.Box(pb.position, pb.rotation, boxBHalf);
                var cyl = new CylinderBoxSolver.Cylinder(pa.position, pa.rotation, cylAHalfHeight, cylARadius);

                if (!CylinderBoxSolver.Solve(box, cyl, out cps))
                    return false;

                SimpleShapeSolvers.FlipManifold(ref cps);
                return true;
            }

            // ---------- Cylinder - Cylinder ----------
            // Cylinders are currently ignored for localPose correctness work.
            if (TryGetBodyCylinder(a, out cylAHalfHeight, out cylARadius, out matCylA) &&
                TryGetBodyCylinder(b, out cylBHalfHeight, out cylBRadius, out matCylB))
            {
                CombineMaterials(matCylA, matCylB, out restitution, out muS, out muD, out muR);

                var cylA = new CylinderCylinderSolver.Cylinder(pa.position, pa.rotation, cylAHalfHeight, cylARadius);
                var cylB = new CylinderCylinderSolver.Cylinder(pb.position, pb.rotation, cylBHalfHeight, cylBRadius);

                return CylinderCylinderSolver.Solve(cylA, cylB, out cps);
            }

            return false;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void BuildCachedContactsForSubstep()
        {
            const float kDepthSlop = 1e-5f;

            cachedContacts.Clear();

            // Broadphase is now rebuilt once per substep, not once per solver iteration.
            broadphase.Rebuild(poses, bodyTypes, colliderStore, triangleMeshStore);

            for (int pairIndex = 0; pairIndex < broadphase.PairCount; pairIndex++)
            {
                ShardBodyPair pair = broadphase.GetPair(pairIndex);
                int a = pair.a;
                int b = pair.b;

                bool aDyn = bodyTypes[a] == BodyType.Dynamic;
                bool bDyn = bodyTypes[b] == BodyType.Dynamic;
                if (!aDyn && !bDyn)
                    continue;

                Pose pa = poses[a];
                Pose pb = poses[b];

                bool gotContact = GetContactManifold(
                    a,
                    b,
                    pa,
                    pb,
                    out ContactPointManifold cps,
                    out float restitution,
                    out float muS,
                    out float muD,
                    out float muR);

                if (!gotContact)
                    continue;

                float3 globalN = cps.globalPenAxis; // A -> B
                float depth = cps.globalPenDepth;
                if (depth <= kDepthSlop)
                    continue;

                // Positional correction happens once when contacts are built.
                // Iterations should solve impulses only; otherwise cached depth would be applied repeatedly.
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

                if (cps.numContactPoints <= 0)
                    continue;

                cachedContacts.Add(new CachedContact
                {
                    a = a,
                    b = b,
                    cps = cps,
                    restitution = restitution,
                    muS = muS,
                    muD = muD,
                    muR = muR
                });
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SolveCachedContacts(float dt, bool applyBias)
        {
            const float kStaticVelEps = 0.05f;  // m/s threshold to treat as trying to stick
            const float kRollEps = 1e-8f;

            for (int contactIndex = 0; contactIndex < cachedContacts.Length; contactIndex++)
            {
                CachedContact contact = cachedContacts[contactIndex];

                int a = contact.a;
                int b = contact.b;

                bool aDyn = bodyTypes[a] == BodyType.Dynamic;
                bool bDyn = bodyTypes[b] == BodyType.Dynamic;
                if (!aDyn && !bDyn)
                    continue;

                Pose pa = poses[a];
                Pose pb = poses[b];
                ContactPointManifold cps = contact.cps;

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

                // Solve per-contact impulses: normal + friction + rolling drag.
                for (int ci = 0; ci < count; ci++)
                {
                    var cp = cps[ci];

                    float3 p = cp.point;

                    // Use the contact's own normal when possible.
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

                    // Bias is only applied on the first solver iteration.
                    // With cached contacts, applying full cached depth every iteration can over-correct velocity.
                    const float beta = 0.2f;
                    const float allowedPen = 1e-4f;
                    float biasVel = 0f;
                    if (applyBias)
                    {
                        float pen = math.max(0f, contact.cps.globalPenDepth - allowedPen);
                        biasVel = (pen > 0f)
                            ? (beta * pen / math.max(dt, 1e-6f))
                            : 0f;
                    }

                    // --- Normal impulse ---
                    float3 rAxN = math.cross(rA, n);
                    float3 rBxN = math.cross(rB, n);

                    float3 angA_n = aDyn ? math.cross(math.mul(invIworldA, rAxN), rA) : float3.zero;
                    float3 angB_n = bDyn ? math.cross(math.mul(invIworldB, rBxN), rB) : float3.zero;

                    float denomN = invMassA + invMassB + math.dot(n, angA_n + angB_n);
                    if (denomN <= 1e-12f)
                        continue;

                    // Restitution only for real impacts.
                    float e = contact.restitution;
                    if (-vn < 1.0f) e = 0f;

                    float jn = (-(1f + e) * vn + biasVel) / denomN;

                    // Clamp: no pulling forces.
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

                    // --- Tangential friction impulse ---
                    // Recompute relative velocity after normal impulse.
                    vPointA = va.linearVelocity + math.cross(va.angularVelocity, rA);
                    vPointB = vb.linearVelocity + math.cross(vb.angularVelocity, rB);
                    vRel = vPointB - vPointA;

                    float3 vt = vRel - n * math.dot(vRel, n);
                    float vtLen = math.length(vt);

                    if (vtLen > 1e-8f)
                    {
                        float3 t = vt / vtLen;

                        float3 rAxT = math.cross(rA, t);
                        float3 rBxT = math.cross(rB, t);

                        float3 angA_t = aDyn ? math.cross(math.mul(invIworldA, rAxT), rA) : float3.zero;
                        float3 angB_t = bDyn ? math.cross(math.mul(invIworldB, rBxT), rB) : float3.zero;

                        float denomT = invMassA + invMassB + math.dot(t, angA_t + angB_t);
                        if (denomT > 1e-12f)
                        {
                            float jt = -math.dot(vRel, t) / denomT;

                            float maxStatic = contact.muS * jn;
                            float maxDynamic = contact.muD * jn;
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

                    // --- Rolling friction ---
                    if (contact.muR > 0f && jn > 0f)
                    {
                        float k = contact.muR * jn;
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool TryGetBodySphereShape(
            int dense,
            Pose bodyPose,
            out SimpleShapeSolvers.Sphere sphere,
            out ShardColliderMaterial mat)
        {
            int n = colliderStore.GetHead(dense);
            while (n != -1)
            {
                if (!colliderStore.TryGetNode(n, out ShardCollider c, out int next))
                    break;

                if (c.type == ShardColliderType.Sphere)
                {
                    Pose colliderPose = ComposePose(bodyPose, c.localPose);
                    sphere = new SimpleShapeSolvers.Sphere(colliderPose.position, math.max(0f, c.radius));
                    mat = c.material;
                    return true;
                }

                n = next;
            }

            sphere = default;
            mat = default;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool TryGetBodyBoxShape(
            int dense,
            Pose bodyPose,
            out SimpleShapeSolvers.Box box,
            out ShardColliderMaterial mat)
        {
            int n = colliderStore.GetHead(dense);
            while (n != -1)
            {
                if (!colliderStore.TryGetNode(n, out ShardCollider c, out int next))
                    break;

                if (c.type == ShardColliderType.Box)
                {
                    Pose colliderPose = ComposePose(bodyPose, c.localPose);
                    box = new SimpleShapeSolvers.Box(
                        colliderPose.position,
                        colliderPose.rotation,
                        math.max(c.halfExtents, float3.zero));
                    mat = c.material;
                    return true;
                }

                n = next;
            }

            box = default;
            mat = default;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool TryGetBodyCapsuleShape(
            int dense,
            Pose bodyPose,
            out SimpleShapeSolvers.Capsule capsule,
            out ShardColliderMaterial mat)
        {
            int n = colliderStore.GetHead(dense);
            while (n != -1)
            {
                if (!colliderStore.TryGetNode(n, out ShardCollider c, out int next))
                    break;

                if (c.type == ShardColliderType.Capsule)
                {
                    Pose colliderPose = ComposePose(bodyPose, c.localPose);
                    capsule = new SimpleShapeSolvers.Capsule(
                        colliderPose.position,
                        colliderPose.rotation,
                        math.max(0f, c.height * 0.5f),
                        math.max(0f, c.radius));
                    mat = c.material;
                    return true;
                }

                n = next;
            }

            capsule = default;
            mat = default;
            return false;
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
        private bool TryGetBodySphere(int dense, out float radius, out ShardColliderMaterial mat)
        {
            int n = colliderStore.GetHead(dense);
            while (n != -1)
            {
                if (!colliderStore.TryGetNode(n, out ShardCollider c, out int next))
                    break;

                if (c.type == ShardColliderType.Sphere)
                {
                    radius = c.radius;
                    mat = c.material;
                    return true;
                }

                n = next;
            }

            radius = 0f;
            mat = default;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool TryGetBodyCapsule(
            int dense,
            out float halfHeight,
            out float radius,
            out ShardColliderMaterial mat)
        {
            int n = colliderStore.GetHead(dense);
            while (n != -1)
            {
                if (!colliderStore.TryGetNode(n, out ShardCollider c, out int next))
                    break;

                if (c.type == ShardColliderType.Capsule)
                {
                    // Matches your cylinder convention:
                    // c.height stores full height, solver wants half height.
                    halfHeight = c.height * 0.5f;
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
        private bool TryGetBodyTriangle(
            int dense,
            Pose bodyPose,
            out SimpleShapeSolvers.Triangle triangle,
            out ShardColliderMaterial mat)
        {
            int n = colliderStore.GetHead(dense);
            while (n != -1)
            {
                if (!colliderStore.TryGetNode(n, out ShardCollider c, out int next))
                    break;

                if (c.type == ShardColliderType.Triangle)
                {
                    Pose colliderPose = ComposePose(bodyPose, c.localPose);

                    float3 a = TransformPoint(colliderPose, c.vertexA);
                    float3 b = TransformPoint(colliderPose, c.vertexB);
                    float3 cc = TransformPoint(colliderPose, c.vertexC);

                    triangle = new SimpleShapeSolvers.Triangle(a, b, cc);
                    mat = c.material;
                    return true;
                }

                n = next;
            }

            triangle = default;
            mat = default;
            return false;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool TryGetBodyTriangleMesh(
            int dense,
            Pose bodyPose,
            out ShardCollider meshCollider,
            out Pose meshWorldPose,
            out ShardTriangleMeshInfo meshInfo,
            out ShardColliderMaterial mat)
        {
            int n = colliderStore.GetHead(dense);
            while (n != -1)
            {
                if (!colliderStore.TryGetNode(n, out ShardCollider c, out int next))
                    break;

                if (c.type == ShardColliderType.Mesh &&
                    triangleMeshStore.TryGetMeshInfo(c.meshIndex, out meshInfo))
                {
                    meshCollider = c;
                    meshWorldPose = ComposePose(bodyPose, c.localPose);
                    mat = c.material;
                    return true;
                }

                n = next;
            }

            meshCollider = default;
            meshWorldPose = default;
            meshInfo = default;
            mat = default;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static SimpleShapeSolvers.Triangle TransformTriangle(
            in ShardTriangle localTriangle,
            in Pose meshWorldPose)
        {
            return new SimpleShapeSolvers.Triangle(
                TransformPoint(meshWorldPose, localTriangle.a),
                TransformPoint(meshWorldPose, localTriangle.b),
                TransformPoint(meshWorldPose, localTriangle.c));
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Aabb ComputeSphereAabb(SimpleShapeSolvers.Sphere sphere)
        {
            float r = math.max(0f, sphere.radius);
            return new Aabb(sphere.center - new float3(r), sphere.center + new float3(r));
        }

        private static Aabb ComputeBoxAabb(SimpleShapeSolvers.Box box)
        {
            float3x3 r = new float3x3(box.rotation);

            float3 extents =
                math.abs(r.c0) * box.halfExtents.x +
                math.abs(r.c1) * box.halfExtents.y +
                math.abs(r.c2) * box.halfExtents.z;

            return new Aabb(box.center - extents, box.center + extents);
        }

        private static Aabb ComputeCapsuleAabb(SimpleShapeSolvers.Capsule capsule)
        {
            float3 axis = math.mul(capsule.rotation, new float3(0f, 1f, 0f));
            float3 extents = math.abs(axis) * capsule.halfHeight + new float3(capsule.radius);
            return new Aabb(capsule.center - extents, capsule.center + extents);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Aabb ComputeTriangleAabb(SimpleShapeSolvers.Triangle triangle)
        {
            Aabb aabb = Aabb.Empty;
            aabb.Encapsulate(triangle.a);
            aabb.Encapsulate(triangle.b);
            aabb.Encapsulate(triangle.c);
            return aabb;
        }

        private bool SolveSphereTriangleMesh(
            SimpleShapeSolvers.Sphere sphere,
            int meshIndex,
            ShardTriangleMeshInfo meshInfo,
            Pose meshWorldPose,
            out ContactPointManifold manifold)
        {
            manifold = default;

            Aabb queryAabb = ComputeSphereAabb(sphere);

            bool hit = false;
            float deepest = float.NegativeInfinity;
            float3 accumulatedNormal = float3.zero;

            for (int i = 0; i < meshInfo.triangleCount; i++)
            {
                if (!triangleMeshStore.TryGetTriangle(meshIndex, i, out ShardTriangle localTriangle))
                    continue;

                SimpleShapeSolvers.Triangle worldTriangle =
                    TransformTriangle(localTriangle, meshWorldPose);

                if (!queryAabb.Overlaps(ComputeTriangleAabb(worldTriangle)))
                    continue;

                if (!SimpleShapeSolvers.SolveSphereTriangle(
                        sphere,
                        worldTriangle,
                        out ContactPointManifold candidate))
                    continue;

                hit = true;
                deepest = math.max(deepest, candidate.globalPenDepth);
                accumulatedNormal += candidate.globalPenAxis * math.max(candidate.globalPenDepth, 0.0001f);

                AddTriangleMeshCandidateContact(ref manifold, candidate.p1);
            }

            if (!hit)
                return false;

            manifold.globalPenDepth = deepest;
            manifold.globalPenAxis = NormalizeOrFallback(accumulatedNormal, new float3(0f, 1f, 0f));

            NormalizeTriangleMeshContactNormals(ref manifold, manifold.globalPenAxis);
            return true;
        }
        
        private static void AddTriangleMeshCandidateContact(
            ref ContactPointManifold manifold,
            ContactPoint contact)
        {
            const float duplicateDistanceSq = 0.0001f;

            for (int i = 0; i < manifold.numContactPoints; i++)
            {
                ContactPoint existing = manifold[i];

                if (math.lengthsq(existing.point - contact.point) < duplicateDistanceSq)
                {
                    if (contact.depth > existing.depth)
                        manifold[i] = contact;

                    return;
                }
            }

            if (manifold.numContactPoints < 4)
            {
                manifold[manifold.numContactPoints] = contact;
                manifold.numContactPoints++;
                return;
            }

            int shallowestIndex = 0;
            float shallowestDepth = manifold[0].depth;

            for (int i = 1; i < 4; i++)
            {
                if (manifold[i].depth < shallowestDepth)
                {
                    shallowestDepth = manifold[i].depth;
                    shallowestIndex = i;
                }
            }

            if (contact.depth > shallowestDepth)
                manifold[shallowestIndex] = contact;
        }

        private static void NormalizeTriangleMeshContactNormals(
            ref ContactPointManifold manifold,
            float3 normal)
        {
            for (int i = 0; i < manifold.numContactPoints; i++)
            {
                ContactPoint cp = manifold[i];
                cp.normal = normal;
                manifold[i] = cp;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 NormalizeOrFallback(float3 v, float3 fallback)
        {
            float lenSq = math.lengthsq(v);

            if (lenSq < 1e-8f)
                return fallback;

            return v * math.rsqrt(lenSq);
        }

        private bool SolveBoxTriangleMesh(
            SimpleShapeSolvers.Box box,
            int meshIndex,
            ShardTriangleMeshInfo meshInfo,
            Pose meshWorldPose,
            out ContactPointManifold manifold)
        {
            manifold = default;

            Aabb queryAabb = ComputeBoxAabb(box);

            bool hit = false;
            float deepest = float.NegativeInfinity;
            float3 accumulatedNormal = float3.zero;

            for (int i = 0; i < meshInfo.triangleCount; i++)
            {
                if (!triangleMeshStore.TryGetTriangle(meshIndex, i, out ShardTriangle localTriangle))
                    continue;

                SimpleShapeSolvers.Triangle worldTriangle =
                    TransformTriangle(localTriangle, meshWorldPose);

                if (!queryAabb.Overlaps(ComputeTriangleAabb(worldTriangle)))
                    continue;

                if (!SimpleShapeSolvers.SolveBoxTriangle(
                        box,
                        worldTriangle,
                        out ContactPointManifold candidate))
                    continue;

                hit = true;
                deepest = math.max(deepest, candidate.globalPenDepth);
                accumulatedNormal += candidate.globalPenAxis * math.max(candidate.globalPenDepth, 0.0001f);

                for (int c = 0; c < candidate.numContactPoints; c++)
                    AddTriangleMeshCandidateContact(ref manifold, candidate[c]);
            }

            if (!hit)
                return false;

            manifold.globalPenDepth = deepest;
            manifold.globalPenAxis = NormalizeOrFallback(accumulatedNormal, new float3(0f, 1f, 0f));

            NormalizeTriangleMeshContactNormals(ref manifold, manifold.globalPenAxis);
            return true;
        }

        private bool SolveCapsuleTriangleMesh(
            SimpleShapeSolvers.Capsule capsule,
            int meshIndex,
            ShardTriangleMeshInfo meshInfo,
            Pose meshWorldPose,
            out ContactPointManifold manifold)
        {
            manifold = default;

            Aabb queryAabb = ComputeCapsuleAabb(capsule);

            bool hit = false;
            float deepest = float.NegativeInfinity;
            float3 accumulatedNormal = float3.zero;

            for (int i = 0; i < meshInfo.triangleCount; i++)
            {
                if (!triangleMeshStore.TryGetTriangle(meshIndex, i, out ShardTriangle localTriangle))
                    continue;

                SimpleShapeSolvers.Triangle worldTriangle =
                    TransformTriangle(localTriangle, meshWorldPose);

                if (!queryAabb.Overlaps(ComputeTriangleAabb(worldTriangle)))
                    continue;

                if (!SimpleShapeSolvers.SolveCapsuleTriangle(
                        capsule,
                        worldTriangle,
                        out ContactPointManifold candidate))
                    continue;

                hit = true;
                deepest = math.max(deepest, candidate.globalPenDepth);
                accumulatedNormal += candidate.globalPenAxis * math.max(candidate.globalPenDepth, 0.0001f);

                AddTriangleMeshCandidateContact(ref manifold, candidate.p1);
            }

            if (!hit)
                return false;

            manifold.globalPenDepth = deepest;
            manifold.globalPenAxis = NormalizeOrFallback(accumulatedNormal, new float3(0f, 1f, 0f));

            NormalizeTriangleMeshContactNormals(ref manifold, manifold.globalPenAxis);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Pose ComposePose(Pose bodyPose, Pose localPose)
        {
            return new Pose
            {
                position = bodyPose.position + math.mul(bodyPose.rotation, localPose.position),
                rotation = math.mul(bodyPose.rotation, localPose.rotation)
            };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 TransformPoint(Pose pose, float3 localPoint)
        {
            return pose.position + math.mul(pose.rotation, localPoint);
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

            // geometric mean (nice �middle ground�)
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
