using Unity.Collections;
using Unity.Mathematics;

namespace Shard
{
    public sealed class PhysicsWorld : System.IDisposable
    {
        public readonly Allocator Allocator;
        public int BodyCount => m_bodyCount;

        // Optional: float32 local-origin trick for server
        public double3 WorldOrigin; // global origin in double space
        public uint WorldId;

        // ---- Bodies (SoA) ----
        internal NativeList<Body> Bodies;
        internal NativeList<Pose> Poses;
        internal NativeList<Velocity> Velocities;
        internal NativeList<MassProperties> Masses;
        internal NativeList<Damping> Dampings;

        // Bookkeeping
        internal NativeList<int> FreeBodySlots; // free-list for BodyId reuse
        internal int m_bodyCount;

        // ---- Colliders ----
        internal ColliderStore Colliders;

        // ---- Broadphase / Narrowphase / Solver ----
        internal Broadphase Broadphase;
        internal Narrowphase Narrowphase;
        internal ConstraintGraph Constraints;
        internal Solver Solver;

        // ---- Scratch (per-world) ----
        internal RewindableAllocator Scratch; // per-step temp allocations

        public PhysicsWorld(int bodyCapacity, int colliderCapacity, Allocator allocator, uint worldId = 1)
        {
            Allocator = allocator;
            WorldId = worldId;

            Bodies = new NativeList<Body>(bodyCapacity, allocator);
            Poses = new NativeList<Pose>(bodyCapacity, allocator);
            Velocities = new NativeList<Velocity>(bodyCapacity, allocator);
            Masses = new NativeList<MassProperties>(bodyCapacity, allocator);
            Dampings = new NativeList<Damping>(bodyCapacity, allocator);
            FreeBodySlots = new NativeList<int>(allocator);

            Colliders = new ColliderStore(colliderCapacity, allocator);

            Broadphase = new Broadphase(allocator);
            Narrowphase = new Narrowphase(allocator);
            Constraints = new ConstraintGraph(allocator);
            Solver = new Solver(allocator);

            Scratch = new RewindableAllocator();
            const int scratchBytes = 256 * 1024;
            Scratch.Initialize(scratchBytes);
        }

        public void Dispose()
        {
            Solver.Dispose();
            Constraints.Dispose();
            Narrowphase.Dispose();
            Broadphase.Dispose();
            Colliders.Dispose();

            Scratch.Dispose();

            FreeBodySlots.Dispose();
            Dampings.Dispose();
            Masses.Dispose();
            Velocities.Dispose();
            Poses.Dispose();
            Bodies.Dispose();
        }

        // -------------------------
        // Bodies: allocation + API
        // -------------------------

        private BodyId AllocateBody()
        {
            int index;
            if (FreeBodySlots.Length > 0)
            {
                index = FreeBodySlots[^1];
                FreeBodySlots.RemoveAtSwapBack(FreeBodySlots.Length - 1);
            }
            else
            {
                index = Bodies.Length;

                Bodies.Add(default);
                Poses.Add(default);
                Velocities.Add(default);
                Masses.Add(default);
                Dampings.Add(default);

                m_bodyCount++;
            }

            EnsureBroadphaseCapacity(index + 1);
            return new BodyId(index);
        }

        public BodyId AddBody(
            MotionType motionType,
            Pose pose,
            Velocity velocity,
            MassProperties mass,
            Damping damping,
            ColliderHandle collider,
            ushort materialId = 0,
            ushort flags = 0)
        {
            var id = AllocateBody();

            Bodies[id.Value] = new Body
            {
                MotionType = motionType,
                Collider = collider,
                MaterialId = materialId,
                Flags = flags
            };

            Poses[id.Value] = pose;
            Velocities[id.Value] = velocity;
            Masses[id.Value] = mass;
            Dampings[id.Value] = damping;

            // Placeholder: keep BodyToProxy in sync (real tree comes in step 4)
            SyncProxyForBody(id);

            return id;
        }

        public void RemoveBody(BodyId id)
        {
            if (!id.IsValid || (uint)id.Value >= (uint)Bodies.Length)
                return;

            // Placeholder proxy removal
            EnsureBroadphaseCapacity(id.Value + 1);
            Broadphase.BodyToProxy[id.Value] = -1;

            Bodies[id.Value] = default;
            Poses[id.Value] = default;
            Velocities[id.Value] = default;
            Masses[id.Value] = default;
            Dampings[id.Value] = default;

            FreeBodySlots.Add(id.Value);
        }

        public void SetPose(BodyId id, Pose pose)
        {
            Poses[id.Value] = pose;
            SyncProxyForBody(id);
        }

        public void SetCollider(BodyId id, ColliderHandle collider)
        {
            var b = Bodies[id.Value];
            b.Collider = collider;
            Bodies[id.Value] = b;

            SyncProxyForBody(id);
        }

        // -------------------------
        // Broadphase placeholder plumbing
        // -------------------------

        private void EnsureBroadphaseCapacity(int bodyCount)
        {
            // Ensure BodyToProxy has an entry for every body slot.
            while (Broadphase.BodyToProxy.Length < bodyCount)
                Broadphase.BodyToProxy.Add(-1);
        }

        private void SyncProxyForBody(BodyId id)
        {
            EnsureBroadphaseCapacity(id.Value + 1);

            var body = Bodies[id.Value];
            if (!Colliders.IsValid(body.Collider))
            {
                Broadphase.BodyToProxy[id.Value] = -1;
                return;
            }

            // Compute world AABB using your step (2) helper.
            ref var header = ref Colliders.Resolve(body.Collider);
            Aabb worldAabb = WorldAabb.FromBodyPose(header.LocalAabb, Poses[id.Value]);

            // Placeholder: proxy id == body id
            int proxy = Broadphase.BodyToProxy[id.Value];
            if (proxy == -1)
                Broadphase.BodyToProxy[id.Value] = id.Value;

            // Step 4 will replace this with real Broadphase.CreateProxy / UpdateProxy.
            // For now we just ensure the proxy exists and worldAabb computes cleanly.
            _ = worldAabb;
        }

    }
}
