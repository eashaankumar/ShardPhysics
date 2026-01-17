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
    }
}
