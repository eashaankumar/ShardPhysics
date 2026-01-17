using Unity.Collections;
using Unity.Mathematics;

namespace Shard
{
    internal struct Broadphase : System.IDisposable
    {
        // Dynamic tree, static tree, proxy mapping, etc.
        // Proxies map BodyId -> tree leaf id
        public NativeList<int> BodyToProxy;

        public Broadphase(Allocator allocator)
        {
            BodyToProxy = new NativeList<int>(allocator);
        }

        public void Dispose() => BodyToProxy.Dispose();
    }

    internal struct Narrowphase : System.IDisposable
    {
        // Pair cache + manifolds
        public NativeList<ContactManifold> Manifolds;

        public Narrowphase(Allocator allocator)
        {
            Manifolds = new NativeList<ContactManifold>(allocator);
        }

        public void Dispose() => Manifolds.Dispose();
    }

    public struct ContactPoint
    {
        public float3 Position;
        public float Penetration;
        public uint FeatureId; // important for warm starting & mesh/voxel caching
    }

    public struct ContactManifold
    {
        public BodyId A;
        public BodyId B;
        public float3 Normal; // from A -> B
        public byte PointCount;
        public ContactPoint P0, P1, P2, P3;

        // Cached impulses per point for warm start (normal + friction)
        public float ImpulseN0, ImpulseN1, ImpulseN2, ImpulseN3;
        public float ImpulseT0, ImpulseT1, ImpulseT2, ImpulseT3;
    }

    internal struct ConstraintGraph : System.IDisposable
    {
        // Constraints as edges; built from joints + contacts.
        public NativeList<Constraint> Constraints;

        public ConstraintGraph(Allocator allocator)
        {
            Constraints = new NativeList<Constraint>(allocator);
        }

        public void Dispose() => Constraints.Dispose();
    }

    public enum ConstraintType : byte
    {
        Contact,
        Hinge,
        BallSocket,
        Prismatic,
        ConeTwist,
        Distance,
        Motor,
        Custom
    }

    public struct Constraint
    {
        public ConstraintType Type;
        public BodyId A;
        public BodyId B;
        public int PayloadIndex;   // points into per-type constraint payload pools
        public ushort Flags;
    }

    internal struct Solver : System.IDisposable
    {
        // Scratch + row buffers, impulses, etc. (per-world)
        public NativeList<SolverRow> Rows;

        public Solver(Allocator allocator)
        {
            Rows = new NativeList<SolverRow>(allocator);
        }

        public void Dispose() => Rows.Dispose();
    }

    /// Row-based solver core unit (Jacobian row / impulse clamp)
    public struct SolverRow
    {
        public BodyId A, B;

        public float3 JA_Linear;
        public float3 JA_Angular;
        public float3 JB_Linear;
        public float3 JB_Angular;

        public float EffectiveMass;
        public float Bias;
        public float Impulse;
        public float MinImpulse;
        public float MaxImpulse;
    }
}
