using Unity.Mathematics;

namespace Shard
{
    /// "Body" is the entity-like record.
    public struct Body
    {
        public MotionType MotionType;
        public ColliderHandle Collider;
        public ushort MaterialId;
        public ushort Flags; // sleeping, CCD, etc.
    }

    /// Pose is stored separately so it’s hot in integration & broadphase.
    public struct Pose
    {
        public float3 Position;
        public quaternion Rotation;
    }

    public struct Velocity
    {
        public float3 Linear;
        public float3 Angular;
    }

    public struct MassProperties
    {
        public float InverseMass;          // 0 for static
        public float3 InverseInertiaLocal; // diagonal in local space (starter)
        public float3 CenterOfMassLocal;   // optional
    }

    public struct Damping
    {
        public float Linear;
        public float Angular;
    }
}
