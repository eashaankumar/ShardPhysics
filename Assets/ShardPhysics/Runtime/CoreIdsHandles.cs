namespace Shard
{
    public readonly struct BodyId
    {
        public readonly int Value;
        public BodyId(int value) => Value = value;
        public bool IsValid => Value >= 0;
        public static readonly BodyId Invalid = new BodyId(-1);
        public override string ToString() => $"Body({Value})";
    }

    public enum ColliderType : byte
    {
        Sphere,
        Capsule,
        Box,

        Cone,       // NEW
        Cylinder,   // NEW

        Convex,
        Mesh,
        Compound,
        Voxel
    }

    /// Handle is stable; payload can be replaced in-place by versioning.
    public readonly struct ColliderHandle
    {
        public readonly int Slot;      // index into handle table
        public readonly ushort Version; // ABA protection
        public ColliderHandle(int slot, ushort version) { Slot = slot; Version = version; }
        public bool IsValid => Slot >= 0;
        public static readonly ColliderHandle Invalid = new ColliderHandle(-1, 0);
        public override string ToString() => $"Col({Slot}:{Version})";
    }

    public readonly struct ConstraintId
    {
        public readonly int Value;
        public ConstraintId(int value) => Value = value;
        public bool IsValid => Value >= 0;
        public static readonly ConstraintId Invalid = new ConstraintId(-1);
    }

    public enum MotionType : byte
    {
        Static,
        Dynamic,
        Kinematic
    }
}
