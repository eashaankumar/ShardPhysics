using Unity.Collections;
using Unity.Mathematics;

namespace Shard
{
    public struct CompoundChild
    {
        public ColliderHandle Collider;
        public TransformQv LocalTransform;
        public Aabb LocalAabb; // cached child bounds (local to compound root)
    }

    public struct CompoundBvhNode
    {
        public Aabb Bounds;
        public int Left;   // node index or -1
        public int Right;  // node index or -1
        public int ChildIndex; // leaf: index into Children, else -1
    }

    public struct CompoundCollider
    {
        // Stored in NativeArrays allocated per collider instance.
        // Handle table points here; you can also pool these allocations.
        public NativeArray<CompoundChild> Children;
        public NativeArray<CompoundBvhNode> Bvh;
        public Aabb LocalAabb;
    }
}
