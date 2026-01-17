using Unity.Collections;
using Unity.Mathematics;

namespace Shard
{
    public enum VoxelStorageKind : byte
    {
        BrickBitmask, // recommended baseline
        RleRuns        // optional alt
    }

    /// 8x8x8 brick occupancy (512 bits). Stored as 8 ulongs (8*64=512).
    public struct BrickMask512
    {
        public ulong M0, M1, M2, M3, M4, M5, M6, M7;
    }

    public struct VoxelBrick
    {
        public int3 BrickCoord;     // brick grid coordinate in voxel space
        public BrickMask512 Mask;   // occupancy
        public Aabb LocalAabb;      // bounds for this brick (occupied cells only if you want tighter)
    }

    public struct VoxelBvhNode
    {
        public Aabb Bounds;
        public int Left;
        public int Right;
        public int BrickIndex; // leaf if >= 0
    }

    public struct VoxelColliderData
    {
        public VoxelStorageKind StorageKind;

        // Voxel grid metadata
        public float CellSize;   // size of one voxel cell
        public int3 VoxelMin;    // optional bounds
        public int3 VoxelMax;

        // Compressed storage (baseline: bricks)
        public NativeArray<VoxelBrick> Bricks;
        public NativeArray<VoxelBvhNode> Bvh;

        public Aabb LocalAabb;
        public uint Version; // bump when bricks/occupancy changes
    }
}
