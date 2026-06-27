using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Shard.Runtime
{
    public readonly struct ShardBodyPair
    {
        public readonly int a;
        public readonly int b;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ShardBodyPair(int a, int b)
        {
            this.a = a;
            this.b = b;
        }
    }

    public sealed class Broadphase : IDisposable
    {
        private NativeList<Aabb> bodyAabbs;
        private NativeList<byte> hasBodyAabb;
        private NativeList<ShardBodyPair> pairs;

        // Spatial hash storage. Bodies are inserted into every grid cell touched by their AABB.
        // Pair keys prevent the same body pair from being emitted multiple times when two AABBs
        // overlap in more than one cell.
        private NativeParallelMultiHashMap<int, int> cellToBodies;
        private NativeParallelHashSet<ulong> emittedPairKeys;
        private NativeList<int> spatialBodies;
        private NativeList<int> oversizedBodies;

        private readonly Allocator allocator;

        public int PairCount => pairs.Length;

        public bool EnableSpatialHashing = true;
        public int SpatialHashBodyThreshold = 96;
        public float SpatialHashCellSize = 4f;
        public int MaxCellsPerBody = 128;

        public Broadphase(int initialBodyCapacity, Allocator allocator)
        {
            this.allocator = allocator;

            bodyAabbs = new NativeList<Aabb>(initialBodyCapacity, allocator);
            hasBodyAabb = new NativeList<byte>(initialBodyCapacity, allocator);
            pairs = new NativeList<ShardBodyPair>(math.max(16, initialBodyCapacity * 2), allocator);

            cellToBodies = new NativeParallelMultiHashMap<int, int>(math.max(64, initialBodyCapacity * 4), allocator);
            emittedPairKeys = new NativeParallelHashSet<ulong>(math.max(64, initialBodyCapacity * 4), allocator);
            spatialBodies = new NativeList<int>(initialBodyCapacity, allocator);
            oversizedBodies = new NativeList<int>(math.max(4, initialBodyCapacity / 8), allocator);
        }

        public void Dispose()
        {
            if (bodyAabbs.IsCreated) bodyAabbs.Dispose();
            if (hasBodyAabb.IsCreated) hasBodyAabb.Dispose();
            if (pairs.IsCreated) pairs.Dispose();
            if (cellToBodies.IsCreated) cellToBodies.Dispose();
            if (emittedPairKeys.IsCreated) emittedPairKeys.Dispose();
            if (spatialBodies.IsCreated) spatialBodies.Dispose();
            if (oversizedBodies.IsCreated) oversizedBodies.Dispose();
        }

        public void Rebuild(
            NativeList<Pose> poses,
            NativeList<BodyType> bodyTypes,
            ShardColliderStore colliderStore,
            ShardTriangleMeshStore triangleMeshStore)
        {
            int bodyCount = poses.Length;

            bodyAabbs.ResizeUninitialized(bodyCount);
            hasBodyAabb.ResizeUninitialized(bodyCount);
            pairs.Clear();

            int validAabbCount = 0;
            for (int i = 0; i < bodyCount; i++)
            {
                if (ComputeBodyAabb(i, poses[i], colliderStore, triangleMeshStore, out Aabb aabb))
                {
                    bodyAabbs[i] = aabb;
                    hasBodyAabb[i] = 1;
                    validAabbCount++;
                }
                else
                {
                    bodyAabbs[i] = default;
                    hasBodyAabb[i] = 0;
                }
            }

            if (EnableSpatialHashing && validAabbCount >= SpatialHashBodyThreshold && SpatialHashCellSize > 1e-5f)
                RebuildSpatialHash(bodyCount, bodyTypes);
            else
                RebuildBruteForce(bodyCount, bodyTypes);
        }

        private void RebuildBruteForce(int bodyCount, NativeList<BodyType> bodyTypes)
        {
            for (int a = 0; a < bodyCount; a++)
            {
                if (hasBodyAabb[a] == 0)
                    continue;

                bool aDynamic = bodyTypes[a] == BodyType.Dynamic;

                for (int b = a + 1; b < bodyCount; b++)
                {
                    if (hasBodyAabb[b] == 0)
                        continue;

                    bool bDynamic = bodyTypes[b] == BodyType.Dynamic;

                    if (!aDynamic && !bDynamic)
                        continue;

                    if (bodyAabbs[a].Overlaps(bodyAabbs[b]))
                        pairs.Add(new ShardBodyPair(a, b));
                }
            }
        }

        private void RebuildSpatialHash(int bodyCount, NativeList<BodyType> bodyTypes)
        {
            cellToBodies.Clear();
            emittedPairKeys.Clear();
            spatialBodies.Clear();
            oversizedBodies.Clear();

            int estimatedCellWrites = 0;
            for (int i = 0; i < bodyCount; i++)
            {
                if (hasBodyAabb[i] == 0)
                    continue;

                int cellCount = CountTouchedCells(bodyAabbs[i]);
                if (cellCount <= 0 || cellCount > MaxCellsPerBody)
                    oversizedBodies.Add(i);
                else
                {
                    spatialBodies.Add(i);
                    estimatedCellWrites += cellCount;
                }
            }

            EnsureSpatialCapacities(estimatedCellWrites, math.max(64, spatialBodies.Length * 4));

            // Insert incrementally. Querying cells before insertion naturally avoids same-index pairs.
            for (int i = 0; i < spatialBodies.Length; i++)
            {
                int body = spatialBodies[i];
                AddPairsAgainstBodiesAlreadyInTouchedCells(body, bodyTypes);
                InsertBodyIntoTouchedCells(body);
            }

            // Huge AABBs, such as large static terrain meshes, are intentionally not expanded into
            // hundreds or thousands of hash cells. Test them separately against every body that can
            // produce a dynamic pair. This keeps the hash grid useful even in terrain scenes.
            for (int i = 0; i < oversizedBodies.Length; i++)
            {
                int big = oversizedBodies[i];
                for (int other = 0; other < bodyCount; other++)
                {
                    if (other == big || hasBodyAabb[other] == 0)
                        continue;

                    TryAddPair(big, other, bodyTypes);
                }
            }
        }

        private void AddPairsAgainstBodiesAlreadyInTouchedCells(int body, NativeList<BodyType> bodyTypes)
        {
            GetCellRange(bodyAabbs[body], out int3 minCell, out int3 maxCell);

            for (int z = minCell.z; z <= maxCell.z; z++)
            for (int y = minCell.y; y <= maxCell.y; y++)
            for (int x = minCell.x; x <= maxCell.x; x++)
            {
                int hash = HashCell(new int3(x, y, z));

                if (cellToBodies.TryGetFirstValue(hash, out int other, out NativeParallelMultiHashMapIterator<int> iterator))
                {
                    do
                    {
                        TryAddPair(body, other, bodyTypes);
                    }
                    while (cellToBodies.TryGetNextValue(out other, ref iterator));
                }
            }
        }

        private void InsertBodyIntoTouchedCells(int body)
        {
            GetCellRange(bodyAabbs[body], out int3 minCell, out int3 maxCell);

            for (int z = minCell.z; z <= maxCell.z; z++)
            for (int y = minCell.y; y <= maxCell.y; y++)
            for (int x = minCell.x; x <= maxCell.x; x++)
                cellToBodies.Add(HashCell(new int3(x, y, z)), body);
        }

        private void TryAddPair(int a, int b, NativeList<BodyType> bodyTypes)
        {
            if (a == b)
                return;

            int min = math.min(a, b);
            int max = math.max(a, b);

            bool minDynamic = bodyTypes[min] == BodyType.Dynamic;
            bool maxDynamic = bodyTypes[max] == BodyType.Dynamic;
            if (!minDynamic && !maxDynamic)
                return;

            if (!bodyAabbs[min].Overlaps(bodyAabbs[max]))
                return;

            ulong key = PairKey(min, max);
            if (!emittedPairKeys.Add(key))
                return;

            pairs.Add(new ShardBodyPair(min, max));
        }

        private void EnsureSpatialCapacities(int estimatedCellWrites, int estimatedPairs)
        {
            if (estimatedCellWrites > cellToBodies.Capacity)
                cellToBodies.Capacity = math.ceilpow2(estimatedCellWrites);

            if (estimatedPairs > emittedPairKeys.Capacity)
                emittedPairKeys.Capacity = math.ceilpow2(estimatedPairs);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int CountTouchedCells(Aabb aabb)
        {
            GetCellRange(aabb, out int3 minCell, out int3 maxCell);
            int3 counts = maxCell - minCell + new int3(1);

            if (counts.x <= 0 || counts.y <= 0 || counts.z <= 0)
                return 0;

            // Prevent integer overflow for pathological AABBs.
            if (counts.x > MaxCellsPerBody || counts.y > MaxCellsPerBody || counts.z > MaxCellsPerBody)
                return MaxCellsPerBody + 1;

            long total = (long)counts.x * counts.y * counts.z;
            return total > int.MaxValue ? int.MaxValue : (int)total;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void GetCellRange(Aabb aabb, out int3 minCell, out int3 maxCell)
        {
            float inv = 1f / SpatialHashCellSize;
            minCell = (int3)math.floor(aabb.min * inv);
            maxCell = (int3)math.floor(aabb.max * inv);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int HashCell(int3 cell)
        {
            unchecked
            {
                int h = cell.x * 73856093;
                h ^= cell.y * 19349663;
                h ^= cell.z * 83492791;
                return h;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ulong PairKey(int a, int b)
        {
            return ((ulong)(uint)a << 32) | (uint)b;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ShardBodyPair GetPair(int index)
        {
            return pairs[index];
        }

        private static bool ComputeBodyAabb(
            int denseBodyIndex,
            Pose bodyPose,
            ShardColliderStore colliderStore,
            ShardTriangleMeshStore triangleMeshStore,
            out Aabb bodyAabb)
        {
            bodyAabb = Aabb.Empty;
            bool hasAnyCollider = false;

            int node = colliderStore.GetHead(denseBodyIndex);

            while (node != -1)
            {
                if (!colliderStore.TryGetNode(node, out ShardCollider collider, out int next))
                    break;

                if (ComputeColliderAabb(bodyPose, collider, triangleMeshStore, out Aabb colliderAabb))
                {
                    bodyAabb.min = math.min(bodyAabb.min, colliderAabb.min);
                    bodyAabb.max = math.max(bodyAabb.max, colliderAabb.max);
                    hasAnyCollider = true;
                }

                node = next;
            }

            return hasAnyCollider;
        }

        private static bool ComputeColliderAabb(
            Pose bodyPose,
            in ShardCollider collider,
            ShardTriangleMeshStore triangleMeshStore,
            out Aabb aabb)
        {
            Pose worldPose = ComposePose(bodyPose, collider.localPose);

            switch (collider.type)
            {
                case ShardColliderType.Sphere:
                {
                    float r = math.max(0f, collider.radius);
                    aabb = FromCenterExtents(worldPose.position, new float3(r));
                    return true;
                }

                case ShardColliderType.Box:
                {
                    float3 half = math.max(collider.halfExtents, float3.zero);
                    aabb = FromOrientedBox(worldPose.position, worldPose.rotation, half);
                    return true;
                }

                case ShardColliderType.Capsule:
                case ShardColliderType.Cylinder:
                case ShardColliderType.Cone:
                {
                    float halfHeight = math.max(0f, collider.height * 0.5f);
                    float radius = math.max(0f, collider.radius);

                    aabb = FromVerticalSweptRadius(
                        worldPose.position,
                        worldPose.rotation,
                        halfHeight,
                        radius);

                    return true;
                }

                case ShardColliderType.Triangle:
                {
                    aabb = Aabb.Empty;
                    aabb.Encapsulate(TransformPoint(worldPose, collider.vertexA));
                    aabb.Encapsulate(TransformPoint(worldPose, collider.vertexB));
                    aabb.Encapsulate(TransformPoint(worldPose, collider.vertexC));
                    return true;
                }

                case ShardColliderType.Mesh:
                {
                    if (!triangleMeshStore.TryGetMeshInfo(collider.meshIndex, out ShardTriangleMeshInfo meshInfo))
                    {
                        aabb = default;
                        return false;
                    }

                    aabb = TransformLocalAabb(meshInfo.localBounds, worldPose);
                    return true;
                }

                default:
                {
                    aabb = default;
                    return false;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Aabb FromCenterExtents(float3 center, float3 extents)
        {
            return new Aabb(center - extents, center + extents);
        }

        private static Aabb FromOrientedBox(float3 center, quaternion rotation, float3 halfExtents)
        {
            float3x3 r = new float3x3(rotation);

            float3 worldExtents =
                math.abs(r.c0) * halfExtents.x +
                math.abs(r.c1) * halfExtents.y +
                math.abs(r.c2) * halfExtents.z;

            return FromCenterExtents(center, worldExtents);
        }

        private static Aabb FromVerticalSweptRadius(
            float3 center,
            quaternion rotation,
            float halfHeight,
            float radius)
        {
            float3 axis = math.mul(rotation, new float3(0f, 1f, 0f));
            float3 extents = math.abs(axis) * halfHeight + new float3(radius);

            return FromCenterExtents(center, extents);
        }

        private static Aabb TransformLocalAabb(Aabb localAabb, Pose worldPose)
        {
            float3 center = TransformPoint(worldPose, localAabb.Center);
            return FromOrientedBox(center, worldPose.rotation, localAabb.Extents);
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
    }
}
