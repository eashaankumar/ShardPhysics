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

        public int PairCount => pairs.Length;

        public Broadphase(int initialBodyCapacity, Allocator allocator)
        {
            bodyAabbs = new NativeList<Aabb>(initialBodyCapacity, allocator);
            hasBodyAabb = new NativeList<byte>(initialBodyCapacity, allocator);
            pairs = new NativeList<ShardBodyPair>(math.max(16, initialBodyCapacity * 2), allocator);
        }

        public void Dispose()
        {
            if (bodyAabbs.IsCreated) bodyAabbs.Dispose();
            if (hasBodyAabb.IsCreated) hasBodyAabb.Dispose();
            if (pairs.IsCreated) pairs.Dispose();
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

            for (int i = 0; i < bodyCount; i++)
            {
                if (ComputeBodyAabb(i, poses[i], colliderStore, triangleMeshStore, out Aabb aabb))
                {
                    bodyAabbs[i] = aabb;
                    hasBodyAabb[i] = 1;
                }
                else
                {
                    bodyAabbs[i] = default;
                    hasBodyAabb[i] = 0;
                }
            }

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