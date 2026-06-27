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
        private NativeList<Aabb> _bodyAabbs;
        private NativeList<byte> _hasBodyAabb;
        private NativeList<ShardBodyPair> _pairs;

        public NativeArray<ShardBodyPair>.ReadOnly Pairs => _pairs.AsArray().AsReadOnly();
        public int PairCount => _pairs.Length;

        public Broadphase(int initialBodyCapacity, Allocator allocator)
        {
            _bodyAabbs = new NativeList<Aabb>(initialBodyCapacity, allocator);
            _hasBodyAabb = new NativeList<byte>(initialBodyCapacity, allocator);
            _pairs = new NativeList<ShardBodyPair>(math.max(16, initialBodyCapacity * 2), allocator);
        }

        public void Dispose()
        {
            if (_bodyAabbs.IsCreated) _bodyAabbs.Dispose();
            if (_hasBodyAabb.IsCreated) _hasBodyAabb.Dispose();
            if (_pairs.IsCreated) _pairs.Dispose();
        }

        public void Rebuild(
            NativeList<Pose> poses,
            NativeList<BodyType> bodyTypes,
            ShardColliderStore colliderStore,
            ShardTriangleMeshStore triangleMeshStore)
        {
            int bodyCount = poses.Length;

            _bodyAabbs.ResizeUninitialized(bodyCount);
            _hasBodyAabb.ResizeUninitialized(bodyCount);
            _pairs.Clear();

            for (int i = 0; i < bodyCount; i++)
            {
                if (ComputeBodyAabb(i, poses[i], colliderStore, triangleMeshStore, out Aabb aabb))
                {
                    _bodyAabbs[i] = aabb;
                    _hasBodyAabb[i] = 1;
                }
                else
                {
                    _bodyAabbs[i] = default;
                    _hasBodyAabb[i] = 0;
                }
            }

            for (int a = 0; a < bodyCount; a++)
            {
                if (_hasBodyAabb[a] == 0)
                    continue;

                bool aDyn = bodyTypes[a] == BodyType.Dynamic;

                for (int b = a + 1; b < bodyCount; b++)
                {
                    if (_hasBodyAabb[b] == 0)
                        continue;

                    bool bDyn = bodyTypes[b] == BodyType.Dynamic;
                    if (!aDyn && !bDyn)
                        continue;

                    if (_bodyAabbs[a].Overlaps(_bodyAabbs[b]))
                        _pairs.Add(new ShardBodyPair(a, b));
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ShardBodyPair GetPair(int index)
        {
            return _pairs[index];
        }

        private static bool ComputeBodyAabb(
            int denseBodyIndex,
            Pose bodyPose,
            ShardColliderStore colliderStore,
            ShardTriangleMeshStore triangleMeshStore,
            out Aabb bodyAabb)
        {
            bodyAabb = Aabb.Empty;
            bool hasAny = false;

            int node = colliderStore.GetHead(denseBodyIndex);
            while (node != -1)
            {
                if (!colliderStore.TryGetNode(node, out ShardCollider collider, out int next))
                    break;

                if (ComputeColliderAabb(bodyPose, collider, triangleMeshStore, out Aabb colliderAabb))
                {
                    bodyAabb.min = math.min(bodyAabb.min, colliderAabb.min);
                    bodyAabb.max = math.max(bodyAabb.max, colliderAabb.max);
                    hasAny = true;
                }

                node = next;
            }

            return hasAny;
        }

        private static bool ComputeColliderAabb(
            Pose bodyPose,
            in ShardCollider collider,
            ShardTriangleMeshStore triangleMeshStore,
            out Aabb aabb)
        {
            Pose worldPose = ComposePoseSafe(bodyPose, collider.localPose);

            switch (collider.type)
            {
                case ShardColliderType.Sphere:
                    aabb = FromCenterExtents(worldPose.position, new float3(math.max(0f, collider.radius)));
                    return true;

                case ShardColliderType.Box:
                    aabb = FromOrientedBox(worldPose.position, worldPose.rotation, math.max(collider.halfExtents, float3.zero));
                    return true;

                case ShardColliderType.Capsule:
                case ShardColliderType.Cylinder:
                case ShardColliderType.Cone:
                    aabb = FromVerticalSweptRadius(
                        worldPose.position,
                        worldPose.rotation,
                        math.max(0f, collider.height * 0.5f),
                        math.max(0f, collider.radius));
                    return true;

                case ShardColliderType.Triangle:
                    aabb = Aabb.Empty;
                    aabb.Encapsulate(TransformPoint(worldPose, collider.vertexA));
                    aabb.Encapsulate(TransformPoint(worldPose, collider.vertexB));
                    aabb.Encapsulate(TransformPoint(worldPose, collider.vertexC));
                    return true;

                case ShardColliderType.TriangleMesh:
                    if (!triangleMeshStore.TryGetMeshInfo(collider.meshIndex, out ShardTriangleMeshInfo meshInfo))
                    {
                        aabb = default;
                        return false;
                    }

                    aabb = TransformAabb(meshInfo.localBounds, worldPose);
                    return true;

                default:
                    aabb = default;
                    return false;
            }
        }

        private static Aabb FromCenterExtents(float3 center, float3 extents)
        {
            return new Aabb(center - extents, center + extents);
        }

        private static Aabb FromOrientedBox(float3 center, quaternion rotation, float3 halfExtents)
        {
            float3x3 r = new float3x3(SanitizeRotation(rotation));

            float3 worldExtents =
                math.abs(r.c0) * halfExtents.x +
                math.abs(r.c1) * halfExtents.y +
                math.abs(r.c2) * halfExtents.z;

            return FromCenterExtents(center, worldExtents);
        }

        private static Aabb FromVerticalSweptRadius(float3 center, quaternion rotation, float halfHeight, float radius)
        {
            quaternion q = SanitizeRotation(rotation);
            float3 axis = math.mul(q, new float3(0f, 1f, 0f));
            float3 extents = math.abs(axis) * halfHeight + new float3(radius);

            return FromCenterExtents(center, extents);
        }

        private static Aabb TransformAabb(Aabb localAabb, Pose pose)
        {
            float3 worldCenter = TransformPoint(pose, localAabb.Center);
            return FromOrientedBox(worldCenter, pose.rotation, localAabb.Extents);
        }

        private static Pose ComposePoseSafe(Pose bodyPose, Pose localPose)
        {
            quaternion bodyRot = SanitizeRotation(bodyPose.rotation);
            quaternion localRot = SanitizeRotation(localPose.rotation);

            return new Pose
            {
                position = bodyPose.position + math.mul(bodyRot, localPose.position),
                rotation = math.mul(bodyRot, localRot)
            };
        }

        private static float3 TransformPoint(Pose pose, float3 localPoint)
        {
            return pose.position + math.mul(SanitizeRotation(pose.rotation), localPoint);
        }

        private static quaternion SanitizeRotation(quaternion q)
        {
            float lenSq = math.lengthsq(q.value);
            return lenSq > 1e-12f ? math.normalize(q) : quaternion.identity;
        }
    }
}