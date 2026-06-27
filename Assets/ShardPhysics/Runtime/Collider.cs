using Unity.Mathematics;
using UnityEngine;

namespace Shard.Runtime
{
    public enum ShardColliderType : byte
    {
        Sphere,
        Box,
        Capsule,
        Cylinder,
        Cone,
        Mesh,
        Triangle,
        TriangleMesh,
        Voxel
    }

    public struct ShardColliderMaterial
    {
        public float bounciness;
        public float frictionStatic;
        public float frictionDynamic;
        public float frictionRolling;
    }

    public struct ShardCollider
    {
        public ShardColliderType type;
        public ShardColliderMaterial material;

        public Pose localPose;
        public float density;

        public float radius;
        public float3 halfExtents;
        public float height;

        // Used only by single Triangle collider.
        public float3 vertexA;
        public float3 vertexB;
        public float3 vertexC;

        // Used only by TriangleMesh collider.
        // This is an index into ShardTriangleMeshStore.
        public int meshIndex;
    }
}