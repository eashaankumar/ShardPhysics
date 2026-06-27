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

        public float3 vertexA;
        public float3 vertexB;
        public float3 vertexC;
        public int meshIndex;
    }
}