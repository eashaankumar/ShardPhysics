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
        Voxel
        // later: Convex, Mesh, Voxel, Heightfield...
    }

    public struct ShardCollider
    {
        public ShardColliderType type;

        public Pose localPose;     // collider pose relative to body frame (body origin)
        public float density;      // optional; OR use body mass distribution later

        // Shape params (use only what type needs)
        public float radius;       // sphere/capsule
        public float3 halfExtents; // box
        public float height;       // capsule (cylinder height, excluding hemispheres, define your convention)
    }
}
