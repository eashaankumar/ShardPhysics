using Unity.Mathematics;

namespace Shard
{
    public struct SphereCollider
    {
        public float Radius;
        public float3 Center; // local
    }

    public struct CapsuleCollider
    {
        public float Radius;
        public float HalfHeight; // along local Y (starter)
        public float3 Center;    // local
    }

    public struct BoxCollider
    {
        public float3 HalfExtents;
        public float3 Center; // local
        public quaternion Orientation; // local box rotation
    }

    public struct ConeCollider
    {
        public float BaseRadius;      // radius at y = -HalfHeight
        public float HalfHeight;      // half of cone height (height = 2*HalfHeight)
        public float RoundingRadius;  // “fillet” radius: rounds tip + edges (Minkowski inflate)
        public float3 Center;         // local center offset
        public quaternion Orientation;// local orientation (optional; can also keep axis fixed)
    }

    public struct CylinderCollider
    {
        public float Radius;          // cylinder radius (before rounding)
        public float HalfHeight;      // half height
        public float RoundingRadius;  // rounds sharp edges (Minkowski inflate, but we’ll treat ends as flat in narrowphase)
        public float3 Center;         // local center offset
        public quaternion Orientation;// local orientation
    }
}
