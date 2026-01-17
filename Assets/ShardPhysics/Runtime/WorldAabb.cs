using Unity.Mathematics;

namespace Shard
{
    internal static class WorldAabb
    {
        /// <summary>
        /// Transforms a collider's local-space AABB into world space using the body's pose.
        /// Assumes the collider's local AABB already includes collider Center/Orientation.
        /// </summary>
        public static Aabb FromBodyPose(in Aabb localAabb, in Pose bodyPose)
        {
            float3 localCenter = localAabb.Center;
            float3 localExt = localAabb.Extents;

            float3x3 R = new float3x3(bodyPose.Rotation);

            // Conservative extents under rotation
            float3 ax = math.abs(R.c0) * localExt.x;
            float3 ay = math.abs(R.c1) * localExt.y;
            float3 az = math.abs(R.c2) * localExt.z;
            float3 worldExt = ax + ay + az;

            float3 worldCenter = bodyPose.Position + math.mul(bodyPose.Rotation, localCenter);
            return Aabb.FromCenterExtents(worldCenter, worldExt);
        }
    }
}
