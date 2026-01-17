using Unity.Collections;
using Unity.Mathematics;

namespace Shard
{
    public struct Triangle
    {
        public int I0, I1, I2;
    }

    public struct MeshBvhNode
    {
        public Aabb Bounds;
        public int Left;
        public int Right;
        public int TriStart;
        public int TriCount; // leaf if >0
    }

    public struct MeshColliderData
    {
        public NativeArray<float3> Vertices;     // mutable (for per-frame vertex updates)
        public NativeArray<Triangle> Triangles;  // usually stable topology
        public NativeArray<MeshBvhNode> Bvh;     // refit each frame if vertices change
        public Aabb LocalAabb;
        public uint TopologyVersion; // bump if triangles change
        public uint VertexVersion;   // bump when vertices change
    }
}
