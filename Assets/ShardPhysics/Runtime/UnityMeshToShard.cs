using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Shard.Runtime
{
    public static class UnityMeshToShard
    {
        public static ShardTriangleMeshHandle AddTriangleMesh(
            ShardPhysicsWorld world,
            NativeArray<float3> vertices,
            NativeArray<int> indices)
        {
            if (world == null)
                return ShardTriangleMeshHandle.Invalid;

            if (!vertices.IsCreated || !indices.IsCreated)
                return ShardTriangleMeshHandle.Invalid;

            return world.AddTriangleMesh(vertices, indices);
        }

        public static ShardTriangleMeshHandle AddUnityMesh(
            ShardPhysicsWorld world,
            Mesh mesh,
            Allocator tempAllocator = Allocator.Temp)
        {
            if (world == null || mesh == null)
                return ShardTriangleMeshHandle.Invalid;

            Vector3[] unityVertices = mesh.vertices;
            int[] unityTriangles = mesh.triangles;

            var vertices = new NativeArray<float3>(unityVertices.Length, tempAllocator);
            var indices = new NativeArray<int>(unityTriangles.Length, tempAllocator);

            for (int i = 0; i < unityVertices.Length; i++)
                vertices[i] = unityVertices[i];

            for (int i = 0; i < unityTriangles.Length; i++)
                indices[i] = unityTriangles[i];

            ShardTriangleMeshHandle handle = world.AddTriangleMesh(vertices, indices);

            vertices.Dispose();
            indices.Dispose();

            return handle;
        }

        public static ShardCollider CreateTriangleMeshCollider(
            ShardTriangleMeshHandle meshHandle,
            Pose localPose = default,
            ShardColliderMaterial material = default)
        {
            return ShardCreateBody.CreateTriangleMesh(
                meshHandle,
                density: 0f,
                localPose: localPose,
                material: material);
        }

        public static ShardCollider CreateUnityMeshCollider(
            ShardPhysicsWorld world,
            Mesh mesh,
            Pose localPose = default,
            ShardColliderMaterial material = default,
            Allocator tempAllocator = Allocator.Temp)
        {
            ShardTriangleMeshHandle meshHandle = AddUnityMesh(world, mesh, tempAllocator);

            return CreateTriangleMeshCollider(
                meshHandle,
                localPose,
                material);
        }
    }
}