using NUnit.Framework;
using Shard.Runtime;
using Unity.Collections;
using Unity.Mathematics;

namespace Shard.Tests.Performance
{
    public class ShardTriangleMeshBvhPerformanceTests
    {
        [Test]
        public void BvhQuery_ReturnsFarFewerCandidatesThanFullTriangleScan()
        {
            const int gridSize = 64;
            int vertexCount = (gridSize + 1) * (gridSize + 1);
            int triangleCount = gridSize * gridSize * 2;

            var vertices = new NativeArray<float3>(vertexCount, Allocator.Temp);
            var indices = new NativeArray<int>(triangleCount * 3, Allocator.Temp);
            var results = new NativeList<int>(Allocator.Temp);
            var stack = new NativeList<int>(Allocator.Temp);

            var store = new ShardTriangleMeshStore(
                initialVertexCapacity: vertexCount,
                initialIndexCapacity: triangleCount * 3,
                initialMeshCapacity: 1,
                allocator: Allocator.Persistent);

            try
            {
                FillFlatGrid(vertices, indices, gridSize);

                ShardTriangleMeshHandle mesh = store.AddMesh(vertices, indices);
                Assert.IsTrue(mesh.IsValid);
                Assert.IsTrue(store.TryGetMeshInfo(mesh, out ShardTriangleMeshInfo info));
                Assert.AreEqual(triangleCount, info.triangleCount);
                Assert.Greater(info.bvhNodeCount, 0);

                Aabb query = new Aabb(
                    new float3(31.75f, -0.25f, 31.75f),
                    new float3(32.25f,  0.25f, 32.25f));

                ShardTriangleMeshQueryStats stats = default;
                bool usedBvh = store.QueryTriangles(mesh, query, results, stack, ref stats);

                Assert.IsTrue(usedBvh);
                Assert.Greater(stats.nodesVisited, 0);
                Assert.Greater(stats.leafNodesVisited, 0);
                Assert.Greater(stats.triangleCandidatesReturned, 0);
                Assert.AreEqual(results.Length, stats.triangleCandidatesReturned);

                // This is intentionally loose so median-split/leaf-size tuning can change
                // without breaking the test. The important regression check is that we are
                // no longer returning the entire mesh for a tiny local query.
                Assert.Less(results.Length, triangleCount / 8);
                Assert.Less(stats.nodesVisited, info.bvhNodeCount);
            }
            finally
            {
                if (store.IsCreated) store.Dispose();
                if (vertices.IsCreated) vertices.Dispose();
                if (indices.IsCreated) indices.Dispose();
                if (results.IsCreated) results.Dispose();
                if (stack.IsCreated) stack.Dispose();
            }
        }

        [Test]
        public void PhysicsWorld_RecordsMeshBvhCountersDuringSimulation()
        {
            const int gridSize = 64;
            int vertexCount = (gridSize + 1) * (gridSize + 1);
            int triangleCount = gridSize * gridSize * 2;

            var vertices = new NativeArray<float3>(vertexCount, Allocator.Temp);
            var indices = new NativeArray<int>(triangleCount * 3, Allocator.Temp);
            var meshColliders = new NativeArray<ShardCollider>(1, Allocator.Temp);
            var sphereColliders = new NativeArray<ShardCollider>(1, Allocator.Temp);

            var world = new ShardPhysicsWorld();

            try
            {
                world.gravity = float3.zero;
                FillFlatGrid(vertices, indices, gridSize);

                ShardTriangleMeshHandle mesh = world.AddTriangleMesh(vertices, indices);
                Assert.IsTrue(mesh.IsValid);

                meshColliders[0] = ShardCreateBody.CreateTriangleMesh(mesh);
                world.CreateBody(
                    new Pose { position = float3.zero, rotation = quaternion.identity },
                    BodyType.Static,
                    0f,
                    meshColliders);

                sphereColliders[0] = ShardCreateBody.CreateSphere(radius: 0.5f);
                world.CreateBody(
                    new Pose { position = new float3(32f, 0.35f, 32f), rotation = quaternion.identity },
                    BodyType.Dynamic,
                    1f,
                    sphereColliders);

                world.Simulate(1f / 60f, substeps: 1, collisionIterations: 1);

                ShardPhysicsPerformanceStats perf = world.LastPerformanceStats;

                Assert.Greater(perf.bodyPairsFromBroadphase, 0);
                Assert.Greater(perf.meshBvhQueries, 0);
                Assert.AreEqual(0, perf.meshFullScanFallbacks);
                Assert.Greater(perf.meshBvhNodesVisited, 0);
                Assert.Greater(perf.meshBvhCandidateTrianglesReturned, 0);
                Assert.Greater(perf.meshTriangleAabbTests, 0);
                Assert.Less(perf.meshTriangleAabbTests, triangleCount / 8);
            }
            finally
            {
                world.Dispose();
                if (vertices.IsCreated) vertices.Dispose();
                if (indices.IsCreated) indices.Dispose();
                if (meshColliders.IsCreated) meshColliders.Dispose();
                if (sphereColliders.IsCreated) sphereColliders.Dispose();
            }
        }

        private static void FillFlatGrid(
            NativeArray<float3> vertices,
            NativeArray<int> indices,
            int gridSize)
        {
            int v = 0;
            for (int z = 0; z <= gridSize; z++)
            {
                for (int x = 0; x <= gridSize; x++)
                {
                    vertices[v++] = new float3(x, 0f, z);
                }
            }

            int i = 0;
            int stride = gridSize + 1;

            for (int z = 0; z < gridSize; z++)
            {
                for (int x = 0; x < gridSize; x++)
                {
                    int v00 = z * stride + x;
                    int v10 = v00 + 1;
                    int v01 = v00 + stride;
                    int v11 = v01 + 1;

                    indices[i++] = v00;
                    indices[i++] = v01;
                    indices[i++] = v10;

                    indices[i++] = v10;
                    indices[i++] = v01;
                    indices[i++] = v11;
                }
            }
        }
    }
}
