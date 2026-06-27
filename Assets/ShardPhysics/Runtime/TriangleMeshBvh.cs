using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Shard.Runtime
{
    public struct ShardTriangleMeshBvhNode
    {
        public Aabb bounds;
        public int left;
        public int right;
        public int firstTriangle;
        public int triangleCount;

        public byte IsLeaf => triangleCount > 0 ? (byte)1 : (byte)0;
    }

    internal static class TriangleMeshBvh
    {
        private const int LeafTriangleCount = 8;

        public static void BuildForMesh(
            ref ShardTriangleMeshInfo mesh,
            ref NativeList<float3> vertices,
            ref NativeList<int> indices,
            ref NativeList<ShardTriangleMeshBvhNode> nodes,
            ref NativeList<int> triangleIndices)
        {
            int triangleStart = triangleIndices.Length;

            for (int i = 0; i < mesh.triangleCount; i++)
                triangleIndices.Add(i);

            int nodeStart = nodes.Length;
            int root = BuildNode(mesh, ref vertices, ref indices, ref nodes, ref triangleIndices, triangleStart, mesh.triangleCount);

            mesh.bvhRootNode = root;
            mesh.bvhNodeStart = nodeStart;
            mesh.bvhNodeCount = nodes.Length - nodeStart;
            mesh.bvhTriangleStart = triangleStart;
            mesh.bvhTriangleCount = mesh.triangleCount;
        }

        public static bool QueryTriangles(
            in ShardTriangleMeshInfo mesh,
            Aabb queryLocalAabb,
            ref NativeList<ShardTriangleMeshBvhNode> nodes,
            ref NativeList<int> triangleIndices,
            ref NativeList<int> results,
            ref NativeList<int> stack)
        {
            results.Clear();
            stack.Clear();

            if (mesh.alive == 0 || mesh.bvhRootNode < 0 || mesh.bvhNodeCount <= 0)
                return false;

            stack.Add(mesh.bvhRootNode);

            while (stack.Length > 0)
            {
                int last = stack.Length - 1;
                int nodeIndex = stack[last];
                stack.RemoveAtSwapBack(last);

                if ((uint)nodeIndex >= (uint)nodes.Length)
                    continue;

                ShardTriangleMeshBvhNode node = nodes[nodeIndex];

                if (!queryLocalAabb.Overlaps(node.bounds))
                    continue;

                if (node.triangleCount > 0)
                {
                    int start = node.firstTriangle;
                    int end = start + node.triangleCount;

                    for (int i = start; i < end; i++)
                        results.Add(triangleIndices[i]);

                    continue;
                }

                if (node.left >= 0)
                    stack.Add(node.left);

                if (node.right >= 0)
                    stack.Add(node.right);
            }

            return true;
        }

        private static int BuildNode(
            ShardTriangleMeshInfo mesh,
            ref NativeList<float3> vertices,
            ref NativeList<int> indices,
            ref NativeList<ShardTriangleMeshBvhNode> nodes,
            ref NativeList<int> triangleIndices,
            int firstTriangle,
            int triangleCount)
        {
            Aabb bounds = Aabb.Empty;
            Aabb centroidBounds = Aabb.Empty;

            for (int i = 0; i < triangleCount; i++)
            {
                int triangleIndex = triangleIndices[firstTriangle + i];
                ShardTriangle tri = GetTriangleUnchecked(mesh, ref vertices, ref indices, triangleIndex);

                bounds.Encapsulate(tri.a);
                bounds.Encapsulate(tri.b);
                bounds.Encapsulate(tri.c);
                centroidBounds.Encapsulate((tri.a + tri.b + tri.c) * (1f / 3f));
            }

            int nodeIndex = nodes.Length;
            nodes.Add(new ShardTriangleMeshBvhNode
            {
                bounds = bounds,
                left = -1,
                right = -1,
                firstTriangle = firstTriangle,
                triangleCount = triangleCount
            });

            if (triangleCount <= LeafTriangleCount)
                return nodeIndex;

            float3 centroidExtents = centroidBounds.max - centroidBounds.min;
            int axis = 0;

            if (centroidExtents.y > centroidExtents.x && centroidExtents.y >= centroidExtents.z)
                axis = 1;
            else if (centroidExtents.z > centroidExtents.x && centroidExtents.z > centroidExtents.y)
                axis = 2;

            if (GetAxis(centroidExtents, axis) <= 1e-6f)
                return nodeIndex;

            SortTriangleRangeByCentroid(mesh, ref vertices, ref indices, ref triangleIndices, firstTriangle, triangleCount, axis);

            int leftCount = triangleCount / 2;
            int rightCount = triangleCount - leftCount;

            int left = BuildNode(mesh, ref vertices, ref indices, ref nodes, ref triangleIndices, firstTriangle, leftCount);
            int right = BuildNode(mesh, ref vertices, ref indices, ref nodes, ref triangleIndices, firstTriangle + leftCount, rightCount);

            nodes[nodeIndex] = new ShardTriangleMeshBvhNode
            {
                bounds = bounds,
                left = left,
                right = right,
                firstTriangle = -1,
                triangleCount = 0
            };

            return nodeIndex;
        }

        private static void SortTriangleRangeByCentroid(
            ShardTriangleMeshInfo mesh,
            ref NativeList<float3> vertices,
            ref NativeList<int> indices,
            ref NativeList<int> triangleIndices,
            int firstTriangle,
            int triangleCount,
            int axis)
        {
            QuickSortTriangleRangeByCentroid(
                mesh,
                ref vertices,
                ref indices,
                ref triangleIndices,
                firstTriangle,
                firstTriangle + triangleCount - 1,
                axis);
        }

        private static void QuickSortTriangleRangeByCentroid(
            ShardTriangleMeshInfo mesh,
            ref NativeList<float3> vertices,
            ref NativeList<int> indices,
            ref NativeList<int> triangleIndices,
            int left,
            int right,
            int axis)
        {
            while (left < right)
            {
                int i = left;
                int j = right;
                float pivot = GetTriangleCentroidAxis(
                    mesh,
                    ref vertices,
                    ref indices,
                    triangleIndices[left + ((right - left) >> 1)],
                    axis);

                while (i <= j)
                {
                    while (GetTriangleCentroidAxis(mesh, ref vertices, ref indices, triangleIndices[i], axis) < pivot)
                        i++;

                    while (GetTriangleCentroidAxis(mesh, ref vertices, ref indices, triangleIndices[j], axis) > pivot)
                        j--;

                    if (i <= j)
                    {
                        int tmp = triangleIndices[i];
                        triangleIndices[i] = triangleIndices[j];
                        triangleIndices[j] = tmp;
                        i++;
                        j--;
                    }
                }

                if (j - left < right - i)
                {
                    if (left < j)
                        QuickSortTriangleRangeByCentroid(mesh, ref vertices, ref indices, ref triangleIndices, left, j, axis);
                    left = i;
                }
                else
                {
                    if (i < right)
                        QuickSortTriangleRangeByCentroid(mesh, ref vertices, ref indices, ref triangleIndices, i, right, axis);
                    right = j;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float GetTriangleCentroidAxis(
            ShardTriangleMeshInfo mesh,
            ref NativeList<float3> vertices,
            ref NativeList<int> indices,
            int triangleIndex,
            int axis)
        {
            ShardTriangle tri = GetTriangleUnchecked(mesh, ref vertices, ref indices, triangleIndex);
            float3 centroid = (tri.a + tri.b + tri.c) * (1f / 3f);
            return GetAxis(centroid, axis);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ShardTriangle GetTriangleUnchecked(
            ShardTriangleMeshInfo mesh,
            ref NativeList<float3> vertices,
            ref NativeList<int> indices,
            int triangleIndex)
        {
            int i = mesh.indexStart + triangleIndex * 3;

            int ia = indices[i + 0];
            int ib = indices[i + 1];
            int ic = indices[i + 2];

            return new ShardTriangle(
                vertices[ia],
                vertices[ib],
                vertices[ic]);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float GetAxis(float3 v, int axis)
        {
            if (axis == 0) return v.x;
            if (axis == 1) return v.y;
            return v.z;
        }
    }
}
