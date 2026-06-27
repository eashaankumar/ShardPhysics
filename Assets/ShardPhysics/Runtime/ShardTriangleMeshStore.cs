using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Shard.Runtime
{
    public readonly struct ShardTriangleMeshHandle
    {
        public readonly int value;

        internal ShardTriangleMeshHandle(int value)
        {
            this.value = value;
        }

        public bool IsValid => value >= 0;

        public static ShardTriangleMeshHandle Invalid => new ShardTriangleMeshHandle(-1);
    }

    public struct ShardTriangleMeshInfo
    {
        public int vertexStart;
        public int vertexCount;

        public int indexStart;
        public int indexCount;

        public int triangleCount;

        public int bvhRootNode;
        public int bvhNodeStart;
        public int bvhNodeCount;
        public int bvhTriangleStart;
        public int bvhTriangleCount;

        public Aabb localBounds;

        public byte alive;
    }

    public struct ShardTriangle
    {
        public float3 a;
        public float3 b;
        public float3 c;

        public ShardTriangle(float3 a, float3 b, float3 c)
        {
            this.a = a;
            this.b = b;
            this.c = c;
        }
    }

    public struct Aabb
    {
        public float3 min;
        public float3 max;

        public Aabb(float3 min, float3 max)
        {
            this.min = min;
            this.max = max;
        }

        public float3 Center => (min + max) * 0.5f;
        public float3 Extents => (max - min) * 0.5f;

        public static Aabb Empty => new Aabb(
            new float3(float.PositiveInfinity),
            new float3(float.NegativeInfinity));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Encapsulate(float3 p)
        {
            min = math.min(min, p);
            max = math.max(max, p);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Overlaps(Aabb other)
        {
            return min.x <= other.max.x && max.x >= other.min.x &&
                   min.y <= other.max.y && max.y >= other.min.y &&
                   min.z <= other.max.z && max.z >= other.min.z;
        }
    }

    public sealed class ShardTriangleMeshStore : IDisposable
    {
        private NativeList<float3> _vertices;
        private NativeList<int> _indices;
        private NativeList<ShardTriangleMeshInfo> _meshes;
        private NativeList<int> _freeMeshes;
        private NativeList<ShardTriangleMeshBvhNode> _bvhNodes;
        private NativeList<int> _bvhTriangleIndices;

        public ShardTriangleMeshStore(
            int initialVertexCapacity,
            int initialIndexCapacity,
            int initialMeshCapacity,
            Allocator allocator)
        {
            _vertices = new NativeList<float3>(initialVertexCapacity, allocator);
            _indices = new NativeList<int>(initialIndexCapacity, allocator);
            _meshes = new NativeList<ShardTriangleMeshInfo>(initialMeshCapacity, allocator);
            _freeMeshes = new NativeList<int>(initialMeshCapacity, allocator);
            _bvhNodes = new NativeList<ShardTriangleMeshBvhNode>(math.max(64, initialIndexCapacity / 3), allocator);
            _bvhTriangleIndices = new NativeList<int>(math.max(64, initialIndexCapacity / 3), allocator);
        }

        public bool IsCreated =>
            _vertices.IsCreated &&
            _indices.IsCreated &&
            _meshes.IsCreated &&
            _freeMeshes.IsCreated &&
            _bvhNodes.IsCreated &&
            _bvhTriangleIndices.IsCreated;

        public int MeshCount => _meshes.IsCreated ? _meshes.Length : 0;
        public int VertexCount => _vertices.IsCreated ? _vertices.Length : 0;
        public int IndexCount => _indices.IsCreated ? _indices.Length : 0;
        public int BvhNodeCount => _bvhNodes.IsCreated ? _bvhNodes.Length : 0;
        public int BvhTriangleIndexCount => _bvhTriangleIndices.IsCreated ? _bvhTriangleIndices.Length : 0;

        public NativeArray<float3>.ReadOnly Vertices => _vertices.AsArray().AsReadOnly();
        public NativeArray<int>.ReadOnly Indices => _indices.AsArray().AsReadOnly();
        public NativeArray<ShardTriangleMeshInfo>.ReadOnly Meshes => _meshes.AsArray().AsReadOnly();

        public void Dispose()
        {
            if (_vertices.IsCreated) _vertices.Dispose();
            if (_indices.IsCreated) _indices.Dispose();
            if (_meshes.IsCreated) _meshes.Dispose();
            if (_freeMeshes.IsCreated) _freeMeshes.Dispose();
            if (_bvhNodes.IsCreated) _bvhNodes.Dispose();
            if (_bvhTriangleIndices.IsCreated) _bvhTriangleIndices.Dispose();
        }

        public ShardTriangleMeshHandle AddMesh(
            NativeArray<float3> vertices,
            NativeArray<int> indices)
        {
            if (!vertices.IsCreated || vertices.Length == 0)
                return ShardTriangleMeshHandle.Invalid;

            if (!indices.IsCreated || indices.Length < 3 || indices.Length % 3 != 0)
                return ShardTriangleMeshHandle.Invalid;

            int meshIndex = AllocateMeshIndex();

            int vertexStart = _vertices.Length;
            int indexStart = _indices.Length;

            Aabb bounds = Aabb.Empty;

            for (int i = 0; i < vertices.Length; i++)
            {
                float3 v = vertices[i];
                _vertices.Add(v);
                bounds.Encapsulate(v);
            }

            for (int i = 0; i < indices.Length; i++)
            {
                int srcIndex = indices[i];

                if ((uint)srcIndex >= (uint)vertices.Length)
                    srcIndex = 0;

                _indices.Add(vertexStart + srcIndex);
            }

            _meshes[meshIndex] = new ShardTriangleMeshInfo
            {
                vertexStart = vertexStart,
                vertexCount = vertices.Length,
                indexStart = indexStart,
                indexCount = indices.Length,
                triangleCount = indices.Length / 3,
                bvhRootNode = -1,
                bvhNodeStart = _bvhNodes.Length,
                bvhNodeCount = 0,
                bvhTriangleStart = _bvhTriangleIndices.Length,
                bvhTriangleCount = 0,
                localBounds = bounds,
                alive = 1
            };

            BuildBvhForMesh(meshIndex);

            return new ShardTriangleMeshHandle(meshIndex);
        }

        public ShardTriangleMeshHandle AddMesh(
            ReadOnlySpan<float3> vertices,
            ReadOnlySpan<int> indices)
        {
            if (vertices.Length == 0)
                return ShardTriangleMeshHandle.Invalid;

            if (indices.Length < 3 || indices.Length % 3 != 0)
                return ShardTriangleMeshHandle.Invalid;

            int meshIndex = AllocateMeshIndex();

            int vertexStart = _vertices.Length;
            int indexStart = _indices.Length;

            Aabb bounds = Aabb.Empty;

            for (int i = 0; i < vertices.Length; i++)
            {
                float3 v = vertices[i];
                _vertices.Add(v);
                bounds.Encapsulate(v);
            }

            for (int i = 0; i < indices.Length; i++)
            {
                int srcIndex = indices[i];

                if ((uint)srcIndex >= (uint)vertices.Length)
                    srcIndex = 0;

                _indices.Add(vertexStart + srcIndex);
            }

            _meshes[meshIndex] = new ShardTriangleMeshInfo
            {
                vertexStart = vertexStart,
                vertexCount = vertices.Length,
                indexStart = indexStart,
                indexCount = indices.Length,
                triangleCount = indices.Length / 3,
                bvhRootNode = -1,
                bvhNodeStart = _bvhNodes.Length,
                bvhNodeCount = 0,
                bvhTriangleStart = _bvhTriangleIndices.Length,
                bvhTriangleCount = 0,
                localBounds = bounds,
                alive = 1
            };

            BuildBvhForMesh(meshIndex);

            return new ShardTriangleMeshHandle(meshIndex);
        }

        public bool RemoveMesh(ShardTriangleMeshHandle handle)
        {
            int meshIndex = handle.value;

            if ((uint)meshIndex >= (uint)_meshes.Length)
                return false;

            ShardTriangleMeshInfo info = _meshes[meshIndex];

            if (info.alive == 0)
                return false;

            info.alive = 0;
            _meshes[meshIndex] = info;
            _freeMeshes.Add(meshIndex);

            // NativeMemory style note:
            // We intentionally do not compact vertices/indices here because existing mesh handles
            // and collider meshIndex references must stay stable.
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetMeshInfo(ShardTriangleMeshHandle handle, out ShardTriangleMeshInfo info)
        {
            int meshIndex = handle.value;

            if ((uint)meshIndex >= (uint)_meshes.Length)
            {
                info = default;
                return false;
            }

            info = _meshes[meshIndex];
            return info.alive != 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetMeshInfo(int meshIndex, out ShardTriangleMeshInfo info)
        {
            if ((uint)meshIndex >= (uint)_meshes.Length)
            {
                info = default;
                return false;
            }

            info = _meshes[meshIndex];
            return info.alive != 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetTriangle(
            ShardTriangleMeshHandle handle,
            int triangleIndex,
            out ShardTriangle triangle)
        {
            return TryGetTriangle(handle.value, triangleIndex, out triangle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetTriangle(
            int meshIndex,
            int triangleIndex,
            out ShardTriangle triangle)
        {
            triangle = default;

            if (!TryGetMeshInfo(meshIndex, out ShardTriangleMeshInfo mesh))
                return false;

            if ((uint)triangleIndex >= (uint)mesh.triangleCount)
                return false;

            int i = mesh.indexStart + triangleIndex * 3;

            int ia = _indices[i + 0];
            int ib = _indices[i + 1];
            int ic = _indices[i + 2];

            triangle = new ShardTriangle(
                _vertices[ia],
                _vertices[ib],
                _vertices[ic]);

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public NativeArray<float3>.ReadOnly GetVertexArrayReadOnly()
        {
            return _vertices.AsArray().AsReadOnly();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public NativeArray<int>.ReadOnly GetIndexArrayReadOnly()
        {
            return _indices.AsArray().AsReadOnly();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public NativeArray<ShardTriangleMeshInfo>.ReadOnly GetMeshInfoArrayReadOnly()
        {
            return _meshes.AsArray().AsReadOnly();
        }

        public bool QueryTriangles(
            int meshIndex,
            Aabb queryLocalAabb,
            NativeList<int> results,
            NativeList<int> stack)
        {
            results.Clear();
            stack.Clear();

            if (!TryGetMeshInfo(meshIndex, out ShardTriangleMeshInfo mesh))
                return false;

            if (mesh.bvhRootNode < 0 || mesh.bvhNodeCount <= 0)
                return false;

            stack.Add(mesh.bvhRootNode);

            while (stack.Length > 0)
            {
                int last = stack.Length - 1;
                int nodeIndex = stack[last];
                stack.RemoveAtSwapBack(last);

                if ((uint)nodeIndex >= (uint)_bvhNodes.Length)
                    continue;

                ShardTriangleMeshBvhNode node = _bvhNodes[nodeIndex];

                if (!queryLocalAabb.Overlaps(node.bounds))
                    continue;

                if (node.triangleCount > 0)
                {
                    int start = node.firstTriangle;
                    int end = start + node.triangleCount;

                    for (int i = start; i < end; i++)
                        results.Add(_bvhTriangleIndices[i]);

                    continue;
                }

                if (node.left >= 0)
                    stack.Add(node.left);

                if (node.right >= 0)
                    stack.Add(node.right);
            }

            return true;
        }

        public bool QueryTriangles(
            ShardTriangleMeshHandle handle,
            Aabb queryLocalAabb,
            NativeList<int> results,
            NativeList<int> stack)
        {
            return QueryTriangles(handle.value, queryLocalAabb, results, stack);
        }

        private void BuildBvhForMesh(int meshIndex)
        {
            if (!TryGetMeshInfo(meshIndex, out ShardTriangleMeshInfo mesh))
                return;

            int triangleStart = _bvhTriangleIndices.Length;

            for (int i = 0; i < mesh.triangleCount; i++)
                _bvhTriangleIndices.Add(i);

            int nodeStart = _bvhNodes.Length;
            int root = BuildBvhNode(mesh, triangleStart, mesh.triangleCount);

            mesh.bvhRootNode = root;
            mesh.bvhNodeStart = nodeStart;
            mesh.bvhNodeCount = _bvhNodes.Length - nodeStart;
            mesh.bvhTriangleStart = triangleStart;
            mesh.bvhTriangleCount = mesh.triangleCount;
            _meshes[meshIndex] = mesh;
        }

        private int BuildBvhNode(ShardTriangleMeshInfo mesh, int firstTriangle, int triangleCount)
        {
            const int LeafTriangleCount = 8;

            Aabb bounds = Aabb.Empty;
            Aabb centroidBounds = Aabb.Empty;

            for (int i = 0; i < triangleCount; i++)
            {
                int triangleIndex = _bvhTriangleIndices[firstTriangle + i];
                ShardTriangle tri = GetTriangleUnchecked(mesh, triangleIndex);

                bounds.Encapsulate(tri.a);
                bounds.Encapsulate(tri.b);
                bounds.Encapsulate(tri.c);
                centroidBounds.Encapsulate((tri.a + tri.b + tri.c) * (1f / 3f));
            }

            int nodeIndex = _bvhNodes.Length;
            _bvhNodes.Add(new ShardTriangleMeshBvhNode
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

            SortTriangleRangeByCentroid(mesh, firstTriangle, triangleCount, axis);

            int leftCount = triangleCount / 2;
            int rightCount = triangleCount - leftCount;

            int left = BuildBvhNode(mesh, firstTriangle, leftCount);
            int right = BuildBvhNode(mesh, firstTriangle + leftCount, rightCount);

            _bvhNodes[nodeIndex] = new ShardTriangleMeshBvhNode
            {
                bounds = bounds,
                left = left,
                right = right,
                firstTriangle = -1,
                triangleCount = 0
            };

            return nodeIndex;
        }

        private void SortTriangleRangeByCentroid(
            ShardTriangleMeshInfo mesh,
            int firstTriangle,
            int triangleCount,
            int axis)
        {
            QuickSortTriangleRangeByCentroid(mesh, firstTriangle, firstTriangle + triangleCount - 1, axis);
        }

        private void QuickSortTriangleRangeByCentroid(
            ShardTriangleMeshInfo mesh,
            int left,
            int right,
            int axis)
        {
            while (left < right)
            {
                int i = left;
                int j = right;
                float pivot = GetTriangleCentroidAxis(mesh, _bvhTriangleIndices[left + ((right - left) >> 1)], axis);

                while (i <= j)
                {
                    while (GetTriangleCentroidAxis(mesh, _bvhTriangleIndices[i], axis) < pivot)
                        i++;

                    while (GetTriangleCentroidAxis(mesh, _bvhTriangleIndices[j], axis) > pivot)
                        j--;

                    if (i <= j)
                    {
                        int tmp = _bvhTriangleIndices[i];
                        _bvhTriangleIndices[i] = _bvhTriangleIndices[j];
                        _bvhTriangleIndices[j] = tmp;
                        i++;
                        j--;
                    }
                }

                if (j - left < right - i)
                {
                    if (left < j)
                        QuickSortTriangleRangeByCentroid(mesh, left, j, axis);
                    left = i;
                }
                else
                {
                    if (i < right)
                        QuickSortTriangleRangeByCentroid(mesh, i, right, axis);
                    right = j;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private float GetTriangleCentroidAxis(ShardTriangleMeshInfo mesh, int triangleIndex, int axis)
        {
            ShardTriangle tri = GetTriangleUnchecked(mesh, triangleIndex);
            float3 centroid = (tri.a + tri.b + tri.c) * (1f / 3f);
            return GetAxis(centroid, axis);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private ShardTriangle GetTriangleUnchecked(ShardTriangleMeshInfo mesh, int triangleIndex)
        {
            int i = mesh.indexStart + triangleIndex * 3;

            int ia = _indices[i + 0];
            int ib = _indices[i + 1];
            int ic = _indices[i + 2];

            return new ShardTriangle(
                _vertices[ia],
                _vertices[ib],
                _vertices[ic]);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float GetAxis(float3 v, int axis)
        {
            if (axis == 0) return v.x;
            if (axis == 1) return v.y;
            return v.z;
        }

        private int AllocateMeshIndex()
        {
            if (_freeMeshes.Length > 0)
            {
                int last = _freeMeshes.Length - 1;
                int meshIndex = _freeMeshes[last];
                _freeMeshes.RemoveAtSwapBack(last);
                return meshIndex;
            }

            int newIndex = _meshes.Length;
            _meshes.Add(default);
            return newIndex;
        }
    }
}