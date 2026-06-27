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
        }

        public bool IsCreated =>
            _vertices.IsCreated &&
            _indices.IsCreated &&
            _meshes.IsCreated &&
            _freeMeshes.IsCreated;

        public int MeshCount => _meshes.IsCreated ? _meshes.Length : 0;
        public int VertexCount => _vertices.IsCreated ? _vertices.Length : 0;
        public int IndexCount => _indices.IsCreated ? _indices.Length : 0;

        public NativeArray<float3>.ReadOnly Vertices => _vertices.AsArray().AsReadOnly();
        public NativeArray<int>.ReadOnly Indices => _indices.AsArray().AsReadOnly();
        public NativeArray<ShardTriangleMeshInfo>.ReadOnly Meshes => _meshes.AsArray().AsReadOnly();

        public void Dispose()
        {
            if (_vertices.IsCreated) _vertices.Dispose();
            if (_indices.IsCreated) _indices.Dispose();
            if (_meshes.IsCreated) _meshes.Dispose();
            if (_freeMeshes.IsCreated) _freeMeshes.Dispose();
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
                localBounds = bounds,
                alive = 1
            };

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
                localBounds = bounds,
                alive = 1
            };

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