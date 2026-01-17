using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Shard
{
    internal struct ColliderHeader
    {
        public ColliderType Type;
        public ushort Version;
        public int PayloadIndex;   // index into the relevant pool
        public Aabb LocalAabb;     // local-space bounds for fast world AABB updates
        public ushort Flags;       // trigger, two-sided, etc.
        public ushort MaterialId;  // collider-level material
    }

    internal struct ColliderSlot
    {
        public ColliderHeader Header;
        public int NextFree; // for free list
        public byte IsAlive;
    }

    public struct Material
    {
        public float Friction;
        public float Restitution;
        public float RollingFriction; // optional
    }


    internal sealed class ColliderStore : IDisposable
    {
        private readonly Allocator _allocator;

        public NativeList<ColliderSlot> Slots;
        public NativeList<int> FreeSlots;

        // Per-type pools
        public NativeList<SphereCollider> Spheres;
        public NativeList<CapsuleCollider> Capsules;
        public NativeList<BoxCollider> Boxes;
        public NativeList<ConeCollider> Cones;
        public NativeList<CylinderCollider> Cylinders;

        public NativeList<MeshColliderData> Meshes;
        public NativeList<CompoundCollider> Compounds;
        public NativeList<VoxelColliderData> Voxels;

        // Convex later
        // public NativeList<ConvexColliderData> Convexes;

        public ColliderStore(int capacity, Allocator allocator)
        {
            _allocator = allocator;

            Slots = new NativeList<ColliderSlot>(capacity, allocator);
            FreeSlots = new NativeList<int>(allocator);

            Spheres = new NativeList<SphereCollider>(allocator);
            Capsules = new NativeList<CapsuleCollider>(allocator);
            Boxes = new NativeList<BoxCollider>(allocator);
            Cones = new NativeList<ConeCollider>(allocator);
            Cylinders = new NativeList<CylinderCollider>(allocator);

            Meshes = new NativeList<MeshColliderData>(allocator);
            Compounds = new NativeList<CompoundCollider>(allocator);
            Voxels = new NativeList<VoxelColliderData>(allocator);
        }

        public void Dispose()
        {
            // Dispose per-instance arrays inside payloads
            for (int i = 0; i < Meshes.Length; i++)
            {
                if (Meshes[i].Vertices.IsCreated) Meshes[i].Vertices.Dispose();
                if (Meshes[i].Triangles.IsCreated) Meshes[i].Triangles.Dispose();
                if (Meshes[i].Bvh.IsCreated) Meshes[i].Bvh.Dispose();
            }

            for (int i = 0; i < Compounds.Length; i++)
            {
                if (Compounds[i].Children.IsCreated) Compounds[i].Children.Dispose();
                if (Compounds[i].Bvh.IsCreated) Compounds[i].Bvh.Dispose();
            }

            for (int i = 0; i < Voxels.Length; i++)
            {
                if (Voxels[i].Bricks.IsCreated) Voxels[i].Bricks.Dispose();
                if (Voxels[i].Bvh.IsCreated) Voxels[i].Bvh.Dispose();
            }

            Voxels.Dispose();
            Compounds.Dispose();
            Meshes.Dispose();

            Cylinders.Dispose();
            Cones.Dispose();
            Boxes.Dispose();
            Capsules.Dispose();
            Spheres.Dispose();

            FreeSlots.Dispose();
            Slots.Dispose();
        }

        // -------------------------
        // Slot allocation / resolve
        // -------------------------

        private int AllocateSlot()
        {
            int slotIndex;
            if (FreeSlots.Length > 0)
            {
                slotIndex = FreeSlots[^1];
                FreeSlots.RemoveAtSwapBack(FreeSlots.Length - 1);

                var s = Slots[slotIndex];
                s.IsAlive = 1;
                s.Header.Version++;
                Slots[slotIndex] = s;
            }
            else
            {
                slotIndex = Slots.Length;
                Slots.Add(new ColliderSlot
                {
                    Header = new ColliderHeader { Version = 1 },
                    IsAlive = 1,
                    NextFree = -1
                });
            }
            return slotIndex;
        }

        private void FreeSlot(int slotIndex)
        {
            var s = Slots[slotIndex];
            s.IsAlive = 0;
            s.NextFree = -1;
            Slots[slotIndex] = s;
            FreeSlots.Add(slotIndex);
        }

        public bool IsValid(ColliderHandle handle)
        {
            if ((uint)handle.Slot >= (uint)Slots.Length) return false;
            var s = Slots[handle.Slot];
            return s.IsAlive != 0 && s.Header.Version == handle.Version;
        }

        public ref ColliderHeader Resolve(ColliderHandle handle)
        {
            if ((uint)handle.Slot >= (uint)Slots.Length)
                throw new InvalidOperationException("Invalid collider handle slot.");

            ref var s = ref Slots.ElementAt(handle.Slot);
            if (s.IsAlive == 0 || s.Header.Version != handle.Version)
                throw new InvalidOperationException("Stale/destroyed collider handle.");

            return ref s.Header;
        }

        public void Destroy(ColliderHandle handle)
        {
            if (!IsValid(handle)) return;
            FreeSlot(handle.Slot);
        }

        // -------------------------
        // Create: Primitives
        // -------------------------

        public ColliderHandle CreateSphere(float radius, float3 center, ushort materialId = 0, ushort flags = 0)
        {
            int slot = AllocateSlot();
            ref var header = ref Slots.ElementAt(slot).Header;

            int payloadIndex = Spheres.Length;
            var p = new SphereCollider { Radius = radius, Center = center };
            Spheres.Add(p);

            header.Type = ColliderType.Sphere;
            header.PayloadIndex = payloadIndex;
            header.MaterialId = materialId;
            header.Flags = flags;
            header.LocalAabb = ComputeSphereLocalAabb(p);

            return new ColliderHandle(slot, header.Version);
        }

        public ColliderHandle CreateCapsule(float radius, float halfHeight, float3 center, ushort materialId = 0, ushort flags = 0)
        {
            int slot = AllocateSlot();
            ref var header = ref Slots.ElementAt(slot).Header;

            int payloadIndex = Capsules.Length;
            var p = new CapsuleCollider { Radius = radius, HalfHeight = halfHeight, Center = center };
            Capsules.Add(p);

            header.Type = ColliderType.Capsule;
            header.PayloadIndex = payloadIndex;
            header.MaterialId = materialId;
            header.Flags = flags;
            header.LocalAabb = ComputeCapsuleLocalAabb(p);

            return new ColliderHandle(slot, header.Version);
        }

        public ColliderHandle CreateBox(float3 halfExtents, float3 center, quaternion orientation, ushort materialId = 0, ushort flags = 0)
        {
            int slot = AllocateSlot();
            ref var header = ref Slots.ElementAt(slot).Header;

            int payloadIndex = Boxes.Length;
            var p = new BoxCollider { HalfExtents = halfExtents, Center = center, Orientation = orientation };
            Boxes.Add(p);

            header.Type = ColliderType.Box;
            header.PayloadIndex = payloadIndex;
            header.MaterialId = materialId;
            header.Flags = flags;
            header.LocalAabb = ComputeBoxLocalAabb(p);

            return new ColliderHandle(slot, header.Version);
        }

        public ColliderHandle CreateCone(float baseRadius, float halfHeight, float roundingRadius, float3 center, quaternion orientation, ushort materialId = 0, ushort flags = 0)
        {
            int slot = AllocateSlot();
            ref var header = ref Slots.ElementAt(slot).Header;

            int payloadIndex = Cones.Length;
            var p = new ConeCollider
            {
                BaseRadius = baseRadius,
                HalfHeight = halfHeight,
                RoundingRadius = roundingRadius,
                Center = center,
                Orientation = orientation
            };
            Cones.Add(p);

            header.Type = ColliderType.Cone;
            header.PayloadIndex = payloadIndex;
            header.MaterialId = materialId;
            header.Flags = flags;
            header.LocalAabb = ComputeConeLocalAabb(p);

            return new ColliderHandle(slot, header.Version);
        }

        public ColliderHandle CreateCylinder(float radius, float halfHeight, float roundingRadius, float3 center, quaternion orientation, ushort materialId = 0, ushort flags = 0)
        {
            int slot = AllocateSlot();
            ref var header = ref Slots.ElementAt(slot).Header;

            int payloadIndex = Cylinders.Length;
            var p = new CylinderCollider
            {
                Radius = radius,
                HalfHeight = halfHeight,
                RoundingRadius = roundingRadius,
                Center = center,
                Orientation = orientation
            };
            Cylinders.Add(p);

            header.Type = ColliderType.Cylinder;
            header.PayloadIndex = payloadIndex;
            header.MaterialId = materialId;
            header.Flags = flags;
            header.LocalAabb = ComputeCylinderLocalAabb(p);

            return new ColliderHandle(slot, header.Version);
        }

        // -------------------------
        // Create: Mesh
        // -------------------------

        public ColliderHandle CreateMesh(
            NativeArray<float3> vertices,
            NativeArray<Triangle> triangles,
            NativeArray<MeshBvhNode> bvh,
            ushort materialId = 0,
            ushort flags = 0)
        {
            int slot = AllocateSlot();
            ref var header = ref Slots.ElementAt(slot).Header;

            // Deep copy into store allocator
            var vCopy = new NativeArray<float3>(vertices.Length, _allocator, NativeArrayOptions.UninitializedMemory);
            var tCopy = new NativeArray<Triangle>(triangles.Length, _allocator, NativeArrayOptions.UninitializedMemory);
            var bCopy = new NativeArray<MeshBvhNode>(bvh.Length, _allocator, NativeArrayOptions.UninitializedMemory);

            NativeArray<float3>.Copy(vertices, vCopy);
            NativeArray<Triangle>.Copy(triangles, tCopy);
            NativeArray<MeshBvhNode>.Copy(bvh, bCopy);

            var p = new MeshColliderData
            {
                Vertices = vCopy,
                Triangles = tCopy,
                Bvh = bCopy,
                LocalAabb = (bCopy.Length > 0) ? bCopy[0].Bounds : ComputeVerticesAabb(vCopy),
                TopologyVersion = 1,
                VertexVersion = 1
            };

            int payloadIndex = Meshes.Length;
            Meshes.Add(p);

            header.Type = ColliderType.Mesh;
            header.PayloadIndex = payloadIndex;
            header.MaterialId = materialId;
            header.Flags = flags;
            header.LocalAabb = p.LocalAabb;

            return new ColliderHandle(slot, header.Version);
        }

        // -------------------------
        // Create: Compound
        // -------------------------

        public ColliderHandle CreateCompound(
            NativeArray<CompoundChild> children,
            NativeArray<CompoundBvhNode> bvh,
            ushort materialId = 0,
            ushort flags = 0)
        {
            int slot = AllocateSlot();
            ref var header = ref Slots.ElementAt(slot).Header;

            var cCopy = new NativeArray<CompoundChild>(children.Length, _allocator, NativeArrayOptions.UninitializedMemory);
            var bCopy = new NativeArray<CompoundBvhNode>(bvh.Length, _allocator, NativeArrayOptions.UninitializedMemory);
            NativeArray<CompoundChild>.Copy(children, cCopy);
            NativeArray<CompoundBvhNode>.Copy(bvh, bCopy);

            // Compute local AABB: union of transformed child aabbs (conservative)
            var localAabb = Aabb.Empty();
            for (int i = 0; i < cCopy.Length; i++)
            {
                var child = cCopy[i];
                var childWorld = TransformAabb(child.LocalAabb, child.LocalTransform);
                localAabb.Encapsulate(childWorld);
            }

            var p = new CompoundCollider
            {
                Children = cCopy,
                Bvh = bCopy,
                LocalAabb = (bCopy.Length > 0) ? bCopy[0].Bounds : localAabb
            };

            int payloadIndex = Compounds.Length;
            Compounds.Add(p);

            header.Type = ColliderType.Compound;
            header.PayloadIndex = payloadIndex;
            header.MaterialId = materialId;
            header.Flags = flags;
            header.LocalAabb = p.LocalAabb;

            return new ColliderHandle(slot, header.Version);
        }

        // -------------------------
        // Create: Voxel
        // -------------------------

        public ColliderHandle CreateVoxelBricks(
            float cellSize,
            int3 voxelMin,
            int3 voxelMax,
            NativeArray<VoxelBrick> bricks,
            NativeArray<VoxelBvhNode> bvh,
            ushort materialId = 0,
            ushort flags = 0)
        {
            int slot = AllocateSlot();
            ref var header = ref Slots.ElementAt(slot).Header;

            var bricksCopy = new NativeArray<VoxelBrick>(bricks.Length, _allocator, NativeArrayOptions.UninitializedMemory);
            var bvhCopy = new NativeArray<VoxelBvhNode>(bvh.Length, _allocator, NativeArrayOptions.UninitializedMemory);
            NativeArray<VoxelBrick>.Copy(bricks, bricksCopy);
            NativeArray<VoxelBvhNode>.Copy(bvh, bvhCopy);

            // Local AABB: either BVH root or union of brick bounds
            Aabb localAabb = Aabb.Empty();
            if (bvhCopy.Length > 0) localAabb = bvhCopy[0].Bounds;
            else
            {
                for (int i = 0; i < bricksCopy.Length; i++)
                    localAabb.Encapsulate(bricksCopy[i].LocalAabb);
            }

            var p = new VoxelColliderData
            {
                StorageKind = VoxelStorageKind.BrickBitmask,
                CellSize = cellSize,
                VoxelMin = voxelMin,
                VoxelMax = voxelMax,
                Bricks = bricksCopy,
                Bvh = bvhCopy,
                LocalAabb = localAabb,
                Version = 1
            };

            int payloadIndex = Voxels.Length;
            Voxels.Add(p);

            header.Type = ColliderType.Voxel;
            header.PayloadIndex = payloadIndex;
            header.MaterialId = materialId;
            header.Flags = flags;
            header.LocalAabb = p.LocalAabb;

            return new ColliderHandle(slot, header.Version);
        }

        // -------------------------
        // AABB helpers
        // -------------------------

        private static Aabb ComputeSphereLocalAabb(in SphereCollider s)
        {
            float r = math.max(0f, s.Radius);
            return Aabb.FromCenterExtents(s.Center, new float3(r, r, r));
        }

        // Capsule assumed canonical along local Y
        private static Aabb ComputeCapsuleLocalAabb(in CapsuleCollider c)
        {
            float r = math.max(0f, c.Radius);
            float hy = math.max(0f, c.HalfHeight);
            return Aabb.FromCenterExtents(c.Center, new float3(r, hy + r, r));
        }

        private static Aabb ComputeBoxLocalAabb(in BoxCollider b)
        {
            float3 ext = math.max(b.HalfExtents, 0f);
            return AabbForOrientedExtents(b.Center, b.Orientation, ext);
        }

        private static Aabb ComputeConeLocalAabb(in ConeCollider c)
        {
            float r = math.max(0f, c.BaseRadius) + math.max(0f, c.RoundingRadius);
            float hy = math.max(0f, c.HalfHeight) + math.max(0f, c.RoundingRadius);
            return AabbForOrientedExtents(c.Center, c.Orientation, new float3(r, hy, r));
        }

        private static Aabb ComputeCylinderLocalAabb(in CylinderCollider c)
        {
            float r = math.max(0f, c.Radius) + math.max(0f, c.RoundingRadius);
            float hy = math.max(0f, c.HalfHeight) + math.max(0f, c.RoundingRadius);
            return AabbForOrientedExtents(c.Center, c.Orientation, new float3(r, hy, r));
        }

        private static Aabb ComputeVerticesAabb(NativeArray<float3> verts)
        {
            var a = Aabb.Empty();
            for (int i = 0; i < verts.Length; i++)
            {
                a.Min = math.min(a.Min, verts[i]);
                a.Max = math.max(a.Max, verts[i]);
            }
            return a;
        }

        private static Aabb AabbForOrientedExtents(float3 center, quaternion rot, float3 extents)
        {
            float3x3 R = new float3x3(rot);
            float3 ax = math.abs(R.c0) * extents.x;
            float3 ay = math.abs(R.c1) * extents.y;
            float3 az = math.abs(R.c2) * extents.z;
            float3 aabbExt = ax + ay + az;
            return Aabb.FromCenterExtents(center, aabbExt);
        }

        private static Aabb TransformAabb(in Aabb localAabb, in TransformQv t)
        {
            // Conservative AABB transform (rotate extents)
            float3 localCenter = localAabb.Center;
            float3 localExt = localAabb.Extents;

            float3x3 R = new float3x3(t.Rotation);
            float3 ax = math.abs(R.c0) * localExt.x;
            float3 ay = math.abs(R.c1) * localExt.y;
            float3 az = math.abs(R.c2) * localExt.z;
            float3 worldExt = ax + ay + az;

            float3 worldCenter = t.Position + math.mul(t.Rotation, localCenter);
            return Aabb.FromCenterExtents(worldCenter, worldExt);
        }
    }

}
