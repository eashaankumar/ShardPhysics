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

    internal sealed class ColliderStore : System.IDisposable
    {
        public NativeList<ColliderSlot> Slots;
        public NativeList<int> FreeSlots;

        // Per-type pools
        public NativeList<SphereCollider> Spheres;
        public NativeList<CapsuleCollider> Capsules;
        public NativeList<BoxCollider> Boxes;
        public NativeList<MeshColliderData> Meshes;
        public NativeList<CompoundCollider> Compounds;
        public NativeList<VoxelColliderData> Voxels;
        public NativeList<ConeCollider> Cones;
        public NativeList<CylinderCollider> Cylinders;


        public ColliderStore(int capacity, Allocator allocator)
        {
            Slots = new NativeList<ColliderSlot>(capacity, allocator);
            FreeSlots = new NativeList<int>(allocator);

            Spheres = new NativeList<SphereCollider>(allocator);
            Capsules = new NativeList<CapsuleCollider>(allocator);
            Boxes = new NativeList<BoxCollider>(allocator);
            Meshes = new NativeList<MeshColliderData>(allocator);
            Compounds = new NativeList<CompoundCollider>(allocator);
            Voxels = new NativeList<VoxelColliderData>(allocator);
            Cones = new NativeList<ConeCollider>(allocator);
            Cylinders = new NativeList<CylinderCollider>(allocator);
        }

        public void Dispose()
        {
            Voxels.Dispose();
            Compounds.Dispose();
            Meshes.Dispose();
            Boxes.Dispose();
            Capsules.Dispose();
            Spheres.Dispose();
            Cylinders.Dispose();
            Cones.Dispose();

            FreeSlots.Dispose();
            Slots.Dispose();
        }

        private int AllocateSlot()
        {
            int slotIndex;

            if (FreeSlots.Length > 0)
            {
                // reuse a previously freed slot
                slotIndex = FreeSlots[^1];
                FreeSlots.RemoveAtSwapBack(FreeSlots.Length - 1);

                var slot = Slots[slotIndex];
                slot.IsAlive = 1;
                slot.Header.Version++; // CRITICAL: version bump (ABA protection)
                Slots[slotIndex] = slot;
            }
            else
            {
                // create a new slot
                slotIndex = Slots.Length;

                Slots.Add(new ColliderSlot
                {
                    Header = new ColliderHeader
                    {
                        Version = 1
                    },
                    IsAlive = 1,
                    NextFree = -1
                });
            }

            return slotIndex;
        }

        private void FreeSlot(int slotIndex)
        {
            var slot = Slots[slotIndex];

            slot.IsAlive = 0;
            slot.NextFree = -1;

            Slots[slotIndex] = slot;
            FreeSlots.Add(slotIndex);
        }

        public ref ColliderHeader Resolve(ColliderHandle handle)
        {
            if ((uint)handle.Slot >= (uint)Slots.Length)
                throw new InvalidOperationException("Invalid collider handle slot");

            ref var slot = ref Slots.ElementAt(handle.Slot);

            if (slot.IsAlive == 0 || slot.Header.Version != handle.Version)
                throw new InvalidOperationException("Stale or destroyed collider handle");

            return ref slot.Header;
        }

        public void Destroy(ColliderHandle handle)
        {
            ref var slot = ref Resolve(handle);

            // optional: reclaim payload later (or leave tombstones)
            // payload pools are usually compacted lazily

            FreeSlot(handle.Slot);
        }



        public ColliderHandle CreateCone(float baseRadius, float halfHeight, float roundingRadius, float3 center, quaternion orientation, ushort materialId = 0, ushort flags = 0)
        {
            // allocate handle slot (same pattern you use for spheres/boxes)
            int slot = AllocateSlot();
            ushort version = (ushort)(Slots[slot].Header.Version + 1);

            var payload = new ConeCollider
            {
                BaseRadius = baseRadius,
                HalfHeight = halfHeight,
                RoundingRadius = roundingRadius,
                Center = center,
                Orientation = orientation
            };

            int payloadIndex = Cones.Length;
            Cones.Add(payload);

            var header = new ColliderHeader
            {
                Type = ColliderType.Cone,
                Version = version,
                PayloadIndex = payloadIndex,
                LocalAabb = Aabb.ComputeConeLocalAabb(payload),
                Flags = flags,
                MaterialId = materialId
            };

            Slots[slot] = new ColliderSlot { Header = header, IsAlive = 1, NextFree = -1 };
            return new ColliderHandle(slot, version);
        }

        public ColliderHandle CreateCylinder(float radius, float halfHeight, float roundingRadius, float3 center, quaternion orientation, ushort materialId = 0, ushort flags = 0)
        {
            int slot = AllocateSlot();
            ushort version = (ushort)(Slots[slot].Header.Version + 1);

            var payload = new CylinderCollider
            {
                Radius = radius,
                HalfHeight = halfHeight,
                RoundingRadius = roundingRadius,
                Center = center,
                Orientation = orientation
            };

            int payloadIndex = Cylinders.Length;
            Cylinders.Add(payload);

            var header = new ColliderHeader
            {
                Type = ColliderType.Cylinder,
                Version = version,
                PayloadIndex = payloadIndex,
                LocalAabb = Aabb.ComputeCylinderLocalAabb(payload),
                Flags = flags,
                MaterialId = materialId
            };

            Slots[slot] = new ColliderSlot { Header = header, IsAlive = 1, NextFree = -1 };
            return new ColliderHandle(slot, version);
        }

    }
}
