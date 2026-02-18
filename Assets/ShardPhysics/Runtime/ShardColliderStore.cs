using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using UnityEngine;

namespace Shard.Runtime
{
    public readonly struct ShardColliderHandle
    {
        public readonly int handle; // node index in pool
        internal ShardColliderHandle(int h) { handle = h; }
        public bool IsValid => handle >= 0;
    }

    internal struct ColliderNode
    {
        public ShardCollider collider;
        public int next;        // -1 end
        public int ownerDense;  // dense body index that owns this node
        public byte alive;      // 1 alive, 0 free
    }

    internal struct ColliderList
    {
        public int head;    // node index or -1
        public int count;
        public byte dirty;  // set to 1 when changed; user can use to recompute inertia later
    }

    /// <summary>
    /// Per-world collider storage:
    /// - A pool of collider nodes (NativeList)
    /// - A per-body (dense) linked list head/count
    /// Supports frequent add/remove/edit without moving other bodies' collider data.
    /// </summary>
    public sealed class ShardColliderStore : IDisposable
    {
        private NativeList<ColliderNode> _nodes;
        private NativeList<int> _free;              // free node indices
        private NativeList<ColliderList> _lists;     // indexed by dense body index

        public ShardColliderStore(int initialNodeCapacity, int initialBodyCapacity, Allocator allocator)
        {
            _nodes = new NativeList<ColliderNode>(initialNodeCapacity, allocator);
            _free = new NativeList<int>(initialNodeCapacity, allocator);
            _lists = new NativeList<ColliderList>(initialBodyCapacity, allocator);
        }

        public void Dispose()
        {
            if (_nodes.IsCreated) _nodes.Dispose();
            if (_free.IsCreated) _free.Dispose();
            if (_lists.IsCreated) _lists.Dispose();
        }

        // -------- Dense body lifecycle hooks --------

        /// <summary>Call when a new dense body row is appended.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void OnBodyAdded()
        {
            _lists.Add(new ColliderList { head = -1, count = 0, dirty = 0 });
        }

        /// <summary>
        /// Call when body 'fromDense' was swap-moved into 'toDense'.
        /// This happens in your DestroyBody when you copy last -> dense.
        /// </summary>
        public void OnBodySwapMoved(int fromDense, int toDense)
        {
            _lists[toDense] = _lists[fromDense];

            // Update ownerDense for the moved list
            int n = _lists[toDense].head;
            while (n != -1)
            {
                var node = _nodes[n];
                node.ownerDense = toDense;
                _nodes[n] = node;
                n = node.next;
            }
        }

        /// <summary>
        /// Call right before removing the last dense body row (pop-back).
        /// Frees all colliders owned by that dense index.
        /// </summary>
        public void OnBodyRemoving(int dense)
        {
            // Free its collider nodes
            int n = _lists[dense].head;
            while (n != -1)
            {
                var node = _nodes[n];
                int next = node.next;

                node.alive = 0;
                node.next = -1;
                node.ownerDense = -1;
                _nodes[n] = node;

                _free.Add(n);
                n = next;
            }

            _lists[dense] = new ColliderList { head = -1, count = 0, dirty = 0 };
        }

        /// <summary>Call after you pop-back your dense arrays.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void OnBodyPoppedBack()
        {
            _lists.RemoveAt(_lists.Length - 1);
        }

        // -------- Per-body operations --------

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetColliderCount(int dense) => _lists[dense].count;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsDirty(int dense) => _lists[dense].dirty != 0;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ClearDirty(int dense)
        {
            var l = _lists[dense];
            l.dirty = 0;
            _lists[dense] = l;
        }

        public void ClearColliders(int dense)
        {
            OnBodyRemoving(dense);
            // Mark dirty for mass props
            var l = _lists[dense];
            l.dirty = 1;
            _lists[dense] = l;
        }

        public ShardColliderHandle AddCollider(int dense, in ShardCollider collider)
        {
            int nodeIndex = AllocateNode();
            _nodes[nodeIndex] = new ColliderNode
            {
                collider = collider,
                next = _lists[dense].head,
                ownerDense = dense,
                alive = 1
            };

            var l = _lists[dense];
            l.head = nodeIndex;
            l.count++;
            l.dirty = 1;
            _lists[dense] = l;

            return new ShardColliderHandle(nodeIndex);
        }

        public bool RemoveCollider(int dense, ShardColliderHandle h)
        {
            int idx = h.handle;
            if ((uint)idx >= (uint)_nodes.Length)
                return false;

            var node = _nodes[idx];
            if (node.alive == 0 || node.ownerDense != dense)
                return false;

            // unlink from singly-linked list
            int prev = -1;
            int cur = _lists[dense].head;

            while (cur != -1)
            {
                if (cur == idx)
                {
                    int next = _nodes[cur].next;
                    if (prev == -1)
                    {
                        var l = _lists[dense];
                        l.head = next;
                        l.count--;
                        l.dirty = 1;
                        _lists[dense] = l;
                    }
                    else
                    {
                        var prevNode = _nodes[prev];
                        prevNode.next = next;
                        _nodes[prev] = prevNode;

                        var l = _lists[dense];
                        l.count--;
                        l.dirty = 1;
                        _lists[dense] = l;
                    }

                    FreeNode(cur);
                    return true;
                }

                prev = cur;
                cur = _nodes[cur].next;
            }

            return false;
        }

        // -------- Per-collider operations (fast per-update edits) --------

        public bool TryGetCollider(ShardColliderHandle h, out ShardCollider collider, out int ownerDense)
        {
            int idx = h.handle;
            if ((uint)idx >= (uint)_nodes.Length)
            {
                collider = default;
                ownerDense = -1;
                return false;
            }

            var n = _nodes[idx];
            if (n.alive == 0)
            {
                collider = default;
                ownerDense = -1;
                return false;
            }

            collider = n.collider;
            ownerDense = n.ownerDense;
            return true;
        }

        public bool SetCollider(ShardColliderHandle h, in ShardCollider collider)
        {
            int idx = h.handle;
            if ((uint)idx >= (uint)_nodes.Length)
                return false;

            var n = _nodes[idx];
            if (n.alive == 0)
                return false;

            n.collider = collider;
            _nodes[idx] = n;

            MarkDirty(n.ownerDense);
            return true;
        }

        public bool SetColliderLocalPose(ShardColliderHandle h, in Pose localPose)
        {
            int idx = h.handle;
            if ((uint)idx >= (uint)_nodes.Length)
                return false;

            var n = _nodes[idx];
            if (n.alive == 0)
                return false;

            var c = n.collider;
            c.localPose = localPose;
            n.collider = c;
            _nodes[idx] = n;

            MarkDirty(n.ownerDense);
            return true;
        }

        // Optional: iterate all colliders of a body (for inertia calc, broadphase, etc.)
        public int GetHead(int dense) => _lists[dense].head;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetNode(int nodeIndex, out ShardCollider collider, out int next)
        {
            if ((uint)nodeIndex >= (uint)_nodes.Length)
            {
                collider = default;
                next = -1;
                return false;
            }

            var n = _nodes[nodeIndex];
            if (n.alive == 0)
            {
                collider = default;
                next = -1;
                return false;
            }

            collider = n.collider;
            next = n.next;
            return true;
        }

        // -------- Internals --------

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void MarkDirty(int dense)
        {
            if (dense < 0 || dense >= _lists.Length) return;
            var l = _lists[dense];
            l.dirty = 1;
            _lists[dense] = l;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int AllocateNode()
        {
            if (_free.Length > 0)
            {
                int last = _free.Length - 1;
                int idx = _free[last];
                _free.RemoveAtSwapBack(last);
                return idx;
            }

            int newIdx = _nodes.Length;
            _nodes.Add(default);
            return newIdx;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void FreeNode(int idx)
        {
            var n = _nodes[idx];
            n.alive = 0;
            n.next = -1;
            n.ownerDense = -1;
            _nodes[idx] = n;

            _free.Add(idx);
        }
    }
}
