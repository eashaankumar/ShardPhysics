using System;
using System.Runtime.CompilerServices;
using Unity.Collections;

namespace Shard.Runtime
{
    /// <summary>
    /// Maintains stable "slot ids" that map to a dense index [0..denseCount-1].
    /// Destroy is swap-remove on dense storage; mappings are updated accordingly.
    /// </summary>
    public sealed class DenseSlotMap : IDisposable
    {
        // slot -> dense index, -1 means free
        private NativeList<int> _slotToDense;

        // dense index -> slot id
        private NativeList<int> _denseToSlot;

        // stack of freed slot ids for reuse
        private NativeList<int> _freeSlots;

        public DenseSlotMap(int initialCapacity, Allocator allocator)
        {
            _slotToDense = new NativeList<int>(initialCapacity, allocator);
            _denseToSlot = new NativeList<int>(initialCapacity, allocator);
            _freeSlots = new NativeList<int>(initialCapacity, allocator);
        }

        public void Dispose()
        {
            if (_slotToDense.IsCreated) _slotToDense.Dispose();
            if (_denseToSlot.IsCreated) _denseToSlot.Dispose();
            if (_freeSlots.IsCreated) _freeSlots.Dispose();
        }

        public int DenseCount => _denseToSlot.Length;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsAlive(int slot)
        {
            return (uint)slot < (uint)_slotToDense.Length && _slotToDense[slot] >= 0;
        }

        /// <summary>
        /// Allocate a slot id and bind it to the next dense index (denseCount).
        /// Returns (slot, denseIndex).
        /// Call this right before you append to your SoA lists.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Allocate(out int slot, out int denseIndex)
        {
            denseIndex = _denseToSlot.Length; // next append position

            if (_freeSlots.Length > 0)
            {
                int last = _freeSlots.Length - 1;
                slot = _freeSlots[last];
                _freeSlots.RemoveAtSwapBack(last);
            }
            else
            {
                slot = _slotToDense.Length;
                _slotToDense.Add(-1);
            }

            // Bind new dense index -> slot
            _denseToSlot.Add(slot);
            _slotToDense[slot] = denseIndex;
        }

        /// <summary>
        /// Resolve slot -> dense index. Returns false if slot is invalid/freed.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryResolveDense(int slot, out int denseIndex)
        {
            denseIndex = -1;
            if ((uint)slot >= (uint)_slotToDense.Length) return false;

            int d = _slotToDense[slot];
            if (d < 0) return false;

            denseIndex = d;
            return true;
        }

        /// <summary>
        /// Called AFTER you swap-remove dense storage at denseIndex by moving lastDense into denseIndex.
        /// Updates the mapping for the moved element.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void OnDenseElementMoved(int fromLastDense, int toDense)
        {
            // moved slot id that used to live at lastDense
            int movedSlot = _denseToSlot[fromLastDense];

            // update denseToSlot at destination
            _denseToSlot[toDense] = movedSlot;

            // update slotToDense for moved slot
            _slotToDense[movedSlot] = toDense;
        }

        /// <summary>
        /// Free a slot and remove the last dense entry from denseToSlot.
        /// Must be called AFTER you've removed last element from all dense arrays.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Free(int slot)
        {
            // mark slot free
            _slotToDense[slot] = -1;
            _freeSlots.Add(slot);

            // remove last dense->slot mapping (caller already swap-removed body data)
            _denseToSlot.RemoveAtSwapBack(_denseToSlot.Length - 1);
        }

        /// <summary>
        /// Get the slot id stored at dense index (useful for debugging).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int DenseToSlot(int denseIndex) => _denseToSlot[denseIndex];
    }
}
