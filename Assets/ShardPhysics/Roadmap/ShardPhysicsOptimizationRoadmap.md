# Shard Physics Optimization Roadmap

## Current State
The engine currently performs:

1. Brute-force body pair testing (`O(n²)`).
2. If a body pair is tested, TriangleMesh collisions iterate over **every triangle** in the mesh.

Example:

```
Sphere
    ↓
Terrain Mesh (1000 triangles)

Current:
Sphere vs Triangle 0
Sphere vs Triangle 1
...
Sphere vs Triangle 999
```

This is correct but extremely slow.

---

# Step 1 — Body Broadphase (Next)

Add a simple body-level broadphase.

Compute one world AABB for every body each frame.

```
Body AABB
        vs
Body AABB
```

If two body AABBs don't overlap:

```
Skip GetContactManifold()
```

This dramatically reduces body pair tests.

Example:

```
100 bodies

Brute force:
4950 body pairs

Broadphase:
~150 body pairs
```

---

# Step 2 — TriangleMesh Narrowphase Prefilter

This is **not** broadphase.

It happens **after** broadphase decides that a body overlaps a terrain chunk.

Instead of testing every triangle:

```
Sphere AABB
      vs
Triangle AABB
```

Only nearby triangles are tested.

Example:

```
Terrain
1000 triangles

Current:
Sphere -> 1000 triangle tests

With prefilter:
Sphere -> ~8 triangle tests
```

This is a huge speed improvement while keeping the implementation simple.

---

# Step 3 — Future Broadphase Improvements

After the engine is stable:

- Spatial hash grid
- Sweep and Prune
- BVH
- Sharded multithreaded broadphase

These replace the simple body-AABB broadphase.

---

# Recommended Order

1. ✅ Brute-force (current)
2. ➜ Body AABB broadphase
3. ➜ TriangleMesh triangle-AABB prefilter
4. ➜ Advanced broadphase (grid/BVH/SAP)

This keeps the engine simple while providing large performance gains at each stage.