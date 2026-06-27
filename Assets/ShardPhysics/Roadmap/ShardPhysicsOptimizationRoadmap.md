# Shard Physics Optimization Roadmap

## Current State

The engine now performs:

1. **Body-level broadphase** using world-space AABBs to reject non-overlapping body pairs.
2. Narrowphase collision detection only for candidate body pairs.
3. TriangleMesh collisions still iterate over every triangle inside an overlapping mesh.

Example:

```text
Sphere
    ↓
Terrain Mesh (1000 triangles)

Current:
Sphere vs Triangle 0
Sphere vs Triangle 1
...
Sphere vs Triangle 999
```

Body broadphase greatly reduces the number of body pairs, but TriangleMesh collision is still brute-force within each overlapping mesh.

---

# Step 1 — Body Broadphase

~~Compute one world AABB for every body each frame.~~

~~Test body AABB overlap before running narrowphase.~~

~~Skip GetContactManifold() when body AABBs do not overlap.~~

Example:

```text
100 bodies

Brute force:
4950 body pairs

Body Broadphase:
~150 body pairs
```

---

# Step 2 — TriangleMesh Narrowphase Prefilter

This is **not** broadphase.

It happens **after** body broadphase has determined that two bodies may collide.

Instead of testing every triangle:

```text
Sphere AABB
      vs
Triangle AABB
```

Only nearby triangles are sent to the narrowphase.

Example:

```text
Terrain
1000 triangles

Current:
Sphere -> 1000 triangle tests

With triangle prefilter:
Sphere -> ~8 triangle tests
```

This is expected to provide the next major performance improvement while keeping the implementation relatively simple.

---

# Step 3 — Future Broadphase Improvements

After the engine is stable:

* Spatial hash grid
* Sweep and Prune (SAP)
* Dynamic AABB Tree
* Bounding Volume Hierarchy (BVH)
* Sharded multithreaded broadphase

These will eventually replace the current simple body-AABB broadphase.

---

# Recommended Order

1. ~~Brute-force body pair testing~~
2. ~~Body AABB broadphase~~
3. ➜ TriangleMesh triangle-AABB prefilter
4. ➜ Advanced broadphase (Grid / SAP / BVH)

This approach keeps the engine simple while providing significant performance gains at each stage.

---

# TODO

## Completed

* ~~Brute-force body pair testing~~
* ~~Body AABB generation~~
* ~~Body AABB overlap testing~~
* ~~Candidate body pair generation~~
* ~~Skip static-static body pairs~~
* ~~Basic `Broadphase.cs` implementation~~
* ~~Broadphase integration into collision pair generation~~

## Remaining

### TriangleMesh Optimization

* Triangle AABB prefilter
* Triangle candidate generation
* Skip distant triangles before narrowphase
* Profile terrain performance improvements

### Advanced Broadphase

* Spatial hash grid
* Sweep and Prune (SAP)
* Dynamic AABB Tree
* BVH
* Pair cache
* Incremental broadphase updates
* Multithreaded broadphase

### Performance

* Benchmark broadphase scalability
* Benchmark TriangleMesh optimization
* Compare against previous brute-force implementation
