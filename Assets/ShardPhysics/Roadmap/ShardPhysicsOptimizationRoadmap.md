# Shard Physics Optimization Roadmap

## Current State

The engine now performs:

1. **Body-level broadphase** using world-space AABBs to reject non-overlapping body pairs.
2. Narrowphase collision detection only for candidate body pairs.
3. TriangleMesh collisions use a **triangle AABB prefilter** before running primitive-vs-triangle narrowphase.
4. Box↔Mesh collision now works through Box↔Triangle contacts, including terrain-style box-vertex-to-triangle-plane contacts.

Example:

```text
Sphere / Box / Capsule
    ↓
Terrain Mesh (1000 triangles)

Current:
Primitive AABB vs Triangle 0 AABB
Primitive AABB vs Triangle 1 AABB
...
Primitive AABB vs Triangle 999 AABB

Only overlapping triangles are sent to narrowphase.
```

Body broadphase greatly reduces the number of body pairs. Triangle AABB prefilter then reduces the number of mesh triangles sent into narrowphase.

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

This is **not** body broadphase.

It happens **after** body broadphase has determined that two bodies may collide.

Instead of running narrowphase against every triangle:

```text
Primitive AABB
      vs
Triangle AABB
```

Only nearby triangles are sent to the narrowphase.

Example:

```text
Terrain
1000 triangles

Previous:
Sphere / Box / Capsule -> 1000 triangle narrowphase tests

With triangle prefilter:
Sphere / Box / Capsule -> ~8 triangle narrowphase tests
```

~~TriangleMesh triangle-AABB prefilter is now implemented for Sphere, Box, and Capsule mesh collisions.~~

---

# Step 3 — Box ↔ Mesh Collision Robustness

Box↔Mesh collision works by reducing mesh collision into Box↔Triangle tests.

Current support:

* ~~Box↔Triangle SAT overlap testing~~
* ~~Box vertices crossing triangle plane / terrain-style face contact~~
* ~~Box↔Mesh collision through triangle iteration~~
* ~~Box↔Mesh support for heightfield terrain~~

Remaining robustness work:

* Triangle vertices inside box contact generation
* Edge-edge closest-point contacts
* Multi-contact manifold clipping
* Better support for vertical walls, sharp mesh corners, and thin geometry
* Stress testing on arbitrary non-heightfield meshes

---

# Step 4 — Future Mesh Acceleration

Triangle AABB prefilter still scans every triangle in an overlapping mesh.

That means this is improved:

```text
1000 triangles
→ 1000 cheap AABB tests
→ ~8 narrowphase tests
```

But a future BVH would improve it further:

```text
1000 triangles
→ traverse mesh BVH
→ visit only nearby nodes
→ test only nearby triangle AABBs
→ run narrowphase on final candidates
```

Future mesh acceleration work:

* Mesh BVH construction
* Mesh BVH traversal
* Terrain chunk BVH support
* Optional spatial hash grid for terrain-like meshes

---

# Step 5 — Future Broadphase Improvements

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
3. ~~TriangleMesh triangle-AABB prefilter~~
4. ~~Basic Box↔Mesh terrain collision~~
5. ➜ Box↔Mesh robustness improvements
6. ➜ Mesh BVH
7. ➜ Advanced body broadphase (Grid / SAP / Dynamic AABB Tree)

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
* ~~Primitive localPose support for Sphere / Box / Capsule~~
* ~~Triangle AABB prefilter~~
* ~~Triangle candidate generation~~
* ~~Skip distant triangles before narrowphase~~
* ~~Basic Box↔Triangle SAT overlap~~
* ~~Box-vertex-to-triangle-plane contact path~~
* ~~Basic Box↔Mesh terrain collision~~

## Remaining

### Box ↔ Mesh Robustness

* Triangle vertices inside box contact generation
* Edge-edge closest-point contacts
* Multi-contact manifold clipping
* Better arbitrary mesh support
* Vertical wall testing
* Mesh edge/corner testing
* Thin triangle testing

### Mesh Optimization

* Mesh BVH construction
* Mesh BVH traversal
* Avoid scanning every triangle in large meshes
* Terrain chunk acceleration
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
* Benchmark Box↔Mesh collision cost
