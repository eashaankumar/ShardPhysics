# Shard Physics Optimization Roadmap

## Current State

The engine now performs:

1.  **Spatial-hash body broadphase** using world-space AABBs to reject
    non-overlapping body pairs without brute-forcing every body against
    every other body in large scenes.
2.  A small-scene **brute-force AABB fallback** for low body counts where
    hash setup overhead is not worth it.
3.  Oversized-AABB fallback handling for very large bodies, terrain-like
    bodies, or objects that would occupy too many spatial hash cells.
4.  Narrowphase collision detection only for candidate body pairs.
5.  TriangleMesh collisions use a **TriangleMesh BVH** to query only
    nearby triangles before narrowphase.
6.  Candidate triangles then pass through a **triangle AABB prefilter**
    before primitive-vs-triangle narrowphase.
7.  **Collision detection runs once per simulation substep instead of
    once per solver iteration.**
8.  **Generated contact manifolds are cached and reused across every
    solver iteration.**
9.  **Broadphase and narrowphase are no longer repeatedly executed
    during iterative impulse solving.**
10. Box↔Mesh collision now works through robust Box↔Triangle contacts.
11. Box↔Mesh collision supports terrain-style surfaces, vertical walls,
    corners, edges, and thin triangle cases.
12. The entire collision pipeline remains **NativeMemory-based**, with
    no managed allocations during simulation.
13. Built-in **performance instrumentation** tracks BVH efficiency and
    collision pipeline statistics.
14. Permanent regression tests verify BVH effectiveness and help catch
    future performance regressions.

Current collision pipeline:

``` text
Simulation Substep

Spatial Hash Body Broadphase
        ↓
TriangleMesh BVH Query
        ↓
Triangle AABB Prefilter
        ↓
Generate Contact Manifolds
        ↓
Cache Contacts
        ↓
Solver Iteration 1
Solver Iteration 2
Solver Iteration 3
Solver Iteration 4
Solver Iteration 5
Solver Iteration 6
```

Collision detection is now separated from constraint solving. Contacts
are generated once, cached, and reused throughout the solver iterations.
Triangle meshes are spatially accelerated using a BVH before
narrowphase, greatly reducing unnecessary triangle testing. Large body
sets are now accelerated with a spatial hash broadphase before
narrowphase, reducing unnecessary body-pair checks.

------------------------------------------------------------------------

# Step 1 --- Body Broadphase

~~Compute one world AABB for every body each frame.~~

~~Test body AABB overlap before running narrowphase.~~

~~Skip GetContactManifold() when body AABBs do not overlap.~~

Example:

``` text
100 bodies

Brute force:
4950 body pairs

Body Broadphase:
~150 body pairs
```

------------------------------------------------------------------------

# Step 2 --- TriangleMesh Narrowphase Prefilter

This occurs **after** body broadphase.

Primitive AABBs are compared against candidate triangle AABBs before
narrowphase.

``` text
Primitive AABB
      vs
Triangle AABB
```

Only nearby triangles proceed into primitive-vs-triangle collision.

~~TriangleMesh triangle-AABB prefilter is implemented for Sphere, Box,
and Capsule mesh collisions.~~

------------------------------------------------------------------------

# Step 3 --- Cached Contact Generation

Previously every solver iteration rebuilt collision information.

Previous architecture:

``` text
Iteration
    Broadphase
    Generate Contacts
    Solve
```

Repeated six times every substep.

Current architecture:

``` text
Simulation Substep

Broadphase
      ↓
Generate Contact Manifolds
      ↓
Cache Contacts
      ↓
Solver Iterations
```

Completed:

-   ~~Broadphase rebuilt once per simulation substep~~
-   ~~Collision detection executed once per simulation substep~~
-   ~~Cached contact manifolds~~
-   ~~Solver iterations reuse cached contacts~~
-   ~~Removed repeated broadphase~~
-   ~~Removed repeated narrowphase~~

Benefits:

-   Dramatically lower CPU usage
-   Significantly higher FPS
-   Solver cost scales much better with iteration count

------------------------------------------------------------------------

# Step 4 --- Box ↔ Mesh Collision Robustness

Current support:

-   ~~Box↔Triangle SAT overlap testing~~
-   ~~Box vertices crossing triangle plane~~
-   ~~Box↔Mesh terrain collision~~
-   ~~Triangle vertices inside box~~
-   ~~Edge-edge closest-point contacts~~
-   ~~Improved Box↔Triangle manifold generation~~
-   ~~Better vertical wall support~~
-   ~~Better sharp corner support~~
-   ~~Better thin triangle support~~

Remaining work:

-   Multi-contact manifold clipping
-   Contact reduction / prioritization
-   Additional stress testing on arbitrary meshes

------------------------------------------------------------------------

# Step 5 --- TriangleMesh BVH

Previously every overlapping mesh required scanning every triangle:

``` text
Mesh
 ↓

Triangle 0
Triangle 1
Triangle 2
...
Triangle 999
```

Current implementation:

``` text
Mesh
 ↓
TriangleMesh BVH
 ↓
Nearby BVH Nodes
 ↓
Candidate Triangles
 ↓
Triangle AABB Prefilter
 ↓
Primitive-vs-Triangle Narrowphase
```

Completed:

-   ~~NativeMemory BVH node storage~~
-   ~~Native triangle index storage~~
-   ~~Median split BVH builder~~
-   ~~Iterative BVH traversal~~
-   ~~BVH integration for Sphere↔Mesh~~
-   ~~BVH integration for Box↔Mesh~~
-   ~~BVH integration for Capsule↔Mesh~~
-   ~~No managed allocations during BVH traversal~~
-   ~~No GC during simulation~~

Benefits:

-   Avoids scanning every triangle in large meshes
-   Scales significantly better with terrain size
-   NativeMemory-only implementation
-   Reusable for future raycasts and spatial queries

Remaining improvements:

-   Surface Area Heuristic (SAH) builder
-   Parallel BVH construction
-   BVH refit for deformable meshes
-   Debug BVH visualization
-   Profiling and tuning of leaf size
-   Large-world performance profiling

------------------------------------------------------------------------

# Step 6 --- Performance Instrumentation & Regression Testing

Permanent instrumentation has been added throughout the collision
pipeline.

Current counters include:

-   BVH queries
-   BVH nodes visited
-   BVH leaf nodes visited
-   Candidate triangles returned
-   Triangle AABB tests
-   Triangle narrowphase tests
-   Contact manifolds generated
-   Broadphase candidate pairs
-   Full triangle scan fallbacks

Permanent regression tests verify:

-   BVH reduces candidate triangle counts
-   BVH traversal remains allocation-free
-   Collision optimizations continue functioning after future engine
    changes

Benefits:

-   Detect performance regressions early
-   Measure BVH effectiveness objectively
-   Validate optimization improvements
-   Guide future tuning

------------------------------------------------------------------------

# Step 7 --- Spatial Hash Body Broadphase

The body broadphase has been upgraded from simple brute-force AABB pair
checking to a spatial hash grid for larger scenes.

Previous body broadphase:

``` text
for each body A
    for each body B after A
        skip static-static
        test AABB overlap
        emit candidate pair
```

This was still **O(n²)** over bodies, even though it avoided expensive
narrowphase work for non-overlapping AABBs.

Current body broadphase:

``` text
Body AABB
      ↓
Insert body into spatial hash cells
      ↓
Query nearby bodies sharing cells
      ↓
Suppress duplicate pairs
      ↓
Final AABB overlap check
      ↓
Emit candidate body pair
```

Completed:

-   ~~Spatial hash body broadphase~~
-   ~~Small-scene brute-force fallback~~
-   ~~Oversized-AABB fallback for huge bodies / terrain-like bodies~~
-   ~~Duplicate pair suppression~~
-   ~~Final AABB overlap validation after hash lookup~~
-   ~~Static-static pair rejection~~
-   ~~Existing Broadphase API preserved~~

Benefits:

-   Reduces body-pair broadphase cost in large scenes
-   Avoids brute-forcing every dynamic body against every other body
-   Keeps the cheaper brute-force path for small scenes
-   Handles huge terrain bodies without exploding hash-cell insertion
-   Preserves the existing collision pipeline structure

Remaining improvements:

-   Benchmark broadphase scalability
-   Tune spatial hash cell size
-   Add broadphase counters for hash cell occupancy
-   Add regression tests for duplicate pair suppression
-   Add regression tests for oversized-body fallback
-   Compare spatial hash against Sweep and Prune for long, flat worlds

------------------------------------------------------------------------

# Step 8 --- Future Broadphase Improvements

After the spatial hash broadphase is stable and profiled:

-   Sweep and Prune (SAP)
-   Dynamic AABB Tree
-   Pair cache
-   Incremental broadphase updates
-   Sharded multithreaded broadphase

These would only replace or supplement the spatial hash if profiling
shows the hash grid is not ideal for a specific scene type.

------------------------------------------------------------------------

# Recommended Order

1.  ~~Brute-force body pair testing~~
2.  ~~Body AABB broadphase~~
3.  ~~TriangleMesh triangle-AABB prefilter~~
4.  ~~Cached contact generation~~
5.  ~~Box↔Mesh robustness~~
6.  ~~TriangleMesh BVH~~
7.  ~~Performance instrumentation & regression testing~~
8.  ~~Spatial hash body broadphase~~
9.  ➜ Multi-contact manifold clipping
10. ➜ Contact reduction / prioritization
11. ➜ Additional arbitrary mesh stress testing
12. ➜ Spatial hash benchmarking and tuning
13. ➜ BVH optimization (SAH / tuning / profiling)
14. ➜ Sleeping and island management
15. ➜ Future broadphase alternatives if profiling justifies them

------------------------------------------------------------------------

# TODO

## Completed

### Broadphase

-   ~~Brute-force body pair testing~~
-   ~~Body AABB generation~~
-   ~~Body AABB overlap testing~~
-   ~~Candidate body pair generation~~
-   ~~Skip static-static body pairs~~
-   ~~Broadphase integration~~
-   ~~Spatial hash grid broadphase~~
-   ~~Small-scene brute-force fallback~~
-   ~~Oversized-AABB fallback~~
-   ~~Duplicate body-pair suppression~~
-   ~~Final AABB overlap check after hash lookup~~

### TriangleMesh

-   ~~Triangle AABB prefilter~~
-   ~~Triangle candidate generation~~
-   ~~Skip distant triangles~~
-   ~~TriangleMesh BVH implementation~~
-   ~~Native BVH node storage~~
-   ~~Median split BVH builder~~
-   ~~Iterative BVH traversal~~
-   ~~Sphere↔Mesh BVH queries~~
-   ~~Box↔Mesh BVH queries~~
-   ~~Capsule↔Mesh BVH queries~~
-   ~~No GC BVH traversal~~

### Solver

-   ~~Broadphase rebuilt once per simulation substep~~
-   ~~Collision detection executed once per simulation substep~~
-   ~~Cached contact manifold generation~~
-   ~~Reuse cached contacts across all solver iterations~~
-   ~~Removed repeated broadphase~~
-   ~~Removed repeated narrowphase~~

### Box ↔ Mesh

-   ~~Basic Box↔Triangle SAT~~
-   ~~Box-vertex-to-triangle-plane contacts~~
-   ~~Triangle vertices inside box~~
-   ~~Edge-edge closest-point contacts~~
-   ~~Improved Box↔Triangle manifold generation~~
-   ~~Improved Box↔Mesh collision behavior~~
-   ~~Improved terrain collision~~
-   ~~Improved wall, corner, and thin triangle support~~

### Performance

-   ~~Permanent physics performance counters~~
-   ~~BVH query statistics~~
-   ~~Candidate triangle statistics~~
-   ~~Triangle narrowphase statistics~~
-   ~~Automated BVH regression tests~~

------------------------------------------------------------------------

## Remaining

### Box ↔ Mesh

-   Multi-contact manifold clipping
-   Contact reduction / prioritization
-   Additional arbitrary mesh stress testing

### Broadphase

-   Benchmark spatial hash broadphase scalability
-   Tune spatial hash cell size
-   Add spatial hash occupancy counters
-   Add duplicate-pair regression tests
-   Add oversized-body fallback regression tests
-   Compare spatial hash against SAP for long, flat worlds

### TriangleMesh BVH

-   SAH BVH builder
-   BVH refit support
-   Parallel BVH construction
-   Debug visualization
-   Benchmark counters
-   Tune leaf size
-   Performance profiling

### Future Broadphase

-   Sweep and Prune (SAP)
-   Dynamic AABB Tree
-   Pair cache
-   Incremental updates
-   Multithreaded broadphase

### Solver / Simulation Scale

-   Sleeping
-   Island management
-   Wake/sleep propagation
-   Skip inactive islands

### Performance

-   Compare BVH against previous full triangle scan
-   Profile large terrain performance
-   Tune BVH leaf size using collected statistics
-   Profile spatial hash performance across dense and sparse scenes
