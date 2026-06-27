# Shard Physics Optimization Roadmap

## Current State

The engine now performs:

1.  **Body-level broadphase** using world-space AABBs to reject
    non-overlapping body pairs.
2.  Narrowphase collision detection only for candidate body pairs.
3.  TriangleMesh collisions use a **TriangleMesh BVH** to query only
    nearby triangles before narrowphase.
4.  Candidate triangles then pass through a **triangle AABB prefilter**
    before primitive-vs-triangle narrowphase.
5.  **Collision detection runs once per simulation substep instead of
    once per solver iteration.**
6.  **Generated contact manifolds are cached and reused across every
    solver iteration.**
7.  **Broadphase and narrowphase are no longer repeatedly executed
    during iterative impulse solving.**
8.  Box↔Mesh collision now works through robust Box↔Triangle contacts.
9.  Box↔Mesh collision supports terrain-style surfaces, vertical walls,
    corners, edges, and thin triangle cases.
10. The entire collision pipeline remains **NativeMemory-based**, with
    no managed allocations during simulation.
11. Built-in **performance instrumentation** tracks BVH efficiency and
    collision pipeline statistics.
12. Permanent regression tests verify BVH effectiveness and help catch
    future performance regressions.

Current collision pipeline:

``` text
Simulation Substep

Body Broadphase
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
narrowphase, greatly reducing unnecessary triangle testing.

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

# Step 7 --- Future Broadphase Improvements

After the engine is stable:

-   Spatial hash grid
-   Sweep and Prune (SAP)
-   Dynamic AABB Tree
-   Pair cache
-   Incremental broadphase updates
-   Sharded multithreaded broadphase

These would replace the current simple body-AABB broadphase for large
dynamic scenes.

------------------------------------------------------------------------

# Recommended Order

1.  ~~Brute-force body pair testing~~
2.  ~~Body AABB broadphase~~
3.  ~~TriangleMesh triangle-AABB prefilter~~
4.  ~~Cached contact generation~~
5.  ~~Box↔Mesh robustness~~
6.  ~~TriangleMesh BVH~~
7.  ~~Performance instrumentation & regression testing~~
8.  ➜ BVH optimization (SAH / tuning / profiling)
9.  ➜ Advanced body broadphase (Grid / SAP / Dynamic AABB Tree)

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

------------------------------------------------------------------------

## Remaining

### Box ↔ Mesh

-   Multi-contact manifold clipping
-   Contact reduction / prioritization
-   Additional arbitrary mesh stress testing

### TriangleMesh BVH

-   SAH BVH builder
-   BVH refit support
-   Parallel BVH construction
-   Debug visualization
-   Benchmark counters
-   Tune leaf size
-   Performance profiling

### Performance

-   ~~Permanent physics performance counters~~
-   ~~BVH query statistics~~
-   ~~Candidate triangle statistics~~
-   ~~Triangle narrowphase statistics~~
-   ~~Automated BVH regression tests~~

### Advanced Broadphase

-   Spatial hash grid
-   Sweep and Prune (SAP)
-   Dynamic AABB Tree
-   Pair cache
-   Incremental updates
-   Multithreaded broadphase

### Performance

-   Benchmark broadphase scalability
-   Compare BVH against previous full triangle scan
-   Profile large terrain performance
-   Tune BVH leaf size using collected statistics
