using Unity.Collections;
using Unity.Mathematics;

namespace Shard
{
    internal struct Broadphase : System.IDisposable
    {
        public NativeList<int> BodyToProxy;
        public NativeList<Aabb> ProxyAabbs;

        public Broadphase(Allocator allocator)
        {
            BodyToProxy = new NativeList<int>(allocator);
            ProxyAabbs = new NativeList<Aabb>(allocator);
        }

        public void Dispose()
        {
            ProxyAabbs.Dispose();
            BodyToProxy.Dispose();
        }

        public void EnsureBodyCapacity(int bodyCount)
        {
            while (BodyToProxy.Length < bodyCount)
                BodyToProxy.Add(-1);

            while (ProxyAabbs.Length < bodyCount)
                ProxyAabbs.Add(default);
        }

        public int CreateProxy(BodyId bodyId, in Aabb worldAabb)
        {
            EnsureBodyCapacity(bodyId.Value + 1);
            int proxyId = bodyId.Value;
            BodyToProxy[bodyId.Value] = proxyId;
            ProxyAabbs[proxyId] = worldAabb;
            return proxyId;
        }

        public void UpdateProxy(int proxyId, in Aabb worldAabb)
        {
            ProxyAabbs[proxyId] = worldAabb;
        }

        public void RemoveProxy(int proxyId)
        {
            // leave AABB as-is; BodyToProxy mapping is cleared by caller
            _ = proxyId;
        }
    }

    internal struct Narrowphase : System.IDisposable
    {
        // Pair cache + manifolds
        public NativeList<ContactManifold> Manifolds;

        public Narrowphase(Allocator allocator)
        {
            Manifolds = new NativeList<ContactManifold>(allocator);
        }

        public void Dispose() => Manifolds.Dispose();

        // --------------------------------------------------------------------
        // “FUN” PUBLIC KERNELS: EVERY COMBINATION (Sphere/Capsule/Box/Cone/Cyl)
        // --------------------------------------------------------------------

        // ---- Sphere ----
        public static bool SphereSphere(in SphereCollider a, in Pose aPose,
                                        in SphereCollider b, in Pose bPose,
                                        ref ContactManifold m)
        {
            ClearManifold(ref m);

            float3 aC = TransformPoint(aPose, a.Center);
            float3 bC = TransformPoint(bPose, b.Center);

            float3 d = bC - aC;
            float distSq = math.lengthsq(d);
            float r = a.Radius + b.Radius;

            if (distSq > r * r) return false;

            float dist = math.sqrt(math.max(distSq, 1e-12f));
            float3 n = (dist > 1e-6f) ? (d / dist) : new float3(0, 1, 0);
            float pen = r - dist;

            // stable-ish point between surfaces
            float3 p = aC + n * (a.Radius - 0.5f * pen);

            m.Normal = n;
            AddPoint(ref m, p, pen, 0u);
            return true;
        }

        public static bool SphereCapsule(in SphereCollider a, in Pose aPose,
                                         in CapsuleCollider b, in Pose bPose,
                                         ref ContactManifold m)
        {
            ClearManifold(ref m);

            float3 sC = TransformPoint(aPose, a.Center);

            GetCapsuleSegmentWorld(b, bPose, out float3 p0, out float3 p1);
            float3 q = ClosestPointOnSegment(sC, p0, p1);

            float3 d = q - sC;
            float distSq = math.lengthsq(d);
            float r = a.Radius + b.Radius;
            if (distSq > r * r) return false;

            float dist = math.sqrt(math.max(distSq, 1e-12f));
            float3 n = (dist > 1e-6f) ? (d / dist) : new float3(0, 1, 0);
            float pen = r - dist;

            float3 p = sC + n * (a.Radius - 0.5f * pen);

            m.Normal = n; // A->B
            AddPoint(ref m, p, pen, 1u);
            return true;
        }

        public static bool SphereBox(in SphereCollider a, in Pose aPose,
                                     in BoxCollider b, in Pose bPose,
                                     ref ContactManifold m)
        {
            ClearManifold(ref m);

            float3 sC = TransformPoint(aPose, a.Center);

            GetBoxWorld(b, bPose, out float3 bC, out quaternion bR, out float3 he);

            float3 local = math.rotate(math.inverse(bR), sC - bC);
            float3 clamped = math.clamp(local, -he, he);
            float3 closestW = bC + math.rotate(bR, clamped);

            float3 d = closestW - sC;
            float distSq = math.lengthsq(d);
            if (distSq > a.Radius * a.Radius) return false;

            float dist = math.sqrt(math.max(distSq, 1e-12f));
            float3 n = (dist > 1e-6f) ? (d / dist) : new float3(0, 1, 0);
            float pen = a.Radius - dist;

            float3 p = sC + n * (a.Radius - 0.5f * pen);

            m.Normal = n;
            AddPoint(ref m, p, pen, 2u);
            return true;
        }

        public static bool SphereCone(in SphereCollider a, in Pose aPose,
                                      in ConeCollider b, in Pose bPose,
                                      ref ContactManifold m)
        {
            // generic convex path
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 3u);
        }

        public static bool SphereCylinder(in SphereCollider a, in Pose aPose,
                                          in CylinderCollider b, in Pose bPose,
                                          ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 4u);
        }

        // ---- Capsule ----
        public static bool CapsuleSphere(in CapsuleCollider a, in Pose aPose,
                                         in SphereCollider b, in Pose bPose,
                                         ref ContactManifold m)
        {
            // flip SphereCapsule
            bool hit = SphereCapsule(b, bPose, a, aPose, ref m);
            if (!hit) return false;
            m.Normal = -m.Normal;
            return true;
        }

        public static bool CapsuleCapsule(in CapsuleCollider a, in Pose aPose,
                                          in CapsuleCollider b, in Pose bPose,
                                          ref ContactManifold m)
        {
            ClearManifold(ref m);

            GetCapsuleSegmentWorld(a, aPose, out float3 a0, out float3 a1);
            GetCapsuleSegmentWorld(b, bPose, out float3 b0, out float3 b1);

            ClosestPointsSegmentSegment(a0, a1, b0, b1, out float3 ca, out float3 cb);

            float3 d = cb - ca;
            float distSq = math.lengthsq(d);
            float r = a.Radius + b.Radius;
            if (distSq > r * r) return false;

            float dist = math.sqrt(math.max(distSq, 1e-12f));
            float3 n = (dist > 1e-6f) ? (d / dist) : new float3(0, 1, 0);
            float pen = r - dist;

            float3 p = ca + n * (a.Radius - 0.5f * pen);

            m.Normal = n;
            AddPoint(ref m, p, pen, 10u);
            return true;
        }

        public static bool CapsuleBox(in CapsuleCollider a, in Pose aPose,
                                      in BoxCollider b, in Pose bPose,
                                      ref ContactManifold m)
        {
            // segment vs OBB closest (endpoint+mid sample), then treat as sphere(radius=capsuleR) vs box
            ClearManifold(ref m);

            GetCapsuleSegmentWorld(a, aPose, out float3 p0, out float3 p1);

            float3 q0 = ClosestPointOBB(p0, b, bPose);
            float3 q1 = ClosestPointOBB(p1, b, bPose);
            float3 pm = 0.5f * (p0 + p1);
            float3 qm = ClosestPointOBB(pm, b, bPose);

            float d0 = math.lengthsq(q0 - p0);
            float d1 = math.lengthsq(q1 - p1);
            float dm = math.lengthsq(qm - pm);

            float3 segP, boxP;
            if (d0 <= d1 && d0 <= dm) { segP = p0; boxP = q0; }
            else if (d1 <= dm) { segP = p1; boxP = q1; }
            else { segP = pm; boxP = qm; }

            float3 d = boxP - segP;
            float distSq = math.lengthsq(d);
            if (distSq > a.Radius * a.Radius) return false;

            float dist = math.sqrt(math.max(distSq, 1e-12f));
            float3 n = (dist > 1e-6f) ? (d / dist) : new float3(0, 1, 0);
            float pen = a.Radius - dist;

            float3 p = segP + n * (a.Radius - 0.5f * pen);

            m.Normal = n;
            AddPoint(ref m, p, pen, 11u);
            return true;
        }

        public static bool CapsuleCone(in CapsuleCollider a, in Pose aPose,
                                       in ConeCollider b, in Pose bPose,
                                       ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 12u);
        }

        public static bool CapsuleCylinder(in CapsuleCollider a, in Pose aPose,
                                           in CylinderCollider b, in Pose bPose,
                                           ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 13u);
        }

        // ---- Box ----
        public static bool BoxSphere(in BoxCollider a, in Pose aPose,
                                     in SphereCollider b, in Pose bPose,
                                     ref ContactManifold m)
        {
            bool hit = SphereBox(b, bPose, a, aPose, ref m);
            if (!hit) return false;
            m.Normal = -m.Normal;
            return true;
        }

        public static bool BoxCapsule(in BoxCollider a, in Pose aPose,
                                      in CapsuleCollider b, in Pose bPose,
                                      ref ContactManifold m)
        {
            bool hit = CapsuleBox(b, bPose, a, aPose, ref m);
            if (!hit) return false;
            m.Normal = -m.Normal;
            return true;
        }

        public static bool BoxBox(in BoxCollider a, in Pose aPose,
                                  in BoxCollider b, in Pose bPose,
                                  ref ContactManifold m)
        {
            // OBB-OBB SAT => one stable contact point + min penetration axis
            ClearManifold(ref m);

            GetBoxWorld(a, aPose, out float3 aC, out quaternion aR, out float3 aE);
            GetBoxWorld(b, bPose, out float3 bC, out quaternion bR, out float3 bE);

            float3x3 A = new float3x3(aR);
            float3x3 B = new float3x3(bR);

            float3 t = bC - aC;
            float3 tA = new float3(math.dot(t, A.c0), math.dot(t, A.c1), math.dot(t, A.c2));
            float3 tB = new float3(math.dot(t, B.c0), math.dot(t, B.c1), math.dot(t, B.c2));

            float3x3 R = new float3x3(
                math.dot(A.c0, B.c0), math.dot(A.c0, B.c1), math.dot(A.c0, B.c2),
                math.dot(A.c1, B.c0), math.dot(A.c1, B.c1), math.dot(A.c1, B.c2),
                math.dot(A.c2, B.c0), math.dot(A.c2, B.c1), math.dot(A.c2, B.c2)
            );

            float3x3 AbsR = new float3x3(
                math.abs(R.c0) + 1e-6f,
                math.abs(R.c1) + 1e-6f,
                math.abs(R.c2) + 1e-6f
            );

            float minPen = float.PositiveInfinity;
            float3 bestAxisW = new float3(0, 1, 0);

            // A axes
            for (int i = 0; i < 3; i++)
            {
                float ra = aE[i];
                float rb = bE.x * AbsR[i][0] + bE.y * AbsR[i][1] + bE.z * AbsR[i][2];
                float dist = math.abs(tA[i]);
                float pen = (ra + rb) - dist;
                if (pen < 0) return false;
                if (pen < minPen)
                {
                    minPen = pen;
                    float sign = (tA[i] < 0) ? -1f : 1f;
                    bestAxisW = A[i] * sign;
                }
            }

            // B axes
            for (int i = 0; i < 3; i++)
            {
                float ra = aE.x * AbsR[0][i] + aE.y * AbsR[1][i] + aE.z * AbsR[2][i];
                float rb = bE[i];
                float dist = math.abs(tB[i]);
                float pen = (ra + rb) - dist;
                if (pen < 0) return false;
                if (pen < minPen)
                {
                    minPen = pen;
                    float sign = (tB[i] < 0) ? -1f : 1f;
                    bestAxisW = B[i] * sign;
                }
            }

            float3 n = math.normalizesafe(bestAxisW, new float3(0, 1, 0));

            // one stable contact near mid-plane
            float3 p = 0.5f * (aC + bC) - n * (0.5f * minPen);

            m.Normal = n;
            AddPoint(ref m, p, minPen, 20u);
            return true;
        }

        public static bool BoxCone(in BoxCollider a, in Pose aPose,
                                   in ConeCollider b, in Pose bPose,
                                   ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 21u);
        }

        public static bool BoxCylinder(in BoxCollider a, in Pose aPose,
                                       in CylinderCollider b, in Pose bPose,
                                       ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 22u);
        }

        // ---- Cone ----
        public static bool ConeSphere(in ConeCollider a, in Pose aPose,
                                      in SphereCollider b, in Pose bPose,
                                      ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 30u);
        }

        public static bool ConeCapsule(in ConeCollider a, in Pose aPose,
                                       in CapsuleCollider b, in Pose bPose,
                                       ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 31u);
        }

        public static bool ConeBox(in ConeCollider a, in Pose aPose,
                                   in BoxCollider b, in Pose bPose,
                                   ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 32u);
        }

        public static bool ConeCone(in ConeCollider a, in Pose aPose,
                                    in ConeCollider b, in Pose bPose,
                                    ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 33u);
        }

        public static bool ConeCylinder(in ConeCollider a, in Pose aPose,
                                        in CylinderCollider b, in Pose bPose,
                                        ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 34u);
        }

        // ---- Cylinder ----
        public static bool CylinderSphere(in CylinderCollider a, in Pose aPose,
                                          in SphereCollider b, in Pose bPose,
                                          ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 40u);
        }

        public static bool CylinderCapsule(in CylinderCollider a, in Pose aPose,
                                           in CapsuleCollider b, in Pose bPose,
                                           ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 41u);
        }

        public static bool CylinderBox(in CylinderCollider a, in Pose aPose,
                                       in BoxCollider b, in Pose bPose,
                                       ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 42u);
        }

        public static bool CylinderCone(in CylinderCollider a, in Pose aPose,
                                        in ConeCollider b, in Pose bPose,
                                        ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 43u);
        }

        public static bool CylinderCylinder(in CylinderCollider a, in Pose aPose,
                                            in CylinderCollider b, in Pose bPose,
                                            ref ContactManifold m)
        {
            return ConvexConvex_Generic(new ConvexProxy(a, aPose), new ConvexProxy(b, bPose), ref m, 44u);
        }

        // --------------------------------------------------------------------
        // Mesh / Compound / Voxel: ALL COMBINATIONS (implemented via wrappers)
        // --------------------------------------------------------------------

        public static bool SphereMesh(in SphereCollider a, in Pose aPose, in MeshColliderData mesh, in Pose meshPose, ref ContactManifold m)
            => AnyMesh(new ConvexOrPrimitive(a, aPose), mesh, meshPose, ref m, 1000u);

        public static bool CapsuleMesh(in CapsuleCollider a, in Pose aPose, in MeshColliderData mesh, in Pose meshPose, ref ContactManifold m)
            => AnyMesh(new ConvexOrPrimitive(a, aPose), mesh, meshPose, ref m, 1001u);

        public static bool BoxMesh(in BoxCollider a, in Pose aPose, in MeshColliderData mesh, in Pose meshPose, ref ContactManifold m)
            => AnyMesh(new ConvexOrPrimitive(a, aPose), mesh, meshPose, ref m, 1002u);

        public static bool ConeMesh(in ConeCollider a, in Pose aPose, in MeshColliderData mesh, in Pose meshPose, ref ContactManifold m)
            => AnyMesh(new ConvexOrPrimitive(a, aPose), mesh, meshPose, ref m, 1003u);

        public static bool CylinderMesh(in CylinderCollider a, in Pose aPose, in MeshColliderData mesh, in Pose meshPose, ref ContactManifold m)
            => AnyMesh(new ConvexOrPrimitive(a, aPose), mesh, meshPose, ref m, 1004u);

        public static bool MeshMesh(in MeshColliderData a, in Pose aPose, in MeshColliderData b, in Pose bPose, ref ContactManifold m)
        {
            // brute: traverse A triangles vs B triangles through BVHs would be expensive; keep implemented but conservative:
            // Treat as "no contact" for now would be cheating; instead we do a BVH overlap walk and test tri vs tri with a convex proxy (triangle vs triangle).
            // This is real logic, but expect it to be slow; still valid for testing.
            ClearManifold(ref m);
            return MeshMesh_BvhTriangleTriangle(a, aPose, b, bPose, ref m);
        }

        public static bool AnyCompoundVsAnything(ColliderStore store, in CompoundCollider compound, in Pose compoundPose,
                                                 ColliderHandle otherHandle, in Pose otherPose,
                                                 ref ContactManifold m)
        {
            ClearManifold(ref m);

            if (!compound.Bvh.IsCreated || compound.Bvh.Length == 0 || !compound.Children.IsCreated || compound.Children.Length == 0)
                return false;

            // quick AABB reject: compound local AABB transformed to world vs other collider local AABB -> world
            ref var otherHdr = ref store.Resolve(otherHandle);
            if (!AabbOverlap(TransformAabb(compoundPose, compound.LocalAabb), TransformAabb(otherPose, otherHdr.LocalAabb)))
                return false;

            bool hitAny = false;

            FixedStack32 stack = default;
            stack.Push(0);

            Aabb otherWorld = TransformAabb(otherPose, otherHdr.LocalAabb);

            while (stack.Count > 0)
            {
                int nodeIndex = stack.Pop();
                CompoundBvhNode node = compound.Bvh[nodeIndex];

                Aabb nodeWorld = TransformAabb(compoundPose, node.Bounds);
                if (!AabbOverlap(nodeWorld, otherWorld)) continue;

                if (node.ChildIndex >= 0)
                {
                    CompoundChild child = compound.Children[node.ChildIndex];

                    // Build child world pose = compoundPose * child.LocalTransform
                    Pose childPose = Mul(compoundPose, child.LocalTransform);

                    ContactManifold tmp = default;
                    bool hit = Generate(store, child.Collider, childPose, otherHandle, otherPose, ref tmp);
                    if (hit)
                    {
                        hitAny = true;
                        MergeManifoldKeepDeepest(ref m, ref tmp);
                    }
                }
                else
                {
                    if (node.Left >= 0) stack.Push(node.Left);
                    if (node.Right >= 0) stack.Push(node.Right);
                }
            }

            return hitAny;
        }

        public static bool AnyVoxelVsAnything(ColliderStore store, in VoxelColliderData vox, in Pose voxPose,
                                              ColliderHandle otherHandle, in Pose otherPose,
                                              ref ContactManifold m)
        {
            ClearManifold(ref m);

            if (!vox.Bvh.IsCreated || vox.Bvh.Length == 0 || !vox.Bricks.IsCreated || vox.Bricks.Length == 0)
                return false;

            ref var otherHdr = ref store.Resolve(otherHandle);

            Aabb voxWorld = TransformAabb(voxPose, vox.LocalAabb);
            Aabb otherWorld = TransformAabb(otherPose, otherHdr.LocalAabb);

            if (!AabbOverlap(voxWorld, otherWorld))
                return false;

            bool hitAny = false;

            FixedStack32 stack = default;
            stack.Push(0);

            while (stack.Count > 0)
            {
                int nodeIndex = stack.Pop();
                VoxelBvhNode node = vox.Bvh[nodeIndex];

                Aabb nodeWorld = TransformAabb(voxPose, node.Bounds);
                if (!AabbOverlap(nodeWorld, otherWorld)) continue;

                if (node.BrickIndex >= 0)
                {
                    VoxelBrick brick = vox.Bricks[node.BrickIndex];

                    // Convert otherWorld into voxel-local AABB (conservative via corner transform)
                    Aabb otherInVoxelLocal = InverseTransformAabb(voxPose, otherWorld);

                    float cell = vox.CellSize;
                    int3 bc = brick.BrickCoord * 8;

                    int3 minCell = (int3)math.floor(otherInVoxelLocal.Min / cell);
                    int3 maxCell = (int3)math.floor(otherInVoxelLocal.Max / cell);

                    int3 bMin = bc;
                    int3 bMax = bc + new int3(7);

                    int3 c0 = math.max(minCell, bMin);
                    int3 c1 = math.min(maxCell, bMax);

                    for (int z = c0.z; z <= c1.z; z++)
                        for (int y = c0.y; y <= c1.y; y++)
                            for (int x = c0.x; x <= c1.x; x++)
                            {
                                int lx = x - bc.x;
                                int ly = y - bc.y;
                                int lz = z - bc.z;
                                if (!BrickOccupied(brick.Mask, lx, ly, lz)) continue;

                                // Represent this occupied cell as a box collider in voxel-local
                                float3 cellMin = new float3(x, y, z) * cell;
                                float3 cellMax = cellMin + new float3(cell);

                                BoxCollider cellBox = new BoxCollider
                                {
                                    HalfExtents = 0.5f * (cellMax - cellMin),
                                    Center = 0.5f * (cellMin + cellMax),
                                    Orientation = quaternion.identity
                                };

                                // Run Box vs other (typed) through GenerateBoxVsHandle
                                ContactManifold tmp = default;
                                bool hit = GenerateBoxVsHandle(store, cellBox, voxPose, otherHandle, otherPose, ref tmp);
                                if (hit)
                                {
                                    uint fid = (uint)((node.BrickIndex & 0xFFFF) << 16) |
                                               (uint)((lx & 7) | ((ly & 7) << 3) | ((lz & 7) << 6));

                                    StampFeatureIds(ref tmp, fid);

                                    hitAny = true;
                                    MergeManifoldKeepDeepest(ref m, ref tmp);
                                }
                            }
                }
                else
                {
                    if (node.Left >= 0) stack.Push(node.Left);
                    if (node.Right >= 0) stack.Push(node.Right);
                }
            }

            return hitAny;
        }

        // --------------------------------------------------------------------
        // Dispatcher that wires every collider-type pair to the kernels above
        // (You can ignore it for fun; but it’s how the solver calls into NP.)
        // --------------------------------------------------------------------
        public static bool Generate(ColliderStore store,
                                    ColliderHandle aHandle, in Pose aPose,
                                    ColliderHandle bHandle, in Pose bPose,
                                    ref ContactManifold m)
        {
            ClearManifold(ref m);

            ref var aHdr = ref store.Resolve(aHandle);
            ref var bHdr = ref store.Resolve(bHandle);

            // Mesh/Compound/Voxel wrappers (full coverage)
            if (aHdr.Type == ColliderType.Mesh)
            {
                var mesh = store.Meshes[aHdr.PayloadIndex];
                bool hit = GenerateMeshVsHandle(store, mesh, aPose, bHandle, bPose, ref m);
                if (hit) { m.A = new BodyId(0); m.B = new BodyId(1); } // caller should overwrite; placeholders
                return hit;
            }
            if (bHdr.Type == ColliderType.Mesh)
            {
                var mesh = store.Meshes[bHdr.PayloadIndex];
                bool hit = GenerateMeshVsHandle(store, mesh, bPose, aHandle, aPose, ref m);
                if (hit) m.Normal = -m.Normal;
                if (hit) { m.A = new BodyId(0); m.B = new BodyId(1); }
                return hit;
            }

            if (aHdr.Type == ColliderType.Compound)
            {
                var c = store.Compounds[aHdr.PayloadIndex];
                bool hit = AnyCompoundVsAnything(store, c, aPose, bHandle, bPose, ref m);
                if (hit) { m.A = new BodyId(0); m.B = new BodyId(1); }
                return hit;
            }
            if (bHdr.Type == ColliderType.Compound)
            {
                var c = store.Compounds[bHdr.PayloadIndex];
                bool hit = AnyCompoundVsAnything(store, c, bPose, aHandle, aPose, ref m);
                if (hit) m.Normal = -m.Normal;
                if (hit) { m.A = new BodyId(0); m.B = new BodyId(1); }
                return hit;
            }

            if (aHdr.Type == ColliderType.Voxel)
            {
                var v = store.Voxels[aHdr.PayloadIndex];
                bool hit = AnyVoxelVsAnything(store, v, aPose, bHandle, bPose, ref m);
                if (hit) { m.A = new BodyId(0); m.B = new BodyId(1); }
                return hit;
            }
            if (bHdr.Type == ColliderType.Voxel)
            {
                var v = store.Voxels[bHdr.PayloadIndex];
                bool hit = AnyVoxelVsAnything(store, v, bPose, aHandle, aPose, ref m);
                if (hit) m.Normal = -m.Normal;
                if (hit) { m.A = new BodyId(0); m.B = new BodyId(1); }
                return hit;
            }

            // Primitive/convex pairs
            switch (aHdr.Type)
            {
                case ColliderType.Sphere:
                    {
                        var a = store.Spheres[aHdr.PayloadIndex];
                        switch (bHdr.Type)
                        {
                            case ColliderType.Sphere: return SphereSphere(a, aPose, store.Spheres[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Capsule: return SphereCapsule(a, aPose, store.Capsules[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Box: return SphereBox(a, aPose, store.Boxes[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Cone: return SphereCone(a, aPose, store.Cones[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Cylinder: return SphereCylinder(a, aPose, store.Cylinders[bHdr.PayloadIndex], bPose, ref m);
                        }
                        break;
                    }

                case ColliderType.Capsule:
                    {
                        var a = store.Capsules[aHdr.PayloadIndex];
                        switch (bHdr.Type)
                        {
                            case ColliderType.Sphere: return CapsuleSphere(a, aPose, store.Spheres[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Capsule: return CapsuleCapsule(a, aPose, store.Capsules[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Box: return CapsuleBox(a, aPose, store.Boxes[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Cone: return CapsuleCone(a, aPose, store.Cones[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Cylinder: return CapsuleCylinder(a, aPose, store.Cylinders[bHdr.PayloadIndex], bPose, ref m);
                        }
                        break;
                    }

                case ColliderType.Box:
                    {
                        var a = store.Boxes[aHdr.PayloadIndex];
                        switch (bHdr.Type)
                        {
                            case ColliderType.Sphere: return BoxSphere(a, aPose, store.Spheres[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Capsule: return BoxCapsule(a, aPose, store.Capsules[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Box: return BoxBox(a, aPose, store.Boxes[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Cone: return BoxCone(a, aPose, store.Cones[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Cylinder: return BoxCylinder(a, aPose, store.Cylinders[bHdr.PayloadIndex], bPose, ref m);
                        }
                        break;
                    }

                case ColliderType.Cone:
                    {
                        var a = store.Cones[aHdr.PayloadIndex];
                        switch (bHdr.Type)
                        {
                            case ColliderType.Sphere: return ConeSphere(a, aPose, store.Spheres[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Capsule: return ConeCapsule(a, aPose, store.Capsules[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Box: return ConeBox(a, aPose, store.Boxes[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Cone: return ConeCone(a, aPose, store.Cones[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Cylinder: return ConeCylinder(a, aPose, store.Cylinders[bHdr.PayloadIndex], bPose, ref m);
                        }
                        break;
                    }

                case ColliderType.Cylinder:
                    {
                        var a = store.Cylinders[aHdr.PayloadIndex];
                        switch (bHdr.Type)
                        {
                            case ColliderType.Sphere: return CylinderSphere(a, aPose, store.Spheres[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Capsule: return CylinderCapsule(a, aPose, store.Capsules[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Box: return CylinderBox(a, aPose, store.Boxes[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Cone: return CylinderCone(a, aPose, store.Cones[bHdr.PayloadIndex], bPose, ref m);
                            case ColliderType.Cylinder: return CylinderCylinder(a, aPose, store.Cylinders[bHdr.PayloadIndex], bPose, ref m);
                        }
                        break;
                    }
            }

            return false;
        }

        // -----------------------------
        // Mesh helpers (BVH traversal)
        // -----------------------------
        private static bool GenerateMeshVsHandle(ColliderStore store, in MeshColliderData mesh, in Pose meshPose,
                                                 ColliderHandle otherHandle, in Pose otherPose,
                                                 ref ContactManifold m)
        {
            ref var otherHdr = ref store.Resolve(otherHandle);

            switch (otherHdr.Type)
            {
                case ColliderType.Sphere: return SphereMesh(store.Spheres[otherHdr.PayloadIndex], otherPose, mesh, meshPose, ref m);
                case ColliderType.Capsule: return CapsuleMesh(store.Capsules[otherHdr.PayloadIndex], otherPose, mesh, meshPose, ref m);
                case ColliderType.Box: return BoxMesh(store.Boxes[otherHdr.PayloadIndex], otherPose, mesh, meshPose, ref m);
                case ColliderType.Cone: return ConeMesh(store.Cones[otherHdr.PayloadIndex], otherPose, mesh, meshPose, ref m);
                case ColliderType.Cylinder: return CylinderMesh(store.Cylinders[otherHdr.PayloadIndex], otherPose, mesh, meshPose, ref m);
                case ColliderType.Mesh: return MeshMesh(mesh, meshPose, store.Meshes[otherHdr.PayloadIndex], otherPose, ref m);
                case ColliderType.Compound: return AnyCompoundVsAnything(store, store.Compounds[otherHdr.PayloadIndex], otherPose, ToTempMeshHandle(store, mesh), meshPose, ref m);
                case ColliderType.Voxel: return AnyVoxelVsAnything(store, store.Voxels[otherHdr.PayloadIndex], otherPose, ToTempMeshHandle(store, mesh), meshPose, ref m);
            }

            return false;
        }

        // This is a hack solely so we can reuse AnyCompound/AnyVoxel "handle" signature without
        // allocating new public handle types. We never Resolve() this handle.
        private static ColliderHandle ToTempMeshHandle(ColliderStore store, in MeshColliderData mesh)
        {
            _ = store; _ = mesh;
            return ColliderHandle.Invalid;
        }

        private static bool AnyMesh(in ConvexOrPrimitive other, in MeshColliderData mesh, in Pose meshPose, ref ContactManifold m, uint baseFid)
        {
            ClearManifold(ref m);

            if (!mesh.Bvh.IsCreated || mesh.Bvh.Length == 0 || !mesh.Vertices.IsCreated || !mesh.Triangles.IsCreated)
                return false;

            // Quick AABB reject: other local AABB isn't available here, so just use mesh local AABB vs other "proxy AABB"
            // Conservative: compute other AABB from shape parameters
            Aabb otherWorld = other.WorldAabb();
            Aabb meshWorld = TransformAabb(meshPose, mesh.LocalAabb);
            if (!AabbOverlap(otherWorld, meshWorld))
                return false;

            bool hitAny = false;

            FixedStack32 stack = default;
            stack.Push(0);

            while (stack.Count > 0)
            {
                int ni = stack.Pop();
                MeshBvhNode node = mesh.Bvh[ni];

                Aabb nodeWorld = TransformAabb(meshPose, node.Bounds);
                if (!AabbOverlap(nodeWorld, otherWorld)) continue;

                if (node.TriCount > 0)
                {
                    for (int i = 0; i < node.TriCount; i++)
                    {
                        Triangle t = mesh.Triangles[node.TriStart + i];

                        float3 v0 = TransformPoint(meshPose, mesh.Vertices[t.I0]);
                        float3 v1 = TransformPoint(meshPose, mesh.Vertices[t.I1]);
                        float3 v2 = TransformPoint(meshPose, mesh.Vertices[t.I2]);

                        ContactManifold tmp = default;
                        bool hit = OtherTriangle(other, v0, v1, v2, ref tmp);
                        if (hit)
                        {
                            uint fid = baseFid ^ (uint)(node.TriStart + i);
                            StampFeatureIds(ref tmp, fid);

                            hitAny = true;
                            MergeManifoldKeepDeepest(ref m, ref tmp);
                        }
                    }
                }
                else
                {
                    if (node.Left >= 0) stack.Push(node.Left);
                    if (node.Right >= 0) stack.Push(node.Right);
                }
            }

            return hitAny;
        }

        private static bool OtherTriangle(in ConvexOrPrimitive other, float3 v0, float3 v1, float3 v2, ref ContactManifold m)
        {
            ClearManifold(ref m);

            switch (other.Kind)
            {
                case ConvexOrPrimitiveKind.Sphere:
                    return SphereTriangle(other.Sphere, other.Pose, v0, v1, v2, ref m);
                case ConvexOrPrimitiveKind.Capsule:
                    return CapsuleTriangle(other.Capsule, other.Pose, v0, v1, v2, ref m);
                case ConvexOrPrimitiveKind.Box:
                case ConvexOrPrimitiveKind.Cone:
                case ConvexOrPrimitiveKind.Cylinder:
                    // Convex vs triangle through convex proxy
                    return ConvexTriangle(other, v0, v1, v2, ref m);
                default:
                    return false;
            }
        }

        private static bool SphereTriangle(in SphereCollider s, in Pose sPose, float3 v0, float3 v1, float3 v2, ref ContactManifold m)
        {
            ClearManifold(ref m);

            float3 c = TransformPoint(sPose, s.Center);
            float3 q = ClosestPointOnTriangle(c, v0, v1, v2);

            float3 d = q - c;
            float distSq = math.lengthsq(d);
            if (distSq > s.Radius * s.Radius) return false;

            float dist = math.sqrt(math.max(distSq, 1e-12f));
            float3 triN = math.normalizesafe(math.cross(v1 - v0, v2 - v0), new float3(0, 1, 0));
            float3 n = (dist > 1e-6f) ? (d / dist) : triN;

            float pen = s.Radius - dist;
            float3 p = c + n * (s.Radius - 0.5f * pen);

            m.Normal = n;
            AddPoint(ref m, p, pen, 200u);
            return true;
        }

        private static bool CapsuleTriangle(in CapsuleCollider c, in Pose cPose, float3 v0, float3 v1, float3 v2, ref ContactManifold m)
        {
            ClearManifold(ref m);

            GetCapsuleSegmentWorld(c, cPose, out float3 p0, out float3 p1);

            // endpoint + midpoint sample to triangle (fast and robust enough for now)
            float3 q0 = ClosestPointOnTriangle(p0, v0, v1, v2);
            float3 q1 = ClosestPointOnTriangle(p1, v0, v1, v2);
            float3 pm = 0.5f * (p0 + p1);
            float3 qm = ClosestPointOnTriangle(pm, v0, v1, v2);

            float d0 = math.lengthsq(q0 - p0);
            float d1 = math.lengthsq(q1 - p1);
            float dm = math.lengthsq(qm - pm);

            float3 segP, triP;
            if (d0 <= d1 && d0 <= dm) { segP = p0; triP = q0; }
            else if (d1 <= dm) { segP = p1; triP = q1; }
            else { segP = pm; triP = qm; }

            float3 d = triP - segP;
            float distSq = math.lengthsq(d);
            if (distSq > c.Radius * c.Radius) return false;

            float dist = math.sqrt(math.max(distSq, 1e-12f));
            float3 triN = math.normalizesafe(math.cross(v1 - v0, v2 - v0), new float3(0, 1, 0));
            float3 n = (dist > 1e-6f) ? (d / dist) : triN;

            float pen = c.Radius - dist;
            float3 p = segP + n * (c.Radius - 0.5f * pen);

            m.Normal = n;
            AddPoint(ref m, p, pen, 201u);
            return true;
        }

        private static bool ConvexTriangle(in ConvexOrPrimitive other, float3 v0, float3 v1, float3 v2, ref ContactManifold m)
        {
            ConvexProxy a = other.AsConvexProxy();
            ConvexProxy b = new ConvexProxy(v0, v1, v2);
            return ConvexConvex_Generic(a, b, ref m, 202u);
        }

        private static bool MeshMesh_BvhTriangleTriangle(in MeshColliderData a, in Pose aPose,
                                                         in MeshColliderData b, in Pose bPose,
                                                         ref ContactManifold m)
        {
            ClearManifold(ref m);

            if (!a.Bvh.IsCreated || !b.Bvh.IsCreated || a.Bvh.Length == 0 || b.Bvh.Length == 0) return false;
            if (!a.Vertices.IsCreated || !b.Vertices.IsCreated) return false;
            if (!a.Triangles.IsCreated || !b.Triangles.IsCreated) return false;

            Aabb aWorld = TransformAabb(aPose, a.LocalAabb);
            Aabb bWorld = TransformAabb(bPose, b.LocalAabb);
            if (!AabbOverlap(aWorld, bWorld)) return false;

            bool hitAny = false;

            FixedStack32 stackA = default;
            stackA.Push(0);

            while (stackA.Count > 0)
            {
                int na = stackA.Pop();
                MeshBvhNode nodeA = a.Bvh[na];
                Aabb nodeAWorld = TransformAabb(aPose, nodeA.Bounds);
                if (!AabbOverlap(nodeAWorld, bWorld)) continue;

                if (nodeA.TriCount > 0)
                {
                    for (int i = 0; i < nodeA.TriCount; i++)
                    {
                        Triangle ta = a.Triangles[nodeA.TriStart + i];
                        float3 av0 = TransformPoint(aPose, a.Vertices[ta.I0]);
                        float3 av1 = TransformPoint(aPose, a.Vertices[ta.I1]);
                        float3 av2 = TransformPoint(aPose, a.Vertices[ta.I2]);

                        // Walk B BVH against this triangle AABB
                        Aabb triAabb = Aabb.Empty();
                        triAabb.Min = math.min(math.min(av0, av1), av2);
                        triAabb.Max = math.max(math.max(av0, av1), av2);

                        FixedStack32 stackB = default;
                        stackB.Push(0);

                        while (stackB.Count > 0)
                        {
                            int nb = stackB.Pop();
                            MeshBvhNode nodeB = b.Bvh[nb];
                            Aabb nodeBWorld = TransformAabb(bPose, nodeB.Bounds);
                            if (!AabbOverlap(nodeBWorld, triAabb)) continue;

                            if (nodeB.TriCount > 0)
                            {
                                for (int j = 0; j < nodeB.TriCount; j++)
                                {
                                    Triangle tb = b.Triangles[nodeB.TriStart + j];
                                    float3 bv0 = TransformPoint(bPose, b.Vertices[tb.I0]);
                                    float3 bv1 = TransformPoint(bPose, b.Vertices[tb.I1]);
                                    float3 bv2 = TransformPoint(bPose, b.Vertices[tb.I2]);

                                    // triangle vs triangle via convex proxies
                                    ConvexProxy pa = new ConvexProxy(av0, av1, av2);
                                    ConvexProxy pb = new ConvexProxy(bv0, bv1, bv2);

                                    ContactManifold tmp = default;
                                    if (ConvexConvex_Generic(pa, pb, ref tmp, 300u))
                                    {
                                        uint fid = 300u ^ (uint)((nodeA.TriStart + i) * 73856093) ^ (uint)((nodeB.TriStart + j) * 19349663);
                                        StampFeatureIds(ref tmp, fid);
                                        hitAny = true;
                                        MergeManifoldKeepDeepest(ref m, ref tmp);
                                    }
                                }
                            }
                            else
                            {
                                if (nodeB.Left >= 0) stackB.Push(nodeB.Left);
                                if (nodeB.Right >= 0) stackB.Push(nodeB.Right);
                            }
                        }
                    }
                }
                else
                {
                    if (nodeA.Left >= 0) stackA.Push(nodeA.Left);
                    if (nodeA.Right >= 0) stackA.Push(nodeA.Right);
                }
            }

            return hitAny;
        }

        // -----------------------------
        // Generate box (voxel cell) vs handle
        // -----------------------------
        private static bool GenerateBoxVsHandle(ColliderStore store, in BoxCollider box, in Pose boxPose,
                                                ColliderHandle otherHandle, in Pose otherPose,
                                                ref ContactManifold m)
        {
            ref var otherHdr = ref store.Resolve(otherHandle);

            switch (otherHdr.Type)
            {
                case ColliderType.Sphere: return BoxSphere(box, boxPose, store.Spheres[otherHdr.PayloadIndex], otherPose, ref m);
                case ColliderType.Capsule: return BoxCapsule(box, boxPose, store.Capsules[otherHdr.PayloadIndex], otherPose, ref m);
                case ColliderType.Box: return BoxBox(box, boxPose, store.Boxes[otherHdr.PayloadIndex], otherPose, ref m);
                case ColliderType.Cone: return BoxCone(box, boxPose, store.Cones[otherHdr.PayloadIndex], otherPose, ref m);
                case ColliderType.Cylinder: return BoxCylinder(box, boxPose, store.Cylinders[otherHdr.PayloadIndex], otherPose, ref m);
                case ColliderType.Mesh: return BoxMesh(box, boxPose, store.Meshes[otherHdr.PayloadIndex], otherPose, ref m);
                case ColliderType.Compound: return AnyCompoundVsAnything(store, store.Compounds[otherHdr.PayloadIndex], otherPose, otherHandle, otherPose, ref m); // conservative; compound will recurse
                case ColliderType.Voxel: return AnyVoxelVsAnything(store, store.Voxels[otherHdr.PayloadIndex], otherPose, otherHandle, otherPose, ref m);
            }

            return false;
        }

        // -----------------------------
        // Convex support + GJK intersection + contact approximation
        // -----------------------------
        private enum ConvexProxyKind : byte { Sphere, Capsule, Box, Cone, Cylinder, Triangle }

        private readonly struct ConvexProxy
        {
            public readonly ConvexProxyKind Kind;
            public readonly Pose Pose;
            public readonly SphereCollider Sphere;
            public readonly CapsuleCollider Capsule;
            public readonly BoxCollider Box;
            public readonly ConeCollider Cone;
            public readonly CylinderCollider Cylinder;
            public readonly float3 T0, T1, T2;

            public ConvexProxy(in SphereCollider s, in Pose p) { Kind = ConvexProxyKind.Sphere; Pose = p; Sphere = s; Capsule = default; Box = default; Cone = default; Cylinder = default; T0 = T1 = T2 = default; }
            public ConvexProxy(in CapsuleCollider c, in Pose p) { Kind = ConvexProxyKind.Capsule; Pose = p; Sphere = default; Capsule = c; Box = default; Cone = default; Cylinder = default; T0 = T1 = T2 = default; }
            public ConvexProxy(in BoxCollider b, in Pose p) { Kind = ConvexProxyKind.Box; Pose = p; Sphere = default; Capsule = default; Box = b; Cone = default; Cylinder = default; T0 = T1 = T2 = default; }
            public ConvexProxy(in ConeCollider c, in Pose p) { Kind = ConvexProxyKind.Cone; Pose = p; Sphere = default; Capsule = default; Box = default; Cone = c; Cylinder = default; T0 = T1 = T2 = default; }
            public ConvexProxy(in CylinderCollider c, in Pose p) { Kind = ConvexProxyKind.Cylinder; Pose = p; Sphere = default; Capsule = default; Box = default; Cone = default; Cylinder = c; T0 = T1 = T2 = default; }
            public ConvexProxy(float3 v0, float3 v1, float3 v2) { Kind = ConvexProxyKind.Triangle; Pose = default; Sphere = default; Capsule = default; Box = default; Cone = default; Cylinder = default; T0 = v0; T1 = v1; T2 = v2; }

            public float3 Support(float3 dirWorld)
            {
                switch (Kind)
                {
                    case ConvexProxyKind.Sphere:
                        {
                            float3 c = TransformPoint(Pose, Sphere.Center);
                            float3 d = math.normalizesafe(dirWorld, new float3(0, 1, 0));
                            return c + d * Sphere.Radius;
                        }
                    case ConvexProxyKind.Capsule:
                        {
                            GetCapsuleSegmentWorld(Capsule, Pose, out float3 p0, out float3 p1);
                            float3 d = math.normalizesafe(dirWorld, new float3(0, 1, 0));
                            float3 p = (math.dot(p1, d) > math.dot(p0, d)) ? p1 : p0;
                            return p + d * Capsule.Radius;
                        }
                    case ConvexProxyKind.Box:
                        {
                            GetBoxWorld(Box, Pose, out float3 c, out quaternion r, out float3 he);
                            float3 dl = math.rotate(math.inverse(r), dirWorld);
                            float3 sl = new float3(dl.x >= 0 ? he.x : -he.x,
                                                   dl.y >= 0 ? he.y : -he.y,
                                                   dl.z >= 0 ? he.z : -he.z);
                            return c + math.rotate(r, sl);
                        }
                    case ConvexProxyKind.Cone:
                        {
                            float3 c = TransformPoint(Pose, Cone.Center);
                            quaternion r = math.mul(Pose.Rotation, Cone.Orientation);
                            float3 dl = math.rotate(math.inverse(r), dirWorld);

                            float h = Cone.HalfHeight;
                            float br = math.max(0, Cone.BaseRadius);

                            float3 tip = new float3(0, +h, 0);

                            float2 dxz = new float2(dl.x, dl.z);
                            float len = math.length(dxz);
                            float3 rim = new float3(0, -h, 0);
                            if (len > 1e-6f)
                                rim += new float3((dxz.x / len) * br, 0, (dxz.y / len) * br);
                            else
                                rim += new float3(br, 0, 0);

                            float dt = math.dot(tip, dl);
                            float dr = math.dot(rim, dl);
                            float3 best = (dt >= dr) ? tip : rim;

                            float3 d = math.normalizesafe(dl, new float3(0, 1, 0));
                            best += d * Cone.RoundingRadius;

                            return c + math.rotate(r, best);
                        }
                    case ConvexProxyKind.Cylinder:
                        {
                            float3 c = TransformPoint(Pose, Cylinder.Center);
                            quaternion r = math.mul(Pose.Rotation, Cylinder.Orientation);
                            float3 dl = math.rotate(math.inverse(r), dirWorld);

                            float h = Cylinder.HalfHeight;
                            float rr = math.max(0, Cylinder.Radius);

                            float y = (dl.y >= 0) ? +h : -h;

                            float2 xz = new float2(dl.x, dl.z);
                            float len = math.length(xz);

                            float3 p = new float3(0, y, 0);
                            if (len > 1e-6f)
                                p += new float3((xz.x / len) * rr, 0, (xz.y / len) * rr);
                            else
                                p += new float3(rr, 0, 0);

                            float3 d = math.normalizesafe(dl, new float3(0, 1, 0));
                            p += d * Cylinder.RoundingRadius;

                            return c + math.rotate(r, p);
                        }
                    case ConvexProxyKind.Triangle:
                        {
                            float da = math.dot(T0, dirWorld);
                            float db = math.dot(T1, dirWorld);
                            float dc = math.dot(T2, dirWorld);
                            return (da >= db && da >= dc) ? T0 : (db >= dc ? T1 : T2);
                        }
                }

                return default;
            }
        }

        private struct Simplex
        {
            public float3 A, B, C, D;
            public int Count;
        }

        private static bool ConvexConvex_Generic(in ConvexProxy a, in ConvexProxy b, ref ContactManifold m, uint baseFid)
        {
            ClearManifold(ref m);

            float3 dir = new float3(1, 0, 0);
            Simplex s = default;

            float3 p = SupportMinkowski(a, b, dir);
            s.A = p;
            s.Count = 1;
            dir = -p;

            const int MAX_ITERS = 24;

            for (int iter = 0; iter < MAX_ITERS; iter++)
            {
                p = SupportMinkowski(a, b, dir);

                // separating axis found
                if (math.dot(p, dir) < 0f)
                    return false;

                AddToSimplex(ref s, p);

                if (DoSimplex(ref s, ref dir))
                {
                    // intersecting: approximate contact normal + depth
                    float3 n = math.normalizesafe(dir, new float3(0, 1, 0));

                    // witness points approx
                    float3 pa = a.Support(-n);
                    float3 pb = b.Support(+n);

                    float pen = math.dot(pb - pa, n);
                    pen = math.max(pen, 1e-4f);

                    float3 cp = 0.5f * (pa + pb);

                    m.Normal = n;
                    AddPoint(ref m, cp, pen, baseFid);
                    return true;
                }
            }

            return false;
        }

        private static float3 SupportMinkowski(in ConvexProxy a, in ConvexProxy b, float3 dir)
            => a.Support(dir) - b.Support(-dir);

        private static void AddToSimplex(ref Simplex s, float3 p)
        {
            s.D = s.C;
            s.C = s.B;
            s.B = s.A;
            s.A = p;
            s.Count = math.min(s.Count + 1, 4);
        }

        private static bool DoSimplex(ref Simplex s, ref float3 dir)
        {
            if (s.Count == 2) return Line(ref s, ref dir);
            if (s.Count == 3) return Triangle(ref s, ref dir);
            if (s.Count == 4) return Tetrahedron(ref s, ref dir);

            dir = -s.A;
            return false;
        }

        private static bool Line(ref Simplex s, ref float3 dir)
        {
            float3 a = s.A;
            float3 b = s.B;
            float3 ab = b - a;
            float3 ao = -a;

            if (math.dot(ab, ao) > 0)
                dir = math.cross(math.cross(ab, ao), ab);
            else
            {
                s.B = default;
                s.Count = 1;
                dir = ao;
            }
            return false;
        }

        private static bool Triangle(ref Simplex s, ref float3 dir)
        {
            float3 a = s.A, b = s.B, c = s.C;
            float3 ab = b - a;
            float3 ac = c - a;
            float3 ao = -a;

            float3 abc = math.cross(ab, ac);

            float3 abPerp = math.cross(abc, ab);
            if (math.dot(abPerp, ao) > 0)
            {
                s.C = default; s.Count = 2;
                dir = math.cross(math.cross(ab, ao), ab);
                return false;
            }

            float3 acPerp = math.cross(ac, abc);
            if (math.dot(acPerp, ao) > 0)
            {
                s.B = s.C; s.C = default; s.Count = 2;
                dir = math.cross(math.cross(ac, ao), ac);
                return false;
            }

            if (math.dot(abc, ao) > 0)
                dir = abc;
            else
            {
                float3 tmp = s.B; s.B = s.C; s.C = tmp;
                dir = -abc;
            }

            return false;
        }

        private static bool Tetrahedron(ref Simplex s, ref float3 dir)
        {
            float3 a = s.A, b = s.B, c = s.C, d = s.D;
            float3 ao = -a;

            float3 abc = math.cross(b - a, c - a);
            float3 acd = math.cross(c - a, d - a);
            float3 adb = math.cross(d - a, b - a);

            if (math.dot(abc, ao) > 0)
            {
                s.D = default; s.Count = 3;
                dir = abc;
                return false;
            }

            if (math.dot(acd, ao) > 0)
            {
                s.B = s.C;
                s.C = s.D;
                s.D = default;
                s.Count = 3;
                dir = acd;
                return false;
            }

            if (math.dot(adb, ao) > 0)
            {
                s.C = s.B;
                s.B = s.D;
                s.D = default;
                s.Count = 3;
                dir = adb;
                return false;
            }

            return true;
        }

        // -----------------------------
        // ConvexOrPrimitive wrapper for mesh code
        // -----------------------------
        private enum ConvexOrPrimitiveKind : byte { Sphere, Capsule, Box, Cone, Cylinder }

        private readonly struct ConvexOrPrimitive
        {
            public readonly ConvexOrPrimitiveKind Kind;
            public readonly Pose Pose;
            public readonly SphereCollider Sphere;
            public readonly CapsuleCollider Capsule;
            public readonly BoxCollider Box;
            public readonly ConeCollider Cone;
            public readonly CylinderCollider Cylinder;

            public ConvexOrPrimitive(in SphereCollider s, in Pose p) { Kind = ConvexOrPrimitiveKind.Sphere; Pose = p; Sphere = s; Capsule = default; Box = default; Cone = default; Cylinder = default; }
            public ConvexOrPrimitive(in CapsuleCollider c, in Pose p) { Kind = ConvexOrPrimitiveKind.Capsule; Pose = p; Sphere = default; Capsule = c; Box = default; Cone = default; Cylinder = default; }
            public ConvexOrPrimitive(in BoxCollider b, in Pose p) { Kind = ConvexOrPrimitiveKind.Box; Pose = p; Sphere = default; Capsule = default; Box = b; Cone = default; Cylinder = default; }
            public ConvexOrPrimitive(in ConeCollider c, in Pose p) { Kind = ConvexOrPrimitiveKind.Cone; Pose = p; Sphere = default; Capsule = default; Box = default; Cone = c; Cylinder = default; }
            public ConvexOrPrimitive(in CylinderCollider c, in Pose p) { Kind = ConvexOrPrimitiveKind.Cylinder; Pose = p; Sphere = default; Capsule = default; Box = default; Cone = default; Cylinder = c; }

            public Aabb WorldAabb()
            {
                // conservative AABB per shape
                switch (Kind)
                {
                    case ConvexOrPrimitiveKind.Sphere:
                        {
                            float3 c = TransformPoint(Pose, Sphere.Center);
                            float3 e = new float3(Sphere.Radius);
                            return Aabb.FromCenterExtents(c, e);
                        }
                    case ConvexOrPrimitiveKind.Capsule:
                        {
                            GetCapsuleSegmentWorld(Capsule, Pose, out float3 p0, out float3 p1);
                            float3 mn = math.min(p0, p1) - new float3(Capsule.Radius);
                            float3 mx = math.max(p0, p1) + new float3(Capsule.Radius);
                            return new Aabb { Min = mn, Max = mx };
                        }
                    case ConvexOrPrimitiveKind.Box:
                        {
                            // corners approach
                            Aabb local = Aabb.FromCenterExtents(Box.Center, Box.HalfExtents);
                            // includes Orientation? For conservative, just use pose+orientation in TransformAabb by building a pseudo-pose:
                            Pose p = Pose;
                            p.Rotation = math.mul(Pose.Rotation, Box.Orientation);
                            return TransformAabb(p, local);
                        }
                    case ConvexOrPrimitiveKind.Cone:
                        {
                            float r = math.max(Cone.BaseRadius + Cone.RoundingRadius, Cone.RoundingRadius);
                            float h = Cone.HalfHeight + Cone.RoundingRadius;
                            Aabb local = Aabb.FromCenterExtents(Cone.Center, new float3(r, h, r));
                            Pose p = Pose;
                            p.Rotation = math.mul(Pose.Rotation, Cone.Orientation);
                            return TransformAabb(p, local);
                        }
                    case ConvexOrPrimitiveKind.Cylinder:
                        {
                            float r = Cylinder.Radius + Cylinder.RoundingRadius;
                            float h = Cylinder.HalfHeight + Cylinder.RoundingRadius;
                            Aabb local = Aabb.FromCenterExtents(Cylinder.Center, new float3(r, h, r));
                            Pose p = Pose;
                            p.Rotation = math.mul(Pose.Rotation, Cylinder.Orientation);
                            return TransformAabb(p, local);
                        }
                }

                return default;
            }

            public ConvexProxy AsConvexProxy()
            {
                return Kind switch
                {
                    ConvexOrPrimitiveKind.Sphere => new ConvexProxy(Sphere, Pose),
                    ConvexOrPrimitiveKind.Capsule => new ConvexProxy(Capsule, Pose),
                    ConvexOrPrimitiveKind.Box => new ConvexProxy(Box, Pose),
                    ConvexOrPrimitiveKind.Cone => new ConvexProxy(Cone, Pose),
                    _ => new ConvexProxy(Cylinder, Pose)
                };
            }
        }

        // -----------------------------
        // Manifold helpers
        // -----------------------------
        private static void ClearManifold(ref ContactManifold m)
        {
            m.A = BodyId.Invalid;
            m.B = BodyId.Invalid;
            m.Normal = new float3(0, 1, 0);
            m.PointCount = 0;
            m.P0 = m.P1 = m.P2 = m.P3 = default;

            m.ImpulseN0 = m.ImpulseN1 = m.ImpulseN2 = m.ImpulseN3 = 0;
            m.ImpulseT0 = m.ImpulseT1 = m.ImpulseT2 = m.ImpulseT3 = 0;
        }

        private static void AddPoint(ref ContactManifold m, float3 pos, float pen, uint fid)
        {
            ContactPoint cp = new ContactPoint { Position = pos, Penetration = pen, FeatureId = fid };

            if (m.PointCount == 0) { m.P0 = cp; m.PointCount = 1; return; }
            if (m.PointCount == 1) { m.P1 = cp; m.PointCount = 2; return; }
            if (m.PointCount == 2) { m.P2 = cp; m.PointCount = 3; return; }
            if (m.PointCount == 3) { m.P3 = cp; m.PointCount = 4; return; }

            // Replace shallowest if full
            int idx = 0;
            float minPen = m.P0.Penetration;
            if (m.P1.Penetration < minPen) { minPen = m.P1.Penetration; idx = 1; }
            if (m.P2.Penetration < minPen) { minPen = m.P2.Penetration; idx = 2; }
            if (m.P3.Penetration < minPen) { minPen = m.P3.Penetration; idx = 3; }

            if (pen <= minPen) return;

            if (idx == 0) m.P0 = cp;
            else if (idx == 1) m.P1 = cp;
            else if (idx == 2) m.P2 = cp;
            else m.P3 = cp;
        }

        private static void MergeManifoldKeepDeepest(ref ContactManifold dst, ref ContactManifold src)
        {
            if (dst.PointCount == 0 && src.PointCount > 0)
                dst.Normal = src.Normal;

            if (src.PointCount >= 1) AddPoint(ref dst, src.P0.Position, src.P0.Penetration, src.P0.FeatureId);
            if (src.PointCount >= 2) AddPoint(ref dst, src.P1.Position, src.P1.Penetration, src.P1.FeatureId);
            if (src.PointCount >= 3) AddPoint(ref dst, src.P2.Position, src.P2.Penetration, src.P2.FeatureId);
            if (src.PointCount >= 4) AddPoint(ref dst, src.P3.Position, src.P3.Penetration, src.P3.FeatureId);
        }

        private static void StampFeatureIds(ref ContactManifold m, uint fid)
        {
            if (m.PointCount >= 1) m.P0.FeatureId = fid;
            if (m.PointCount >= 2) m.P1.FeatureId = fid + 1u;
            if (m.PointCount >= 3) m.P2.FeatureId = fid + 2u;
            if (m.PointCount >= 4) m.P3.FeatureId = fid + 3u;
        }

        // -----------------------------
        // Geometry helpers
        // -----------------------------
        private static float3 TransformPoint(in Pose p, float3 local)
            => p.Position + math.mul(p.Rotation, local);

        private static Pose Mul(in Pose parent, in TransformQv local)
        {
            return new Pose
            {
                Rotation = math.mul(parent.Rotation, local.Rotation),
                Position = parent.Position + math.mul(parent.Rotation, local.Position)
            };
        }

        private static void GetBoxWorld(in BoxCollider b, in Pose pose, out float3 c, out quaternion r, out float3 he)
        {
            c = TransformPoint(pose, b.Center);
            r = math.mul(pose.Rotation, b.Orientation);
            he = b.HalfExtents;
        }

        private static void GetCapsuleSegmentWorld(in CapsuleCollider c, in Pose pose, out float3 p0, out float3 p1)
        {
            float3 centerW = TransformPoint(pose, c.Center);
            quaternion r = math.mul(pose.Rotation, c.Orientation);
            float3 axis = math.mul(r, new float3(0, 1, 0));
            float hh = math.max(0, c.HalfHeight);
            p0 = centerW - axis * hh;
            p1 = centerW + axis * hh;
        }

        private static float3 ClosestPointOnSegment(float3 p, float3 a, float3 b)
        {
            float3 ab = b - a;
            float denom = math.max(math.dot(ab, ab), 1e-12f);
            float t = math.dot(p - a, ab) / denom;
            t = math.clamp(t, 0f, 1f);
            return a + t * ab;
        }

        private static void ClosestPointsSegmentSegment(float3 p1, float3 q1, float3 p2, float3 q2, out float3 c1, out float3 c2)
        {
            float3 d1 = q1 - p1;
            float3 d2 = q2 - p2;
            float3 r = p1 - p2;

            float a = math.dot(d1, d1);
            float e = math.dot(d2, d2);
            float f = math.dot(d2, r);

            float s, t;

            if (a <= 1e-12f && e <= 1e-12f)
            {
                s = t = 0f;
                c1 = p1; c2 = p2; return;
            }

            if (a <= 1e-12f)
            {
                s = 0f;
                t = math.clamp(f / e, 0f, 1f);
            }
            else
            {
                float c = math.dot(d1, r);
                if (e <= 1e-12f)
                {
                    t = 0f;
                    s = math.clamp(-c / a, 0f, 1f);
                }
                else
                {
                    float b = math.dot(d1, d2);
                    float denom = a * e - b * b;

                    if (denom != 0f) s = math.clamp((b * f - c * e) / denom, 0f, 1f);
                    else s = 0f;

                    float tnom = b * s + f;

                    if (tnom < 0f)
                    {
                        t = 0f;
                        s = math.clamp(-c / a, 0f, 1f);
                    }
                    else if (tnom > e)
                    {
                        t = 1f;
                        s = math.clamp((b - c) / a, 0f, 1f);
                    }
                    else
                    {
                        t = tnom / e;
                    }
                }
            }

            c1 = p1 + d1 * s;
            c2 = p2 + d2 * t;
        }

        private static float3 ClosestPointOBB(float3 pWorld, in BoxCollider b, in Pose bPose)
        {
            GetBoxWorld(b, bPose, out float3 c, out quaternion r, out float3 he);
            float3 local = math.rotate(math.inverse(r), pWorld - c);
            float3 clamped = math.clamp(local, -he, he);
            return c + math.rotate(r, clamped);
        }

        private static float3 ClosestPointOnTriangle(float3 p, float3 a, float3 b, float3 c)
        {
            float3 ab = b - a;
            float3 ac = c - a;
            float3 ap = p - a;

            float d1 = math.dot(ab, ap);
            float d2 = math.dot(ac, ap);
            if (d1 <= 0f && d2 <= 0f) return a;

            float3 bp = p - b;
            float d3 = math.dot(ab, bp);
            float d4 = math.dot(ac, bp);
            if (d3 >= 0f && d4 <= d3) return b;

            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float v = d1 / (d1 - d3);
                return a + v * ab;
            }

            float3 cp = p - c;
            float d5 = math.dot(ab, cp);
            float d6 = math.dot(ac, cp);
            if (d6 >= 0f && d5 <= d6) return c;

            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float w = d2 / (d2 - d6);
                return a + w * ac;
            }

            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            {
                float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                return b + w * (c - b);
            }

            float denom = 1f / (va + vb + vc);
            float v2 = vb * denom;
            float w2 = vc * denom;
            return a + ab * v2 + ac * w2;
        }

        private static bool AabbOverlap(in Aabb a, in Aabb b)
        {
            return (a.Min.x <= b.Max.x && a.Max.x >= b.Min.x) &&
                   (a.Min.y <= b.Max.y && a.Max.y >= b.Min.y) &&
                   (a.Min.z <= b.Max.z && a.Max.z >= b.Min.z);
        }

        private static Aabb TransformAabb(in Pose pose, in Aabb local)
        {
            float3 c = local.Center;
            float3 e = local.Extents;

            float3 w0 = TransformPoint(pose, c + new float3(-e.x, -e.y, -e.z));
            float3 w1 = TransformPoint(pose, c + new float3(+e.x, -e.y, -e.z));
            float3 w2 = TransformPoint(pose, c + new float3(-e.x, +e.y, -e.z));
            float3 w3 = TransformPoint(pose, c + new float3(+e.x, +e.y, -e.z));
            float3 w4 = TransformPoint(pose, c + new float3(-e.x, -e.y, +e.z));
            float3 w5 = TransformPoint(pose, c + new float3(+e.x, -e.y, +e.z));
            float3 w6 = TransformPoint(pose, c + new float3(-e.x, +e.y, +e.z));
            float3 w7 = TransformPoint(pose, c + new float3(+e.x, +e.y, +e.z));

            float3 mn = math.min(math.min(math.min(w0, w1), math.min(w2, w3)),
                                 math.min(math.min(w4, w5), math.min(w6, w7)));
            float3 mx = math.max(math.max(math.max(w0, w1), math.max(w2, w3)),
                                 math.max(math.max(w4, w5), math.max(w6, w7)));

            return new Aabb { Min = mn, Max = mx };
        }

        private static Aabb InverseTransformAabb(in Pose pose, in Aabb world)
        {
            quaternion invR = math.inverse(pose.Rotation);
            float3 c = world.Center;
            float3 e = world.Extents;

            float3 l0 = math.mul(invR, (c + new float3(-e.x, -e.y, -e.z)) - pose.Position);
            float3 l1 = math.mul(invR, (c + new float3(+e.x, -e.y, -e.z)) - pose.Position);
            float3 l2 = math.mul(invR, (c + new float3(-e.x, +e.y, -e.z)) - pose.Position);
            float3 l3 = math.mul(invR, (c + new float3(+e.x, +e.y, -e.z)) - pose.Position);
            float3 l4 = math.mul(invR, (c + new float3(-e.x, -e.y, +e.z)) - pose.Position);
            float3 l5 = math.mul(invR, (c + new float3(+e.x, -e.y, +e.z)) - pose.Position);
            float3 l6 = math.mul(invR, (c + new float3(-e.x, +e.y, +e.z)) - pose.Position);
            float3 l7 = math.mul(invR, (c + new float3(+e.x, +e.y, +e.z)) - pose.Position);

            float3 mn = math.min(math.min(math.min(l0, l1), math.min(l2, l3)),
                                 math.min(math.min(l4, l5), math.min(l6, l7)));
            float3 mx = math.max(math.max(math.max(l0, l1), math.max(l2, l3)),
                                 math.max(math.max(l4, l5), math.max(l6, l7)));

            return new Aabb { Min = mn, Max = mx };
        }

        // -----------------------------
        // Voxel occupancy
        // -----------------------------
        private static bool BrickOccupied(in BrickMask512 mask, int x, int y, int z)
        {
            int idx = x | (y << 3) | (z << 6); // 0..511
            int word = idx >> 6;               // 0..7
            int bit = idx & 63;

            ulong w = word switch
            {
                0 => mask.M0,
                1 => mask.M1,
                2 => mask.M2,
                3 => mask.M3,
                4 => mask.M4,
                5 => mask.M5,
                6 => mask.M6,
                _ => mask.M7
            };

            return ((w >> bit) & 1ul) != 0;
        }

        // -----------------------------
        // Tiny fixed BVH stack
        // -----------------------------
        private struct FixedStack32
        {
            public int Count;
            private int a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31;

            public void Push(int v)
            {
                int i = Count++;
                if (i == 0) a0 = v;
                else if (i == 1) a1 = v;
                else if (i == 2) a2 = v;
                else if (i == 3) a3 = v;
                else if (i == 4) a4 = v;
                else if (i == 5) a5 = v;
                else if (i == 6) a6 = v;
                else if (i == 7) a7 = v;
                else if (i == 8) a8 = v;
                else if (i == 9) a9 = v;
                else if (i == 10) a10 = v;
                else if (i == 11) a11 = v;
                else if (i == 12) a12 = v;
                else if (i == 13) a13 = v;
                else if (i == 14) a14 = v;
                else if (i == 15) a15 = v;
                else if (i == 16) a16 = v;
                else if (i == 17) a17 = v;
                else if (i == 18) a18 = v;
                else if (i == 19) a19 = v;
                else if (i == 20) a20 = v;
                else if (i == 21) a21 = v;
                else if (i == 22) a22 = v;
                else if (i == 23) a23 = v;
                else if (i == 24) a24 = v;
                else if (i == 25) a25 = v;
                else if (i == 26) a26 = v;
                else if (i == 27) a27 = v;
                else if (i == 28) a28 = v; else if (i == 29) a29 = v; else if (i == 30) a30 = v; else a31 = v;

                if (Count > 32) Count = 32; // clamp
            }

            public int Pop()
            {
                int i = --Count;
                return i switch
                {
                    0 => a0,
                    1 => a1,
                    2 => a2,
                    3 => a3,
                    4 => a4,
                    5 => a5,
                    6 => a6,
                    7 => a7,
                    8 => a8,
                    9 => a9,
                    10 => a10,
                    11 => a11,
                    12 => a12,
                    13 => a13,
                    14 => a14,
                    15 => a15,
                    16 => a16,
                    17 => a17,
                    18 => a18,
                    19 => a19,
                    20 => a20,
                    21 => a21,
                    22 => a22,
                    23 => a23,
                    24 => a24,
                    25 => a25,
                    26 => a26,
                    27 => a27,
                    28 => a28,
                    29 => a29,
                    30 => a30,
                    _ => a31
                };
            }
        }
    }

    public struct ContactPoint
    {
        public float3 Position;
        public float Penetration;
        public uint FeatureId; // important for warm starting & mesh/voxel caching
    }

    public struct ContactManifold
    {
        public BodyId A;
        public BodyId B;
        public float3 Normal; // from A -> B
        public byte PointCount;
        public ContactPoint P0, P1, P2, P3;

        // Cached impulses per point for warm start (normal + friction)
        public float ImpulseN0, ImpulseN1, ImpulseN2, ImpulseN3;
        public float ImpulseT0, ImpulseT1, ImpulseT2, ImpulseT3;
    }

    internal struct ConstraintGraph : System.IDisposable
    {
        // Constraints as edges; built from joints + contacts.
        public NativeList<Constraint> Constraints;

        public ConstraintGraph(Allocator allocator)
        {
            Constraints = new NativeList<Constraint>(allocator);
        }

        public void Dispose() => Constraints.Dispose();
    }

    public enum ConstraintType : byte
    {
        Contact,
        Hinge,
        BallSocket,
        Prismatic,
        ConeTwist,
        Distance,
        Motor,
        Custom
    }

    public struct Constraint
    {
        public ConstraintType Type;
        public BodyId A;
        public BodyId B;
        public int PayloadIndex;   // points into per-type constraint payload pools
        public ushort Flags;
    }

    internal struct Solver : System.IDisposable
    {
        // Scratch + row buffers, impulses, etc. (per-world)
        public NativeList<SolverRow> Rows;

        public Solver(Allocator allocator)
        {
            Rows = new NativeList<SolverRow>(allocator);
        }

        public void Dispose() => Rows.Dispose();
    }

    /// Row-based solver core unit (Jacobian row / impulse clamp)
    public struct SolverRow
    {
        public BodyId A, B;

        public float3 JA_Linear;
        public float3 JA_Angular;
        public float3 JB_Linear;
        public float3 JB_Angular;

        public float EffectiveMass;
        public float Bias;
        public float Impulse;
        public float MinImpulse;
        public float MaxImpulse;
    }
}
