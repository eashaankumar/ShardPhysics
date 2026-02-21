using System;
using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace Shard.Dev
{
    public static class CylinderCylinderSolver
    {
        public struct ContactPoint
        {
            public float3 point;   // stable shared point (midpoint of two witnesses)
            public float3 normal;  // global MTV axis (A -> B)
            public float depth;    // penetration along normal
        }

        public struct CylinderCylinderContactPoints
        {
            public ContactPoint p1;
            public ContactPoint p2;
            public ContactPoint p3;
            public ContactPoint p4;
            public int numContactPoints;

            public float3 globalPenAxis;  // MTV axis (A -> B)
            public float globalPenDepth;  // MTV depth

            public ContactPoint this[int index]
            {
                get
                {
                    if (index == 0) return p1;
                    if (index == 1) return p2;
                    if (index == 2) return p3;
                    if (index == 3) return p4;
                    return default;
                }
            }
        }

        public struct Cylinder
        {
            public readonly float3 center;
            public readonly quaternion rot;
            public readonly float halfHeight;
            public readonly float radius;

            public Cylinder(float3 center, quaternion rot, float halfHeight, float radius)
            {
                this.center = center;
                this.rot = rot;
                this.halfHeight = halfHeight;
                this.radius = radius;
            }
        }

        private const float kSlop = 1e-6f;
        private const float kAxisEps = 1e-10f;
        private const float kDedupEpsSq = 1e-10f;

        private const float kCapSideParallelEps = 0.02f; // keep your existing
        private const float kTiny = 1e-12f;

        // ============================================================
        // Public API (UNCHANGED)
        // ============================================================
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Solve(in Cylinder a, in Cylinder b, out CylinderCylinderContactPoints cc)
        {
            cc = default;
            InvalidateAll(ref cc);

            float3 aAxis = GetAxis(in a);
            float3 bAxis = GetAxis(in b);

            // SAT overlap + MTV
            if (!SAT_CylinderCylinder(in a, in b, aAxis, bAxis, out float3 n, out float depth))
                return false;

            if (depth <= kSlop)
                return false;

            // Orient MTV A->B (must be done BEFORE any classification/manifold)
            float3 cTo = b.center - a.center;
            if (math.dot(n, cTo) < 0f) n = -n;

            cc.globalPenAxis = n;
            cc.globalPenDepth = depth;

            float axesDot = math.abs(math.dot(aAxis, bAxis));
            bool axesParallel = axesDot > 0.9995f;

            bool sideNormal =
                math.abs(math.dot(n, aAxis)) < 0.2f &&
                math.abs(math.dot(n, bAxis)) < 0.2f;

            bool capNormal =
                math.abs(math.dot(n, aAxis)) > 0.9f &&
                math.abs(math.dot(n, bAxis)) > 0.9f;

            // ---------------------------
            // 1) PARALLEL CAP–CAP (keep your existing behavior)
            // ---------------------------
            if (axesParallel && capNormal)
            {
                if (BuildCapCapParallelManifold(in a, in b, aAxis, bAxis, n, depth, ref cc))
                {
                    DedupAndCompact(ref cc);
                    cc.globalPenAxis = n;
                    cc.globalPenDepth = depth;
                    return cc.numContactPoints > 0;
                }
            }

            // ---------------------------
            // 2) PARALLEL SIDE–SIDE (keep your existing behavior)
            // ---------------------------
            if (axesParallel && sideNormal)
            {
                if (BuildSideSideParallelManifold(in a, in b, aAxis, bAxis, n, depth, ref cc))
                {
                    DedupAndCompact(ref cc);
                    cc.globalPenAxis = n;
                    cc.globalPenDepth = depth;
                    return cc.numContactPoints > 0;
                }
            }

            // ---------------------------
            // 3) CAP–SIDE EDGE @ ~90° (THIS MUST RUN BEFORE CONVEX FALLBACK)
            // Emits EXACTLY 2 points (min/max bounds of intersecting edge)
            // ---------------------------


            // We want a forgiving "near 90°" test. (Adjust window if needed.)
            bool axesPerp = axesDot < 0.05f;

            if (axesPerp)
            {
                // Try A cap vs B side
                if (TryBuildCapSideEdge(in a, in b, aAxis, bAxis, n, depth, ref cc))
                {
                    DedupAndCompact(ref cc);
                    cc.globalPenAxis = n;
                    cc.globalPenDepth = depth;
                    return cc.numContactPoints > 0; // IMPORTANT: prevent 4-pt convex fallback
                }

                // Try B cap vs A side
                if (TryBuildCapSideEdge(in b, in a, bAxis, aAxis, n, depth, ref cc))
                {
                    DedupAndCompact(ref cc);
                    cc.globalPenAxis = n;
                    cc.globalPenDepth = depth;
                    return cc.numContactPoints > 0; // IMPORTANT: prevent 4-pt convex fallback
                }
            }

            // ---------------------------
            // 4) FALLBACK: generic convex-style reduction
            // (This can emit up to 4 — by design.)
            // ---------------------------
            BuildManifold_ConvexStyle(in a, in b, aAxis, bAxis, n, depth, ref cc);

            DedupAndCompact(ref cc);
            cc.globalPenAxis = n;
            cc.globalPenDepth = depth;
            return cc.numContactPoints > 0;
        }


        private static bool TryBuildCapSideEdge(
    in Cylinder capCyl, in Cylinder sideCyl,
    float3 capAxis, float3 sideAxis,
    float3 nAB, float satDepth,
    ref CylinderCylinderContactPoints cc)
        {
            // near 90° gate (looser so we don't fall back)
            if (math.abs(math.dot(sideAxis, capAxis)) > 0.10f)
                return false;

            // IMPORTANT:
            // nCap must point CAP -> SIDE for cap selection + "deepest edge" direction.
            float3 nCap = nAB;
            float3 capToSide = sideCyl.center - capCyl.center;
            if (math.dot(nCap, capToSide) < 0f)
                nCap = -nCap;

            // Pick facing cap using nCap (cap -> side), not global ordering.
            float s = (math.dot(nCap, capAxis) >= 0f) ? +1f : -1f;

            float3 capCenter = capCyl.center + capAxis * (s * capCyl.halfHeight);
            float3 capNormalOut = capAxis * s; // outward normal of that cap face (points toward side)

            // Build on deepest penetrating side edge (direction decided by capNormalOut)
            return BuildCapSide_DeepestPenetratingEdge_TwoPoints(
                in capCyl, capCenter, capNormalOut,
                in sideCyl, sideAxis,
                nAB, satDepth,   // keep GLOBAL normal for cp.normal / solver direction
                ref cc);
        }

        private static bool BuildCapSide_DeepestPenetratingEdge_TwoPoints(
    in Cylinder cap, float3 capCenter, float3 capNormalOut,
    in Cylinder side, float3 sideAxis,
    float3 nAB, float satDepth,
    ref CylinderCylinderContactPoints cc)
        {
            // Preconditions:
            // - dot(sideAxis, capNormalOut) ~ 0 (side axis lies in cap plane)
            // - capCenter is chosen cap center
            // - capNormalOut points OUT of the cap face

            // Build in-plane basis for the cap plane:
            // d = sideAxis projected into cap plane (should already be almost in plane)
            float3 d = sideAxis - capNormalOut * math.dot(sideAxis, capNormalOut);
            float dLenSq = math.lengthsq(d);
            if (dLenSq < kTiny) return false;
            d *= math.rsqrt(dLenSq);

            // p = perpendicular in cap plane
            float3 p = math.cross(capNormalOut, d);
            float pLenSq = math.lengthsq(p);
            if (pLenSq < kTiny) return false;
            p *= math.rsqrt(pLenSq);

            // Signed distance of side axis (through side.center) to the cap plane along capNormalOut:
            float hOut = math.dot(side.center - capCenter, capNormalOut);

            // The deepest penetrating edge of the side cylinder is the generatrix shifted
            // into the cap by radius along -capNormalOut.
            // Distance of that edge to the plane: distEdge = hOut - side.radius.
            float distEdge = hOut - side.radius;

            // If the deepest edge is not behind the cap plane, there is no cap penetration by that edge.
            if (distEdge >= -1e-6f)
                return false;

            // Deepest penetration magnitude relative to the cap plane (along capNormalOut).
            float penDeepPlane = -distEdge; // = side.radius - hOut

            // Project side axis line onto the cap plane (a point on the side axis in the cap plane):
            // axisOnPlane = side.center - capNormalOut * hOut
            float3 axisOnPlane = side.center - capNormalOut * hOut;

            // In cap-plane coordinates relative to capCenter:
            float3 rel = axisOnPlane - capCenter;
            float xAxis = math.dot(rel, p); // perpendicular-to-axis offset in plane
            float yAxis = math.dot(rel, d); // along-axis offset in plane

            float R = cap.radius;

            // Intersect the projected axis line with the cap disk:
            // Condition: xAxis^2 + (yAxis + t)^2 <= R^2  where t is along d (and equals sideAxis param)
            float yMaxSq = R * R - xAxis * xAxis;
            if (yMaxSq <= 1e-10f)
                return false; // tangent / no segment => let fallback handle

            float yMax = math.sqrt(yMaxSq);

            // Solve |yAxis + t| <= yMax  =>  t in [-yAxis - yMax, -yAxis + yMax]
            float tMinDisk = -yAxis - yMax;
            float tMaxDisk = -yAxis + yMax;

            // Clip by finite side height along sideAxis: t in [-hh, +hh]
            float t0 = math.max(tMinDisk, -side.halfHeight);
            float t1 = math.min(tMaxDisk, +side.halfHeight);

            if (t1 < t0 + 1e-7f)
                return false;

            // Build the two deepest-edge points ON THE SIDE CYLINDER (penetrating feature):
            float3 inward = -capNormalOut;
            float3 wSide0 = side.center + sideAxis * t0 + inward * side.radius;
            float3 wSide1 = side.center + sideAxis * t1 + inward * side.radius;

            // Corresponding points on the CAP FACE (cap plane) are the projection of the side points
            // along capNormalOut back to the plane by distEdge (constant):
            // Since distEdge = dot(wSide - capCenter, capNormalOut), the plane witness is:
            float3 wCap0 = wSide0 - capNormalOut * distEdge;
            float3 wCap1 = wSide1 - capNormalOut * distEdge;

            // Depth should be the deepest-edge penetration ALONG nAB (stable for solver push).
            // Use witness difference along nAB (cap -> side), ensure positivity.
            float depth0 = math.abs(math.dot(wCap0 - wSide0, nAB));
            float depth1 = math.abs(math.dot(wCap1 - wSide1, nAB));
            float depth = math.max(depth0, depth1);

            // Numerical guard: if nAB is slightly off, fall back to plane penetration projected onto nAB.
            //float depthFallback = penDeepPlane * math.max(0f, math.dot(capNormalOut, nAB));
            //if (depth0 <= kSlop) depth0 = depthFallback;
            //if (depth1 <= kSlop) depth1 = depthFallback;

            //float depth = math.max(depth0, depth1);
            if (depth <= kSlop)
                return false;

            // Emit exactly 2 contacts, points located ON the penetrating edge (wSide*).
            cc = default;
            InvalidateAll(ref cc);

            ContactPoint cp0; cp0.point = wSide0; cp0.normal = nAB; cp0.depth = depth;
            ContactPoint cp1; cp1.point = wSide1; cp1.normal = nAB; cp1.depth = depth;

            Write(ref cc, 0, cp0);
            Write(ref cc, 1, cp1);
            cc.numContactPoints = 2;
            cc.globalPenAxis = nAB;
            cc.globalPenDepth = math.max(satDepth, depth); // keep MTV at least as big as this feature depth
            return true;
        }

        private static bool BuildCapCapParallelManifold(
    in Cylinder a, in Cylinder b,
    float3 aAxis, float3 bAxis,
    float3 nAB, float depth,
    ref CylinderCylinderContactPoints cc)
        {
            if (depth <= kSlop)
                return false;

            // Pick facing caps (toward each other along nAB)
            // A faces +nAB, B faces -nAB
            float aSign = (math.dot(aAxis, nAB) >= 0f) ? +1f : -1f;
            float bSign = (math.dot(bAxis, nAB) >= 0f) ? -1f : +1f;

            float3 aCapC = a.center + aAxis * (aSign * a.halfHeight);
            float3 bCapC = b.center + bAxis * (bSign * b.halfHeight);

            // Build stable plane basis (u,v) orthonormal to nAB
            BuildStableOrthoBasis(nAB, out float3 u, out float3 v);

            // Project B cap center into A cap plane along nAB so both disks live in same plane
            float3 bCproj = bCapC - nAB * math.dot(bCapC - aCapC, nAB);

            float rA = a.radius;
            float rB = b.radius;

            // Mid-plane between the two cap planes (your "intersecting plane")
            float planeOffset = 0.5f * math.dot(bCapC - aCapC, nAB);
            float3 midPlaneOrigin = aCapC + nAB * planeOffset;

            // Find a point inside the lens (disk ∩ disk) in that plane
            float3 lensCenter = ClosestPointInDiskIntersection_OnPlane(aCapC, rA, bCproj, rB, u, v);

            // If there is no intersection, no cap-cap manifold (shouldn't happen if SAT classified cap-cap)
            if (!PointInBothDisks_OnPlane(lensCenter, aCapC, rA, bCproj, rB, u, v))
                return false;

            // Emit 4 clipped points around lensCenter in u/v directions
            const float push = 2f; // push factor (scaled by min radius)
            float big = math.min(rA, rB) * push;

            float3 p0 = ClipToDiskIntersection_OnPlane(lensCenter + u * big, aCapC, rA, bCproj, rB, u, v);
            float3 p1 = ClipToDiskIntersection_OnPlane(lensCenter + v * big, aCapC, rA, bCproj, rB, u, v);
            float3 p2 = ClipToDiskIntersection_OnPlane(lensCenter - u * big, aCapC, rA, bCproj, rB, u, v);
            float3 p3 = ClipToDiskIntersection_OnPlane(lensCenter - v * big, aCapC, rA, bCproj, rB, u, v);

            // Force to rim-ish while staying in lens (prevents drifting inward)
            p0 = ForceToRimAndClip(p0, aCapC, rA, bCproj, rB, u, v);
            p1 = ForceToRimAndClip(p1, aCapC, rA, bCproj, rB, u, v);
            p2 = ForceToRimAndClip(p2, aCapC, rA, bCproj, rB, u, v);
            p3 = ForceToRimAndClip(p3, aCapC, rA, bCproj, rB, u, v);

            // Convert each plane point to the shared MID-PLANE (same u/v, mid between caps)
            // This ensures "clipped onto the intersecting plane".
            p0 = ToMidPlane(p0, aCapC, midPlaneOrigin, nAB);
            p1 = ToMidPlane(p1, aCapC, midPlaneOrigin, nAB);
            p2 = ToMidPlane(p2, aCapC, midPlaneOrigin, nAB);
            p3 = ToMidPlane(p3, aCapC, midPlaneOrigin, nAB);

            // Write 4 contacts
            cc = default;
            InvalidateAll(ref cc);

            int count = 0;
            Write(ref cc, count++, MakeCP(p0, nAB, depth));
            Write(ref cc, count++, MakeCP(p1, nAB, depth));
            Write(ref cc, count++, MakeCP(p2, nAB, depth));
            Write(ref cc, count++, MakeCP(p3, nAB, depth));

            cc.numContactPoints = count;
            cc.globalPenAxis = nAB;
            cc.globalPenDepth = depth;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ContactPoint MakeCP(float3 p, float3 n, float depth)
        {
            ContactPoint cp;
            cp.point = p;
            cp.normal = n;
            cp.depth = depth;
            return cp;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ToMidPlane(float3 pOnACapPlane, float3 aCapC, float3 midPlaneOrigin, float3 nAB)
        {
            // Remove any numerical normal component relative to A cap plane, keep u/v offset,
            // then re-add it to midPlaneOrigin.
            float3 d = pOnACapPlane - aCapC;
            d -= nAB * math.dot(d, nAB);
            return midPlaneOrigin + d;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool PointInBothDisks_OnPlane(
            float3 p, float3 cA, float rA, float3 cB, float rB, float3 u, float3 v)
        {
            float2 a2 = ToPlane2D(p - cA, u, v);
            float2 b2 = ToPlane2D(p - cB, u, v);
            return math.dot(a2, a2) <= rA * rA + 1e-8f && math.dot(b2, b2) <= rB * rB + 1e-8f;
        }

        private static float3 ClosestPointInDiskIntersection_OnPlane(
            float3 cA, float rA, float3 cB, float rB, float3 u, float3 v)
        {
            // Iterative clamp between disks
            float3 p = 0.5f * (cA + cB);
            for (int i = 0; i < 6; i++)
            {
                p = ClampToDisk_OnPlane(p, cA, rA, u, v);
                p = ClampToDisk_OnPlane(p, cB, rB, u, v);
            }
            return p;
        }

        private static float3 ClipToDiskIntersection_OnPlane(
            float3 p, float3 cA, float rA, float3 cB, float rB, float3 u, float3 v)
        {
            // Alternate clamp a few times to get into lens
            for (int i = 0; i < 4; i++)
            {
                p = ClampToDisk_OnPlane(p, cA, rA, u, v);
                p = ClampToDisk_OnPlane(p, cB, rB, u, v);
            }
            p = ClampToDisk_OnPlane(p, cA, rA, u, v);
            return p;
        }

        private static float3 ForceToRimAndClip(
            float3 p, float3 cA, float rA, float3 cB, float rB, float3 u, float3 v)
        {
            float2 a2 = ToPlane2D(p - cA, u, v);
            float2 b2 = ToPlane2D(p - cB, u, v);

            float da = math.length(a2);
            float db = math.length(b2);

            float3 q = p;

            // Push toward the more "active" rim (whichever is closer to being outside)
            if (da >= db)
            {
                float2 dir = (da > 1e-12f) ? (a2 / da) : new float2(1, 0);
                q = cA + u * (dir.x * rA) + v * (dir.y * rA);
            }
            else
            {
                float2 dir = (db > 1e-12f) ? (b2 / db) : new float2(1, 0);
                q = cB + u * (dir.x * rB) + v * (dir.y * rB);
            }

            // Then clip back into the lens
            return ClipToDiskIntersection_OnPlane(q, cA, rA, cB, rB, u, v);
        }

        private static float3 ClampToDisk_OnPlane(float3 p, float3 c, float r, float3 u, float3 v)
        {
            float2 d2 = ToPlane2D(p - c, u, v);
            float lenSq = math.dot(d2, d2);
            float r2 = r * r;

            if (lenSq <= r2)
                return p;

            float invLen = math.rsqrt(math.max(lenSq, 1e-20f));
            float2 dN = d2 * invLen;
            return c + u * (dN.x * r) + v * (dN.y * r);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float2 ToPlane2D(float3 d, float3 u, float3 v)
        {
            return new float2(math.dot(d, u), math.dot(d, v));
        }


        private static bool BuildSideSideParallelManifold(
    in Cylinder a, in Cylinder b,
    float3 aAxis, float3 bAxis,
    float3 nAB, float depth,
    ref CylinderCylinderContactPoints cc)
        {
            // Build radial dir between axes (perp to aAxis) – must be stable & perpendicular.
            float3 cTo = b.center - a.center;

            // Project center-to-center off A axis to get a clean radial
            float3 r = cTo - aAxis * math.dot(cTo, aAxis);
            float rLenSq = math.lengthsq(r);
            if (rLenSq < 1e-12f)
                return false;

            float3 radial = r * math.rsqrt(rLenSq);

            // Orient radial A->B
            if (math.dot(radial, cTo) < 0f) radial = -radial;

            // Compute overlap band along A axis where both finite cylinders exist.
            // Map B's axial interval into A's axis coordinate.
            float cAlongA = math.dot(cTo, aAxis);

            // bAxis may be parallel or anti-parallel: treat by sign.
            float sign = math.sign(math.dot(bAxis, aAxis));
            if (sign == 0f) sign = 1f;

            // B's extent along A is halfHeight (axes parallel), so interval is [cAlongA - hhB, cAlongA + hhB]
            float bMinOnA = cAlongA - b.halfHeight;
            float bMaxOnA = cAlongA + b.halfHeight;

            float aMin = -a.halfHeight;
            float aMax = +a.halfHeight;

            float bandMin = math.max(aMin, bMinOnA);
            float bandMax = math.min(aMax, bMaxOnA);

            if (bandMax < bandMin + 1e-6f)
                return false;

            // Endpoints of the shared band on A axis
            float s0 = bandMin;
            float s1 = bandMax;

            // For each endpoint, compute witnesses on side surfaces at that axial position, then a deep shared point.
            int count = 0;
            if (EmitParallelSidePoint(in a, in b, aAxis, bAxis, radial, nAB, depth, s0, ref cc, ref count)) { }
            if (EmitParallelSidePoint(in a, in b, aAxis, bAxis, radial, nAB, depth, s1, ref cc, ref count)) { }

            cc.numContactPoints = count;
            return count > 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool EmitParallelSidePoint(
            in Cylinder a, in Cylinder b,
            float3 aAxis, float3 bAxis,
            float3 radial,          // A->B, perp to aAxis
            float3 nAB, float satDepth,
            float sA,               // A axis coordinate
            ref CylinderCylinderContactPoints cc, ref int count)
        {
            // A axis point
            float3 pA_axis = a.center + aAxis * sA;

            // Compute corresponding B axis coordinate (project same world point onto B axis)
            float tB = math.dot(pA_axis - b.center, bAxis);
            tB = math.clamp(tB, -b.halfHeight, +b.halfHeight);
            float3 pB_axis = b.center + bAxis * tB;

            // Use the radial dir (perp to A axis) for side witnesses
            float3 wA = pA_axis + radial * a.radius;
            float3 wB = pB_axis - radial * b.radius;

            // Penetration along radial (more correct for side-side than satDepth)
            float distAxis = math.length((pB_axis - pA_axis) - aAxis * math.dot((pB_axis - pA_axis), aAxis));
            float pen = (a.radius + b.radius) - distAxis;

            if (pen <= 1e-6f)
                pen = satDepth;

            // Deep shared point guaranteed inside both (midpoint of overlap along normal)
            // Use nAB (SAT axis) for consistency with solver push direction.
            // But for parallel side-side, nAB should be ~radial; enforce that for stability:
            float3 n = nAB;
            if (math.dot(n, radial) < 0f) n = -n;

            float3 deep = wA - n * (0.5f * pen);

            // Hard clamp deep point into BOTH cylinders by clamping its axial coordinate to the shared band
            // (this prevents “one endpoint outside” when the other axis clamps differently)
            {
                // Clamp along A
                float ha = math.dot(deep - a.center, aAxis);
                ha = math.clamp(ha, -a.halfHeight, +a.halfHeight);
                float3 baseA = a.center + aAxis * ha;
                float3 ra = deep - baseA;
                ra -= aAxis * math.dot(ra, aAxis);
                float raLenSq = math.lengthsq(ra);
                if (raLenSq > 1e-20f) ra *= (a.radius * math.rsqrt(raLenSq));
                else ra = radial * a.radius;
                deep = baseA + ra;

                // Clamp along B (project onto B axis, then put it on B side surface in the opposite radial)
                float hb = math.dot(deep - b.center, bAxis);
                hb = math.clamp(hb, -b.halfHeight, +b.halfHeight);
                float3 baseB = b.center + bAxis * hb;
                float3 rb = deep - baseB;
                rb -= bAxis * math.dot(rb, bAxis);
                float rbLenSq = math.lengthsq(rb);
                if (rbLenSq > 1e-20f) rb *= (b.radius * math.rsqrt(rbLenSq));
                else rb = -radial * b.radius;

                // Re-center deep point between the two clamped surface points
                float3 deepB = baseB + rb;
                deep = 0.5f * (deep + deepB);
            }

            ContactPoint cp;
            cp.point = deep;
            cp.normal = nAB;
            cp.depth = pen;

            Write(ref cc, count++, cp);
            return true;
        }


        // ============================================================
        // SAT (exact cylinder interval projection; same as your original idea)
        // ============================================================
        private static bool SAT_CylinderCylinder(
            in Cylinder a, in Cylinder b,
            float3 aAxis, float3 bAxis,
            out float3 mtvAxis, out float mtvDepth)
        {
            mtvAxis = float3.zero;
            mtvDepth = float.MaxValue;

            float3 cTo = b.center - a.center;

            // Candidate axes:
            // - cylinder axes
            // - cross(aAxis,bAxis) (if valid)
            // - radial center-to-center projected off aAxis (fallback if cross degenerate)
            float3 ax0 = aAxis;
            float3 ax1 = bAxis;

            float3 ax2 = math.cross(aAxis, bAxis);
            float ax2LenSq = math.lengthsq(ax2);
            bool ax2Valid = ax2LenSq > 1e-10f;
            if (ax2Valid) ax2 *= math.rsqrt(ax2LenSq);

            float3 radial = cTo - aAxis * math.dot(cTo, aAxis);
            float rLenSq = math.lengthsq(radial);
            bool radialValid = rLenSq > 1e-10f;
            if (radialValid) radial *= math.rsqrt(rLenSq);

            // Test base axes
            if (!TestAxis(ax0, in a, in b, aAxis, bAxis, ref mtvAxis, ref mtvDepth)) return false;
            if (!TestAxis(ax1, in a, in b, aAxis, bAxis, ref mtvAxis, ref mtvDepth)) return false;

            // Cross or fallback radial
            if (ax2Valid)
            {
                if (!TestAxis(ax2, in a, in b, aAxis, bAxis, ref mtvAxis, ref mtvDepth)) return false;
            }
            else if (radialValid)
            {
                if (!TestAxis(radial, in a, in b, aAxis, bAxis, ref mtvAxis, ref mtvDepth)) return false;
            }

            // Orient A->B
            if (math.dot(mtvAxis, cTo) < 0f) mtvAxis = -mtvAxis;
            return true;
        }

        private static bool TestAxis(
            float3 n,
            in Cylinder a, in Cylinder b,
            float3 aAxis, float3 bAxis,
            ref float3 mtvAxis,
            ref float mtvDepth)
        {
            float nLenSq = math.lengthsq(n);
            if (nLenSq < kAxisEps) return true;

            n *= math.rsqrt(nLenSq);

            ProjectCylinderInterval(in a, aAxis, n, out float aMin, out float aMax);
            ProjectCylinderInterval(in b, bAxis, n, out float bMin, out float bMax);

            float overlap = math.min(aMax, bMax) - math.max(aMin, bMin);
            if (overlap <= kSlop) return false;

            if (overlap < mtvDepth)
            {
                mtvDepth = overlap;
                mtvAxis = n;
            }
            return true;
        }

        private static void ProjectCylinderInterval(
            in Cylinder cyl, float3 cylAxis, float3 n,
            out float min, out float max)
        {
            float c = math.dot(cyl.center, n);

            float a = math.abs(math.dot(cylAxis, n));
            float radial = math.sqrt(math.max(0f, 1f - a * a));

            float extent = cyl.halfHeight * a + cyl.radius * radial;

            min = c - extent;
            max = c + extent;
        }

        // ============================================================
        // Bepu-style convex manifold: probe support in contact plane + reduce
        // ============================================================
        private static void BuildManifold_ConvexStyle(
            in Cylinder a, in Cylinder b,
            float3 aAxis, float3 bAxis,
            float3 nAB, float depth,
            ref CylinderCylinderContactPoints cc)
        {
            // Contact plane basis
            BuildStableOrthoBasis(nAB, out float3 t0, out float3 t1);

            // Collect candidates (support probes around circle in tangent plane)
            Span<ContactPoint> cand = stackalloc ContactPoint[16];
            int cCount = 0;

            // Always include a center-ish candidate (closest approach along nAB)
            {
                // A: max along +nAB
                float3 wA = SupportCylinder(in a, +nAB);
                // B: min along +nAB == support along -nAB
                float3 wB = SupportCylinder(in b, -nAB);
                AddCandidate(wA, wB, nAB, depth, ref cand, ref cCount);
            }

            // Ring samples: 8 directions around tangent plane
            const int RING = 8;
            const float tilt = 0.35f; // small tilt toward tangents

            for (int i = 0; i < RING; i++)
            {
                float ang = (2f * math.PI) * (i / (float)RING);
                float cs = math.cos(ang);
                float sn = math.sin(ang);

                float3 tan = t0 * cs + t1 * sn;
                float3 dA = math.normalize(+nAB + tan * tilt);

                // A witness: max along dA
                float3 wA = SupportCylinder(in a, dA);

                // B witness: min along dA == support along -dA
                float3 wB = SupportCylinder(in b, -dA);

                AddCandidate(wA, wB, nAB, depth, ref cand, ref cCount);
            }

            if (cCount == 0)
                return;

            // Reduce to <=4 deterministically in tangent plane
            Span<ContactPoint> reduced = stackalloc ContactPoint[4];
            int rCount = ReduceTo4(cand, cCount, nAB, t0, t1, ref reduced);

            // Write out
            InvalidateAll(ref cc);
            cc = default;

            for (int i = 0; i < rCount; i++)
                Write(ref cc, i, reduced[i]);

            cc.numContactPoints = rCount;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void AddCandidate(
            float3 wA, float3 wB,
            float3 nAB, float globalDepth,
            ref Span<ContactPoint> cand, ref int cCount)
        {
            float pen = math.dot(wA - wB, nAB);
            if (pen <= kSlop) return;

            // Clamp for sanity: candidates vary across contact patch but shouldn’t exceed MTV by much
            pen = math.min(pen, globalDepth + 1e-3f);

            ContactPoint cp;
            cp.point = 0.5f * (wA + wB);
            cp.normal = nAB;
            cp.depth = pen;

            if (cCount < cand.Length)
                cand[cCount++] = cp;
        }

        private static int ReduceTo4(
            Span<ContactPoint> cand, int cCount,
            float3 nAB, float3 t0, float3 t1,
            ref Span<ContactPoint> out4)
        {
            // Filter finite + positive
            Span<ContactPoint> tmp = stackalloc ContactPoint[16];
            int tCount = 0;
            for (int i = 0; i < cCount; i++)
            {
                if (cand[i].depth > kSlop && math.all(math.isfinite(cand[i].point)))
                    tmp[tCount++] = cand[i];
            }

            if (tCount == 0) return 0;
            if (tCount == 1) { out4[0] = tmp[0]; return 1; }

            float2 To2(float3 p) => new float2(math.dot(p, t0), math.dot(p, t1));

            // 1) deepest
            int i0 = 0;
            float bestD = tmp[0].depth;
            for (int i = 1; i < tCount; i++)
            {
                if (tmp[i].depth > bestD) { bestD = tmp[i].depth; i0 = i; }
            }

            out4[0] = tmp[i0];
            float2 p0 = To2(out4[0].point);

            // 2) farthest from p0
            int i1 = -1;
            float best = -1f;
            for (int i = 0; i < tCount; i++)
            {
                if (i == i0) continue;
                float2 q = To2(tmp[i].point);
                float d2 = math.lengthsq(q - p0);
                if (d2 > best) { best = d2; i1 = i; }
            }
            if (i1 < 0 || best < 1e-14f) return 1;

            out4[1] = tmp[i1];
            float2 p1 = To2(out4[1].point);

            // 3) farthest from segment
            int i2 = -1;
            best = -1f;
            for (int i = 0; i < tCount; i++)
            {
                if (i == i0 || i == i1) continue;
                float2 q = To2(tmp[i].point);
                float d2 = DistPointSegmentSq(q, p0, p1);
                if (d2 > best) { best = d2; i2 = i; }
            }
            if (i2 < 0 || best < 1e-14f) return 2;

            out4[2] = tmp[i2];
            float2 p2 = To2(out4[2].point);

            // 4) farthest from triangle (outside distance)
            int i3 = -1;
            best = -1f;
            for (int i = 0; i < tCount; i++)
            {
                if (i == i0 || i == i1 || i == i2) continue;
                float2 q = To2(tmp[i].point);
                float d2 = DistPointTriangleSq2D(q, p0, p1, p2);
                if (d2 > best) { best = d2; i3 = i; }
            }
            if (i3 < 0 || best < 1e-14f) return 3;

            out4[3] = tmp[i3];
            return 4;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float DistPointSegmentSq(float2 p, float2 a, float2 b)
        {
            float2 ab = b - a;
            float denom = math.dot(ab, ab);
            if (denom < 1e-20f) return math.lengthsq(p - a);

            float t = math.dot(p - a, ab) / denom;
            t = math.clamp(t, 0f, 1f);
            float2 q = a + ab * t;
            return math.lengthsq(p - q);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float Cross2(float2 a, float2 b) => a.x * b.y - a.y * b.x;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float DistPointTriangleSq2D(float2 p, float2 a, float2 b, float2 c)
        {
            bool s1 = Cross2(b - a, p - a) >= 0f;
            bool s2 = Cross2(c - b, p - b) >= 0f;
            bool s3 = Cross2(a - c, p - c) >= 0f;
            bool inside = (s1 == s2) && (s2 == s3);
            if (inside) return 0f;

            float d0 = DistPointSegmentSq(p, a, b);
            float d1 = DistPointSegmentSq(p, b, c);
            float d2 = DistPointSegmentSq(p, c, a);
            return math.min(d0, math.min(d1, d2));
        }

        // ============================================================
        // Exact analytic support for finite cylinder (native, no mesh)
        // ============================================================
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 SupportCylinder(in Cylinder c, float3 dirW)
        {
            float3 dirL = math.rotate(math.inverse(c.rot), dirW);

            float y = (dirL.y >= 0f) ? c.halfHeight : -c.halfHeight;

            float2 xz = new float2(dirL.x, dirL.z);
            float lenSq = math.dot(xz, xz);

            float x = 0f, z = 0f;
            if (lenSq > 1e-20f)
            {
                float invLen = math.rsqrt(lenSq);
                float2 n2 = xz * invLen;
                x = n2.x * c.radius;
                z = n2.y * c.radius;
            }

            float3 pL = new float3(x, y, z);
            return c.center + math.rotate(c.rot, pL);
        }

        // ============================================================
        // Utility
        // ============================================================
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 GetAxis(in Cylinder c)
        {
            return math.normalize(math.rotate(c.rot, math.up()));
        }

        // Stable orthonormal basis from n (avoids AnyPerp flipping)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void BuildStableOrthoBasis(float3 n, out float3 u, out float3 v)
        {
            n = math.normalize(n);
            if (n.z < -0.9999999f)
            {
                u = new float3(0f, -1f, 0f);
                v = new float3(-1f, 0f, 0f);
                return;
            }

            float a = 1f / (1f + n.z);
            float b = -n.x * n.y * a;

            u = new float3(1f - n.x * n.x * a, b, -n.x);
            v = new float3(b, 1f - n.y * n.y * a, -n.y);

            u = math.normalize(u);
            v = math.normalize(v);
        }

        private static void DedupAndCompact(ref CylinderCylinderContactPoints cc)
        {
            Span<ContactPoint> tmp = stackalloc ContactPoint[4];
            int count = 0;

            if (cc.p1.depth > 0f) tmp[count++] = cc.p1;
            if (cc.p2.depth > 0f) tmp[count++] = cc.p2;
            if (cc.p3.depth > 0f) tmp[count++] = cc.p3;
            if (cc.p4.depth > 0f) tmp[count++] = cc.p4;

            Span<ContactPoint> outPts = stackalloc ContactPoint[4];
            int outCount = 0;

            for (int i = 0; i < count; i++)
            {
                bool dup = false;
                for (int j = 0; j < outCount; j++)
                {
                    if (math.lengthsq(outPts[j].point - tmp[i].point) <= kDedupEpsSq)
                    {
                        dup = true;
                        break;
                    }
                }

                if (!dup)
                    outPts[outCount++] = tmp[i];
            }

            var axis = cc.globalPenAxis;
            var depth = cc.globalPenDepth;

            cc = default;
            InvalidateAll(ref cc);

            if (outCount > 0) cc.p1 = outPts[0];
            if (outCount > 1) cc.p2 = outPts[1];
            if (outCount > 2) cc.p3 = outPts[2];
            if (outCount > 3) cc.p4 = outPts[3];
            cc.numContactPoints = outCount;

            cc.globalPenAxis = axis;
            cc.globalPenDepth = depth;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void Write(ref CylinderCylinderContactPoints cc, int index, ContactPoint cp)
        {
            if (index == 0) cc.p1 = cp;
            else if (index == 1) cc.p2 = cp;
            else if (index == 2) cc.p3 = cp;
            else if (index == 3) cc.p4 = cp;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void InvalidateAll(ref CylinderCylinderContactPoints cc)
        {
            cc.p1.depth = -1f;
            cc.p2.depth = -1f;
            cc.p3.depth = -1f;
            cc.p4.depth = -1f;
            cc.numContactPoints = 0;
        }
    }
}
