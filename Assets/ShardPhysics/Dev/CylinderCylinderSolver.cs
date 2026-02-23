using System;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using UnityEngine;

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

            if (!SAT_CylinderCylinder(in a, in b, aAxis, bAxis, out float3 n, out float depth))
                return false;

            if (depth <= kSlop)
                return false;

            // Orient MTV A->B
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

            bool built = false;

            // ------------------------------------------------------------
            // 1) PARALLEL CAP–CAP
            // ------------------------------------------------------------
            if (!built && axesParallel && capNormal)
            {
                built = BuildCapCapParallelManifold(in a, in b, aAxis, bAxis, n, depth, ref cc);
            }

            // ------------------------------------------------------------
            // 2) PARALLEL SIDE–SIDE
            // ------------------------------------------------------------
            if (!built && axesParallel && sideNormal)
            {
                built = BuildSideSideParallelManifold(in a, in b, aAxis, bAxis, n, depth, ref cc);
            }

            // ------------------------------------------------------------
            // 3) CAP–SIDE (closest-points classifier)
            // ------------------------------------------------------------
            if (!built)
            {
                Debug.Log("Cap side " + Time.time);
                built = TryBuildCapSide_FromClosestPoints(in a, in b, aAxis, bAxis, n, depth, ref cc);
            }

            // ------------------------------------------------------------
            // 3.5) RIM–CAP (both orderings)
            // ------------------------------------------------------------
            if (!built)
            {
                Debug.Log("Rim cap a b " + Time.time);
                built = TryBuildEdgeCapRim(in a, in b, aAxis, bAxis, n, depth, ref cc);
            }

            if (!built)
            {
                Debug.Log("Rim cap b a " + Time.time);
                built = TryBuildEdgeCapRim(in b, in a, bAxis, aAxis, n, depth, ref cc);
            }

            // ------------------------------------------------------------
            // 2.5) SKEW SIDE–SIDE
            // ------------------------------------------------------------
            if (!built && !axesParallel && sideNormal)
            {
                built = TryBuildSideSideSkewSinglePoint_NeverFail(in a, in b, aAxis, bAxis, n, depth, ref cc);
            }

            // ------------------------------------------------------------
            // 4) FALLBACK
            // ------------------------------------------------------------
            if (!built)
            {
                BuildManifold_ConvexStyle(in a, in b, aAxis, bAxis, n, depth, ref cc);
                built = cc.numContactPoints > 0;
            }

            // ------------------------------------------------------------
            // FINALIZE (always run)
            // ------------------------------------------------------------
            if (!built)
                return false;

            // Keep SAT axis/depth stable, but fix points so they are inside BOTH volumes.
            DedupAndCompact(ref cc);
            //RepairAndFilterManifold(in a, in b, aAxis, bAxis, ref cc);

            cc.globalPenAxis = n;
            cc.globalPenDepth = depth;

            //ForcePointsOnCylinderA_AndFixNormals(in a, aAxis, n, ref cc);

            // optional: small dedup again since projection can collapse points
            //DedupAndCompact(ref cc);

            return cc.numContactPoints > 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool BuildCapSide_EdgeParallelToCapFace_TwoPoints(
    in Cylinder Cc, float3 capCenter, float3 capNormalOut, float3 CcAxis,
    in Cylinder Ce, float3 CeAxis,
    float3 O, float3 D,                 // Ce edge line: P(s)=O + D*s, with D == CeAxis (unit)
    float3 nAB, float satDepth,
    ref CylinderCylinderContactPoints cc)
        {
            // Parallel-to-plane and coplanar tolerances (scale-aware)
            float parEps = 1e-6f; // how parallel D is to cap plane
            float planeEps = math.max(1e-5f, 1e-3f * math.min(Cc.radius, Ce.radius)); // how close line is to plane

            float dn = math.abs(math.dot(D, capNormalOut));
            if (dn > parEps)
                return false; // not the "parallel" subcase

            float sd0 = math.dot(O - capCenter, capNormalOut);
            if (math.abs(sd0) > planeEps)
                return false; // line is parallel but not (nearly) in the cap plane => no disk intersection

            // Build cap plane basis (u,v)
            BuildStableOrthoBasis(capNormalOut, out float3 u, out float3 v);

            // 2D line (in cap plane): P2(s) = o2 + d2*s
            float3 OC = O - capCenter;
            float2 o2 = new float2(math.dot(OC, u), math.dot(OC, v));

            float2 d2 = new float2(math.dot(D, u), math.dot(D, v));
            float d2LenSq = math.dot(d2, d2);
            if (d2LenSq < 1e-12f)
                return false;

            // Normalize in 2D so the quadratic is stable and s is in world units (since D is unit).
            float invD2Len = math.rsqrt(d2LenSq);
            d2 *= invD2Len;

            // Circle: |o2 + d2*s|^2 = R^2
            float R = Cc.radius;
            float a = 1f; // dot(d2,d2) == 1
            float b = 2f * math.dot(o2, d2);
            float c = math.dot(o2, o2) - R * R;

            float disc = b * b - 4f * a * c;
            if (disc < 0f)
                return false;

            float sDisc = math.sqrt(disc);
            float s0 = (-b - sDisc) * 0.5f;
            float s1 = (-b + sDisc) * 0.5f;
            if (s1 < s0) { float tmp = s0; s0 = s1; s1 = tmp; }

            // Intersect with the FINITE edge segment param range
            float tMin = -Ce.halfHeight;
            float tMax = +Ce.halfHeight;

            float sEnter = math.max(s0, tMin);
            float sExit = math.min(s1, tMax);

            if (sExit < sEnter - 1e-7f)
                return false;

            float3 p0 = O + D * sEnter;
            float3 p1 = O + D * sExit;

            // Clamp to cap disk (numerical safety)
            p0 = ClampPointToDiskOnPlane(p0, capCenter, capNormalOut, R);
            p1 = ClampPointToDiskOnPlane(p1, capCenter, capNormalOut, R);

            // Must be inside BOTH volumes (your rule)
            if (!InsideBoth(in Cc, CcAxis, in Ce, CeAxis, p0) &&
                !InsideBoth(in Cc, CcAxis, in Ce, CeAxis, p1))
                return false;

            // Emit up to 2 points (skip invalids individually)
            cc = default;
            InvalidateAll(ref cc);

            int count = 0;

            if (InsideBoth(in Cc, CcAxis, in Ce, CeAxis, p0))
            {
                ContactPoint cp;
                cp.point = p0;
                cp.normal = nAB;
                cp.depth = satDepth;
                Write(ref cc, count++, cp);
            }

            if (InsideBoth(in Cc, CcAxis, in Ce, CeAxis, p1))
            {
                // Avoid duplicating if tangent/near-tangent collapses
                if (count == 0 || math.lengthsq(p1 - cc.p1.point) > 1e-12f)
                {
                    ContactPoint cp;
                    cp.point = p1;
                    cp.normal = nAB;
                    cp.depth = satDepth;
                    Write(ref cc, count++, cp);
                }
            }

            cc.numContactPoints = count;
            cc.globalPenAxis = nAB;
            cc.globalPenDepth = satDepth;

            return count > 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ClampPointToDiskOnPlane(float3 p, float3 c, float3 n, float R)
        {
            // project to plane
            float sd = math.dot(p - c, n);
            float3 q = p - n * sd;

            // clamp radius in-plane
            float3 d = q - c;
            d -= n * math.dot(d, n);
            float dsq = math.lengthsq(d);

            float r2 = R * R;
            if (dsq > r2 && dsq > 1e-20f)
                q = c + d * (R * math.rsqrt(dsq));

            return q;
        }


        private static bool TryBuildCapSide_FromClosestPoints(
    in Cylinder a, in Cylinder b,
    float3 aAxis, float3 bAxis,
    float3 nAB, float satDepth,
    ref CylinderCylinderContactPoints cc)
        {
            // Closest points between finite axis segments (used only to choose which cap face sign)
            float3 a0 = a.center - aAxis * a.halfHeight;
            float3 a1 = a.center + aAxis * a.halfHeight;
            float3 b0 = b.center - bAxis * b.halfHeight;
            float3 b1 = b.center + bAxis * b.halfHeight;

            ClosestPoints_SegmentSegment(a0, a1, b0, b1, out float3 pA, out float3 pB);

            float hA = math.dot(pA - a.center, aAxis);
            float hB = math.dot(pB - b.center, bAxis);

            // Use SAT normal alignment to decide which cylinder is more "cap-plane provider"
            float aAlign = math.abs(math.dot(aAxis, nAB));
            float bAlign = math.abs(math.dot(bAxis, nAB));
            bool aIsMoreCapLike = aAlign >= bAlign;

            // Try the more cap-like ordering first, but NEVER hard-reject based on heuristics.
            // If it fails, try the opposite ordering.
            if (aIsMoreCapLike)
            {
                if (TryBuildCapSide_Attempt(in a, in b, aAxis, bAxis, hA, nAB, satDepth, ref cc))
                    return true;

                if (TryBuildCapSide_Attempt(in b, in a, bAxis, aAxis, hB, nAB, satDepth, ref cc))
                    return true;
            }
            else
            {
                if (TryBuildCapSide_Attempt(in b, in a, bAxis, aAxis, hB, nAB, satDepth, ref cc))
                    return true;

                if (TryBuildCapSide_Attempt(in a, in b, aAxis, bAxis, hA, nAB, satDepth, ref cc))
                    return true;
            }

            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool TryBuildCapSide_Attempt(
            in Cylinder capCyl, in Cylinder sideCyl,
            float3 capAxis, float3 sideAxis,
            float hCapClosest,                 // axial coord of closest point on capCyl axis (used to choose cap face sign)
            float3 nAB, float satDepth,
            ref CylinderCylinderContactPoints cc)
        {
            // Pick the actual cap face by sign of closest axial coordinate
            float s = (hCapClosest >= 0f) ? +1f : -1f;
            float3 capCenter = capCyl.center + capAxis * (s * capCyl.halfHeight);
            float3 capNormalOut = capAxis * s;

            // If the side axis is parallel to cap plane, we want the 2-point edge-line intersection.
            bool areCapEdgeParallel = math.abs(math.dot(capNormalOut, sideAxis)) < 0.01f;
            if (areCapEdgeParallel)
            {
                GetFacingEdgeLineTowardCap(
                    in sideCyl, sideAxis,
                    capCyl.center,                 // IMPORTANT: stable facing choice
                    out float3 O, out float3 D);

                if (BuildCapSide_EdgeParallel_TwoPoints_OnEdgeLine(
                    in capCyl, capCenter, capNormalOut, capAxis,
                    in sideCyl, sideAxis,
                    O, D,
                    nAB, satDepth,
                    ref cc))
                    return true;

                // If the edge-parallel 2pt path fails (numerical/InsideBoth), fall through to 1pt regen.
            }

            return BuildCapSide_Regen_OnePoint(
                in capCyl, capCenter, capNormalOut, capAxis,
                in sideCyl, sideAxis,
                nAB, satDepth,
                ref cc);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void GetFacingEdgeLineTowardCap(
    in Cylinder edgeCyl, float3 edgeAxis,
    float3 capCylCenter,                // use cap cylinder CENTER (stable)
    out float3 O, out float3 D)
        {
            D = edgeAxis; // unit

            // Radial direction from edge axis toward the cap cylinder (in edge radial plane)
            float3 toCap = capCylCenter - edgeCyl.center;
            float3 r = toCap - edgeAxis * math.dot(toCap, edgeAxis);
            float rLenSq = math.lengthsq(r);

            if (rLenSq < 1e-12f)
            {
                // degenerate: cap center lies on edge axis -> choose any perpendicular
                BuildStableOrthoBasis(edgeAxis, out float3 u, out _);
                r = u;
                rLenSq = 1f;
            }

            float3 rDir = r * math.rsqrt(rLenSq);

            // This is the *edge line* (generatrix) on the side of edgeCyl facing the cap cylinder
            O = edgeCyl.center + rDir * edgeCyl.radius;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool BuildCapSide_EdgeParallel_TwoPoints_OnEdgeLine(
    in Cylinder capCyl, float3 capCenter, float3 capNormalOut, float3 capAxis,
    in Cylinder edgeCyl, float3 edgeAxis,
    float3 O, float3 D,                 // edge line: P(s)=O + D*s (D unit)
    float3 nAB, float satDepth,
    ref CylinderCylinderContactPoints cc)
        {
            // Must be parallel to cap plane
            if (math.abs(math.dot(D, capNormalOut)) > 1e-6f)
                return false;

            // Signed distance from the line to the cap plane (constant along line)
            float sd = math.dot(O - capCenter, capNormalOut);

            // Project cap center onto the line's parallel plane so the 2D circle test is consistent
            float3 capCenterProj = capCenter + capNormalOut * sd;

            // Cap plane basis
            BuildStableOrthoBasis(capNormalOut, out float3 u, out float3 v);

            // 2D line in cap plane
            float3 OC = O - capCenterProj;
            float2 o2 = new float2(math.dot(OC, u), math.dot(OC, v));

            float2 d2 = new float2(math.dot(D, u), math.dot(D, v));
            float d2LenSq = math.dot(d2, d2);
            if (d2LenSq < 1e-12f)
                return false;

            // Normalize projected direction so a=1
            float inv = math.rsqrt(d2LenSq);
            d2 *= inv;

            float R = capCyl.radius;

            // Solve |o2 + d2*s|^2 = R^2
            float b = 2f * math.dot(o2, d2);
            float c = math.dot(o2, o2) - R * R;

            float disc = b * b - 4f * c;
            if (disc < 0f)
                return false;

            float sDisc = math.sqrt(disc);
            float s0 = (-b - sDisc) * 0.5f;
            float s1 = (-b + sDisc) * 0.5f;
            if (s1 < s0) { float t = s0; s0 = s1; s1 = t; }

            // Clamp to finite edge segment
            float tMin = -edgeCyl.halfHeight;
            float tMax = +edgeCyl.halfHeight;

            float sEnter = math.max(s0, tMin);
            float sExit = math.min(s1, tMax);

            if (sExit < sEnter - 1e-7f)
                return false;

            // Primary points (always on edge line)
            float3 p0 = O + D * sEnter;
            float3 p1 = O + D * sExit;

            // If it collapses numerically, FORCE a stable 2-point set on the segment extents.
            // This is the key to preventing the "single point near exit" drop.
            float minScale = math.min(capCyl.radius, edgeCyl.radius);
            float collapseEps = math.max(1e-5f, 1e-3f * minScale);

            float segLen = sExit - sEnter;
            if (segLen < collapseEps)
            {
                // Use endpoints first (they are by definition clamped to extents)
                float3 pMin = O + D * tMin;
                float3 pMax = O + D * tMax;

                // Pick which endpoint is closer to the computed point (for stability)
                float dMinSq = math.lengthsq(p0 - pMin);
                float dMaxSq = math.lengthsq(p0 - pMax);

                // Order them deterministically: closest first, then other.
                if (dMinSq <= dMaxSq)
                {
                    p0 = pMin;
                    p1 = pMax;
                }
                else
                {
                    p0 = pMax;
                    p1 = pMin;
                }
            }

            // Validate using your rule (inside both volumes).
            // IMPORTANT: do not change points if they fail; just decide whether we can emit.
            bool ok0 = InsideBoth(in capCyl, capAxis, in edgeCyl, edgeAxis, p0);
            bool ok1 = InsideBoth(in capCyl, capAxis, in edgeCyl, edgeAxis, p1);

            // If both fail, no manifold here.
            if (!ok0 && !ok1)
                return false;

            // If one fails (common near exit), still emit TWO points by keeping the other endpoint.
            // This matches your original purpose: always return two points clamped to extents.
            if (!ok0 || !ok1)
            {
                float3 pMin = O + D * (-edgeCyl.halfHeight);
                float3 pMax = O + D * (+edgeCyl.halfHeight);

                // Choose the "other" point as the far endpoint from the valid point, to maximize spread.
                if (ok0 && !ok1)
                {
                    float dMinSq = math.lengthsq(p0 - pMin);
                    float dMaxSq = math.lengthsq(p0 - pMax);
                    p1 = (dMinSq >= dMaxSq) ? pMin : pMax;
                    ok1 = InsideBoth(in capCyl, capAxis, in edgeCyl, edgeAxis, p1);
                }
                else if (ok1 && !ok0)
                {
                    float dMinSq = math.lengthsq(p1 - pMin);
                    float dMaxSq = math.lengthsq(p1 - pMax);
                    p0 = (dMinSq >= dMaxSq) ? pMin : pMax;
                    ok0 = InsideBoth(in capCyl, capAxis, in edgeCyl, edgeAxis, p0);
                }

                // If salvage still can’t get two valid points, fall back to emitting the one valid point.
                // (But we keep the method from returning false and triggering rim-cap unnecessarily.)
                cc = default;
                InvalidateAll(ref cc);

                int count = 0;
                if (ok0)
                {
                    ContactPoint cp;
                    cp.point = p0;
                    cp.normal = nAB;
                    cp.depth = satDepth;
                    Write(ref cc, count++, cp);
                }
                if (ok1)
                {
                    if (count == 0 || math.lengthsq(p1 - cc.p1.point) > 1e-12f)
                    {
                        ContactPoint cp;
                        cp.point = p1;
                        cp.normal = nAB;
                        cp.depth = satDepth;
                        Write(ref cc, count++, cp);
                    }
                }

                cc.numContactPoints = count;
                cc.globalPenAxis = nAB;
                cc.globalPenDepth = satDepth;
                return count > 0;
            }

            // Normal 2-point output
            cc = default;
            InvalidateAll(ref cc);

            {
                ContactPoint cp;
                cp.point = p0;
                cp.normal = nAB;
                cp.depth = satDepth;
                Write(ref cc, 0, cp);
            }
            {
                ContactPoint cp;
                cp.point = p1;
                cp.normal = nAB;
                cp.depth = satDepth;
                Write(ref cc, 1, cp);
            }

            cc.numContactPoints = 2;
            cc.globalPenAxis = nAB;
            cc.globalPenDepth = satDepth;
            return true;
        }


        // Coplanar case: segment-circle intersection in cap plane
        private static bool EmitSegmentCircleIntersections_Coplanar(
            in Cylinder capCyl, float3 capCenter, float3 capNormalOut, float3 capAxis,
            in Cylinder sideCyl, float3 sideAxis,
            float3 O, float3 D,
            float3 nAB, float satDepth,
            ref CylinderCylinderContactPoints cc)
        {
            // Build cap plane basis (u,v)
            BuildStableOrthoBasis(capNormalOut, out float3 u, out float3 v);

            // 2D coords
            float3 OC = O - capCenter;
            float2 o2 = new float2(math.dot(OC, u), math.dot(OC, v));
            float2 d2 = new float2(math.dot(D, u), math.dot(D, v));
            float d2LenSq = math.dot(d2, d2);
            if (d2LenSq < 1e-12f) return false;

            // Normalize so a=1 and t is in world-units because D is unit length in 3D (d2 is just its projection)
            float inv = math.rsqrt(d2LenSq);
            d2 *= inv;

            float R = capCyl.radius;

            // Solve |o2 + d2*t|^2 = R^2
            float b = 2f * math.dot(o2, d2);
            float c = math.dot(o2, o2) - R * R;
            float disc = b * b - 4f * c;
            if (disc < 0f) return false;

            float sDisc = math.sqrt(disc);
            float t0 = (-b - sDisc) * 0.5f;
            float t1 = (-b + sDisc) * 0.5f;
            if (t1 < t0) { float tmp = t0; t0 = t1; t1 = tmp; }

            // Clamp to finite edge segment
            float tMin = -sideCyl.halfHeight;
            float tMax = +sideCyl.halfHeight;
            float tEnter = math.max(t0, tMin);
            float tExit = math.min(t1, tMax);
            if (tExit < tEnter - 1e-7f) return false;

            float3 pA = O + D * tEnter;
            float3 pB = O + D * tExit;

            // Clamp to disk (numerical safety)
            pA = ClampPointToDiskOnPlane(pA, capCenter, capNormalOut, R);
            pB = ClampPointToDiskOnPlane(pB, capCenter, capNormalOut, R);

            // Emit up to 2 points, but keep only those inside BOTH cylinders
            cc = default;
            InvalidateAll(ref cc);
            int count = 0;

            if (InsideBoth(in capCyl, capAxis, in sideCyl, sideAxis, pA))
            {
                ContactPoint cp;
                cp.point = pA;
                cp.normal = nAB;
                cp.depth = satDepth;
                Write(ref cc, count++, cp);
            }

            if (InsideBoth(in capCyl, capAxis, in sideCyl, sideAxis, pB))
            {
                if (count == 0 || math.lengthsq(pB - cc.p1.point) > 1e-12f)
                {
                    ContactPoint cp;
                    cp.point = pB;
                    cp.normal = nAB;
                    cp.depth = satDepth;
                    Write(ref cc, count++, cp);
                }
            }

            cc.numContactPoints = count;
            cc.globalPenAxis = nAB;
            cc.globalPenDepth = satDepth;
            return count > 0;
        }

        // === REGENERATED CAP–SIDE MANIFOLD (ONE POINT) ===
        // Cc = cap cylinder being penetrated
        // Ce = edge cylinder doing the penetrating
        private static bool BuildCapSide_Regen_OnePoint(
            in Cylinder Cc, float3 capCenter, float3 capNormalOut, float3 CcAxis,
            in Cylinder Ce, float3 CeAxis,
            float3 nAB, float satDepth,
            ref CylinderCylinderContactPoints cc)
        {
            // ------------------------------------------------------------
            // 0) Choose Ce edge line (generatrix) that penetrates the cap
            // ------------------------------------------------------------
            float3 intoCap = -capNormalOut; // points into Cc

            float3 rDir = intoCap - CeAxis * math.dot(intoCap, CeAxis);
            float rLenSq = math.lengthsq(rDir);

            if (rLenSq < 1e-12f)
            {
                // fallback: capCenter direction
                rDir = (capCenter - Ce.center);
                rDir -= CeAxis * math.dot(rDir, CeAxis);
                rLenSq = math.lengthsq(rDir);
                if (rLenSq < 1e-12f) return false;
            }

            rDir *= math.rsqrt(rLenSq);

            float3 O = Ce.center + rDir * Ce.radius; // point on Ce edge line
            float3 D = CeAxis;                      // line direction (unit)

            

            // ------------------------------------------------------------
            // 1) capEdgeP: (Ce edge line) ∩ (Cc cap plane), then project + clamp to Cc disk
            // ------------------------------------------------------------
            float denomPlane = math.dot(D, capNormalOut);
            float tCapLine;

            if (math.abs(denomPlane) < 1e-10f)
                tCapLine = math.dot(capCenter - O, D); // closest slice
            else
                tCapLine = math.dot(capCenter - O, capNormalOut) / denomPlane;

            // Clamp anchor to Ce segment so root-selection remains stable in near-parallel cases
            tCapLine = math.clamp(tCapLine, -Ce.halfHeight, +Ce.halfHeight);

            float3 pLineAtCap = O + D * tCapLine; // ON THE LINE
            float3 capEdgeP = ProjectToPlane(pLineAtCap, capCenter, capNormalOut);

            // Clamp to Cc disk
            {
                float3 d = capEdgeP - capCenter;
                d -= capNormalOut * math.dot(d, capNormalOut);
                float dsq = math.lengthsq(d);
                float R = Cc.radius;
                if (dsq > R * R && dsq > 1e-20f)
                    capEdgeP = capCenter + d * (R * math.rsqrt(dsq));
            }

            // ------------------------------------------------------------
            // 2) Intersect Ce edge line with infinite Cc SIDE surface -> (t0,t1)
            // ------------------------------------------------------------
            float t0, t1;
            bool hit = LineVsInfiniteCylinderSide(O, D, Cc.center, CcAxis, Cc.radius, out t0, out t1);
            if (!hit) return false;

            // Choose the infinite root "near the cap witness" (stable branch)
            float dt0 = math.abs(t0 - tCapLine);
            float dt1 = math.abs(t1 - tCapLine);
            float tChosenInf = (dt0 <= dt1) ? t0 : t1;

            // ------------------------------------------------------------
            // 3) edgeEdgeClampP with YOUR RULE:
            // If clamp case, pick the clamped endpoint that is inside BOTH cylinders.
            // Otherwise, use the actual intersection (already correct).
            // ------------------------------------------------------------
            float tMin = -Ce.halfHeight;
            float tMax = +Ce.halfHeight;

            float3 edgeEdgeClampP;

            if (tChosenInf >= tMin && tChosenInf <= tMax)
            {
                // Not a clamp case: use actual intersection on the finite edge segment.
                edgeEdgeClampP = O + D * tChosenInf;
            }
            else
            {
                // Clamp case: choose endpoint inside BOTH
                float3 pMin = O + D * tMin;
                float3 pMax = O + D * tMax;

                bool okMin = InsideBoth(in Cc, CcAxis, in Ce, CeAxis, pMin);
                bool okMax = InsideBoth(in Cc, CcAxis, in Ce, CeAxis, pMax);

                if (okMin && !okMax)
                {
                    edgeEdgeClampP = pMin;
                }
                else if (okMax && !okMin)
                {
                    edgeEdgeClampP = pMax;
                }
                else if (okMin && okMax)
                {
                    // both valid: pick closer to the cap witness along the EDGE PARAM
                    // (this keeps the segment glued to the cap-side patch)
                    float dMin = math.abs(tCapLine - tMin);
                    float dMax = math.abs(tCapLine - tMax);
                    edgeEdgeClampP = (dMin <= dMax) ? pMin : pMax;
                }
                else
                {
                    // neither endpoint is inside both -> no valid clamp solution
                    return false;
                }
            }

            // ------------------------------------------------------------
            // 4) edgeMidP
            // ------------------------------------------------------------
            float3 edgeMidP = 0.5f * (capEdgeP + edgeEdgeClampP);

            // ------------------------------------------------------------
            // 5) deepestRimP (YOUR WAY):
            // pick intersection root deepest along intoCap, lift to cap plane, push to rim
            // ------------------------------------------------------------
            float3 pI0 = O + D * t0;
            float3 pI1 = O + D * t1;

            float s0 = math.dot(pI0, intoCap);
            float s1 = math.dot(pI1, intoCap);

            float3 pInt = (s0 >= s1) ? pI0 : pI1;

            float3 pOnCapPlane = ProjectToPlane(pInt, capCenter, capNormalOut);

            float3 rimDir = pOnCapPlane - capCenter;
            rimDir -= capNormalOut * math.dot(rimDir, capNormalOut);
            float rimLenSq = math.lengthsq(rimDir);
            if (rimLenSq < 1e-20f) return false;

            rimDir *= math.rsqrt(rimLenSq);

            float3 deepestRimP = capCenter + rimDir * Cc.radius;

            // You require this
            if (!PointInsideFiniteCylinder(in Ce, CeAxis, deepestRimP, 1e-5f))
                return false;

            // ------------------------------------------------------------
            // 6) Final CP per your spec
            // ------------------------------------------------------------
            float3 sep = edgeMidP - deepestRimP; // deepestRimP -> edgeMidP
            float dist = math.length(sep);
            if (dist <= 1e-8f) return false;

            float3 n = sep / dist;
            if (math.dot(n, nAB) < 0f) n = -n; // keep convention consistent

            float depth = dist;
            depth = math.min(depth, satDepth + 1e-3f); // optional sanity clamp

            cc = default;
            InvalidateAll(ref cc);

            ContactPoint cp;
            cp.point = 0.5f * (deepestRimP + edgeMidP);
            cp.normal = n;
            cp.depth = depth;

            Write(ref cc, 0, cp);
            cc.numContactPoints = 1;
            cc.globalPenAxis = nAB;
            cc.globalPenDepth = satDepth;
            return true;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ProjectToPlane(float3 p, float3 planePoint, float3 planeN)
        {
            float sd = math.dot(p - planePoint, planeN);
            return p - planeN * sd;
        }

        // Solve intersection of line L(t)=O+D*t with infinite cylinder side surface:
        // distance from L(t) to axis line (Cc.center + CcAxis*s) equals radius.
        private static bool LineVsInfiniteCylinderSide(
            float3 O, float3 D,                 // line origin, dir (need not be perp)
            float3 C, float3 A, float R,         // cylinder center, unit axis, radius
            out float t0, out float t1)
        {
            // Decompose into components perpendicular to axis
            float dA = math.dot(D, A);
            float3 Dp = D - A * dA;

            float3 w0 = O - C;
            float wA = math.dot(w0, A);
            float3 w0p = w0 - A * wA;

            float a = math.dot(Dp, Dp);
            float b = 2f * math.dot(w0p, Dp);
            float c = math.dot(w0p, w0p) - R * R;

            if (a < 1e-12f)
            {
                t0 = t1 = 0f;
                return false;
            }

            float disc = b * b - 4f * a * c;
            if (disc < 0f)
            {
                t0 = t1 = 0f;
                return false;
            }

            float sDisc = math.sqrt(disc);
            float inv2a = 0.5f / a;
            t0 = (-b - sDisc) * inv2a;
            t1 = (-b + sDisc) * inv2a;
            if (t1 < t0) { float tmp = t0; t0 = t1; t1 = tmp; }
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 SafeNormalizeInPlane(float3 v, float3 planeN)
        {
            v -= planeN * math.dot(v, planeN);
            float lsq = math.lengthsq(v);
            if (lsq < 1e-20f)
            {
                BuildStableOrthoBasis(planeN, out float3 u, out _);
                return u;
            }
            return v * math.rsqrt(lsq);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool InsideBoth(
    in Cylinder a, float3 aAxis,
    in Cylinder b, float3 bAxis,
    float3 p)
        {
            float eps = math.max(1e-4f, 1e-3f * math.min(a.radius, b.radius));
            return PointInsideFiniteCylinder(in a, aAxis, p, eps) &&
                   PointInsideFiniteCylinder(in b, bAxis, p, eps);
        }



        // Drop-in replacement for your skew side-side builder.
        // Emits exactly 1 CP on the CORRECT (penetrating) rim.
        // Assumes SAT already produced nAB (A->B) and satDepth > 0.
        // Drop-in replacement for your skew side-side builder.
        // Emits exactly 1 CP ON B'S RIM, but ensures the point is ALSO INSIDE A
        // by searching the B rim circle at the chosen axial anchor and picking the
        // deepest (most toward A) point that lies inside A.
        //
        // normal = nAB (A->B), depth = satDepth (stable)
        private static bool TryBuildSideSideSkewSinglePoint_NeverFail(
    in Cylinder a, in Cylinder b,
    float3 aAxis, float3 bAxis,
    float3 nAB, float satDepth,
    ref CylinderCylinderContactPoints cc)
        {
            if (satDepth <= kSlop) return false;

            float3 d = -nAB; // "into A"

            // Closest points between axis segments (gives stable slice)
            float3 a0 = a.center - aAxis * a.halfHeight;
            float3 a1 = a.center + aAxis * a.halfHeight;
            float3 b0 = b.center - bAxis * b.halfHeight;
            float3 b1 = b.center + bAxis * b.halfHeight;

            ClosestPoints_SegmentSegment(a0, a1, b0, b1, out float3 pA, out float3 pB);

            // ------------------------------------------------------------
            // HARD GATE: true side-side requires closest points to be
            // well inside BOTH finite cylinders (not near caps)
            // ------------------------------------------------------------
            {
                float hA = math.dot(pA - a.center, aAxis);
                float hB = math.dot(pB - b.center, bAxis);

                // Scale-aware margins (important!)
                float marginA = math.max(1e-4f, 0.02f * a.radius);
                float marginB = math.max(1e-4f, 0.02f * b.radius);

                if (math.abs(hA) > a.halfHeight - marginA) return false;
                if (math.abs(hB) > b.halfHeight - marginB) return false;
            }

            // Basis for B radial plane
            BuildStableOrthoBasis(bAxis, out float3 uB, out float3 vB);

            // Deep direction on B rim (projection of d onto B radial plane)
            float3 r = d - bAxis * math.dot(d, bAxis);
            float rLenSq = math.lengthsq(r);
            if (rLenSq < 1e-12f)
            {
                // d almost parallel to bAxis -> not a side-side direction; let other manifolds handle
                return false;
            }
            r *= math.rsqrt(rLenSq);

            // Unconstrained deepest rim point on that ring
            float3 p = pB + r * b.radius;

            // If inside A, we're done
            if (PointInsideFiniteCylinder(in a, aAxis, p, 1e-5f))
            {
                EmitOne(ref cc, p, nAB, satDepth);
                return true;
            }

            // Otherwise, we want the deepest point on THIS rim circle that is still inside A.
            // Approx: clamp to the rim-circle point that lies on the plane dot(x,d)=k,
            // where k is A's support value in direction d at that slice height.
            float3 xSupA = SupportFiniteCylinder_NoRot(in a, aAxis, d);
            float k = math.dot(xSupA, d);

            float Cdot = math.dot(pB, d);
            float maxDot = Cdot + b.radius * math.dot(r, d);
            float target = math.min(maxDot, k);

            // Solve dot(rimDir, d) = (target - Cdot)/b.radius
            float rhs = (target - Cdot) / math.max(b.radius, 1e-20f);

            // rimDir = uB*c + vB*s
            float A1 = math.dot(uB, d);
            float B1 = math.dot(vB, d);
            float L = math.sqrt(A1 * A1 + B1 * B1);
            if (L < 1e-12f) return false;

            float rhsN = rhs / L;
            rhsN = math.clamp(rhsN, -1f, 1f);

            float A1n = A1 / L;
            float B1n = B1 / L;

            float t = math.sqrt(math.max(0f, 1f - rhsN * rhsN));

            float2 sol0 = new float2(rhsN * A1n - t * B1n, rhsN * B1n + t * A1n);
            float2 sol1 = new float2(rhsN * A1n + t * B1n, rhsN * B1n - t * A1n);

            float dot0 = sol0.x * A1n + sol0.y * B1n;
            float dot1 = sol1.x * A1n + sol1.y * B1n;
            float2 bestCS = (dot0 >= dot1) ? sol0 : sol1;

            float3 rimDirBest = uB * bestCS.x + vB * bestCS.y;
            float3 pClamped = pB + rimDirBest * b.radius;

            if (!PointInsideFiniteCylinder(in a, aAxis, pClamped, 1e-5f))
                return false;

            EmitOne(ref cc, pClamped, nAB, satDepth);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void EmitOne(ref CylinderCylinderContactPoints cc, float3 p, float3 nAB, float depth)
        {
            cc = default;
            InvalidateAll(ref cc);

            ContactPoint cp;
            cp.point = p;
            cp.normal = nAB;
            cp.depth = depth;

            Write(ref cc, 0, cp);
            cc.numContactPoints = 1;
            cc.globalPenAxis = nAB;
            cc.globalPenDepth = depth;
        }

        // Finite cylinder support for direction d (no rotation, axis provided)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 SupportFiniteCylinder_NoRot(in Cylinder c, float3 cAxis, float3 d)
        {
            // Choose cap along axis
            float sd = math.dot(d, cAxis);
            float h = (sd >= 0f) ? c.halfHeight : -c.halfHeight;

            // Radial part
            float3 dr = d - cAxis * sd;
            float lenSq = math.lengthsq(dr);
            float3 r = (lenSq > 1e-20f) ? (dr * (c.radius * math.rsqrt(lenSq))) : float3.zero;

            return c.center + cAxis * h + r;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool PointInsideFiniteCylinder(in Cylinder c, float3 cAxis, float3 p, float eps)
        {
            float3 d = p - c.center;

            float h = math.dot(d, cAxis);
            if (h > c.halfHeight + eps || h < -c.halfHeight - eps)
                return false;

            float3 radial = d - cAxis * h;
            float rr = (c.radius + eps);
            return math.lengthsq(radial) <= rr * rr;
        }


        #region Edge intersects Cap

        private static bool TryBuildEdgeCapRim(
    in Cylinder edgeCyl, in Cylinder capCyl,
    float3 edgeAxis, float3 capAxis,
    float3 nAB, float satDepth,
    ref CylinderCylinderContactPoints cc)
        {
            // We want nEdge pointing EDGE -> CAP to choose the facing rim.
            float3 nEdge = nAB;
            float3 edgeToCap = capCyl.center - edgeCyl.center;
            if (math.dot(nEdge, edgeToCap) < 0f)
                nEdge = -nEdge;

            // Choose which rim of edgeCyl faces capCyl
            float sEdge = (math.dot(edgeAxis, nEdge) >= 0f) ? +1f : -1f;
            float3 edgeRimCenter = edgeCyl.center + edgeAxis * (sEdge * edgeCyl.halfHeight);

            // Choose which cap of capCyl faces edgeCyl (outward normal toward edge)
            float3 capToEdge = edgeCyl.center - capCyl.center;
            float sCap = (math.dot(capAxis, capToEdge) >= 0f) ? +1f : -1f;
            float3 capCenter = capCyl.center + capAxis * (sCap * capCyl.halfHeight);
            float3 capNormalOut = capAxis * sCap;

            float3 intoCap = -capNormalOut;

            // Rim plane basis (used for stable fallbacks)
            BuildStableOrthoBasis(edgeAxis, out float3 uE, out _);

            // ------------------------------------------------------------
            // 1) Unconstrained deepest rim point (maximize dot(p, intoCap))
            // ------------------------------------------------------------
            float3 dir = intoCap - edgeAxis * math.dot(intoCap, edgeAxis);
            float dirLenSq = math.lengthsq(dir);
            if (dirLenSq < 1e-20f)
            {
                dir = uE;
                dirLenSq = 1f;
            }
            dir *= math.rsqrt(dirLenSq);

            float3 pRim = edgeRimCenter + dir * edgeCyl.radius;

            // If not valid, do the one-shot correction: clamp to CAP volume then re-project to rim
            if (!InsideBoth(in edgeCyl, edgeAxis, in capCyl, capAxis, pRim))
            {
                float3 q = ClampPointToFiniteCylinderVolume(in capCyl, capAxis, pRim);

                // project q onto rim plane, then clamp to rim circle
                float3 qProj = q - edgeAxis * math.dot(q - edgeRimCenter, edgeAxis);
                float3 d = qProj - edgeRimCenter;
                float dsq = math.lengthsq(d);
                if (dsq < 1e-20f) return false;

                d *= math.rsqrt(dsq);
                pRim = edgeRimCenter + d * edgeCyl.radius;

                if (!InsideBoth(in edgeCyl, edgeAxis, in capCyl, capAxis, pRim))
                    return false;
            }

            // ------------------------------------------------------------
            // 2) Project that deepest rim point onto the OTHER'S CAP (disk)
            // ------------------------------------------------------------
            // Project to cap plane
            float sd = math.dot(pRim - capCenter, capNormalOut);
            float3 pCap = pRim - capNormalOut * sd;

            // Clamp to cap disk radius
            {
                float3 inPlane = pCap - capCenter;
                inPlane -= capNormalOut * math.dot(inPlane, capNormalOut); // should be ~0 already
                float dsq = math.lengthsq(inPlane);
                float R = capCyl.radius;

                if (dsq > R * R && dsq > 1e-20f)
                    pCap = capCenter + inPlane * (R * math.rsqrt(dsq));
            }

            // (Optional but recommended) ensure the cap point really lies inside the CAP cylinder.
            // It should, because it's on the chosen cap plane and clamped to disk.
            // If you want to be strict, you can uncomment:
            // if (!PointInsideFiniteCylinder(in capCyl, capAxis, pCap, 1e-5f)) return false;

            // ------------------------------------------------------------
            // 3) Midpoint between rim point and its cap projection
            // ------------------------------------------------------------
            float3 mid = 0.5f * (pRim + pCap);

            cc = default;
            InvalidateAll(ref cc);

            ContactPoint cp;
            cp.point = mid;
            cp.normal = nAB;
            cp.depth = satDepth;

            Write(ref cc, 0, cp);
            cc.numContactPoints = 1;
            cc.globalPenAxis = nAB;
            cc.globalPenDepth = satDepth;
            return true;
        }



        #endregion

        #region Side-side non-perpendicular intersection
        private static bool TryBuildSideSideSkewSinglePoint(
    in Cylinder a, in Cylinder b,
    float3 aAxis, float3 bAxis,
    float3 nAB, float satDepth,
    ref CylinderCylinderContactPoints cc)
        {
            // Find closest points between the two finite axis segments
            float3 a0 = a.center - aAxis * a.halfHeight;
            float3 a1 = a.center + aAxis * a.halfHeight;

            float3 b0 = b.center - bAxis * b.halfHeight;
            float3 b1 = b.center + bAxis * b.halfHeight;

            ClosestPoints_SegmentSegment(a0, a1, b0, b1, out float3 pA, out float3 pB);

            // Build radial dir on A (perp to aAxis)
            float3 c = pB - pA;
            float3 rA = c - aAxis * math.dot(c, aAxis);
            float rALenSq = math.lengthsq(rA);
            if (rALenSq < 1e-12f)
                return false;

            rA *= math.rsqrt(rALenSq);

            // Orient rA from A->B
            if (math.dot(rA, c) < 0f) rA = -rA;

            // Build radial dir on B (perp to bAxis) using opposite connector
            float3 cB = pA - pB;
            float3 rB = cB - bAxis * math.dot(cB, bAxis);
            float rBLenSq = math.lengthsq(rB);
            if (rBLenSq < 1e-12f)
            {
                // Fallback: just use -rA (still stable)
                rB = -rA;
            }
            else
            {
                rB *= math.rsqrt(rBLenSq);
                if (math.dot(rB, cB) < 0f) rB = -rB;
            }

            // Witness points on side surfaces (“rim”) at the closest axial locations
            float3 wA = pA + rA * a.radius;
            float3 wB = pB + rB * b.radius;

            // Compute penetration along solver axis
            float dep = math.abs(math.dot(wA - wB, nAB));
            if (dep <= kSlop) dep = satDepth;
            if (dep <= kSlop) return false;

            // Contact point requested: "at the rim of the deepest part of the overlapping"
            // Put it on A's rim, but shifted half-way toward the opposing witness for stability.
            float3 point = 0.5f * (wA + wB);

            cc = default;
            InvalidateAll(ref cc);

            ContactPoint cp;
            cp.point = point;
            cp.normal = nAB;
            cp.depth = dep;

            Write(ref cc, 0, cp);
            cc.numContactPoints = 1;
            cc.globalPenAxis = nAB;
            cc.globalPenDepth = math.max(satDepth, dep);
            return true;
        }

        // Robust closest points between two segments in 3D
        // Returns points on each segment (pA on [a0,a1], pB on [b0,b1])
        private static void ClosestPoints_SegmentSegment(
            float3 a0, float3 a1,
            float3 b0, float3 b1,
            out float3 pA, out float3 pB)
        {
            float3 d1 = a1 - a0;
            float3 d2 = b1 - b0;
            float3 r = a0 - b0;

            float a = math.dot(d1, d1);
            float e = math.dot(d2, d2);
            float f = math.dot(d2, r);

            float s, t;

            if (a <= 1e-20f && e <= 1e-20f)
            {
                // Both degenerate
                s = t = 0f;
                pA = a0;
                pB = b0;
                return;
            }

            if (a <= 1e-20f)
            {
                // A degenerate
                s = 0f;
                t = math.clamp(f / math.max(e, 1e-20f), 0f, 1f);
            }
            else
            {
                float c = math.dot(d1, r);

                if (e <= 1e-20f)
                {
                    // B degenerate
                    t = 0f;
                    s = math.clamp(-c / math.max(a, 1e-20f), 0f, 1f);
                }
                else
                {
                    float b = math.dot(d1, d2);
                    float denom = a * e - b * b;

                    if (denom != 0f)
                        s = math.clamp((b * f - c * e) / denom, 0f, 1f);
                    else
                        s = 0f; // parallel-ish

                    float tNom = b * s + f;
                    if (tNom < 0f)
                    {
                        t = 0f;
                        s = math.clamp(-c / math.max(a, 1e-20f), 0f, 1f);
                    }
                    else if (tNom > e)
                    {
                        t = 1f;
                        s = math.clamp((b - c) / math.max(a, 1e-20f), 0f, 1f);
                    }
                    else
                    {
                        t = tNom / e;
                    }
                }
            }

            pA = a0 + d1 * s;
            pB = b0 + d2 * t;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool EmitEdgeCapContactFromRimPoint(
    float3 pRim,
    float3 capCenter,
    float3 capNormalOut,
    float3 nAB,
    float satDepth,
    ref CylinderCylinderContactPoints cc)
        {
            // NOTE: we need the cylinders to validate points, so this function must
            // be updated to take them OR we validate in the caller.
            // If you prefer not to change signature, use the caller-validating version below.
            throw new NotImplementedException("Use the caller-validating overload below.");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool ProjectsInsideCapDisk(float3 p, float3 capCenter, float3 capNormal, float capRadius)
        {
            // Project p onto cap plane and test radius in-plane
            float sd = math.dot(p - capCenter, capNormal);
            float3 inPlane = (p - capCenter) - capNormal * sd;
            return math.lengthsq(inPlane) <= capRadius * capRadius + 1e-6f;
        }

        #endregion


        private static bool TryBuildCapSideEdge(
    in Cylinder capCyl, in Cylinder sideCyl,
    float3 capAxis, float3 sideAxis,
    float3 nAB, float satDepth,
    ref CylinderCylinderContactPoints cc)
        {
            // near 90° gate (looser so we don't fall back)
            if (math.abs(math.dot(sideAxis, capAxis)) > 0.10f)
                return false;

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
                in capCyl, capCenter, capAxis, capNormalOut,
                in sideCyl, sideAxis,
                nAB, satDepth,
                ref cc);
        }

        private static bool BuildCapSide_DeepestPenetratingEdge_TwoPoints(
    in Cylinder cap, float3 capCenter, float3 capAxis, float3 capNormalOut,
    in Cylinder side, float3 sideAxis,
    float3 nAB, float satDepth,
    ref CylinderCylinderContactPoints cc)
        {
            // Build in-plane basis for the cap plane:
            float3 d = sideAxis - capNormalOut * math.dot(sideAxis, capNormalOut);
            float dLenSq = math.lengthsq(d);
            if (dLenSq < kTiny) return false;
            d *= math.rsqrt(dLenSq);

            float3 p = math.cross(capNormalOut, d);
            float pLenSq = math.lengthsq(p);
            if (pLenSq < kTiny) return false;
            p *= math.rsqrt(pLenSq);

            // Signed distance of side axis (through side.center) to the cap plane along capNormalOut:
            float hOut = math.dot(side.center - capCenter, capNormalOut);

            // Deepest penetrating generatrix edge distance to plane:
            // distEdge = hOut - side.radius (negative means behind/in the cap plane)
            float distEdge = hOut - side.radius;

            if (distEdge >= -1e-6f)
                return false;

            // Project side axis onto cap plane
            float3 axisOnPlane = side.center - capNormalOut * hOut;

            float3 rel = axisOnPlane - capCenter;
            float xAxis = math.dot(rel, p);
            float yAxis = math.dot(rel, d);

            float R = cap.radius;

            // Disk intersection in that plane
            float yMaxSq = R * R - xAxis * xAxis;
            float3 inward = -capNormalOut;

            // Tangent-ish: emit ONE point, but only if it's inside both.
            if (yMaxSq <= 1e-10f)
            {
                float t = math.clamp(-yAxis, -side.halfHeight, +side.halfHeight);

                float3 wSide = side.center + sideAxis * t + inward * side.radius;

                if (!InsideBoth(in cap, capAxis, in side, sideAxis, wSide))
                    return false;

                float3 wCap = wSide - capNormalOut * distEdge;

                float dep = math.abs(math.dot(wCap - wSide, nAB));
                if (dep <= kSlop) dep = satDepth;
                if (dep <= kSlop) return false;

                cc = default;
                InvalidateAll(ref cc);

                ContactPoint cp;
                cp.point = wSide;     // on penetrating edge feature
                cp.normal = nAB;
                cp.depth = dep;

                Write(ref cc, 0, cp);
                cc.numContactPoints = 1;
                cc.globalPenAxis = nAB;
                cc.globalPenDepth = math.max(satDepth, dep);
                return true;
            }

            float yMax = math.sqrt(yMaxSq);

            float tMinDisk = -yAxis - yMax;
            float tMaxDisk = -yAxis + yMax;

            float t0 = math.max(tMinDisk, -side.halfHeight);
            float t1 = math.min(tMaxDisk, +side.halfHeight);

            if (t1 < t0 + 1e-7f)
                return false;

            float3 wSide0 = side.center + sideAxis * t0 + inward * side.radius;
            float3 wSide1 = side.center + sideAxis * t1 + inward * side.radius;

            // ---- NEW: endpoint validation ----
            bool ok0 = InsideBoth(in cap, capAxis, in side, sideAxis, wSide0);
            bool ok1 = InsideBoth(in cap, capAxis, in side, sideAxis, wSide1);

            if (!ok0 && !ok1)
                return false;

            float3 wCap0 = wSide0 - capNormalOut * distEdge;
            float3 wCap1 = wSide1 - capNormalOut * distEdge;

            float depth0 = math.abs(math.dot(wCap0 - wSide0, nAB));
            float depth1 = math.abs(math.dot(wCap1 - wSide1, nAB));
            float depth = math.max(depth0, depth1);
            if (depth <= kSlop) return false;

            cc = default;
            InvalidateAll(ref cc);

            int count = 0;

            if (ok0)
            {
                ContactPoint cp0;
                cp0.point = wSide0;
                cp0.normal = nAB;
                cp0.depth = depth;
                Write(ref cc, count++, cp0);
            }

            if (ok1)
            {
                ContactPoint cp1;
                cp1.point = wSide1;
                cp1.normal = nAB;
                cp1.depth = depth;
                Write(ref cc, count++, cp1);
            }

            cc.numContactPoints = count;
            cc.globalPenAxis = nAB;
            cc.globalPenDepth = math.max(satDepth, depth);
            return count > 0;
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
        private static void ForcePointsOnCylinderA_AndFixNormals(
    in Cylinder a, float3 aAxis,
    float3 nAB,
    ref CylinderCylinderContactPoints cc)
        {
            // nAB is already oriented A->B in Solve().
            // Force every contact normal to match that convention.
            // Force every contact point to lie ON A's finite cylinder surface.

            if (cc.numContactPoints <= 0) return;

            for (int i = 0; i < cc.numContactPoints; i++)
            {
                ContactPoint cp = cc[i];
                if (cp.depth <= 0f) continue;

                cp.normal = nAB;
                cp.point = ProjectPointToFiniteCylinderSurface(in a, aAxis, cp.point, nAB);

                Write(ref cc, i, cp);
            }

            cc.globalPenAxis = nAB;
            // keep cc.globalPenDepth unchanged (SAT depth)
        }

        // Projects p to a *surface* point on A (side or cap) in a stable way,
        // using nAB to decide the "facing" surface direction.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ProjectPointToFiniteCylinderSurface(
            in Cylinder c, float3 cAxis,
            float3 p,
            float3 nAB)
        {
            // Clamp axial coordinate first (finite cylinder)
            float3 d = p - c.center;
            float h = math.dot(d, cAxis);
            h = math.clamp(h, -c.halfHeight, +c.halfHeight);

            float3 baseP = c.center + cAxis * h;

            // Decide whether we should snap to a CAP or the SIDE.
            // Use how aligned the normal is with the axis as the classifier.
            float axisAlign = math.abs(math.dot(nAB, cAxis));
            bool preferCap = axisAlign > 0.85f; // tweakable; keeps cap manifolds on caps

            // Radial component at this axial slice
            float3 radial = p - baseP;
            radial -= cAxis * math.dot(radial, cAxis);

            if (preferCap)
            {
                // Cap: keep within disk, don't force to rim.
                float rr = c.radius;
                float rLenSq = math.lengthsq(radial);
                if (rLenSq > rr * rr && rLenSq > 1e-20f)
                    radial *= (rr * math.rsqrt(rLenSq));
                return baseP + radial;
            }
            else
            {
                // Side: force to rim in the direction that faces B.
                // Use the normal projected into the radial plane as the stable rim direction.
                float3 rimDir = nAB - cAxis * math.dot(nAB, cAxis);
                float rimLenSq = math.lengthsq(rimDir);

                if (rimLenSq < 1e-20f)
                {
                    // Fallback: use the existing radial direction if normal is axial-ish.
                    float rLenSq = math.lengthsq(radial);
                    if (rLenSq < 1e-20f)
                    {
                        // Degenerate: pick any perpendicular
                        BuildStableOrthoBasis(cAxis, out float3 u, out _);
                        rimDir = u;
                    }
                    else
                    {
                        rimDir = radial * math.rsqrt(rLenSq);
                    }
                }
                else
                {
                    rimDir *= math.rsqrt(rimLenSq);
                }

                return baseP + rimDir * c.radius;
            }
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ClampPointToFiniteCylinderVolume(in Cylinder c, float3 cAxis, float3 p)
        {
            float3 d = p - c.center;

            // clamp axial
            float h = math.dot(d, cAxis);
            h = math.clamp(h, -c.halfHeight, +c.halfHeight);
            float3 axisPt = c.center + cAxis * h;

            // clamp radial
            float3 r = p - axisPt;
            r -= cAxis * math.dot(r, cAxis);
            float rLenSq = math.lengthsq(r);

            if (rLenSq > 1e-20f)
            {
                float rLen = math.sqrt(rLenSq);
                if (rLen > c.radius)
                    r *= (c.radius / rLen);
            }
            else
            {
                r = float3.zero;
            }

            return axisPt + r;
        }

        private static float3 RepairSharedPointInsideBoth(
            in Cylinder a, float3 aAxis,
            in Cylinder b, float3 bAxis,
            float3 p)
        {
            // 2-3 iterations is enough; this is like alternating projections.
            for (int i = 0; i < 3; i++)
            {
                float3 aP = ClampPointToFiniteCylinderVolume(in a, aAxis, p);
                float3 bP = ClampPointToFiniteCylinderVolume(in b, bAxis, p);
                p = 0.5f * (aP + bP);
            }
            return p;
        }

        private static void RepairAndFilterManifold(
            in Cylinder a, in Cylinder b,
            float3 aAxis, float3 bAxis,
            ref CylinderCylinderContactPoints cc)
        {
            float eps = math.max(1e-4f, 1e-3f * math.min(a.radius, b.radius));

            Span<ContactPoint> pts = stackalloc ContactPoint[4];
            int n = 0;

            if (cc.p1.depth > 0f) pts[n++] = cc.p1;
            if (cc.p2.depth > 0f) pts[n++] = cc.p2;
            if (cc.p3.depth > 0f) pts[n++] = cc.p3;
            if (cc.p4.depth > 0f) pts[n++] = cc.p4;

            // repair + keep only valid
            Span<ContactPoint> outPts = stackalloc ContactPoint[4];
            int outCount = 0;

            for (int i = 0; i < n; i++)
            {
                ContactPoint cp = pts[i];

                // Repair to be inside both volumes
                cp.point = RepairSharedPointInsideBoth(in a, aAxis, in b, bAxis, cp.point);

                // Validate
                if (!PointInsideFiniteCylinder(in a, aAxis, cp.point, eps)) continue;
                if (!PointInsideFiniteCylinder(in b, bAxis, cp.point, eps)) continue;

                outPts[outCount++] = cp;
                if (outCount == 4) break;
            }

            // rewrite cc
            float3 axis = cc.globalPenAxis;
            float depth = cc.globalPenDepth;

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
