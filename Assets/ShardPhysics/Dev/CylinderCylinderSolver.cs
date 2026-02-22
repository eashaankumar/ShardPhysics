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
                built = TryBuildEdgeCapRim(in a, in b, aAxis, bAxis, n, depth, ref cc);
            }

            if (!built)
            {
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
            RepairAndFilterManifold(in a, in b, aAxis, bAxis, ref cc);

            cc.globalPenAxis = n;
            cc.globalPenDepth = depth;

            //ForcePointsOnCylinderA_AndFixNormals(in a, aAxis, n, ref cc);

            // optional: small dedup again since projection can collapse points
            DedupAndCompact(ref cc);

            return cc.numContactPoints > 0;
        }


        private static bool TryBuildCapSide_FromClosestPoints(
    in Cylinder a, in Cylinder b,
    float3 aAxis, float3 bAxis,
    float3 nAB, float satDepth,
    ref CylinderCylinderContactPoints cc)
        {
            // Closest points between finite axis segments
            float3 a0 = a.center - aAxis * a.halfHeight;
            float3 a1 = a.center + aAxis * a.halfHeight;
            float3 b0 = b.center - bAxis * b.halfHeight;
            float3 b1 = b.center + bAxis * b.halfHeight;

            ClosestPoints_SegmentSegment(a0, a1, b0, b1, out float3 pA, out float3 pB);

            float hA = math.dot(pA - a.center, aAxis);
            float hB = math.dot(pB - b.center, bAxis);

            // scale-aware "near cap" margin
            float mA = math.max(1e-4f, 0.03f * a.radius);
            float mB = math.max(1e-4f, 0.03f * b.radius);

            bool aNearCap = math.abs(hA) > (a.halfHeight - mA);
            bool bNearCap = math.abs(hB) > (b.halfHeight - mB);

            // We want exactly one cap involved (cap-side). If both caps involved, it's edge/cap-cap-ish.
            if (aNearCap == bNearCap)
                return false;

            // Select cap cyl + side cyl
            Cylinder capCyl, sideCyl;
            float3 capAxis, sideAxis;
            float hCap;

            if (aNearCap)
            {
                capCyl = a; sideCyl = b;
                capAxis = aAxis; sideAxis = bAxis;
                hCap = hA;
            }
            else
            {
                capCyl = b; sideCyl = a;
                capAxis = bAxis; sideAxis = aAxis;
                hCap = hB;
            }

            // Pick the actual cap face by the sign of the closest-point axial coordinate
            float s = (hCap >= 0f) ? +1f : -1f;
            float3 capCenter = capCyl.center + capAxis * (s * capCyl.halfHeight);
            float3 capNormalOut = capAxis * s;

            // Now build 2-point edge segment on the SIDE cylinder, clipped to the CAP disk
            return BuildCapSide_YourSpec(
                in capCyl, capCenter, capNormalOut, capAxis,
                in sideCyl, sideAxis,
                nAB, satDepth,
                ref cc);
        }

        private static bool BuildCapSide_YourSpec(
    in Cylinder Cc, float3 capCenter, float3 capNormalOut, float3 CcAxis,
    in Cylinder Ce, float3 CeAxis,
    float3 nAB, float satDepth,
    ref CylinderCylinderContactPoints cc)
        {
            // ------------------------------------------------------------
            // 0) Build Ce edge line (infinite): O + CeAxis * t
            // Choose rDir on Ce that points toward the cap region.
            // ------------------------------------------------------------
            float3 rDir = capCenter - Ce.center;
            rDir -= CeAxis * math.dot(rDir, CeAxis);
            float rLenSq = math.lengthsq(rDir);
            if (rLenSq < 1e-12f)
            {
                // fallback: use -capNormal projected into Ce radial plane
                rDir = -capNormalOut;
                rDir -= CeAxis * math.dot(rDir, CeAxis);
                rLenSq = math.lengthsq(rDir);
                if (rLenSq < 1e-12f) return false;
            }
            rDir *= math.rsqrt(rLenSq);

            float3 O = Ce.center + rDir * Ce.radius; // a point on Ce edge line
            float3 D = CeAxis;                      // line dir (unit)

            // ------------------------------------------------------------
            // 1) capEdgeP = intersection of Ce edge line with Cc cap plane,
            // then clamp to Cc disk (since you want it "on Cc cap plane").
            // ------------------------------------------------------------
            float denom = math.dot(D, capNormalOut);
            float tCap;
            if (math.abs(denom) < 1e-10f)
                tCap = math.dot(capCenter - O, D);              // closest slice
            else
                tCap = math.dot(capCenter - O, capNormalOut) / denom;

            float3 capEdgeP = O + D * tCap;
            capEdgeP = ProjectToPlane(capEdgeP, capCenter, capNormalOut);

            // clamp to Cc disk
            {
                float3 d = capEdgeP - capCenter;
                d -= capNormalOut * math.dot(d, capNormalOut);
                float dsq = math.lengthsq(d);
                float R = Cc.radius;
                if (dsq > R * R && dsq > 1e-20f)
                    capEdgeP = capCenter + d * (R * math.rsqrt(dsq));
            }

            // ------------------------------------------------------------
            // 2) edgeEdgeClampP = intersection of Ce edge line with infinite
            // Cc SIDE surface, then clamp t to Ce extents.
            // If no real intersection, clamp to whichever endpoint is closest.
            // ------------------------------------------------------------
            float t0, t1;
            bool hit = LineVsInfiniteCylinderSide(O, D, Cc.center, CcAxis, Cc.radius, out t0, out t1);

            float tEdge;

            if (hit)
            {
                // choose the root whose clamped version is closer to the true root
                float t0c = math.clamp(t0, -Ce.halfHeight, +Ce.halfHeight);
                float t1c = math.clamp(t1, -Ce.halfHeight, +Ce.halfHeight);

                // prefer the one less disturbed by clamp
                float err0 = math.abs(t0c - t0);
                float err1 = math.abs(t1c - t1);
                tEdge = (err0 <= err1) ? t0c : t1c;
            }
            else
            {
                // no intersection: clamp to ends (your “clamp if never exists”)
                float tA = -Ce.halfHeight;
                float tB = +Ce.halfHeight;

                // pick endpoint that is *closest* to Cc side surface
                float3 pA = O + D * tA;
                float3 pB = O + D * tB;

                float da = math.sqrt(DistSqPointToAxis(pA, Cc.center, CcAxis));
                float db = math.sqrt(DistSqPointToAxis(pB, Cc.center, CcAxis));

                float ea = math.abs(da - Cc.radius);
                float eb = math.abs(db - Cc.radius);

                tEdge = (ea <= eb) ? tA : tB;
            }

            float3 edgeEdgeClampP = O + D * tEdge;

            // ------------------------------------------------------------
            // 3) edgeMidP
            // ------------------------------------------------------------
            float3 edgeMidP = 0.5f * (capEdgeP + edgeEdgeClampP);

            // ------------------------------------------------------------
            // 4) deepestRimP (YOUR WAY):
            // intersect infinite Ce edge with infinite Cc side -> take that intersection point,
            // project up to Cc cap plane, then push to rim.
            // ------------------------------------------------------------
            float tInf;
            if (hit)
            {
                // choose the intersection closer to the cap plane (more “vertically below the rim”)
                float3 pI0 = O + D * t0;
                float3 pI1 = O + D * t1;
                float a0 = math.abs(math.dot(pI0 - capCenter, capNormalOut));
                float a1 = math.abs(math.dot(pI1 - capCenter, capNormalOut));
                tInf = (a0 <= a1) ? t0 : t1;
            }
            else
            {
                // if no analytic intersection, use unclamped tCap slice as a proxy “below rim”
                tInf = tCap;
            }

            float3 pInt = O + D * tInf;                                 // “below rim” on infinite geometry
            float3 pOnCapPlane = ProjectToPlane(pInt, capCenter, capNormalOut);

            float3 rimDir = SafeNormalizeInPlane(pOnCapPlane - capCenter, capNormalOut);
            float3 deepestRimP = capCenter + rimDir * Cc.radius;

            // ------------------------------------------------------------
            // 5) Final CP per your spec
            // normal = direction(deepestRimP, edgeMidP)
            // depth = dist(deepestRimP, edgeMidP)
            // ------------------------------------------------------------
            float3 sep = edgeMidP - deepestRimP; // direction(deepestRimP -> edgeMidP)
            float dist = math.length(sep);
            if (dist <= 1e-8f) return false;

            float3 n = sep / dist;

            // maintain solver convention (A->B) using SAT axis as reference
            if (math.dot(n, nAB) < 0f) n = -n;

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
        private static float DistSqPointToAxis(float3 p, float3 axisPoint, float3 axisUnit)
        {
            // axisUnit MUST be normalized
            float3 d = p - axisPoint;

            float proj = math.dot(d, axisUnit);     // axial component
            float3 radial = d - axisUnit * proj;    // perpendicular component

            return math.dot(radial, radial);        // squared distance
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
            float3 edgeRimNormal = edgeAxis;

            // Choose which cap of capCyl faces edgeCyl (outward normal toward edge)
            float3 capToEdge = edgeCyl.center - capCyl.center;
            float sCap = (math.dot(capAxis, capToEdge) >= 0f) ? +1f : -1f;
            float3 capCenter = capCyl.center + capAxis * (sCap * capCyl.halfHeight);
            float3 capNormalOut = capAxis * sCap;

            // ------------------------------------------------------------
            // 1) Deepest rim point relative to the cap plane
            // ------------------------------------------------------------
            float3 q = capNormalOut - edgeRimNormal * math.dot(capNormalOut, edgeRimNormal);
            float qLenSq = math.lengthsq(q);

            if (qLenSq > 1e-12f)
            {
                q *= math.rsqrt(qLenSq);

                // Deepest rim point is opposite q (most negative plane distance)
                float3 pDeep = edgeRimCenter - q * edgeCyl.radius;

                if (ProjectsInsideCapDisk(pDeep, capCenter, capNormalOut, capCyl.radius))
                {
                    // --- emit from rim point, but choose a point that is inside BOTH ---
                    float sd = math.dot(pDeep - capCenter, capNormalOut);
                    float3 pCap = pDeep - capNormalOut * sd;
                    float3 shared = 0.5f * (pDeep + pCap);

                    float3 chosen = shared;

                    if (!InsideBoth(in edgeCyl, edgeAxis, in capCyl, capAxis, chosen))
                    {
                        if (InsideBoth(in edgeCyl, edgeAxis, in capCyl, capAxis, pDeep))
                            chosen = pDeep;
                        else if (InsideBoth(in edgeCyl, edgeAxis, in capCyl, capAxis, pCap))
                            chosen = pCap;
                        else
                            goto STEP2; // no valid point from this candidate
                    }

                    cc = default;
                    InvalidateAll(ref cc);

                    ContactPoint cp;
                    cp.point = chosen;
                    cp.normal = nAB;
                    cp.depth = satDepth;

                    Write(ref cc, 0, cp);
                    cc.numContactPoints = 1;
                    cc.globalPenAxis = nAB;
                    cc.globalPenDepth = satDepth;
                    return true;
                }
            }

        STEP2:
            // ------------------------------------------------------------
            // 2) Rim circle intersects cap plane -> pick a valid endpoint
            // ------------------------------------------------------------
            float3 lineDir = math.cross(edgeRimNormal, capNormalOut);
            float lineDirLenSq = math.lengthsq(lineDir);
            if (lineDirLenSq < 1e-12f)
                return false;

            float3 l = lineDir;
            float lLenSq = lineDirLenSq;

            float d0 = math.dot(edgeRimNormal, edgeRimCenter);
            float d1 = math.dot(capNormalOut, capCenter);

            float3 x0 =
                (d0 * math.cross(capNormalOut, l) +
                 d1 * math.cross(l, edgeRimNormal)) / lLenSq;

            lineDir *= math.rsqrt(lineDirLenSq);

            // Intersect line with rim circle
            float3 m = x0 - edgeRimCenter;
            float B = 2f * math.dot(lineDir, m);
            float C = math.dot(m, m) - edgeCyl.radius * edgeCyl.radius;
            float disc = B * B - 4f * C; // A=1

            if (disc < 0f)
                return false;

            float sqrtDisc = math.sqrt(math.max(0f, disc));
            float tA = (-B - sqrtDisc) * 0.5f;
            float tB = (-B + sqrtDisc) * 0.5f;

            float3 pA = x0 + lineDir * tA;
            float3 pB = x0 + lineDir * tB;

            bool okA = ProjectsInsideCapDisk(pA, capCenter, capNormalOut, capCyl.radius);
            bool okB = ProjectsInsideCapDisk(pB, capCenter, capNormalOut, capCyl.radius);

            if (!okA && !okB)
                return false;

            // Prefer endpoint with smallest signed distance (most "behind")
            float bestSd = float.PositiveInfinity;
            float3 bestP = default;

            if (okA)
            {
                float sd = math.dot(pA - capCenter, capNormalOut);
                if (sd < bestSd) { bestSd = sd; bestP = pA; }
            }
            if (okB)
            {
                float sd = math.dot(pB - capCenter, capNormalOut);
                if (sd < bestSd) { bestSd = sd; bestP = pB; }
            }

            // --- emit from bestP, but choose a point that is inside BOTH ---
            {
                float sd = math.dot(bestP - capCenter, capNormalOut);
                float3 pCap = bestP - capNormalOut * sd;
                float3 shared = 0.5f * (bestP + pCap);

                float3 chosen = shared;

                if (!InsideBoth(in edgeCyl, edgeAxis, in capCyl, capAxis, chosen))
                {
                    if (InsideBoth(in edgeCyl, edgeAxis, in capCyl, capAxis, bestP))
                        chosen = bestP;
                    else if (InsideBoth(in edgeCyl, edgeAxis, in capCyl, capAxis, pCap))
                        chosen = pCap;
                    else
                        return false;
                }

                cc = default;
                InvalidateAll(ref cc);

                ContactPoint cp;
                cp.point = chosen;
                cp.normal = nAB;
                cp.depth = satDepth;

                Write(ref cc, 0, cp);
                cc.numContactPoints = 1;
                cc.globalPenAxis = nAB;
                cc.globalPenDepth = satDepth;
                return true;
            }
        }


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

        #endregion

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
