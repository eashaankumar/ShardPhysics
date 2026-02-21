using System;
using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace Shard.Dev
{
    // NOTE:
    // This is a CYLINDER vs CYLINDER solver (finite cylinders, flat caps).
    // It computes:
    //  - globalPenAxis (A -> B)
    //  - globalPenDepth (MTV magnitude)
    //  - 1/2/4 contact points depending on configuration
    //
    // Rules (per your request):
    //  - Side-side collision => 2 contact points (clamped).
    //  - Face-face (cap-cap overlap) => up to 4 contact points, equally distributed,
    //    ALWAYS ON RIMS, clamped to BOTH cylinders, and lying on the intersecting plane.
    //  - 1 contact point only when:
    //      * they don't share a side band OR
    //      * one cap intersects anywhere on the other (cap-involved)
    //
    // IMPORTANT: Do NOT use capsule "segment distance + r" as the overlap test (false positives).
    // We use SAT (or other exact test) to decide if colliding. Manifold generation must not invent collisions.
    public static class CylinderCylinderSolver
    {
        public struct ContactPoint
        {
            public float3 point;   // stable shared point (midpoint of two witnesses)
            public float3 normal;  // global MTV axis (A -> B)
            public float depth;    // per-contact depth (optional / debug)
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

        private const float kSlop = 1e-5f;
        private const float kEpsLenSq = 1e-12f;
        private const float kAxisParallel = 0.9995f;
        private const float kDedupEpsSq = 1e-8f;
        private const float kCapSideParallelEps = 0.02f; // |dot(capAxis, sideAxis)| < eps => cap plane contains side axis
        private const float kSideNormalMinLenSq = 1e-10f;

        // ------------------------------------------------------------
        // Public API (placeholder narrowphase)
        // ------------------------------------------------------------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Solve(in Cylinder a, in Cylinder b, out CylinderCylinderContactPoints cc)
        {
            cc = default;

            float3 aAxis = GetAxis(in a);
            float3 bAxis = GetAxis(in b);

            if (!SAT_CylinderCylinder(in a, in b, aAxis, bAxis, out float3 satAxisAB, out float satDepth))
                return false;

            cc.globalPenAxis = satAxisAB;
            cc.globalPenDepth = satDepth;

            float3 cTo = b.center - a.center;

            // -------------------------
            // CAP-CAP
            // -------------------------
            float da = math.abs(math.dot(satAxisAB, aAxis));
            float db = math.abs(math.dot(satAxisAB, bAxis));
            bool capCap = (da > 0.999f) && (db > 0.999f);

            if (capCap)
            {
                if (BuildCapCapManifold(in a, in b, aAxis, bAxis, satAxisAB, satDepth, out cc))
                    return true;
                return false;
            }

            // -------------------------
            // SIDE-SIDE "looks like" fast path
            // -------------------------
            float daSide = math.abs(math.dot(satAxisAB, aAxis));
            float dbSide = math.abs(math.dot(satAxisAB, bAxis));
            bool looksSideSide = (daSide < 0.85f) && (dbSide < 0.85f);

            if (looksSideSide)
            {
                if (BuildSideSideManifold(in a, in b, aAxis, bAxis, satAxisAB, satDepth, out cc))
                    return true;
            }

            // -------------------------
            // CAP-SIDE tries
            // -------------------------
            if (BuildCapSideManifold(in a, in b, aAxis, bAxis, satAxisAB, satDepth, out cc))
                return true;

            // Force A-as-cap
            {
                float s = math.dot(aAxis, cTo) >= 0f ? 1f : -1f;
                float3 forceAxisA = aAxis * s;
                if (BuildCapSideManifold(in a, in b, aAxis, bAxis, forceAxisA, satDepth, out cc))
                    return true;

                forceAxisA = -forceAxisA;
                if (BuildCapSideManifold(in a, in b, aAxis, bAxis, forceAxisA, satDepth, out cc))
                    return true;
            }

            // Force B-as-cap
            {
                float s = math.dot(bAxis, (a.center - b.center)) >= 0f ? 1f : -1f;
                float3 bTowardA = bAxis * s;
                float3 forceAxisB = -bTowardA;
                if (BuildCapSideManifold(in a, in b, aAxis, bAxis, forceAxisB, satDepth, out cc))
                    return true;

                forceAxisB = -forceAxisB;
                if (BuildCapSideManifold(in a, in b, aAxis, bAxis, forceAxisB, satDepth, out cc))
                    return true;
            }

            // -------------------------
            // FIXED FALLBACK:
            // If we get here, SAT says overlap but neither manifold path triggered.
            // This is your "deadzone". It is usually CAP-SIDE-ish, so we must NOT
            // generate a "side witness on the cap cylinder" or use a cap-plane normal.
            //
            // We'll build a CAP-SIDE contact by clamping a cap point to the cap disk,
            // and using the other cylinder's SIDE radial for the normal.
            // -------------------------
            {
                // Closest points on finite axis segments
                ClosestPoints_SegmentSegment(
                    a.center, aAxis, -a.halfHeight, a.halfHeight,
                    b.center, bAxis, -b.halfHeight, b.halfHeight,
                    out float sA, out float tB);

                // Determine if either segment point is near an end-cap (cap involvement)
                const float endFrac = 0.98f; // tweak if you want earlier/later snap
                bool aNearCap = math.abs(sA) > a.halfHeight * endFrac;
                bool bNearCap = math.abs(tB) > b.halfHeight * endFrac;

                // If cap-ish, prefer that cylinder as the cap cylinder.
                // If both/none, fall back to which axis SAT is more aligned with.
                bool capIsA;
                if (aNearCap ^ bNearCap) capIsA = aNearCap;
                else capIsA = (math.abs(math.dot(satAxisAB, aAxis)) > math.abs(math.dot(satAxisAB, bAxis)));

                Cylinder capCyl = capIsA ? a : b;
                Cylinder sideCyl = capIsA ? b : a;

                float3 capAxis = capIsA ? aAxis : bAxis;
                float3 sideAxis = capIsA ? bAxis : aAxis;

                // Pick which cap (top/bottom) based on the closest axis param sign
                float capParam = capIsA ? sA : tB;
                float capSign = (capParam >= 0f) ? 1f : -1f;
                float3 capCenter = capCyl.center + capAxis * (capSign * capCyl.halfHeight);

                // Build cap point by projecting SIDE axis point onto cap plane and clamping to disk
                float3 sideAxisPoint = sideCyl.center + sideAxis * math.clamp(
                    math.dot((capCenter - sideCyl.center), sideAxis),
                    -sideCyl.halfHeight, +sideCyl.halfHeight);

                // Cap plane basis
                float3 u = AnyPerp(capAxis);
                float3 v = math.normalize(math.cross(capAxis, u));

                // Project side axis point onto cap plane
                float3 proj = sideAxisPoint - capAxis * math.dot(sideAxisPoint - capCenter, capAxis);

                // Clamp to filled cap disk (THIS guarantees inside the cap cylinder)
                float3 capPoint = ClampPointToDiskPlane(proj, capCenter, capCyl.radius, u, v);

                // Side witness on SIDE cylinder (true side surface point)
                float3 wSide = SideWitnessForCapPoint(capPoint, sideCyl.center, sideAxis, sideCyl.halfHeight, sideCyl.radius);

                // Radial normal from SIDE centerline -> cap point (THIS is the radial you want)
                float3 nSideOut = RadialFromSideAxisToPoint_Clamped(capPoint, sideCyl.center, sideAxis, sideCyl.halfHeight);

                // Orient A->B
                float3 nAB = capIsA ? -nSideOut : nSideOut;
                if (math.dot(nAB, cTo) < 0f) nAB = -nAB;

                // Pen depth from side radius minus radial distance
                float t = math.dot(capPoint - sideCyl.center, sideAxis);
                t = math.clamp(t, -sideCyl.halfHeight, +sideCyl.halfHeight);
                float3 pAxis = sideCyl.center + sideAxis * t;

                float3 r = capPoint - pAxis;
                r -= sideAxis * math.dot(r, sideAxis);
                float rLen = math.length(r);
                float penDepth = sideCyl.radius - rLen;
                if (penDepth <= kSlop) penDepth = satDepth;

                InvalidateAll(ref cc);

                ContactPoint cp;
                cp.point = 0.5f * (capPoint + wSide);
                cp.normal = nAB;
                cp.depth = penDepth;

                Write(ref cc, 0, cp);
                cc.numContactPoints = 1;
                cc.globalPenAxis = nAB;
                cc.globalPenDepth = penDepth;

                DedupAndCompact(ref cc);
                return cc.numContactPoints > 0;
            }
        }




        #region Cap-Side
        private static void EmitOneRimPoint(
            float3 circleCenterW, float3 u, float3 v,
            float2 p0, float2 d0, float t,
            in Cylinder side, float3 sideAxis,
            float3 nAB, float penDepth,
            ref CylinderCylinderContactPoints cc, ref int count)
        {
            float2 pt2 = p0 + d0 * t;
            float3 pW = circleCenterW + u * pt2.x + v * pt2.y;

            // Must lie on the FINITE side cylinder (within height)
            float h = math.dot(pW - side.center, sideAxis);
            if (math.abs(h) > side.halfHeight + 1e-5f)
                return;

            ContactPoint cp;
            cp.point = pW;      // both witnesses are on the intersection curve in this case
            cp.normal = nAB;
            cp.depth = penDepth; // keep stable depth (not 0 at rim)
            Write(ref cc, count++, cp);
        }

        // ------------------------------------------------------------
        // CAP-SIDE manifold (EXACT witness + reject)
        // ------------------------------------------------------------
        // Produces EXACTLY 1 CP, only if the cap disk actually penetrates the OTHER cylinder's SIDE surface.
        // Normal is RADIAL of the penetred side surface (oriented A->B).
        // Point is the "deepest" cap point = closest point on cap disk to the side cylinder axis segment.
        private static bool BuildCapSideManifold(
            in Cylinder a, in Cylinder b,
            float3 aAxisW, float3 bAxisW,
            float3 mtvAxis, float satDepth,
            out CylinderCylinderContactPoints cc)
        {
            cc = default;
            InvalidateAll(ref cc);

            // Decide which one is cap-like based on MTV alignment.
            float da = math.abs(math.dot(mtvAxis, aAxisW));
            float db = math.abs(math.dot(mtvAxis, bAxisW));
            bool aIsCap = da > db;

            Cylinder capCyl = aIsCap ? a : b;
            Cylinder sideCyl = aIsCap ? b : a;

            float3 capAxis = aIsCap ? aAxisW : bAxisW;
            float3 sideAxis = aIsCap ? bAxisW : aAxisW;

            // Direction from cap cylinder towards the other (for selecting facing cap)
            float3 towardOther = aIsCap ? mtvAxis : -mtvAxis;

            // Pick facing cap center on the cap cylinder
            float capSign = (math.dot(capAxis, towardOther) >= 0f) ? +1f : -1f;
            float3 capCenter = capCyl.center + capAxis * (capSign * capCyl.halfHeight);

            // Special case: side axis lies in the cap plane -> contact is a chord, emit 2/4 rim points.
            if (math.abs(math.dot(sideAxis, capAxis)) < kCapSideParallelEps)
            {
                if (!BuildCapSideAxisInCapPlaneManifold(
                    in capCyl, capCenter, capAxis,
                    in sideCyl, sideAxis,
                    aIsCap,
                    mtvAxis, satDepth,
                    out cc))
                    return false;

                return true;
            }

            // Compute deepest witnesses for cap disk vs side surface.
            if (!DeepestCapDiskVsSide(
                    in capCyl, capCenter, capAxis,
                    in sideCyl, sideAxis,
                    out float3 wCap, out float3 wSide,
                    out float3 nSideOut, out float penDepth))
            {
                return false; // exact reject (fixes "detecting cps when not colliding")
            }

            // Orient normal as A->B (your convention everywhere else)
            // nSideOut points from SIDE cylinder axis -> cap point (i.e., from SIDE toward CAP).
            // If CAP is A and SIDE is B, then A->B is opposite that.
            float3 nAB = aIsCap ? -aAxisW : -bAxisW;

            // Make depth consistent with the axis you are returning
            float depthAB = math.dot((wSide - wCap), nAB);   // should be positive
            if (depthAB <= kSlop) depthAB = penDepth;        // fallback


            ContactPoint cp;
            cp.point = wSide;
            cp.normal = nAB;
            cp.depth = depthAB;

            Write(ref cc, 0, cp);
            cc.numContactPoints = 1;

            // For cap-side, global penetration axis/depth should match the actual side radial,
            // not the cap normal / SAT MTV.
            cc.globalPenAxis = nAB;
            cc.globalPenDepth = depthAB;

            DedupAndCompact(ref cc);
            return cc.numContactPoints > 0;
        }

        private static bool BuildCapSideAxisInCapPlaneManifold(
    in Cylinder cap, float3 capCenter, float3 capAxis,
    in Cylinder side, float3 sideAxis,
    bool capIsA,
    float3 mtvAxisAB, float satDepth,   // NEW: penetration axis (A->B) and depth
    out CylinderCylinderContactPoints cc)
        {
            cc = default;
            InvalidateAll(ref cc);

            if (satDepth <= kSlop)
                return false;

            // sideAxis lies in cap plane (by caller)
            if (math.abs(math.dot(sideAxis, capAxis)) > kCapSideParallelEps)
                return false;

            // Build in-plane basis:
            float3 d = sideAxis - capAxis * math.dot(sideAxis, capAxis);
            float dLenSq = math.lengthsq(d);
            if (dLenSq < 1e-12f)
                return false;
            d *= math.rsqrt(dLenSq);

            float3 p = math.cross(capAxis, d);
            float pLenSq = math.lengthsq(p);
            if (pLenSq < 1e-12f)
                return false;
            p *= math.rsqrt(pLenSq);

            // Signed distance from side axis to cap plane (along capAxis)
            float h = math.dot(side.center - capCenter, capAxis);
            float hAbs = math.abs(h);

            if (hAbs > side.radius + 1e-6f)
                return false;

            float sMax = math.sqrt(math.max(0f, side.radius * side.radius - h * h));
            if (sMax <= 1e-6f)
                return false;

            float3 axisOnPlane = side.center - capAxis * h;

            float xAxis = math.dot(axisOnPlane - capCenter, p);

            float R = cap.radius;

            // Feasibility: disk interval [-R, +R] overlaps strip interval [xAxis - sMax, xAxis + sMax]
            float overlap1D = (R + sMax) - math.abs(xAxis);
            if (overlap1D <= kSlop)
                return false;

            // Choose a chord within disk at x = clamp(xAxis, -R, +R)
            float xp = math.clamp(xAxis, -R, +R);
            float y = math.sqrt(math.max(0f, R * R - xp * xp));
            if (y <= 1e-8f)
                return false;

            float3 chord0 = capCenter + p * xp + d * (-y);
            float3 chord1 = capCenter + p * xp + d * (+y);

            // Clip chord to finite side height
            if (!ClipChordToFiniteSideHeight(
                    chord0, chord1,
                    side.center, sideAxis, side.halfHeight,
                    out float3 c0, out float3 c1))
            {
                return false;
            }

            // ---- Contact points MUST be ON SIDE surface (your requirement) ----
            float3 wSide0 = SideWitnessForCapPoint(c0, side.center, sideAxis, side.halfHeight, side.radius);
            float3 wSide1 = SideWitnessForCapPoint(c1, side.center, sideAxis, side.halfHeight, side.radius);

            // Normal should be the penetration axis direction (A->B), since your depth is defined along it.
            float3 nAB = mtvAxisAB;
            // (mtvAxisAB should already be oriented A->B by SAT; keep this here for safety if you ever pass a forced axis)
            // if (math.dot(nAB, (capIsA ? (side.center - cap.center) : (cap.center - side.center))) < 0f) nAB = -nAB;

            int count = 0;

            ContactPoint cp0;
            cp0.point = 0.5f * (c0 + wSide0);     // ON SIDE
            cp0.normal = nAB;        // pen axis
            cp0.depth = satDepth;   // pen depth along pen axis (uniform)
            Write(ref cc, count++, cp0);

            ContactPoint cp1;
            cp1.point = 0.5f * (c1 + wSide1);     // ON SIDE
            cp1.normal = nAB;
            cp1.depth = satDepth;
            Write(ref cc, count++, cp1);

            cc.numContactPoints = count;
            cc.globalPenAxis = nAB;
            cc.globalPenDepth = satDepth;

            DedupAndCompact(ref cc);
            return cc.numContactPoints > 0;
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float Eval_SegmentPoint_ToCapDisk_DistSq(
            float t01,
            float3 seg0, float3 seg1,
            float3 capCenter, float3 capAxis, float capRadius,
            float3 u, float3 v,
            out float3 outSegP,
            out float3 outDiskP)
        {
            // segment point
            outSegP = math.lerp(seg0, seg1, t01);

            // project to cap plane
            float3 proj = outSegP - capAxis * math.dot(outSegP - capCenter, capAxis);

            // clamp to filled disk
            outDiskP = ClampPointToDiskPlane(proj, capCenter, capRadius, u, v);

            return math.lengthsq(outSegP - outDiskP);
        }

        // -------------------------------------------------------------------
        // NEW helper #2 (needed by the rewritten BuildCapSideAxisInCapPlaneManifold)
        // Returns SIDE->POINT radial (perp to sideAxis), using clamped axis segment.
        // -------------------------------------------------------------------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 RadialFromSideAxisToPoint_Clamped(
            float3 pointW,
            float3 sideCenter, float3 sideAxis, float sideHalfHeight)
        {
            float t = math.dot(pointW - sideCenter, sideAxis);
            t = math.clamp(t, -sideHalfHeight, +sideHalfHeight);
            float3 pAxis = sideCenter + sideAxis * t;

            float3 r = pointW - pAxis;
            r -= sideAxis * math.dot(r, sideAxis);

            float rLenSq = math.lengthsq(r);
            if (rLenSq < 1e-12f)
                return AnyPerp(sideAxis);

            return r * math.rsqrt(rLenSq);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 SideWitnessForCapPoint(
            float3 capPoint,
            float3 sideCenter, float3 sideAxis, float sideHalfHeight, float sideRadius)
        {
            float t = math.dot(capPoint - sideCenter, sideAxis);
            t = math.clamp(t, -sideHalfHeight, +sideHalfHeight);
            float3 pAxis = sideCenter + sideAxis * t;

            float3 r = capPoint - pAxis;
            r -= sideAxis * math.dot(r, sideAxis);

            float rLenSq = math.lengthsq(r);
            float3 rN = (rLenSq > 1e-12f) ? (r * math.rsqrt(rLenSq)) : AnyPerp(sideAxis);

            return pAxis + rN * sideRadius;
        }

        // Clips the chord endpoints by the finite height of the side cylinder.
        // The chord direction is along d (same as sideAxis projected into cap plane).
        // We simply clip the segment in parameter t along the side axis.
        private static bool ClipChordToFiniteSideHeight(
            float3 p0, float3 p1,
            float3 sideCenter, float3 sideAxis, float sideHalfHeight,
            out float3 out0, out float3 out1)
        {
            out0 = out1 = default;

            // Segment parameter s in [0,1]: P(s) = p0 + (p1-p0)*s
            float3 dP = p1 - p0;

            // Height along side axis is linear in s:
            // h(s) = dot(P(s) - sideCenter, sideAxis) = h0 + s*dh
            float h0 = math.dot(p0 - sideCenter, sideAxis);
            float dh = math.dot(dP, sideAxis);

            float minH = -sideHalfHeight;
            float maxH = +sideHalfHeight;

            // Compute s interval where h(s) in [minH, maxH]
            float sMin = 0f;
            float sMax = 1f;

            if (math.abs(dh) < 1e-12f)
            {
                // Chord parallel to the height slabs; accept only if both inside
                if (h0 < minH - 1e-6f || h0 > maxH + 1e-6f)
                    return false;
            }
            else
            {
                float invDh = 1f / dh;
                float sA = (minH - h0) * invDh;
                float sB = (maxH - h0) * invDh;
                if (sA > sB) { float tmp = sA; sA = sB; sB = tmp; }

                sMin = math.max(sMin, sA);
                sMax = math.min(sMax, sB);

                if (sMax < sMin)
                    return false;
            }

            out0 = p0 + dP * sMin;
            out1 = p0 + dP * sMax;
            return math.lengthsq(out1 - out0) > 1e-20f;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ContactPoint MakeCapSideCP(float3 p, float3 nAB, float depth)
        {
            ContactPoint cp;
            cp.point = p;
            cp.normal = nAB;
            cp.depth = depth;
            return cp;
        }

        private static bool ClipSegmentToCapDisk_NoProject(
            float3 aW, float3 bW,
            float3 capCenterW, float3 capAxisW,
            float capRadius,
            out float3 out0W, out float3 out1W)
        {
            out0W = out1W = default;

            // Build plane basis
            float3 u = AnyPerp(capAxisW);
            float3 v = math.normalize(math.cross(capAxisW, u));

            float2 a2 = new float2(math.dot(aW - capCenterW, u), math.dot(aW - capCenterW, v));
            float2 b2 = new float2(math.dot(bW - capCenterW, u), math.dot(bW - capCenterW, v));

            float2 d2 = b2 - a2;
            float A = math.dot(d2, d2);
            if (A < 1e-20f)
                return false;

            float B = 2f * math.dot(a2, d2);
            float C = math.dot(a2, a2) - capRadius * capRadius;

            float disc = B * B - 4f * A * C;

            // Fully inside check
            float r2 = capRadius * capRadius;
            bool aIn = math.dot(a2, a2) <= r2;
            bool bIn = math.dot(b2, b2) <= r2;

            if (disc < 0f)
            {
                if (aIn && bIn)
                {
                    out0W = aW;
                    out1W = bW;
                    return true;
                }
                return false;
            }

            float s = math.sqrt(math.max(0f, disc));
            float inv2A = 0.5f / A;

            float t0 = (-B - s) * inv2A;
            float t1 = (-B + s) * inv2A;
            if (t0 > t1) { float tmp = t0; t0 = t1; t1 = tmp; }

            float lo = math.max(0f, t0);
            float hi = math.min(1f, t1);
            if (hi < lo)
                return false;

            float2 p0 = a2 + d2 * lo;
            float2 p1 = a2 + d2 * hi;

            out0W = capCenterW + u * p0.x + v * p0.y;
            out1W = capCenterW + u * p1.x + v * p1.y;
            return true;
        }


        // Returns deepest penetration witnesses between:
        //  - a CAP DISK (capCenter, plane normal capAxis, radius cap.radius)
        //  - a CYLINDER SIDE (axis segment, radius side.radius, finite height)
        //
        // Outputs:
        //  wCap    : deepest point on cap disk (in its plane) toward side axis segment
        //  wSide   : corresponding point on side surface (finite, clamped)
        //  nSideOut: radial normal of side surface at contact, pointing SIDE -> CAP (unit)
        //  penDepth: positive penetration depth (side.radius - radialDistance)
        //
        // Rejects if not penetrating beyond slop.

        private static bool DeepestCapDiskVsSide(
    in Cylinder cap, float3 capCenter, float3 capAxis,
    in Cylinder side, float3 sideAxis,
    out float3 wCap, out float3 wSide,
    out float3 nSideOut, out float penDepth)
        {
            wCap = wSide = nSideOut = default;
            penDepth = 0f;

            // Cap plane basis
            float3 u = AnyPerp(capAxis);
            float3 v = math.normalize(math.cross(capAxis, u));

            // Side axis segment endpoints
            float3 seg0 = side.center + sideAxis * (-side.halfHeight);
            float3 seg1 = side.center + sideAxis * (+side.halfHeight);

            // --- coarse search ---
            float bestT = 0f;
            float bestD2 = float.MaxValue;
            float3 bestSegP = default;
            float3 bestDiskP = default;

            const int SAMPLES = 9;
            for (int i = 0; i < SAMPLES; i++)
            {
                float t01 = (SAMPLES == 1) ? 0f : (i / (float)(SAMPLES - 1));
                float3 segP, diskP;
                float d2 = Eval_SegmentPoint_ToCapDisk_DistSq(
                    t01, seg0, seg1,
                    capCenter, capAxis, cap.radius,
                    u, v,
                    out segP, out diskP);

                if (d2 < bestD2)
                {
                    bestD2 = d2;
                    bestT = t01;
                    bestSegP = segP;
                    bestDiskP = diskP;
                }
            }

            // --- local refinement with ternary search on [lo, hi] ---
            float step = 1f / (SAMPLES - 1);
            float lo = math.max(0f, bestT - step);
            float hi = math.min(1f, bestT + step);

            // If step is tiny (degenerate), keep as-is.
            if (hi - lo > 1e-6f)
            {
                for (int it = 0; it < 10; it++)
                {
                    float tA = math.lerp(lo, hi, 1f / 3f);
                    float tB = math.lerp(lo, hi, 2f / 3f);

                    float3 segPA, diskPA;
                    float gA = Eval_SegmentPoint_ToCapDisk_DistSq(
                        tA, seg0, seg1,
                        capCenter, capAxis, cap.radius,
                        u, v,
                        out segPA, out diskPA);

                    float3 segPB, diskPB;
                    float gB = Eval_SegmentPoint_ToCapDisk_DistSq(
                        tB, seg0, seg1,
                        capCenter, capAxis, cap.radius,
                        u, v,
                        out segPB, out diskPB);

                    if (gA < gB) hi = tB;
                    else lo = tA;
                }

                // Final evaluate at refined midpoint
                float tMid = 0.5f * (lo + hi);
                float3 segPM, diskPM;
                float d2M = Eval_SegmentPoint_ToCapDisk_DistSq(
                    tMid, seg0, seg1,
                    capCenter, capAxis, cap.radius,
                    u, v,
                    out segPM, out diskPM);

                if (d2M < bestD2)
                {
                    bestD2 = d2M;
                    bestSegP = segPM;
                    bestDiskP = diskPM;
                }
            }

            // Compute radial from side axis (clamped segment) to cap point
            float3 r = bestDiskP - bestSegP;
            r -= sideAxis * math.dot(r, sideAxis);

            float rLenSq = math.lengthsq(r);
            float rLen = (rLenSq > 1e-20f) ? math.sqrt(rLenSq) : 0f;

            penDepth = side.radius - rLen;
            if (penDepth <= kSlop)
                return false;

            //float3 rN;
            //if (rLen > 1e-10f) rN = r / rLen;
            //else rN = AnyPerp(sideAxis);

            // Witnesses
            wCap = bestDiskP;

            // Compute a stable radial dir (SIDE axis -> CAP point), even in near-zero cases.
            // Use capCenter (or wCap) as the reference for fallback so direction points "toward the cap".
            float3 rN = StableRadialDirFromAxisToPoint(bestSegP, wCap, sideAxis, capCenter);

            // Two possible side surface points (opposite sides of the side cylinder)
            float3 wSideA = bestSegP + rN * side.radius;
            float3 wSideB = bestSegP - rN * side.radius;

            // Choose the one that is INSIDE the cap cylinder (penetrating witness).
            float mA = CapContainmentMargin(in cap, capAxis, wSideA);
            float mB = CapContainmentMargin(in cap, capAxis, wSideB);

            // Prefer the one with bigger containment margin (more inside).
            // If both similar, pick whichever is closer to wCap.
            bool pickA;
            const float marginTie = 1e-6f;

            if (mA > mB + marginTie) pickA = true;
            else if (mB > mA + marginTie) pickA = false;
            else
            {
                float dA = math.lengthsq(wSideA - wCap);
                float dB = math.lengthsq(wSideB - wCap);
                pickA = dA <= dB;
            }

            wSide = pickA ? wSideA : wSideB;

            // Now define nSideOut consistently as SIDE -> CAP at the chosen side point.
            // That’s the radial normal on the side surface pointing toward the cap volume.
            float3 nOut = wCap - bestSegP;
            nOut -= sideAxis * math.dot(nOut, sideAxis);
            float nLenSq = math.lengthsq(nOut);
            if (nLenSq > 1e-20f) nOut *= math.rsqrt(nLenSq);
            else
            {
                // if degenerate, derive it from chosen side
                float3 tmp = (pickA ? +rN : -rN);
                nOut = tmp;
            }

            // Ensure nOut points from SIDE surface toward CAP (so that wSide is the penetrating witness)
            // i.e., wSide should be "behind" the cap along -nOut
            if (math.dot(nOut, (wCap - wSide)) < 0f)
                nOut = -nOut;

            nSideOut = nOut;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float CapContainmentMargin(in Cylinder cap, float3 capAxis, float3 p)
        {
            // Margin > 0 means inside (by that amount). Margin < 0 means outside.
            float t = math.dot(p - cap.center, capAxis);
            float tClamped = math.clamp(t, -cap.halfHeight, +cap.halfHeight);
            float slabOutside = math.abs(t) - cap.halfHeight; // >0 if outside slab

            float3 pAxis = cap.center + capAxis * tClamped;
            float3 r = p - pAxis;
            r -= capAxis * math.dot(r, capAxis);
            float radial = math.length(r);

            float radialMargin = cap.radius - radial; // >0 inside radius
            float slabMargin = -math.max(0f, slabOutside); // 0 if inside slab, negative if outside

            // If outside slab, slabMargin is negative; if inside slab, slabMargin is 0.
            // Combine by taking the minimum constraint (tightest).
            return math.min(radialMargin, slabMargin);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 StableRadialDirFromAxisToPoint(float3 axisPoint, float3 point, float3 axisDir, float3 fallbackRef)
        {
            // returns unit vector perpendicular to axisDir
            float3 r = point - axisPoint;
            r -= axisDir * math.dot(r, axisDir);
            float rLenSq = math.lengthsq(r);

            if (rLenSq > 1e-20f)
                return r * math.rsqrt(rLenSq);

            // fallback: use fallbackRef projected perp to axisDir
            float3 f = fallbackRef - axisPoint;
            f -= axisDir * math.dot(f, axisDir);
            float fLenSq = math.lengthsq(f);

            if (fLenSq > 1e-20f)
                return f * math.rsqrt(fLenSq);

            // final fallback
            return AnyPerp(axisDir);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ClampPointToDiskPlane(float3 p, float3 c, float r, float3 u, float3 v)
        {
            float3 d = p - c;
            float2 d2 = new float2(math.dot(d, u), math.dot(d, v));
            float lenSq = math.dot(d2, d2);
            float r2 = r * r;

            if (lenSq <= r2)
                return p;

            float invLen = math.rsqrt(math.max(lenSq, 1e-20f));
            float2 dn = d2 * invLen;
            return c + u * (dn.x * r) + v * (dn.y * r);
        }

        #endregion

        #region Side-Side
        // ------------------------------------------------------------
        // SIDE-SIDE manifold
        // ------------------------------------------------------------
        // Rules:
        //  - Always tries to emit 2 points (ends of the shared side-band).
        //  - Points are clamped to BOTH finite cylinders (via axis segment clamping).
        //  - Uses MTV axis as the contact normal (A -> B).
        private static bool BuildSideSideManifold(
    in Cylinder a, in Cylinder b,
    float3 aAxisW, float3 bAxisW,
    float3 n, float depth,
    out CylinderCylinderContactPoints cc)
        {
            cc = default;
            InvalidateAll(ref cc);

            if (depth <= kSlop)
                return false;

            float3 cTo = b.center - a.center;

            // Ensure a stable radial normal for side witnesses:
            // remove any component along A axis (side surface normal must be perpendicular to axis).
            float3 nA = n - aAxisW * math.dot(n, aAxisW);
            float nALenSq = math.lengthsq(nA);

            if (nALenSq <= 1e-10f)
            {
                // Fallback: use center-to-center radial projected off A axis
                nA = cTo - aAxisW * math.dot(cTo, aAxisW);
                nALenSq = math.lengthsq(nA);

                if (nALenSq <= 1e-10f)
                {
                    // Final fallback: any perp to A axis
                    nA = AnyPerp(aAxisW);
                    nALenSq = math.lengthsq(nA);
                }
            }

            float3 nSide = nA * math.rsqrt(nALenSq);

            // If axes are NOT parallel-ish, there is no shared side band (unique closest pair) => 1 CP.
            bool axesParallel = math.abs(math.dot(aAxisW, bAxisW)) > kAxisParallel;
            if (!axesParallel)
            {
                ClosestPoints_SegmentSegment(
                    a.center, aAxisW, -a.halfHeight, a.halfHeight,
                    b.center, bAxisW, -b.halfHeight, b.halfHeight,
                    out float sA, out float tB);

                float3 pA_axis = a.center + aAxisW * sA;
                float3 pB_axis = b.center + bAxisW * tB;

                float3 sep = pB_axis - pA_axis;

                // Prefer a radial perpendicular to A axis
                float3 r = sep - aAxisW * math.dot(sep, aAxisW);
                float rLenSq = math.lengthsq(r);

                float3 nLocal;
                float dist;

                if (rLenSq < 1e-12f)
                {
                    nLocal = nSide; // stable fallback
                    dist = 0f;
                }
                else
                {
                    dist = math.sqrt(rLenSq);
                    nLocal = r * (1.0f / dist);
                }

                // Ensure A->B
                if (math.dot(nLocal, cTo) < 0f) nLocal = -nLocal;

                float3 wA = pA_axis + nLocal * a.radius;
                float3 wB = pB_axis - nLocal * b.radius;

                // Pen depth along this radial
                float penDepth = (a.radius + b.radius) - dist;

                // If numerical weirdness makes this tiny/negative, fall back to SAT depth for stability
                if (penDepth <= kSlop) penDepth = depth;

                // ✅ Deepest shared point INSIDE BOTH cylinders
                float3 deepPoint = wA - nLocal * (0.5f * penDepth);

                ContactPoint cp;
                cp.point = deepPoint;
                cp.normal = nLocal;   // radial
                cp.depth = penDepth;

                Write(ref cc, 0, cp);
                cc.numContactPoints = 1;

                cc.globalPenAxis = nLocal;
                cc.globalPenDepth = penDepth;

                DedupAndCompact(ref cc);
                return cc.numContactPoints > 0;
            }

            // --------------------------------------------------------
            // Find the shared "side band" along A's axis where B exists
            // --------------------------------------------------------
            float cAlongA = math.dot(b.center - a.center, aAxisW);
            float bAlongA = math.dot(bAxisW, aAxisW);
            float bExtentAlongA = b.halfHeight * math.abs(bAlongA);

            float aMin = -a.halfHeight;
            float aMax = a.halfHeight;

            float bMinOnA = cAlongA - bExtentAlongA;
            float bMaxOnA = cAlongA + bExtentAlongA;

            float bandMin = math.max(aMin, bMinOnA);
            float bandMax = math.min(aMax, bMaxOnA);

            // If no shared band, this isn't side-side per your rules (cap involved).
            if (bandMax < bandMin + 1e-6f)
                return false;

            // Pick 2 points at the ends of the overlap band (max 2 contacts).
            float s0 = bandMin;
            float s1 = bandMax;

            int count = 0;

            if (EmitSideSidePoint(in a, in b, aAxisW, bAxisW, nSide, n, depth, s0, out var cp0))
                Write(ref cc, count++, cp0);

            if (EmitSideSidePoint(in a, in b, aAxisW, bAxisW, nSide, n, depth, s1, out var cp1))
                Write(ref cc, count++, cp1);

            if (count == 0)
                return false;

            cc.numContactPoints = count;
            cc.globalPenAxis = n;      // keep MTV axis as global penetration axis
            cc.globalPenDepth = depth;

            DedupAndCompact(ref cc);
            return cc.numContactPoints > 0;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool EmitSideSidePoint(
            in Cylinder a, in Cylinder b,
            float3 aAxisW, float3 bAxisW,
            float3 nSideFallback, // stable fallback direction (perp to A axis)
            float3 mtvAxis,       // SAT MTV axis (A -> B), used only as fallback orientation
            float satDepth,
            float sA,             // parameter along A axis in [-a.hh, +a.hh]
            out ContactPoint cp)
        {
            cp = default;

            // Point on A axis segment
            float3 pA_axis = a.center + aAxisW * sA;

            // Closest point on B axis segment to that point
            float tB = math.dot(pA_axis - b.center, bAxisW);
            tB = math.clamp(tB, -b.halfHeight, b.halfHeight);
            float3 pB_axis = b.center + bAxisW * tB;

            float3 sep = pB_axis - pA_axis;

            // Radial direction (perp to A axis)
            float3 r = sep - aAxisW * math.dot(sep, aAxisW);
            float rLenSq = math.lengthsq(r);

            float3 nLocal;
            float dist;

            if (rLenSq < 1e-12f)
            {
                // fallback: use provided stable direction
                nLocal = nSideFallback;
                dist = 0f;
            }
            else
            {
                dist = math.sqrt(rLenSq);
                nLocal = r * (1.0f / dist);
            }

            // Ensure A -> B orientation
            float3 cTo = b.center - a.center;
            if (math.dot(nLocal, cTo) < 0f) nLocal = -nLocal;

            // Side surface witnesses
            float3 wA = pA_axis + nLocal * a.radius;
            float3 wB = pB_axis - nLocal * b.radius;

            // Pen depth along this radial
            float penDepth = (a.radius + b.radius) - dist;

            // If numerical weirdness makes this tiny/negative, fall back to SAT depth for stability
            if (penDepth <= kSlop) penDepth = satDepth;

            // ✅ Deepest shared point INSIDE BOTH cylinders
            float3 deepPoint = wA - nLocal * (0.5f * penDepth);

            cp.point = deepPoint;
            cp.normal = nLocal;   // radial
            cp.depth = penDepth;
            return true;
        }


        #endregion

        private static bool SAT_CylinderCylinder(
            in Cylinder a, in Cylinder b,
            float3 aAxis, float3 bAxis,
            out float3 mtvAxis, out float mtvDepth)
        {
            mtvAxis = float3.zero;
            mtvDepth = float.MaxValue;

            float3 cTo = b.center - a.center;

            // Candidate axes
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

            if (!TestAxis(ax0, in a, in b, aAxis, bAxis, ref mtvAxis, ref mtvDepth)) return false;
            if (!TestAxis(ax1, in a, in b, aAxis, bAxis, ref mtvAxis, ref mtvDepth)) return false;

            if (ax2Valid)
            {
                if (!TestAxis(ax2, in a, in b, aAxis, bAxis, ref mtvAxis, ref mtvDepth))
                    return false;
            }
            else
            {
                if (radialValid)
                {
                    if (!TestAxis(radial, in a, in b, aAxis, bAxis, ref mtvAxis, ref mtvDepth))
                        return false;
                }
            }

            // Orient A -> B
            if (math.dot(mtvAxis, cTo) < 0f)
                mtvAxis = -mtvAxis;

            return true;
        }

        private static bool TestAxis(
            float3 n,
            in Cylinder a, in Cylinder b,
            float3 aAxis, float3 bAxis,
            ref float3 mtvAxis,
            ref float mtvDepth)
        {
            ProjectCylinderInterval(in a, aAxis, n, out float aMin, out float aMax);
            ProjectCylinderInterval(in b, bAxis, n, out float bMin, out float bMax);

            float overlap = math.min(aMax, bMax) - math.max(aMin, bMin);

            if (overlap <= kSlop)
                return false;

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

        // ------------------------------------------------------------
        // CAP-CAP manifold (FACE-FACE)
        // ------------------------------------------------------------
        // Inputs:
        //  - n : unit normal pointing A -> B (axial-ish)
        //  - depth : penetration depth along n (from your narrowphase)
        // Behavior:
        //  - Picks the facing cap of A and facing cap of B
        //  - Builds intersection of their disks in the cap plane
        //  - Emits up to 4 points ON RIMS, equally distributed (0/90/180/270)
        //  - Points lie on the mid-plane between the two cap planes (stable shared plane)
        public static bool BuildCapCapManifold(
            in Cylinder a, in Cylinder b,
            float3 aAxisW, float3 bAxisW,
            float3 n, float depth,
            out CylinderCylinderContactPoints cc)
        {
            cc = default;
            InvalidateAll(ref cc);

            if (depth <= kSlop)
                return false;

            // Choose facing caps (toward each other along n)
            float aCapSign = (math.dot(aAxisW, n) >= 0f) ? +1f : -1f; // +1 => top cap, -1 => bottom cap
            float bCapSign = (math.dot(bAxisW, n) >= 0f) ? -1f : +1f; // B faces opposite direction

            float3 aCapCenter = a.center + aAxisW * (aCapSign * a.halfHeight);
            float3 bCapCenter = b.center + bAxisW * (bCapSign * b.halfHeight);

            // Build a stable plane basis (u,v) orthonormal to n
            float3 u = AnyPerp(n);
            float3 v = math.normalize(math.cross(n, u));

            // Project bCapCenter into A's cap plane along n so both disks live in same plane coords
            float3 bCenterProj = bCapCenter - n * math.dot(bCapCenter - aCapCenter, n);

            // Compute circle-circle intersection in that plane (2D in u,v coordinates)
            // We work in 2D:
            //   A at (0,0), radius rA
            //   B at (d,0) after aligning x axis to (bCenterProj-aCapCenter)
            float2 b2 = ToPlane2D(bCenterProj - aCapCenter, u, v);
            float d = math.length(b2);

            float rA = a.radius;
            float rB = b.radius;

            // No overlap => no manifold (shouldn't happen if narrowphase said cap-cap)
            if (d > rA + rB + 1e-6f)
                return false;

            // If coincident centers (d ~ 0): intersection is full disk of min radius.
            // Your rule: points on rims (use min radius rim).
            if (d < 1e-6f)
            {
                float r = math.min(rA, rB);

                // Use 4 rim points in u/v directions
                EmitCapCapPoint(in a, in b, aCapCenter, bCapCenter, n, depth, u * r, out cc.p1);
                EmitCapCapPoint(in a, in b, aCapCenter, bCapCenter, n, depth, v * r, out cc.p2);
                EmitCapCapPoint(in a, in b, aCapCenter, bCapCenter, n, depth, -u * r, out cc.p3);
                EmitCapCapPoint(in a, in b, aCapCenter, bCapCenter, n, depth, -v * r, out cc.p4);

                cc.numContactPoints = 4;
                cc.globalPenAxis = n;
                cc.globalPenDepth = depth;

                DedupAndCompact(ref cc);
                return cc.numContactPoints > 0;
            }

            // Align x-axis with B center direction in plane
            float2 ex2 = b2 / d;              // 2D unit
            float2 ey2 = new float2(-ex2.y, ex2.x);

            // Circle-circle intersection:
            // x = (d^2 + rA^2 - rB^2) / (2d)
            // y = sqrt(rA^2 - x^2)
            float x = (d * d + rA * rA - rB * rB) / (2f * d);
            float y2 = rA * rA - x * x;
            float y = (y2 > 0f) ? math.sqrt(y2) : 0f;

            // Intersection chord endpoints in 2D (in A-centered plane coords)
            float2 pI0_2 = ex2 * x + ey2 * y;
            float2 pI1_2 = ex2 * x - ey2 * y;

            // If y==0 -> tangent: only one intersection point.
            // But you want up to 4 points on rims within overlap: for tangent, it's basically 1 point.
            // We'll still generate 2 opposite points but clamp will collapse/dedup.
            // Convert endpoints back to 3D points on the plane
            float3 pI0 = aCapCenter + u * pI0_2.x + v * pI0_2.y;
            float3 pI1 = aCapCenter + u * pI1_2.x + v * pI1_2.y;

            // Now we need FOUR points "equally distributed" on the overlap region's rim.
            // A robust way:
            //  - pick a center inside the lens (midpoint of the two circle centers, clamped)
            //  - choose 4 directions (u,v,-u,-v)
            //  - cast each ray from lens center to the boundary of the intersection (clamp to both disks),
            //    then force to rim(s) by pushing to circle boundary and re-clamping.
            float3 lensCenter = ClosestPointInDiskIntersection_OnPlane(aCapCenter, rA, bCenterProj, rB, u, v);

            // Directions in plane
            float3 d0 = u;
            float3 d1 = v;
            float3 d2 = -u;
            float3 d3 = -v;

            // For "rim points", we try to land on boundary of intersection:
            // Start by pushing far enough then clamp to intersection.
            float rPush = math.min(rA, rB) * 2f;

            float3 s0 = ClampPointToDiskIntersection_OnPlane(lensCenter + d0 * rPush, aCapCenter, rA, bCenterProj, rB, u, v);
            float3 s1 = ClampPointToDiskIntersection_OnPlane(lensCenter + d1 * rPush, aCapCenter, rA, bCenterProj, rB, u, v);
            float3 s2 = ClampPointToDiskIntersection_OnPlane(lensCenter + d2 * rPush, aCapCenter, rA, bCenterProj, rB, u, v);
            float3 s3 = ClampPointToDiskIntersection_OnPlane(lensCenter + d3 * rPush, aCapCenter, rA, bCenterProj, rB, u, v);

            // Force them to be on "a rim": push each to nearest circle boundary (A or B) while staying in intersection
            s0 = ForceToRimAndClamp(s0, aCapCenter, rA, bCenterProj, rB, u, v);
            s1 = ForceToRimAndClamp(s1, aCapCenter, rA, bCenterProj, rB, u, v);
            s2 = ForceToRimAndClamp(s2, aCapCenter, rA, bCenterProj, rB, u, v);
            s3 = ForceToRimAndClamp(s3, aCapCenter, rA, bCenterProj, rB, u, v);

            int count = 0;
            if (EmitCapCapSharedPoint(in a, in b, aCapCenter, bCapCenter, n, depth, s0, out var c0)) Write(ref cc, count++, c0);
            if (EmitCapCapSharedPoint(in a, in b, aCapCenter, bCapCenter, n, depth, s1, out var c1)) Write(ref cc, count++, c1);
            if (EmitCapCapSharedPoint(in a, in b, aCapCenter, bCapCenter, n, depth, s2, out var c2)) Write(ref cc, count++, c2);
            if (EmitCapCapSharedPoint(in a, in b, aCapCenter, bCapCenter, n, depth, s3, out var c3)) Write(ref cc, count++, c3);

            if (count == 0)
                return false;

            cc.numContactPoints = count;
            cc.globalPenAxis = n;

            // Use depth (or min per-contact depth if you compute it). For face-face, depth is uniform from narrowphase.
            cc.globalPenDepth = depth;

            DedupAndCompact(ref cc);
            return cc.numContactPoints > 0;
        }

        // Emits a stable shared point on the mid-plane between caps (not biased to either cap plane).
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool EmitCapCapSharedPoint(
            in Cylinder a, in Cylinder b,
            float3 aCapCenter, float3 bCapCenter,
            float3 n, float depth,
            float3 planePoint,
            out ContactPoint cp)
        {
            cp = default;

            // Clamp to A cap disk (world)
            float3 wA = ClosestPointOnCapDiskToPointWorld(in a, aCapCenter, n, planePoint);

            // Clamp to B cap disk (world) - note: its cap plane normal is also n (caps are parallel-ish in cap-cap case)
            float3 wB = ClosestPointOnCapDiskToPointWorld(in b, bCapCenter, n, planePoint);

            // Shared point as midpoint (stable)
            cp.point = 0.5f * (wA + wB);
            cp.normal = n;
            cp.depth = depth;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void EmitCapCapPoint(
            in Cylinder a, in Cylinder b,
            float3 aCapCenter, float3 bCapCenter,
            float3 n, float depth,
            float3 offsetOnPlane,
            out ContactPoint cp)
        {
            float3 p = aCapCenter + offsetOnPlane;
            EmitCapCapSharedPoint(in a, in b, aCapCenter, bCapCenter, n, depth, p, out cp);
        }

        // Clamp a point to the disk of a cap (defined by capCenter and plane normal n)
        // This does NOT choose top/bottom; caller already selected capCenter.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ClosestPointOnCapDiskToPointWorld(in Cylinder cyl, float3 capCenterW, float3 n, float3 pWorld)
        {
            // Build a basis for the cap plane
            float3 u = AnyPerp(n);
            float3 v = math.normalize(math.cross(n, u));

            // Bring point into plane coords relative to cap center
            float3 d = pWorld - capCenterW;
            float2 d2 = new float2(math.dot(d, u), math.dot(d, v));

            float lenSq = math.dot(d2, d2);
            float r = cyl.radius;
            float r2 = r * r;

            float2 clamped;
            if (lenSq <= 1e-20f)
            {
                clamped = new float2(r, 0f); // arbitrary rim direction if exactly at center
            }
            else if (lenSq <= r2)
            {
                clamped = d2;
            }
            else
            {
                float invLen = math.rsqrt(lenSq);
                clamped = d2 * (r * invLen);
            }

            // Return on the cap plane (same plane as capCenterW)
            return capCenterW + u * clamped.x + v * clamped.y;
        }

        // Force a point to the boundary of either disk, then clamp back into intersection.
        // This yields "rim points" that stay inside the overlap lens.
        private static float3 ForceToRimAndClamp(
            float3 p,
            float3 cA, float rA,
            float3 cB, float rB,
            float3 u, float3 v)
        {
            // Determine which circle boundary is closer to being "outside" (push to that rim)
            float2 a2 = ToPlane2D(p - cA, u, v);
            float2 b2 = ToPlane2D(p - cB, u, v);

            float da = math.length(a2);
            float db = math.length(b2);

            float3 q = p;

            // Push to A rim
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

            // Clamp back into lens
            q = ClampPointToDiskIntersection_OnPlane(q, cA, rA, cB, rB, u, v);
            return q;
        }

        // ------------------------------------------------------------
        // Disk intersection helpers (in the plane spanned by u,v)
        // ------------------------------------------------------------
        private static float3 ClosestPointInDiskIntersection_OnPlane(
            float3 cA, float rA,
            float3 cB, float rB,
            float3 u, float3 v)
        {
            // Start at midpoint of centers and iteratively clamp to both disks
            float3 p = 0.5f * (cA + cB);
            for (int i = 0; i < 4; i++)
            {
                p = ClampToDisk_OnPlane(p, cA, rA, u, v);
                p = ClampToDisk_OnPlane(p, cB, rB, u, v);
            }
            return p;
        }

        private static float3 ClampPointToDiskIntersection_OnPlane(
            float3 p,
            float3 cA, float rA,
            float3 cB, float rB,
            float3 u, float3 v)
        {
            p = ClampToDisk_OnPlane(p, cA, rA, u, v);
            p = ClampToDisk_OnPlane(p, cB, rB, u, v);
            p = ClampToDisk_OnPlane(p, cA, rA, u, v);
            return p;
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

        // ------------------------------------------------------------
        // Utility
        // ------------------------------------------------------------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 GetAxis(in Cylinder c)
        {
            return math.normalize(math.rotate(c.rot, math.up()));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 AnyPerp(float3 n)
        {
            float3 a = (math.abs(n.y) < 0.999f) ? math.up() : math.right();
            float3 p = math.cross(n, a);
            float lenSq = math.lengthsq(p);
            if (lenSq <= kEpsLenSq) return new float3(1, 0, 0);
            return p * math.rsqrt(lenSq);
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
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ClosestPoints_SegmentSegment(
            float3 p0, float3 u, float sMin, float sMax,
            float3 q0, float3 v, float tMin, float tMax,
            out float s, out float t)
        {
            float3 w0 = p0 - q0;
            float b = math.dot(u, v);
            float d = math.dot(u, w0);
            float e = math.dot(v, w0);

            float denom = 1f - b * b;

            if (denom > 1e-8f)
            {
                s = (b * e - d) / denom;
                t = (e - b * d) / denom;
            }
            else
            {
                // nearly parallel fallback
                s = 0f;
                t = e;
            }

            s = math.clamp(s, sMin, sMax);

            // clamp t based on s
            t = math.dot((p0 + u * s) - q0, v);
            t = math.clamp(t, tMin, tMax);

            // re-clamp s based on t (one iteration)
            s = math.dot((q0 + v * t) - p0, u);
            s = math.clamp(s, sMin, sMax);
        }

    }
}
