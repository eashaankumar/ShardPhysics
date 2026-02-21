using System;
using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace Shard.Dev
{
    public static class CylinderCylinderSolver
    {
        public struct ContactPoint
        {
            public float3 point;   // stable shared point (midpoint of witnesses)
            public float3 normal;  // global MTV axis (A -> B)
            public float depth;    // per-contact depth (debug/optional)
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

        private const float kAxisParallel = 0.9995f;
        private const float kSlop = 1e-5f;
        private const float kEpsLenSq = 1e-12f;

        // For endpoint classification on axis segments (cap involvement)
        private const float kEndEps = 1e-4f;

        // ------------------------------------------------------------
        // Public solve
        // ------------------------------------------------------------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Solve(in Cylinder a, in Cylinder b, out CylinderCylinderContactPoints cc)
        {
            cc = default;

            float3 aAxis = GetAxis(in a); // unit
            float3 bAxis = GetAxis(in b); // unit

            float3 a0, a1; GetSegment(in a, aAxis, out a0, out a1);
            float3 b0, b1; GetSegment(in b, bAxis, out b0, out b1);

            float align = math.abs(math.dot(aAxis, bAxis));
            if (align >= kAxisParallel)
            {
                return SolveParallel(in a, in b, aAxis, out cc);
            }

            // General (skew) case: measure via closest points between axis segments.
            ClosestPointsSegmentSegment(a0, a1, b0, b1, out float s, out float t, out float3 pA, out float3 pB);

            float3 d = pB - pA;
            float distSq = math.lengthsq(d);
            float rSum = a.radius + b.radius;

            if (distSq > (rSum + kSlop) * (rSum + kSlop))
                return false;

            float dist = math.sqrt(math.max(0f, distSq));
            float3 n = (distSq <= kEpsLenSq) ? StableNormalFromAxes(aAxis, bAxis) : (d / dist);

            float penDepth = rSum - dist;
            if (penDepth <= kSlop)
                return false;

            // Rule:
            // - side-side => 2 CPs (clamped)
            // - 1 CP only if they don't share side OR any cap is involved
            bool aCapInvolved = (s <= kEndEps) || (s >= 1f - kEndEps);
            bool bCapInvolved = (t <= kEndEps) || (t >= 1f - kEndEps);
            bool shareSide = !aCapInvolved && !bCapInvolved;

            // Global MTV axis is A->B
            if (math.dot(n, b.center - a.center) < 0f) n = -n;

            cc.globalPenAxis = n;
            cc.globalPenDepth = penDepth;

            if (!shareSide)
            {
                // Single deepest / stable point
                EmitMidpointWitnessAtAxisPoints(in a, in b, pA, pB, n, penDepth, ref cc.p1);
                cc.numContactPoints = 1;
                return true;
            }

            // Side-side: 2 points, separated along A's axis segment around s
            // (Pick a symmetric offset in parameter space, clamp, then map to B by closest point)
            float ds = 0.25f; // "equally distributed" along the shared side band (coarse but stable)
            float sL = math.clamp(s - ds, 0f, 1f);
            float sR = math.clamp(s + ds, 0f, 1f);

            float3 pA_L = math.lerp(a0, a1, sL);
            float3 pA_R = math.lerp(a0, a1, sR);

            float3 pB_L = ClosestPointOnSegment(b0, b1, pA_L);
            float3 pB_R = ClosestPointOnSegment(b0, b1, pA_R);

            // Contact normals can vary slightly per point in skew configurations.
            // But keep them all aligned to global MTV axis for your solver’s “hard separation”.
            EmitMidpointWitnessAtAxisPoints(in a, in b, pA_L, pB_L, n, penDepth, ref cc.p1);
            EmitMidpointWitnessAtAxisPoints(in a, in b, pA_R, pB_R, n, penDepth, ref cc.p2);

            // De-dupe
            cc.numContactPoints = (math.lengthsq(cc.p2.point - cc.p1.point) > 1e-8f) ? 2 : 1;
            return true;
        }

        // ------------------------------------------------------------
        // Parallel case: decide axial vs radial MTV, then build manifold
        // ------------------------------------------------------------
        private static bool SolveParallel(in Cylinder a, in Cylinder b, float3 axis, out CylinderCylinderContactPoints cc)
        {
            cc = default;

            // Make axis roughly A->B for consistency
            if (math.dot(axis, b.center - a.center) < 0f)
                axis = -axis;

            float3 dC = b.center - a.center;
            float s = math.dot(dC, axis);

            // radial separation between axes
            float3 radial = dC - axis * s;
            float radialDistSq = math.lengthsq(radial);

            float rSum = a.radius + b.radius;
            if (radialDistSq > (rSum + kSlop) * (rSum + kSlop))
                return false;

            float radialDist = math.sqrt(math.max(0f, radialDistSq));
            float radialOverlap = rSum - radialDist;

            float axialOverlap = (a.halfHeight + b.halfHeight) - math.abs(s);
            if (axialOverlap <= kSlop)
                return false;

            // Choose MTV axis (smaller overlap)
            if (axialOverlap < radialOverlap)
            {
                // MTV along axis (cap/cap or cap/side-ish)
                float3 mtvAxis = (s >= 0f) ? axis : -axis;
                float mtvDepth = axialOverlap;

                cc.globalPenAxis = mtvAxis;
                cc.globalPenDepth = mtvDepth;

                // Axial intervals along mtvAxis
                float aC = math.dot(a.center, mtvAxis);
                float bC = math.dot(b.center, mtvAxis);
                float aMin = aC - a.halfHeight;
                float aMax = aC + a.halfHeight;
                float bMin = bC - b.halfHeight;
                float bMax = bC + b.halfHeight;

                float overlapMin = math.max(aMin, bMin);
                float overlapMax = math.min(aMax, bMax);
                if (overlapMax - overlapMin <= kSlop)
                    return false;

                // Determine containment along axis (both caps of one intersect)
                bool aInsideB = (aMin >= bMin - 1e-6f) && (aMax <= bMax + 1e-6f);
                bool bInsideA = (bMin >= aMin - 1e-6f) && (bMax <= aMax + 1e-6f);

                // Orthonormal basis for the cap plane
                float3 u = AnyPerp(mtvAxis);
                float3 v = math.normalize(math.cross(mtvAxis, u));

                if (aInsideB)
                {
                    // Both A caps intersect B somewhere => return 2 points (one per A cap), clamped to BOTH cylinders.
                    float3 capTop = a.center + mtvAxis * a.halfHeight;
                    float3 capBot = a.center - mtvAxis * a.halfHeight;

                    EmitClampedPointOnBothCylinders(in a, in b, capTop, mtvAxis, u, v, a.radius, ref cc.p1, mtvAxis, mtvDepth);
                    EmitClampedPointOnBothCylinders(in a, in b, capBot, mtvAxis, u, v, a.radius, ref cc.p2, mtvAxis, mtvDepth);

                    cc.numContactPoints = (math.lengthsq(cc.p2.point - cc.p1.point) > 1e-8f) ? 2 : 1;
                    return true;
                }

                if (bInsideA)
                {
                    // Both B caps intersect A somewhere => return 2 points (one per B cap), clamped to BOTH cylinders.
                    float3 capTop = b.center + mtvAxis * b.halfHeight;
                    float3 capBot = b.center - mtvAxis * b.halfHeight;

                    EmitClampedPointOnBothCylinders(in a, in b, capTop, mtvAxis, u, v, b.radius, ref cc.p1, mtvAxis, mtvDepth);
                    EmitClampedPointOnBothCylinders(in a, in b, capBot, mtvAxis, u, v, b.radius, ref cc.p2, mtvAxis, mtvDepth);

                    cc.numContactPoints = (math.lengthsq(cc.p2.point - cc.p1.point) > 1e-8f) ? 2 : 1;
                    return true;
                }

                // Otherwise: this is the classic face-face cap overlap (cap of A vs cap of B).
                // Rule: stick to 4 points, equally distributed, clamped to BOTH cylinder intersection surfaces.

                float3 aCapCenter = a.center + mtvAxis * a.halfHeight;   // cap facing B
                float3 bCapCenter = b.center - mtvAxis * b.halfHeight;   // opposing cap facing A

                // Put both disk centers into the same plane (A cap plane)
                float3 bCapProj = bCapCenter - mtvAxis * math.dot(bCapCenter - aCapCenter, mtvAxis);

                // A good "intersection center" seed: closest point in intersection (alternating projections)
                float3 center = ClosestPointInDiskIntersection(aCapCenter, a.radius, bCapProj, b.radius, u, v);

                // Four equally distributed directions (0/90/180/270)
                // Choose a radius that lives comfortably inside the overlap.
                float rPick = math.max(0f, math.min(a.radius, b.radius) * 0.85f);

                float3 c0 = ClampPointToDiskIntersection(center + u * rPick, aCapCenter, a.radius, bCapProj, b.radius, u, v);
                float3 c1 = ClampPointToDiskIntersection(center + v * rPick, aCapCenter, a.radius, bCapProj, b.radius, u, v);
                float3 c2 = ClampPointToDiskIntersection(center - u * rPick, aCapCenter, a.radius, bCapProj, b.radius, u, v);
                float3 c3 = ClampPointToDiskIntersection(center - v * rPick, aCapCenter, a.radius, bCapProj, b.radius, u, v);

                EmitMidpointWitness(in a, in b, c0, mtvAxis, mtvDepth, ref cc.p1);
                EmitMidpointWitness(in a, in b, c1, mtvAxis, mtvDepth, ref cc.p2);
                EmitMidpointWitness(in a, in b, c2, mtvAxis, mtvDepth, ref cc.p3);
                EmitMidpointWitness(in a, in b, c3, mtvAxis, mtvDepth, ref cc.p4);

                // De-dupe if overlap is tiny
                cc.numContactPoints = 1;
                if (math.lengthsq(cc.p2.point - cc.p1.point) > 1e-8f) cc.numContactPoints = 2;
                if (cc.numContactPoints == 2 && math.lengthsq(cc.p3.point - cc.p1.point) > 1e-8f && math.lengthsq(cc.p3.point - cc.p2.point) > 1e-8f) cc.numContactPoints = 3;
                if (cc.numContactPoints == 3 && math.lengthsq(cc.p4.point - cc.p1.point) > 1e-8f && math.lengthsq(cc.p4.point - cc.p2.point) > 1e-8f && math.lengthsq(cc.p4.point - cc.p3.point) > 1e-8f) cc.numContactPoints = 4;

                return true;
            }
            else
            {
                // MTV radially => side-side, generate 2 CPs clamped (unless a cap intersects => 1)
                float3 mtvAxis;
                if (radialDistSq <= kEpsLenSq) mtvAxis = AnyPerp(axis);
                else mtvAxis = radial / radialDist; // A->B
                float mtvDepth = radialOverlap;

                cc.globalPenAxis = mtvAxis;
                cc.globalPenDepth = mtvDepth;

                // Determine axial overlap interval (shared side band)
                float aC = math.dot(a.center, axis);
                float bC = math.dot(b.center, axis);
                float aMin = aC - a.halfHeight;
                float aMax = aC + a.halfHeight;
                float bMin = bC - b.halfHeight;
                float bMax = bC + b.halfHeight;

                float oMin = math.max(aMin, bMin);
                float oMax = math.min(aMax, bMax);

                // If they don't share a side band (i.e., overlap degenerates), treat as cap involvement => 1 CP
                if (oMax - oMin <= 1e-4f)
                {
                    float3 pA = a.center + axis * math.clamp(bC - aC, -a.halfHeight, a.halfHeight);
                    float3 pB = b.center + axis * math.clamp(aC - bC, -b.halfHeight, b.halfHeight);

                    EmitMidpointWitnessAtAxisPoints(in a, in b, pA, pB, mtvAxis, mtvDepth, ref cc.p1);
                    cc.numContactPoints = 1;
                    return true;
                }

                // Two points, equally distributed along the overlapped axial interval
                float t1 = math.lerp(oMin, oMax, 0.25f);
                float t2 = math.lerp(oMin, oMax, 0.75f);

                // Convert to world axis points (same axial coordinate for both, each clamped by its own caps)
                float3 pA1 = a.center + axis * math.clamp(t1 - aC, -a.halfHeight, a.halfHeight);
                float3 pB1 = b.center + axis * math.clamp(t1 - bC, -b.halfHeight, b.halfHeight);

                float3 pA2 = a.center + axis * math.clamp(t2 - aC, -a.halfHeight, a.halfHeight);
                float3 pB2 = b.center + axis * math.clamp(t2 - bC, -b.halfHeight, b.halfHeight);

                EmitMidpointWitnessAtAxisPoints(in a, in b, pA1, pB1, mtvAxis, mtvDepth, ref cc.p1);
                EmitMidpointWitnessAtAxisPoints(in a, in b, pA2, pB2, mtvAxis, mtvDepth, ref cc.p2);

                cc.numContactPoints = (math.lengthsq(cc.p2.point - cc.p1.point) > 1e-8f) ? 2 : 1;
                return true;
            }
        }

        // ------------------------------------------------------------
        // Emission helpers (clamped to BOTH cylinder surfaces)
        // ------------------------------------------------------------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void EmitMidpointWitness(in Cylinder a, in Cylinder b, float3 seedPoint, float3 normal, float depth, ref ContactPoint outCp)
        {
            float3 wa = ClosestPointOnCylinderToPointWorld(in a, seedPoint);
            float3 wb = ClosestPointOnCylinderToPointWorld(in b, seedPoint);
            float3 c = 0.5f * (wa + wb);

            outCp.point = c;
            outCp.normal = normal;
            outCp.depth = depth;
        }

        // Use axis points (on the center-lines) to build witnesses properly:
        // witnessA = axisPointA + normal * rA
        // witnessB = axisPointB - normal * rB
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void EmitMidpointWitnessAtAxisPoints(in Cylinder a, in Cylinder b, float3 axisPointA, float3 axisPointB, float3 normal, float depth, ref ContactPoint outCp)
        {
            float3 wa = axisPointA + normal * a.radius;
            float3 wb = axisPointB - normal * b.radius;
            float3 c = 0.5f * (wa + wb);

            outCp.point = c;
            outCp.normal = normal;
            outCp.depth = depth;
        }

        // For “cap point” seeds: keep it in a given cap plane disk, then clamp to BOTH cylinder volumes.
        private static void EmitClampedPointOnBothCylinders(
            in Cylinder a, in Cylinder b,
            float3 capSeed,
            float3 capPlaneN, float3 u, float3 v, float capRadius,
            ref ContactPoint outCp,
            float3 normal, float depth)
        {
            // Ensure seed stays on the cap disk of the owner (capSeed is itself the disk center in this usage)
            // We’ll just clamp it to the disk, then use midpoint of cylinder clamps.
            float3 p = ClampToDisk(capSeed, capSeed, capRadius, u, v);
            EmitMidpointWitness(in a, in b, p, normal, depth, ref outCp);
        }

        // ------------------------------------------------------------
        // Disk intersection helpers (same plane)
        // ------------------------------------------------------------
        private static float3 ClosestPointInDiskIntersection(
            float3 cA, float rA,
            float3 cB, float rB,
            float3 u, float3 v)
        {
            float3 p = 0.5f * (cA + cB);
            for (int i = 0; i < 3; i++)
            {
                p = ClampToDisk(p, cA, rA, u, v);
                p = ClampToDisk(p, cB, rB, u, v);
            }
            return p;
        }

        private static float3 ClampPointToDiskIntersection(
            float3 p,
            float3 cA, float rA,
            float3 cB, float rB,
            float3 u, float3 v)
        {
            p = ClampToDisk(p, cA, rA, u, v);
            p = ClampToDisk(p, cB, rB, u, v);
            p = ClampToDisk(p, cA, rA, u, v);
            return p;
        }

        private static float3 ClampToDisk(float3 p, float3 c, float r, float3 u, float3 v)
        {
            float2 d2 = new float2(math.dot(p - c, u), math.dot(p - c, v));
            float lenSq = math.dot(d2, d2);
            float r2 = r * r;
            if (lenSq <= r2) return p;

            float invLen = math.rsqrt(math.max(lenSq, 1e-20f));
            float2 dN = d2 * invLen;
            return c + u * (dN.x * r) + v * (dN.y * r);
        }

        // ------------------------------------------------------------
        // Geometry helpers
        // ------------------------------------------------------------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 GetAxis(in Cylinder c)
        {
            return math.normalize(math.rotate(c.rot, math.up())); // local +Y
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void GetSegment(in Cylinder c, float3 axis, out float3 A, out float3 B)
        {
            float3 h = axis * c.halfHeight;
            A = c.center - h;
            B = c.center + h;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 StableNormalFromAxes(float3 aAxis, float3 bAxis)
        {
            float3 cross = math.cross(aAxis, bAxis);
            if (math.lengthsq(cross) > kEpsLenSq)
                return math.normalize(math.cross(cross, aAxis));
            return AnyPerp(aAxis);
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ClosestPointOnSegment(float3 a, float3 b, float3 p)
        {
            float3 ab = b - a;
            float t = math.dot(p - a, ab) / math.max(math.dot(ab, ab), 1e-20f);
            t = math.clamp(t, 0f, 1f);
            return a + ab * t;
        }

        // Closest points between segments (also returns params)
        private static void ClosestPointsSegmentSegment(
            float3 p1, float3 q1,
            float3 p2, float3 q2,
            out float s, out float t,
            out float3 c1, out float3 c2)
        {
            float3 d1 = q1 - p1;
            float3 d2 = q2 - p2;
            float3 r = p1 - p2;

            float a = math.dot(d1, d1);
            float e = math.dot(d2, d2);
            float f = math.dot(d2, r);

            if (a <= 1e-12f && e <= 1e-12f)
            {
                s = t = 0f;
                c1 = p1;
                c2 = p2;
                return;
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

                    if (math.abs(denom) > 1e-12f)
                        s = math.clamp((b * f - c * e) / denom, 0f, 1f);
                    else
                        s = 0f;

                    float tNom = b * s + f;
                    if (tNom < 0f)
                    {
                        t = 0f;
                        s = math.clamp(-c / a, 0f, 1f);
                    }
                    else if (tNom > e)
                    {
                        t = 1f;
                        s = math.clamp((b - c) / a, 0f, 1f);
                    }
                    else
                    {
                        t = tNom / e;
                    }
                }
            }

            c1 = p1 + d1 * s;
            c2 = p2 + d2 * t;
        }

        // Clamp-to-cylinder-volume point (world) – matches your cylinder-box helper behavior
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ClosestPointOnCylinderToPointWorld(in Cylinder cyl, float3 pWorld)
        {
            float3 pL = math.rotate(math.inverse(cyl.rot), pWorld - cyl.center);

            float y = math.clamp(pL.y, -cyl.halfHeight, cyl.halfHeight);

            float2 xz = new float2(pL.x, pL.z);
            float lenSq = math.lengthsq(xz);
            float r = cyl.radius;

            float2 xzClamped;
            if (lenSq <= 1e-12f)
            {
                xzClamped = new float2(r, 0f);
            }
            else
            {
                float len = math.sqrt(lenSq);
                float s = math.min(1f, r / len);
                xzClamped = xz * s;
            }

            float3 qL = new float3(xzClamped.x, y, xzClamped.y);
            return cyl.center + math.rotate(cyl.rot, qL);
        }
    }
}
