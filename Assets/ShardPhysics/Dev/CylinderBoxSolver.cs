using System;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using UnityEngine;

namespace Shard.Dev
{
    public struct CylinderBoxSolver
    {
        public struct ContactPoint
        {
            public float3 point;
            public float3 normal;
            public float depth; // debug / optional bias; NOT used for your hard separation
        }

        public struct CylinderBoxContactPoints
        {
            public ContactPoint p1;
            public ContactPoint p2;
            public ContactPoint p3;
            public ContactPoint p4;
            public int numContactPoints;

            public float3 globalPenAxis;   // MTV axis (box -> cylinder)
            public float globalPenDepth;   // MTV depth

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

        public struct Box
        {
            public readonly float3 center;
            public readonly float3 extents;
            public readonly float3 halfExtents;
            public readonly quaternion rot;

            public Box(float3 center, quaternion rot, float3 halfExtents)
            {
                this.center = center;
                this.rot = rot;
                this.halfExtents = halfExtents;
                this.extents = halfExtents * 2;
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

        public struct BoxAxis
        {
            public float3 right;
            public float3 up;
            public float3 forward;

            public float3 this[int index]
            {
                get
                {
                    if (index == 0) return right;
                    if (index == 1) return up;
                    if (index == 2) return forward;
                    return float3.zero;
                }
                set
                {
                    if (index == 0) right = value;
                    if (index == 1) up = value;
                    if (index == 2) forward = value;
                }
            }
        }

        public struct SeparatingAxes
        {
            // 0..2 : box face axes
            public float3 b0, b1, b2;
            // 3 : cylinder axis
            public float3 c;

            // 4..6 : cross axes bi x c
            public float3 b0xc, b1xc, b2xc;
            public bool b0xcValid, b1xcValid, b2xcValid;

            public static readonly int NUM_AXES = 7;

            public float3 this[int index]
            {
                get
                {
                    if (index == 0) return b0;
                    if (index == 1) return b1;
                    if (index == 2) return b2;
                    if (index == 3) return c;
                    if (index == 4) return b0xc;
                    if (index == 5) return b1xc;
                    if (index == 6) return b2xc;
                    return float3.zero;
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool IsCrossValid(int index)
            {
                if (index == 4) return b0xcValid;
                if (index == 5) return b1xcValid;
                if (index == 6) return b2xcValid;
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void NormalizeCross(float epsLenSq = 1e-8f)
            {
                Normalize(ref b0xc, out b0xcValid, epsLenSq);
                Normalize(ref b1xc, out b1xcValid, epsLenSq);
                Normalize(ref b2xc, out b2xcValid, epsLenSq);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void Normalize(ref float3 axis, out bool valid, float epsLenSq)
            {
                float lenSq = math.lengthsq(axis);
                if (lenSq <= epsLenSq)
                {
                    axis = float3.zero;
                    valid = false;
                    return;
                }
                axis *= math.rsqrt(lenSq);
                valid = true;
            }
        }

        public struct Computed
        {
            public float3 boxToCyl;
            public float minOverlap;
            public int minAxis;
            public float3 penAxis;
        }

        private const float SAT_OVERLAP_SLOP = 1e-5f;

        // Rim contact tuning
        private const float kContactSlop = 1e-4f;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Solve(in Box box, in Cylinder cyl, out CylinderBoxContactPoints cbcp)
        {
            cbcp = default;

            BoxAxis boxAx = GetBoxAxis(box);
            float3 cylAxis = GetCylinderAxis(cyl); // unit

            SeparatingAxes axes = default;
            axes.b0 = boxAx[0];
            axes.b1 = boxAx[1];
            axes.b2 = boxAx[2];
            axes.c = cylAxis;

            axes.b0xc = math.cross(boxAx[0], cylAxis);
            axes.b1xc = math.cross(boxAx[1], cylAxis);
            axes.b2xc = math.cross(boxAx[2], cylAxis);
            axes.NormalizeCross();

            Computed comp = default;
            comp.boxToCyl = cyl.center - box.center;

            if (!SATOverlapTest(in box, in cyl, in boxAx, cylAxis, in axes, comp.boxToCyl, out comp.minOverlap, out comp.minAxis))
                return false;

            // global axis points box -> cylinder
            comp.penAxis = axes[comp.minAxis];
            if (math.dot(comp.penAxis, comp.boxToCyl) < 0f)
                comp.penAxis = -comp.penAxis;

            cbcp.globalPenAxis = comp.penAxis;
            cbcp.globalPenDepth = comp.minOverlap;

            GenerateManifold(in box, in cyl, in boxAx, cylAxis, in axes, in comp, out cbcp);

            return cbcp.numContactPoints > 0;
        }

        // ------------------------------------------------------------
        // SAT
        // ------------------------------------------------------------

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool SATOverlapTest(
            in Box box, in Cylinder cyl,
            in BoxAxis boxAx, float3 cylAxis,
            in SeparatingAxes axes,
            float3 boxToCyl,
            out float minOverlap, out int minAxis)
        {
            minOverlap = float.MaxValue;
            minAxis = -1;

            // Cylinder segment endpoints
            GetCylinderSegment(in cyl, cylAxis, out float3 A, out float3 B);

            for (int i = 0; i < SeparatingAxes.NUM_AXES; i++)
            {
                if (i >= 4 && !axes.IsCrossValid(i))
                    continue;

                float3 n = axes[i];

                // Box interval radius about its center
                float rBox = ProjectBoxRadii(in box, n, in boxAx);

                // Cylinder inflated segment interval
                ProjectCylinderInterval(in cyl, cylAxis, n, out float minC, out float maxC);

                float cBox = math.dot(box.center, n);
                float minB = cBox - rBox;
                float maxB = cBox + rBox;

                float overlap = math.min(maxB, maxC) - math.max(minB, minC);
                if (overlap < SAT_OVERLAP_SLOP)
                    return false;

                if (minAxis < 0 || overlap < minOverlap)
                {
                    minOverlap = overlap;
                    minAxis = i;
                }
            }

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float ProjectBoxRadii(in Box box, float3 n, in BoxAxis ax)
        {
            return box.halfExtents.x * math.abs(math.dot(n, ax[0])) +
                   box.halfExtents.y * math.abs(math.dot(n, ax[1])) +
                   box.halfExtents.z * math.abs(math.dot(n, ax[2]));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ProjectCylinderInterval(in Cylinder cyl, float3 cylAxis, float3 n, out float min, out float max)
        {
            // n and cylAxis are expected unit.
            float c = math.dot(cyl.center, n);

            float a = math.abs(math.dot(cylAxis, n));                 // cosine between axis and n
            float radial = math.sqrt(math.max(0f, 1f - a * a));       // sine magnitude

            float extent = cyl.halfHeight * a + cyl.radius * radial;

            min = c - extent;
            max = c + extent;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ProjectCapsuleInterval(float3 A, float3 B, float r, float3 n, out float min, out float max)
        {
            float a = math.dot(A, n);
            float b = math.dot(B, n);
            if (a < b) { min = a - r; max = b + r; }
            else { min = b - r; max = a + r; }
        }

        // ------------------------------------------------------------
        // Manifold generation
        // ------------------------------------------------------------

        private static void GenerateManifold(
            in Box box, in Cylinder cyl,
            in BoxAxis boxAx, float3 cylAxis,
            in SeparatingAxes axes,
            in Computed comp,
            out CylinderBoxContactPoints cbcp)
        {
            cbcp = default;

            float3 globalN = comp.penAxis; // box -> cyl

            // If the winning axis is a box face axis (0..2), we can do a nice face manifold.
            if (comp.minAxis >= 0 && comp.minAxis <= 2)
            {
                int faceAxis = comp.minAxis;

                // Cap-on-face condition: cylinder axis ~ parallel to face normal (perpendicular to face plane)
                // globalN ~ +/- cylAxis
                float align = math.abs(math.dot(globalN, cylAxis));
                if (align > 0.9f)
                {
                    if (BuildCapOnFaceRimManifold(in box, in cyl, in boxAx, cylAxis, faceAxis, globalN, comp.minOverlap, out cbcp))
                        return;
                }

                // Otherwise use side/feature manifold (1-2 points) based on closest segment↔OBB
                BuildClosestFeatureManifold(in box, in cyl, in boxAx, cylAxis, globalN, comp.minOverlap, out cbcp);
                return;
            }

            // If winning axis is cylinder axis (3) or a cross axis (4..6), keep it robust:
            // generate a closest-feature contact (optionally 2 points).
            BuildClosestFeatureManifold(in box, in cyl, in boxAx, cylAxis, globalN, comp.minOverlap, out cbcp);
        }

        // --- Cap-on-face: rim points on the box face ---
        private static bool BuildCapOnFaceRimManifold(
            in Box box, in Cylinder cyl,
            in BoxAxis boxAx, float3 cylAxis,
            int boxFaceAxisIndex, // 0..2
            float3 globalN,       // box -> cyl
            float globalDepth,
            out CylinderBoxContactPoints cbcp)
        {
            cbcp = default;

            // Determine which box face (positive/negative) is the reference face along globalN
            float3 faceN = boxAx[boxFaceAxisIndex];
            bool positiveFace = math.dot(faceN, globalN) > 0f;
            if (!positiveFace) faceN = -faceN;

            float faceHalfN = GetHalf(box.halfExtents, boxFaceAxisIndex);
            float3 faceCenter = box.center + faceN * faceHalfN;

            // Face basis (u,v) in-plane
            int uI = (boxFaceAxisIndex + 1) % 3;
            int vI = (boxFaceAxisIndex + 2) % 3;
            float3 u = boxAx[uI];
            float3 v = boxAx[vI];
            float uHalf = GetHalf(box.halfExtents, uI);
            float vHalf = GetHalf(box.halfExtents, vI);

            // Choose the cylinder cap that faces toward the box (i.e., along -globalN)
            // If cylAxis points roughly along globalN, the "bottom" cap (center - cylAxis*hh) faces box.
            float s = math.dot(cylAxis, globalN);
            float3 capCenter = (s > 0f)
                ? (cyl.center - cylAxis * cyl.halfHeight)
                : (cyl.center + cylAxis * cyl.halfHeight);

            // Project cap center onto the face plane along face normal direction for stability
            float signedDist = math.dot(faceN, capCenter - faceCenter);
            float3 capProjOnPlane = capCenter - faceN * signedDist;

            // Candidate rim offsets (4-point)
            float r = cyl.radius;
            Span<float3> pts = stackalloc float3[4];
            pts[0] = capProjOnPlane + u * r;
            pts[1] = capProjOnPlane - u * r;
            pts[2] = capProjOnPlane + v * r;
            pts[3] = capProjOnPlane - v * r;

            // Clamp each point into the face rectangle (and keep on the plane)
            int outCount = 0;
            Span<float3> outPts = stackalloc float3[4];

            for (int i = 0; i < 4; i++)
            {
                float3 p = pts[i];

                // express in face basis
                float du = math.dot(p - faceCenter, u);
                float dv = math.dot(p - faceCenter, v);

                du = math.clamp(du, -uHalf, uHalf);
                dv = math.clamp(dv, -vHalf, vHalf);

                float3 pClamped = faceCenter + u * du + v * dv;

                // Ensure point is on/behind the plane slightly (contact slop)
                float planeSigned = math.dot(faceN, pClamped - faceCenter);
                if (planeSigned <= kContactSlop)
                {
                    // De-dupe very close points
                    if (!IsNearExisting(outPts, outCount, pClamped, 1e-6f))
                        outPts[outCount++] = pClamped;
                }
            }

            if (outCount == 0)
                return false;

            // Write contacts with GLOBAL normal. Depth = plane penetration (debug-ish).
            // Since you use global separation, you can just set depths to globalDepth for simplicity.
            WriteContact(ref cbcp.p1, outPts[0], globalN, globalDepth);
            if (outCount > 1) WriteContact(ref cbcp.p2, outPts[1], globalN, globalDepth);
            if (outCount > 2) WriteContact(ref cbcp.p3, outPts[2], globalN, globalDepth);
            if (outCount > 3) WriteContact(ref cbcp.p4, outPts[3], globalN, globalDepth);
            cbcp.numContactPoints = outCount;

            return true;
        }

        // --- Generic robust manifold from closest segment↔OBB features (1–2 points) ---
        // --- Generic robust manifold from closest segment↔OBB features (1–2 points) ---
        // --- Generic robust manifold from closest segment↔OBB features (1–2 points) ---
        private static void BuildClosestFeatureManifold(
            in Box box, in Cylinder cyl,
            in BoxAxis boxAx, float3 cylAxis,
            float3 globalN,
            float globalDepth,
            out CylinderBoxContactPoints cbcp)
        {
            cbcp = default;

            // Cylinder axis segment endpoints
            GetCylinderSegment(in cyl, cylAxis, out float3 A, out float3 B);

            // Closest axis point to the OBB (world)
            ClosestSegmentOBB(in box, in boxAx, A, B, out float3 segPtW, out float3 boxPtW, out float distSq);

            float3 n = globalN;

            // Prefer contact points ON the BOX for debug/readability (change to midpoint/cyl surface if you prefer)
            WriteContact(ref cbcp.p1, boxPtW, n, globalDepth);
            cbcp.numContactPoints = 1;

            // If this is a side-ish contact (not cap-on-face), try to generate 2 stable points along the cylinder axis "contact band".
            float align = math.abs(math.dot(cylAxis, n));
            if (align < 0.9f)
            {
                // t parameter of closest axis point relative to cylinder center (in [-halfHeight, +halfHeight])
                float t0 = math.dot(segPtW - cyl.center, cylAxis);
                t0 = math.clamp(t0, -cyl.halfHeight, cyl.halfHeight);

                // Find interval [tL, tR] where distance(axisPoint(t), OBB) <= radius (+slop)
                const float bandSlop = 1e-4f;
                float r = cyl.radius + bandSlop;

                if (FindAxisContactBand(in box, in boxAx, in cyl, cylAxis, t0, r, out float tL, out float tR))
                {
                    // If the band has meaningful length, use its ends as two contacts
                    if (math.abs(tR - tL) > 1e-3f)
                    {
                        float3 P1 = cyl.center + cylAxis * tL;
                        float3 Q1 = ClosestPointOnOBB(in box, in boxAx, P1);

                        float3 P2 = cyl.center + cylAxis * tR;
                        float3 Q2 = ClosestPointOnOBB(in box, in boxAx, P2);

                        // De-dupe vs first point and each other
                        if (math.lengthsq(Q1 - cbcp.p1.point) > 1e-6f)
                        {
                            WriteContact(ref cbcp.p2, Q1, n, globalDepth);
                            cbcp.numContactPoints = 2;
                        }

                        // If Q1 got added as p2, try to use Q2 as a third? You only have 4 slots,
                        // but if you want strictly "2 points", then choose the two most separated.
                        // Here we enforce exactly 2 points: p1 is closest, p2 becomes the farther end.
                        if (cbcp.numContactPoints == 1)
                        {
                            // p1 was boxPtW. Choose the farther of Q1/Q2 from p1.
                            float d1 = math.lengthsq(Q1 - cbcp.p1.point);
                            float d2 = math.lengthsq(Q2 - cbcp.p1.point);
                            float3 pick = (d2 > d1) ? Q2 : Q1;

                            if (math.lengthsq(pick - cbcp.p1.point) > 1e-6f)
                            {
                                WriteContact(ref cbcp.p2, pick, n, globalDepth);
                                cbcp.numContactPoints = 2;
                            }
                        }
                        else
                        {
                            // p2 is Q1 right now; see if Q2 is actually farther and replace p2 if so.
                            float dCur = math.lengthsq(cbcp.p2.point - cbcp.p1.point);
                            float dAlt = math.lengthsq(Q2 - cbcp.p1.point);
                            if (dAlt > dCur && math.lengthsq(Q2 - cbcp.p1.point) > 1e-6f)
                            {
                                WriteContact(ref cbcp.p2, Q2, n, globalDepth);
                            }
                        }

                        return; // side manifold done
                    }
                }
            }

            // Fallback: single point only (your original behavior), but with box-point placement
            // If you want, you can keep your old "probe" as a last resort, but the band search usually removes the need.
        }

        private static bool FindAxisContactBand(
    in Box box, in BoxAxis boxAx,
    in Cylinder cyl, float3 cylAxis,
    float t0, float r,
    out float tL, out float tR)
        {
            float r2 = r * r;
            float tMin = -cyl.halfHeight;
            float tMax = cyl.halfHeight;

            // Quick check: if even at t0 we're not within radius, no band (numerical oddity)
            if (DistSqPointOBB(cyl.center + cylAxis * t0, in box, in boxAx) > r2)
            {
                tL = tR = t0;
                return false;
            }

            // Search outward to bracket the boundary on each side
            float step = math.min(math.max(cyl.radius * 0.5f, 0.01f), cyl.halfHeight); // sane step

            tL = t0;
            tR = t0;

            // Left side (toward tMin)
            {
                float tIn = t0;
                float tOut = t0;

                // march until we leave the radius zone or hit bound
                while (true)
                {
                    float next = tOut - step;
                    if (next <= tMin) { tOut = tMin; break; }

                    float d2 = DistSqPointOBB(cyl.center + cylAxis * next, in box, in boxAx);
                    if (d2 > r2) { break; }

                    tOut = next;
                }

                // If we hit tMin and still inside, boundary is at tMin
                if (tOut == tMin && DistSqPointOBB(cyl.center + cylAxis * tMin, in box, in boxAx) <= r2)
                {
                    tL = tMin;
                }
                else
                {
                    // We have bracket [tOut, tIn] where tIn is inside and tOut is outside-ish.
                    // Ensure proper bracket
                    float inside = tIn;
                    float outside = tOut - step; // the first outside candidate
                    outside = math.max(outside, tMin);

                    // If outside isn't actually outside (step too big), just use inside
                    if (DistSqPointOBB(cyl.center + cylAxis * outside, in box, in boxAx) <= r2)
                        tL = inside;
                    else
                        tL = BisectionToBoundary(in box, in boxAx, cyl, cylAxis, inside, outside, r2);
                }
            }

            // Right side (toward tMax)
            {
                float tIn = t0;
                float tOut = t0;

                while (true)
                {
                    float next = tOut + step;
                    if (next >= tMax) { tOut = tMax; break; }

                    float d2 = DistSqPointOBB(cyl.center + cylAxis * next, in box, in boxAx);
                    if (d2 > r2) { break; }

                    tOut = next;
                }

                if (tOut == tMax && DistSqPointOBB(cyl.center + cylAxis * tMax, in box, in boxAx) <= r2)
                {
                    tR = tMax;
                }
                else
                {
                    float inside = tIn;
                    float outside = tOut + step;
                    outside = math.min(outside, tMax);

                    if (DistSqPointOBB(cyl.center + cylAxis * outside, in box, in boxAx) <= r2)
                        tR = inside;
                    else
                        tR = BisectionToBoundary(in box, in boxAx, cyl, cylAxis, inside, outside, r2);
                }
            }

            // Make sure ordering
            if (tL > tR) { float tmp = tL; tL = tR; tR = tmp; }

            return true;
        }

        private static float BisectionToBoundary(
            in Box box, in BoxAxis boxAx,
            in Cylinder cyl, float3 cylAxis,
            float tInside, float tOutside, float r2)
        {
            // We expect DistSq(tInside) <= r2, DistSq(tOutside) > r2
            float a = tInside;
            float b = tOutside;

            // 8 iterations is plenty
            for (int i = 0; i < 8; i++)
            {
                float m = 0.5f * (a + b);
                float d2 = DistSqPointOBB(cyl.center + cylAxis * m, in box, in boxAx);
                if (d2 <= r2) a = m;
                else b = m;
            }

            return a; // last inside point near boundary
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float DistSqPointOBB(float3 p, in Box box, in BoxAxis ax)
        {
            float3 q = ClosestPointOnOBB(in box, in ax, p);
            return math.lengthsq(p - q);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ComputeCylinderRadialTowardBox(float3 segPtW, float3 boxPtW, float3 cylAxis, float3 fallbackN)
        {
            // Radial direction = (box - axisPoint) with axis component removed
            float3 d = boxPtW - segPtW;
            d -= cylAxis * math.dot(d, cylAxis);

            float lenSq = math.lengthsq(d);
            if (lenSq <= 1e-12f)
            {
                // Degenerate: box point is (nearly) on the cylinder axis line.
                // Pick any perpendicular to cylAxis, biased by fallbackN if possible.
                float3 perp = fallbackN - cylAxis * math.dot(fallbackN, cylAxis);
                if (math.lengthsq(perp) <= 1e-12f)
                    perp = AnyPerp(cylAxis);
                return math.normalize(perp);
            }

            return d * math.rsqrt(lenSq);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 AnyPerp(float3 n)
        {
            // Return any unit vector perpendicular to n (n assumed unit-ish)
            float3 a = (math.abs(n.y) < 0.999f) ? math.up() : math.right();
            float3 p = math.cross(n, a);
            float lenSq = math.lengthsq(p);
            if (lenSq <= 1e-12f)
                return new float3(1, 0, 0);
            return p * math.rsqrt(lenSq);
        }


        // ------------------------------------------------------------
        // Geometry helpers
        // ------------------------------------------------------------

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void GetCylinderSegment(in Cylinder cyl, float3 cylAxis, out float3 A, out float3 B)
        {
            float3 h = cylAxis * cyl.halfHeight;
            A = cyl.center - h;
            B = cyl.center + h;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 GetCylinderAxis(in Cylinder cyl)
        {
            // Convention: cylinder local axis = +Y
            return math.normalize(math.rotate(cyl.rot, math.up()));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static BoxAxis GetBoxAxis(in Box box)
        {
            BoxAxis ax = default;
            ax.right = math.rotate(box.rot, math.right());
            ax.up = math.rotate(box.rot, math.up());
            ax.forward = math.rotate(box.rot, math.forward());
            return ax;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float GetHalf(float3 halfExtents, int axisIndex)
        {
            if (axisIndex == 0) return halfExtents.x;
            if (axisIndex == 1) return halfExtents.y;
            return halfExtents.z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void WriteContact(ref ContactPoint cp, float3 point, float3 normal, float depth)
        {
            cp.point = point;
            cp.normal = normal;
            cp.depth = depth;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool IsNearExisting(Span<float3> pts, int count, float3 p, float epsSq)
        {
            for (int i = 0; i < count; i++)
                if (math.lengthsq(pts[i] - p) <= epsSq)
                    return true;
            return false;
        }

        // Closest point on OBB to point (world)
        private static float3 ClosestPointOnOBB(in Box box, in BoxAxis ax, float3 p)
        {
            float3 d = p - box.center;
            float3 q = box.center;

            float dist;

            dist = math.dot(d, ax[0]);
            dist = math.clamp(dist, -box.halfExtents.x, box.halfExtents.x);
            q += ax[0] * dist;

            dist = math.dot(d, ax[1]);
            dist = math.clamp(dist, -box.halfExtents.y, box.halfExtents.y);
            q += ax[1] * dist;

            dist = math.dot(d, ax[2]);
            dist = math.clamp(dist, -box.halfExtents.z, box.halfExtents.z);
            q += ax[2] * dist;

            return q;
        }

        // Closest points between cylinder axis segment and OBB:
        // - transform segment into box-local AABB
        // - check segment-AABB intersection
        // - otherwise check endpoints + 12 AABB edges against segment (robust, still cheap)
        private static void ClosestSegmentOBB(
            in Box box, in BoxAxis ax,
            float3 A, float3 B,
            out float3 segPtW,
            out float3 boxPtW,
            out float distSq)
        {
            // Box local transform (rotation only, since ax is orthonormal)
            float3 pA = ToBoxLocal(in box, in ax, A);
            float3 pB = ToBoxLocal(in box, in ax, B);

            float3 min = -box.halfExtents;
            float3 max = box.halfExtents;

            // If segment intersects AABB, closest distance is 0.
            if (SegmentIntersectsAABB(pA, pB, min, max, out float tEnter, out float tExit))
            {
                float tMid = 0.5f * (tEnter + tExit);
                float3 p = pA + (pB - pA) * tMid;

                segPtW = ToWorld(in box, in ax, p);
                boxPtW = segPtW;
                distSq = 0f;
                return;
            }

            // Candidate 1: endpoints to AABB
            float3 qA = ClosestPointOnAABB(pA, min, max);
            float3 qB = ClosestPointOnAABB(pB, min, max);

            float dA = math.lengthsq(pA - qA);
            float dB = math.lengthsq(pB - qB);

            float bestD = dA;
            float3 bestSeg = pA;
            float3 bestBox = qA;

            if (dB < bestD)
            {
                bestD = dB;
                bestSeg = pB;
                bestBox = qB;
            }

            // Candidate 2: segment vs AABB edges (12 edges)
            // Build local AABB corners
            float3 c000 = new float3(min.x, min.y, min.z);
            float3 c001 = new float3(min.x, min.y, max.z);
            float3 c010 = new float3(min.x, max.y, min.z);
            float3 c011 = new float3(min.x, max.y, max.z);
            float3 c100 = new float3(max.x, min.y, min.z);
            float3 c101 = new float3(max.x, min.y, max.z);
            float3 c110 = new float3(max.x, max.y, min.z);
            float3 c111 = new float3(max.x, max.y, max.z);

            // List edges as segments (start,end)
            CheckEdge(c000, c001);
            CheckEdge(c000, c010);
            CheckEdge(c000, c100);

            CheckEdge(c111, c110);
            CheckEdge(c111, c101);
            CheckEdge(c111, c011);

            CheckEdge(c001, c011);
            CheckEdge(c001, c101);

            CheckEdge(c010, c011);
            CheckEdge(c010, c110);

            CheckEdge(c100, c101);
            CheckEdge(c100, c110);

            void CheckEdge(float3 e0, float3 e1)
            {
                ClosestPointsSegmentSegment(pA, pB, e0, e1, out float3 s, out float3 q);
                float d = math.lengthsq(s - q);
                if (d < bestD)
                {
                    bestD = d;
                    bestSeg = s;
                    bestBox = q;
                }
            }

            segPtW = ToWorld(in box, in ax, bestSeg);
            boxPtW = ToWorld(in box, in ax, bestBox);
            distSq = bestD;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ToBoxLocal(in Box box, in BoxAxis ax, float3 pWorld)
        {
            float3 d = pWorld - box.center;
            return new float3(
                math.dot(d, ax[0]),
                math.dot(d, ax[1]),
                math.dot(d, ax[2])
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ToWorld(in Box box, in BoxAxis ax, float3 pLocal)
        {
            return box.center + ax[0] * pLocal.x + ax[1] * pLocal.y + ax[2] * pLocal.z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ClosestPointOnAABB(float3 p, float3 min, float3 max)
        {
            return math.clamp(p, min, max);
        }

        // Slab test for segment vs AABB in local space
        private static bool SegmentIntersectsAABB(float3 a, float3 b, float3 min, float3 max, out float tEnter, out float tExit)
        {
            float3 d = b - a;
            tEnter = 0f;
            tExit = 1f;

            for (int i = 0; i < 3; i++)
            {
                float ai = a[i];
                float di = d[i];
                float minI = min[i];
                float maxI = max[i];

                if (math.abs(di) < 1e-12f)
                {
                    if (ai < minI || ai > maxI)
                        return false;
                }
                else
                {
                    float ood = 1f / di;
                    float t1 = (minI - ai) * ood;
                    float t2 = (maxI - ai) * ood;
                    if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }

                    tEnter = math.max(tEnter, t1);
                    tExit = math.min(tExit, t2);
                    if (tEnter > tExit)
                        return false;
                }
            }

            return true;
        }

        // Same helper you already use (ported here)
        private static void ClosestPointsSegmentSegment(
            float3 p1, float3 q1,
            float3 p2, float3 q2,
            out float3 c1, out float3 c2)
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
    }
}
