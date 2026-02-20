using System;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using UnityEngine;

namespace Shard.Runtime.Solvers
{
    public struct BoxBoxSolver
    {
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

        public struct SeparatingAxises
        {
            // --- Face axes (always valid; should already be unit) ---
            public float3 a1, a2, a3; // A0,A1,A2
            public float3 b1, b2, b3; // B0,B1,B2

            // --- Cross axes (need validity + safe normalize) ---
            public float3 a1xb1, a1xb2, a1xb3;
            public float3 a2xb1, a2xb2, a2xb3;
            public float3 a3xb1, a3xb2, a3xb3;

            public bool a1xb1Valid, a1xb2Valid, a1xb3Valid;
            public bool a2xb1Valid, a2xb2Valid, a2xb3Valid;
            public bool a3xb1Valid, a3xb2Valid, a3xb3Valid;

            public static readonly int NUM_SEPARATING_AXIS = 15;

            // Axis lookup (0..14)
            public float3 this[int index]
            {
                get
                {
                    if (index == 0) return a1;
                    if (index == 1) return a2;
                    if (index == 2) return a3;

                    if (index == 3) return b1;
                    if (index == 4) return b2;
                    if (index == 5) return b3;

                    if (index == 6) return a1xb1;
                    if (index == 7) return a1xb2;
                    if (index == 8) return a1xb3;

                    if (index == 9) return a2xb1;
                    if (index == 10) return a2xb2;
                    if (index == 11) return a2xb3;

                    if (index == 12) return a3xb1;
                    if (index == 13) return a3xb2;
                    if (index == 14) return a3xb3;

                    return float3.zero;
                }
            }

            // Validity lookup (only meaningful for indices 6..14)
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool IsCrossAxisValid(int index)
            {
                // You can call this only for index 6..14 if you want.
                if (index == 6) return a1xb1Valid;
                if (index == 7) return a1xb2Valid;
                if (index == 8) return a1xb3Valid;

                if (index == 9) return a2xb1Valid;
                if (index == 10) return a2xb2Valid;
                if (index == 11) return a2xb3Valid;

                if (index == 12) return a3xb1Valid;
                if (index == 13) return a3xb2Valid;
                if (index == 14) return a3xb3Valid;

                // Face axes (0..5) are always valid
                return true;
            }

            /// <summary>
            /// Safely normalize each cross axis and set its valid flag.
            /// If near-zero length, mark invalid and set axis = 0.
            /// </summary>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void NormalizeCrossAxes(float epsLenSq = 1e-8f)
            {
                NormalizeCross(ref a1xb1, out a1xb1Valid, epsLenSq);
                NormalizeCross(ref a1xb2, out a1xb2Valid, epsLenSq);
                NormalizeCross(ref a1xb3, out a1xb3Valid, epsLenSq);

                NormalizeCross(ref a2xb1, out a2xb1Valid, epsLenSq);
                NormalizeCross(ref a2xb2, out a2xb2Valid, epsLenSq);
                NormalizeCross(ref a2xb3, out a2xb3Valid, epsLenSq);

                NormalizeCross(ref a3xb1, out a3xb1Valid, epsLenSq);
                NormalizeCross(ref a3xb2, out a3xb2Valid, epsLenSq);
                NormalizeCross(ref a3xb3, out a3xb3Valid, epsLenSq);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void NormalizeCross(ref float3 axis, out bool valid, float epsLenSq)
            {
                float lenSq = math.lengthsq(axis);
                if (lenSq <= epsLenSq)
                {
                    axis = float3.zero;
                    valid = false;
                    return;
                }

                axis *= math.rsqrt(lenSq); // normalize without sqrt
                valid = true;
            }
        }
        public struct ComputedQuantities
        {
            public float3 box1ToBox2CenterVec;
            public float minOverlap;
            public int minOverlapAxis;
            public float3 penAxis;
        }

        private const float SAT_OVERLAP_SLOP = 1e-5f; // or 1e-4f depending on scale

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Solve(Box box1, Box box2, out ContactPointManifold bbcps)
        {
            bbcps = default;

            BoxAxis box1Axis = GetBoxAxis(box1);
            BoxAxis box2Axis = GetBoxAxis(box2);

            SeparatingAxises sepAxs = ComputeSATAxisGroups(box1Axis, box2Axis);

            ComputedQuantities computedQuants = default;
            computedQuants.box1ToBox2CenterVec = box2.center - box1.center;

            if (!SATOverlapTest(box1, box2, box1Axis, box2Axis, sepAxs, computedQuants.box1ToBox2CenterVec, out computedQuants.minOverlap, out computedQuants.minOverlapAxis)) return false;

            // produce normal pointing from box1 to box2
            computedQuants.penAxis = sepAxs[computedQuants.minOverlapAxis];
            if (math.dot(computedQuants.penAxis, computedQuants.box1ToBox2CenterVec) < 0f)
                computedQuants.penAxis = -computedQuants.penAxis;

            BoxBoxManifold.GenerateBoxBoxManifold(box1, box2, box1Axis, box2Axis, sepAxs, computedQuants, out bbcps);

            bbcps.globalPenAxis = computedQuants.penAxis;
            bbcps.globalPenDepth = computedQuants.minOverlap;

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool SATOverlapTest(in Box box1, in Box box2, in BoxAxis box1Axis, in BoxAxis box2Axis, in SeparatingAxises sepAxs, float3 box1ToBox2CenterVec, out float minOverlap, out int minOverlapAxis)
        {
            minOverlap = float.MaxValue;
            minOverlapAxis = -1;

            // sat overlap test
            for (int axisI = 0; axisI < SeparatingAxises.NUM_SEPARATING_AXIS; axisI++)
            {
                // Skip invalid cross axes
                if (axisI >= 6 && !sepAxs.IsCrossAxisValid(axisI))
                    continue;
                var n = sepAxs[axisI];
                var rA = ProjectRadii(box1, n, box1Axis);
                var rB = ProjectRadii(box2, n, box2Axis);
                var dist = math.abs(math.dot(box1ToBox2CenterVec, n));
                var overlap = (rA + rB) - dist;

                if (overlap < SAT_OVERLAP_SLOP) return false;

                if (minOverlapAxis < 0)
                {
                    minOverlapAxis = axisI;
                    minOverlap = overlap;
                }
                else if (overlap < minOverlap)
                {
                    minOverlapAxis = axisI;
                    minOverlap = overlap;
                }
            }

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float ProjectRadii(in Box box, float3 n, in BoxAxis boxAxis)
        {
            return box.halfExtents.x * math.abs(math.dot(n, boxAxis[0])) +
                   box.halfExtents.y * math.abs(math.dot(n, boxAxis[1])) +
                   box.halfExtents.z * math.abs(math.dot(n, boxAxis[2]));
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static BoxAxis GetBoxAxis(Box box)
        {
            BoxAxis boxAxis = default;
            boxAxis.right = math.rotate(box.rot, math.right());
            boxAxis.up = math.rotate(box.rot, math.up());
            boxAxis.forward = math.rotate(box.rot, math.forward());
            return boxAxis;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static SeparatingAxises ComputeSATAxisGroups(in BoxAxis a, in BoxAxis b)
        {
            SeparatingAxises groups = default;

            groups.a1 = a[0];
            groups.a2 = a[1];
            groups.a3 = a[2];

            groups.b1 = b[0];
            groups.b2 = b[1];
            groups.b3 = b[2];

            groups.a1xb1 = math.cross(a[0], b[0]);
            groups.a1xb2 = math.cross(a[0], b[1]);
            groups.a1xb3 = math.cross(a[0], b[2]);

            groups.a2xb1 = math.cross(a[1], b[0]);
            groups.a2xb2 = math.cross(a[1], b[1]);
            groups.a2xb3 = math.cross(a[1], b[2]);

            groups.a3xb1 = math.cross(a[2], b[0]);
            groups.a3xb2 = math.cross(a[2], b[1]);
            groups.a3xb3 = math.cross(a[2], b[2]);

            groups.NormalizeCrossAxes();

            return groups;
        }

        public struct BoxBoxManifold
        {
            // Tunables
            private const float kContactSlop = 1e-4f; // accept slightly separated after clipping
            private const float kPlaneEps = 1e-6f;    // clipping epsilon

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void GenerateBoxBoxManifold(
                in Box box1, in Box box2,
                in BoxAxis box1Axis, in BoxAxis box2Axis,
                in SeparatingAxises sepAxs,
                in ComputedQuantities computedQuants,
                out ContactPointManifold bbcps)
            {
                bbcps = default;

                int satCase = computedQuants.minOverlapAxis;

                // Global normal (already oriented box1 -> box2 by Solve)
                float3 n = computedQuants.penAxis;

                if (satCase < 3)
                {
                    BuildFaceManifold(
                        refIsA: true,
                        refAxisIndex: satCase,
                        globalNormal: n,
                        globalDepth: computedQuants.minOverlap,
                        in box1, in box2,
                        in box1Axis, in box2Axis,
                        out bbcps);
                }
                else if (satCase < 6)
                {
                    BuildFaceManifold(
                        refIsA: false,
                        refAxisIndex: satCase - 3,
                        globalNormal: n,
                        globalDepth: computedQuants.minOverlap,
                        in box1, in box2,
                        in box1Axis, in box2Axis,
                        out bbcps);
                }
                else
                {
                    BuildEdgeEdgeManifold(
                        satCase,
                        globalNormal: n,
                        globalDepth: computedQuants.minOverlap,
                        in box1, in box2,
                        in box1Axis, in box2Axis,
                        out bbcps);
                }
            }

            // ------------------------------------------------------------
            // Face manifold (reference face rectangle + clip incident face)
            // Produces up to 4 contacts. depth is per-point (debug): plane penetration.
            // ------------------------------------------------------------
            private static void BuildFaceManifold(
                bool refIsA,
                int refAxisIndex,      // 0..2
                float3 globalNormal,   // box1 -> box2
                float globalDepth,     // SAT minOverlap (optional debug; not used for per-point depths below)
                in Box box1, in Box box2,
                in BoxAxis box1Axis, in BoxAxis box2Axis,
                out ContactPointManifold bbcps)
            {
                bbcps = default;

                // Choose reference/incident
                Box refBox = refIsA ? box1 : box2;
                Box incBox = refIsA ? box2 : box1;
                BoxAxis refAx = refIsA ? box1Axis : box2Axis;
                BoxAxis incAx = refIsA ? box2Axis : box1Axis;

                // Reference face normal starts as the reference axis
                float3 refN = refAx[refAxisIndex];

                // Ensure refN points from reference -> incident
                float3 refToInc = refIsA ? (box2.center - box1.center) : (box1.center - box2.center);
                if (math.dot(refN, refToInc) < 0f)
                    refN = -refN;

                // Reference face center (on surface)
                float refHalfN = GetHalf(refBox.halfExtents, refAxisIndex);
                float3 refFaceCenter = refBox.center + refN * refHalfN;

                // In-face axes (u,v)
                int uI = (refAxisIndex + 1) % 3;
                int vI = (refAxisIndex + 2) % 3;
                float3 u = refAx[uI];
                float3 v = refAx[vI];
                float uHalf = GetHalf(refBox.halfExtents, uI);
                float vHalf = GetHalf(refBox.halfExtents, vI);

                // Pick incident face on incident box whose normal is most anti-parallel to refN
                int incFaceAxisIndex = 0;
                float d0 = math.dot(refN, incAx[0]);
                float d1 = math.dot(refN, incAx[1]);
                float d2 = math.dot(refN, incAx[2]);

                float ad0 = math.abs(d0);
                float ad1 = math.abs(d1);
                float ad2 = math.abs(d2);

                if (ad1 > ad0) { incFaceAxisIndex = 1; ad0 = ad1; d0 = d1; }
                if (ad2 > ad0) { incFaceAxisIndex = 2; d0 = d2; }

                // If dot(refN, incAxis) > 0 then +incAxis points along refN, so choose NEG face; else POS face.
                bool incPositiveFace = (d0 < 0f);

                // Build incident quad (world space)
                Span<float3> polyA = stackalloc float3[8];
                Span<float3> polyB = stackalloc float3[8];

                GetFaceQuadWorld(incBox, incAx, incFaceAxisIndex, incPositiveFace,
                    out polyA[0], out polyA[1], out polyA[2], out polyA[3]);
                int polyCount = 4;

                // Clip against the 4 side planes of the reference face rectangle
                ClipPolyAgainstPlane(polyA, polyCount, u, math.dot(u, refFaceCenter) + uHalf, polyB, out polyCount);
                if (polyCount == 0) return;

                ClipPolyAgainstPlane(polyB, polyCount, -u, math.dot(-u, refFaceCenter) + uHalf, polyA, out polyCount);
                if (polyCount == 0) return;

                ClipPolyAgainstPlane(polyA, polyCount, v, math.dot(v, refFaceCenter) + vHalf, polyB, out polyCount);
                if (polyCount == 0) return;

                ClipPolyAgainstPlane(polyB, polyCount, -v, math.dot(-v, refFaceCenter) + vHalf, polyA, out polyCount);
                if (polyCount == 0) return;

                // Build candidates: keep points behind/on reference plane.
                // depth = -signedDist to ref plane (clamped >=0). This is mainly for debugging.
                Span<float3> candidates = polyA;
                Span<float> depths = stackalloc float[8];

                int candCount = 0;
                float refPlaneD = math.dot(refN, refFaceCenter);

                for (int i = 0; i < polyCount; i++)
                {
                    float3 p = candidates[i];
                    float signedDist = math.dot(refN, p) - refPlaneD; // <=0 behind plane (penetrating)
                    if (signedDist <= kContactSlop)
                    {
                        // Project onto reference plane for stability
                        float3 pOnPlane = p - refN * signedDist;

                        candidates[candCount] = pOnPlane;
                        depths[candCount] = math.max(0f, -signedDist);
                        candCount++;
                    }
                }

                if (candCount == 0) return;

                int outCount = math.min(4, candCount);
                if (candCount > 4)
                    SelectTop4ByDepth(candidates, depths, candCount);

                // Output with GLOBAL normal (consistent with Solve) and per-point depth (debug).
                WriteContact(ref bbcps.p1, candidates[0], globalNormal, depths[0]);
                if (outCount > 1) WriteContact(ref bbcps.p2, candidates[1], globalNormal, depths[1]);
                if (outCount > 2) WriteContact(ref bbcps.p3, candidates[2], globalNormal, depths[2]);
                if (outCount > 3) WriteContact(ref bbcps.p4, candidates[3], globalNormal, depths[3]);

                bbcps.numContactPoints = outCount;
            }

            // ------------------------------------------------------------
            // Edge-edge manifold (closest points between support edges)
            // Produces 1 contact. depth is set to globalDepth (debug).
            // ------------------------------------------------------------
            private static void BuildEdgeEdgeManifold(
                int satCase,           // 6..14
                float3 globalNormal,   // box1 -> box2
                float globalDepth,     // SAT minOverlap (debug)
                in Box box1, in Box box2,
                in BoxAxis box1Axis, in BoxAxis box2Axis,
                out ContactPointManifold bbcps)
            {
                bbcps = default;

                DecodeCrossAxis(satCase, out int aEdgeDirIndex, out int bEdgeDirIndex);

                // Support edges:
                // - On A (box1): edge direction is box1Axis[aEdgeDirIndex], pick edge farthest along +n
                // - On B (box2): edge direction is box2Axis[bEdgeDirIndex], pick edge farthest along -n
                GetSupportEdgeWorld(in box1, in box1Axis, aEdgeDirIndex, globalNormal, out float3 a0, out float3 a1);
                GetSupportEdgeWorld(in box2, in box2Axis, bEdgeDirIndex, -globalNormal, out float3 b0, out float3 b1);

                ClosestPointsSegmentSegment(a0, a1, b0, b1, out float3 ca, out float3 cb);

                float3 cp = (ca + cb) * 0.5f;

                // For debugging, using globalDepth is the most consistent single-number depth here.
                WriteContact(ref bbcps.p1, cp, globalNormal, globalDepth);
                bbcps.numContactPoints = 1;
            }

            // ------------------------------------------------------------
            // Helpers
            // ------------------------------------------------------------

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static void WriteContact(ref ContactPoint cp, float3 point, float3 normal, float depth)
            {
                cp.point = point;
                cp.normal = normal;
                cp.depth = depth;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static float GetHalf(float3 halfExtents, int axisIndex)
            {
                if (axisIndex == 0) return halfExtents.x;
                if (axisIndex == 1) return halfExtents.y;
                return halfExtents.z;
            }

            private static void GetFaceQuadWorld(
                in Box box,
                in BoxAxis ax,
                int faceAxisIndex,     // 0..2
                bool positiveFace,     // true => +axis face, false => -axis face
                out float3 v0, out float3 v1, out float3 v2, out float3 v3)
            {
                int uI = (faceAxisIndex + 1) % 3;
                int vI = (faceAxisIndex + 2) % 3;

                float3 n = ax[faceAxisIndex];
                float3 u = ax[uI];
                float3 v = ax[vI];

                float hn = GetHalf(box.halfExtents, faceAxisIndex);
                float hu = GetHalf(box.halfExtents, uI);
                float hv = GetHalf(box.halfExtents, vI);

                float3 faceCenter = box.center + n * (positiveFace ? hn : -hn);

                v0 = faceCenter + u * hu + v * hv;
                v1 = faceCenter - u * hu + v * hv;
                v2 = faceCenter - u * hu - v * hv;
                v3 = faceCenter + u * hu - v * hv;
            }

            private static void ClipPolyAgainstPlane(
                Span<float3> inPts, int inCount,
                float3 planeN, float planeD,
                Span<float3> outPts, out int outCount)
            {
                outCount = 0;
                if (inCount == 0) return;

                float3 prev = inPts[inCount - 1];
                float prevDist = math.dot(planeN, prev) - planeD; // <=0 inside

                for (int i = 0; i < inCount; i++)
                {
                    float3 curr = inPts[i];
                    float currDist = math.dot(planeN, curr) - planeD;

                    bool prevInside = prevDist <= kPlaneEps;
                    bool currInside = currDist <= kPlaneEps;

                    if (prevInside && currInside)
                    {
                        outPts[outCount++] = curr;
                    }
                    else if (prevInside && !currInside)
                    {
                        outPts[outCount++] = Intersect(prev, curr, prevDist, currDist);
                    }
                    else if (!prevInside && currInside)
                    {
                        outPts[outCount++] = Intersect(prev, curr, prevDist, currDist);
                        outPts[outCount++] = curr;
                    }

                    prev = curr;
                    prevDist = currDist;
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private static float3 Intersect(float3 a, float3 b, float da, float db)
            {
                float denom = (da - db);
                float t = (math.abs(denom) > 1e-12f) ? (da / denom) : 0f;
                return a + (b - a) * t;
            }

            private static void SelectTop4ByDepth(Span<float3> pts, Span<float> depths, int count)
            {
                // Place 4 largest depths into [0..3]
                for (int k = 0; k < 4; k++)
                {
                    int best = k;
                    float bestD = depths[k];
                    for (int i = k + 1; i < count; i++)
                    {
                        if (depths[i] > bestD)
                        {
                            bestD = depths[i];
                            best = i;
                        }
                    }

                    if (best != k)
                    {
                        (pts[k], pts[best]) = (pts[best], pts[k]);
                        (depths[k], depths[best]) = (depths[best], depths[k]);
                    }
                }
            }

            private static void DecodeCrossAxis(int satCase, out int ai, out int bi)
            {
                int k = satCase - 6;  // 0..8
                ai = k / 3;           // 0..2
                bi = k - ai * 3;      // 0..2
            }

            private static void GetSupportEdgeWorld(
                in Box box,
                in BoxAxis ax,
                int edgeDirAxisIndex,   // 0..2 (direction of the segment)
                float3 supportDir,      // choose edge at extreme along this direction
                out float3 p0, out float3 p1)
            {
                int o1, o2;
                if (edgeDirAxisIndex == 0) { o1 = 1; o2 = 2; }
                else if (edgeDirAxisIndex == 1) { o1 = 0; o2 = 2; }
                else { o1 = 0; o2 = 1; }

                float3 a1 = ax[o1];
                float3 a2 = ax[o2];
                float h1 = GetHalf(box.halfExtents, o1);
                float h2 = GetHalf(box.halfExtents, o2);

                float s1 = (math.dot(supportDir, a1) >= 0f) ? 1f : -1f;
                float s2 = (math.dot(supportDir, a2) >= 0f) ? 1f : -1f;

                float3 offset = a1 * (s1 * h1) + a2 * (s2 * h2);

                float3 e = ax[edgeDirAxisIndex] * GetHalf(box.halfExtents, edgeDirAxisIndex);

                float3 basePt = box.center + offset;
                p0 = basePt + e;
                p1 = basePt - e;
            }

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
}
