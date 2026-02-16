using System.Runtime.CompilerServices;
using UnityEngine;

namespace Shard.Manifolds
{
    using Unity.Mathematics;

    // SAT-case driven OBB-OBB manifold:
    // - SAT decides *which feature pair* (FaceA / FaceB / EdgeEdge) is responsible for minimum penetration.
    // - Face cases: reference face is that SAT face (no heuristics), incident face is the most anti-parallel face on the other box,
    //   then clip incident quad against the 4 side planes of the reference face.
    // - EdgeEdge case: pick extreme edges and do segment-segment closest points -> 1 contact.
    //
    // Assumptions (matching your snippet):
    // Pose.Position : float3, Pose.Rotation : quaternion
    // BoxCollider.Center : float3, BoxCollider.Orientation : quaternion, BoxCollider.HalfExtents : float3
    internal static class BoxBoxManifold
    {
        private const float kEps = 1e-6f;
        private const float kParallelAxisLenSq = 1e-10f;
        private const float kPlaneSlop = 1e-6f;

        // -------------------------
        // Public API
        // -------------------------
        public static bool Generate(in BoxCollider a, in Pose aPose,
                                    in BoxCollider b, in Pose bPose,
                                    ref ContactManifold m)
        {
            ClearManifold(ref m);

            GetBoxWorld(a, aPose, out float3 aC, out quaternion aR, out float3 aE);
            GetBoxWorld(b, bPose, out float3 bC, out quaternion bR, out float3 bE);

            float3x3 A = new float3x3(aR); // columns are world axes
            float3x3 B = new float3x3(bR);

            // SAT: normal (A->B), penetration depth, and winning axis identity
            if (!SatCase(aC, A, aE, bC, B, bE, out SatHit hit))
                return false;

            m.Normal = hit.N;

            if (hit.Kind == AxisKind.EdgeEdge)
            {
                // Edge-edge: i = axis on A, j = axis on B
                BuildEdgeEdge(aC, A, aE, bC, B, bE, hit.I, hit.J, hit.N, hit.Depth, ref m);
                return m.PointCount > 0;
            }

            // FaceA(i): ref=A, inc=B
            // FaceB(j): ref=B, inc=A (but manifold normal stays A->B; do NOT flip m.Normal)
            if (hit.Kind == AxisKind.FaceA)
                BuildFace(refBoxIsA: true, aC, A, aE, bC, B, bE, hit.I, hit.N, hit.Depth, ref m);
            else
                BuildFace(refBoxIsA: false, bC, B, bE, aC, A, aE, hit.I, hit.N, hit.Depth, ref m);

            // If clipping degenerates (rare), fall back to a single stable point.
            if (m.PointCount == 0)
            {
                float3 pA = SupportPoint(aC, A, aE, +hit.N);
                float3 pB = SupportPoint(bC, B, bE, -hit.N);
                float3 cp = 0.5f * (pA + pB);
                AddPoint(ref m, cp, hit.Depth, 20u);
            }

            m.Normal = hit.N;
            return m.PointCount > 0;
        }

        // -------------------------
        // SAT case
        // -------------------------
        private enum AxisKind : byte { FaceA, FaceB, EdgeEdge }

        private struct SatHit
        {
            public AxisKind Kind;
            public int I;     // Face axis index (0..2) OR edge axis i for EdgeEdge
            public int J;     // edge axis j for EdgeEdge
            public float3 N;  // normalized, points A -> B
            public float Depth;
        }

        private static bool SatCase(float3 aC, in float3x3 A, float3 aE,
                                    float3 bC, in float3x3 B, float3 bE,
                                    out SatHit hit)
        {
            float3 tW = bC - aC;

            // translation in each local frame
            float3 tA = new float3(math.dot(tW, A.c0), math.dot(tW, A.c1), math.dot(tW, A.c2));
            float3 tB = new float3(math.dot(tW, B.c0), math.dot(tW, B.c1), math.dot(tW, B.c2));

            // R = A^T * B (column-major)
            float3x3 R = new float3x3(
                new float3(math.dot(A.c0, B.c0), math.dot(A.c1, B.c0), math.dot(A.c2, B.c0)),
                new float3(math.dot(A.c0, B.c1), math.dot(A.c1, B.c1), math.dot(A.c2, B.c1)),
                new float3(math.dot(A.c0, B.c2), math.dot(A.c1, B.c2), math.dot(A.c2, B.c2))
            );

            float3x3 AbsR = new float3x3(
                math.abs(R.c0) + kEps,
                math.abs(R.c1) + kEps,
                math.abs(R.c2) + kEps
            );

            // Bias edge-edge slightly so face wins when nearly equal (stability)
            float minHalfExtent = math.cmin(math.min(aE, bE));
            float edgeBias = 2e-3f * (minHalfExtent * 2f);

            float bestPen = float.PositiveInfinity;
            AxisKind bestKind = AxisKind.FaceA;
            int bestI = 0, bestJ = 0;
            float3 bestAxis = new float3(0, 1, 0);

            // --- Face axes A[i] ---
            for (int i = 0; i < 3; i++)
            {
                float ra = aE[i];
                float rb = bE.x * Mij(AbsR, i, 0) + bE.y * Mij(AbsR, i, 1) + bE.z * Mij(AbsR, i, 2);

                float dist = math.abs(tA[i]);
                float pen = (ra + rb) - dist;
                if (pen < 0f) { hit = default; return false; }

                float3 axis = A[i];
                if (math.dot(axis, tW) < 0f) axis = -axis; // point A->B

                if (pen < bestPen)
                {
                    bestPen = pen;
                    bestKind = AxisKind.FaceA;
                    bestI = i; bestJ = 0;
                    bestAxis = axis;
                }
            }

            // --- Face axes B[j] ---
            for (int j = 0; j < 3; j++)
            {
                float ra = aE.x * Mij(AbsR, 0, j) + aE.y * Mij(AbsR, 1, j) + aE.z * Mij(AbsR, 2, j);
                float rb = bE[j];

                float dist = math.abs(tB[j]);
                float pen = (ra + rb) - dist;
                if (pen < 0f) { hit = default; return false; }

                float3 axis = B[j];
                if (math.dot(axis, tW) < 0f) axis = -axis; // point A->B

                if (pen < bestPen)
                {
                    bestPen = pen;
                    bestKind = AxisKind.FaceB;
                    bestI = j; bestJ = 0;
                    bestAxis = axis;
                }
            }

            // --- Cross axes A[i] x B[j] ---
            for (int i = 0; i < 3; i++)
            {
                int i1 = (i + 1) % 3;
                int i2 = (i + 2) % 3;

                for (int j = 0; j < 3; j++)
                {
                    float3 axisRaw = math.cross(A[i], B[j]);
                    float lenSq = math.lengthsq(axisRaw);
                    if (lenSq < kParallelAxisLenSq) continue;

                    int j1 = (j + 1) % 3;
                    int j2 = (j + 2) % 3;

                    float ra = aE[i1] * Mij(AbsR, i2, j) + aE[i2] * Mij(AbsR, i1, j);
                    float rb = bE[j1] * Mij(AbsR, i, j2) + bE[j2] * Mij(AbsR, i, j1);

                    float dist = math.abs(tA[i2] * Mij(R, i1, j) - tA[i1] * Mij(R, i2, j));
                    float pen = (ra + rb) - dist;
                    if (pen < 0f) { hit = default; return false; }

                    float3 axis = axisRaw * math.rsqrt(lenSq);
                    if (math.dot(axis, tW) < 0f) axis = -axis; // point A->B

                    // Edge bias
                    float biased = pen + edgeBias;
                    if (biased < bestPen)
                    {
                        bestPen = biased;
                        bestKind = AxisKind.EdgeEdge;
                        bestI = i;
                        bestJ = j;
                        bestAxis = axis;
                    }
                }
            }

            // Output
            float3 n = math.normalizesafe(bestAxis, new float3(0, 1, 0));

            // Recover actual depth if best was edge-edge (bestPen contains bias)
            float depth = bestPen;
            if (bestKind == AxisKind.EdgeEdge)
            {
                // recompute actual penetration for chosen (i,j) without bias
                int i = bestI, j = bestJ;
                int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
                int j1 = (j + 1) % 3, j2 = (j + 2) % 3;

                float ra = aE[i1] * Mij(AbsR, i2, j) + aE[i2] * Mij(AbsR, i1, j);
                float rb = bE[j1] * Mij(AbsR, i, j2) + bE[j2] * Mij(AbsR, i, j1);
                float dist = math.abs(tA[i2] * Mij(R, i1, j) - tA[i1] * Mij(R, i2, j));
                depth = (ra + rb) - dist;
            }

            hit = new SatHit
            {
                Kind = bestKind,
                I = bestI,
                J = bestJ,
                N = n,
                Depth = depth
            };
            return true;
        }

        // Column-major float3x3 access: row i, col j
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float Mij(in float3x3 M, int i, int j) => M[j][i];

        // -------------------------
        // Face manifold (SAT-driven)
        // -------------------------
        private static void BuildFace(bool refBoxIsA,
                                      float3 refC, in float3x3 refM, float3 refE,
                                      float3 incC, in float3x3 incM, float3 incE,
                                      int refAxis,
                                      float3 nAtoB,
                                      float depth,
                                      ref ContactManifold m)
        {
            // Reference face normal is exactly ±refM[refAxis], aligned with manifold normal.
            float3 refAxisW = refM[refAxis];
            float faceSign = (math.dot(refAxisW, nAtoB) >= 0f) ? 1f : -1f;
            float3 refN = math.normalizesafe(refAxisW * faceSign, nAtoB);

            // Reference face center and tangents (in reference box frame)
            float3 refFaceCenter = refC + refN * refE[refAxis];

            int t0i = (refAxis + 1) % 3;
            int t1i = (refAxis + 2) % 3;

            float3 t0 = refM[t0i];
            float3 t1 = refM[t1i];
            float ext0 = refE[t0i];
            float ext1 = refE[t1i];

            // Side planes of reference face rectangle (clip space is "inside" => SignedDistance >= 0)
            Plane4 side0 = PlaneFromPointNormal(refFaceCenter + t0 * ext0, -t0);
            Plane4 side1 = PlaneFromPointNormal(refFaceCenter - t0 * ext0, +t0);
            Plane4 side2 = PlaneFromPointNormal(refFaceCenter + t1 * ext1, -t1);
            Plane4 side3 = PlaneFromPointNormal(refFaceCenter - t1 * ext1, +t1);

            // Reference plane for depth (keep points behind it)
            Plane4 refPlane = PlaneFromPointNormal(refFaceCenter, refN);

            // Choose incident face on incident box: face whose normal is most anti-parallel to refN
            FindIncidentFace(incM, refN, out int incAxis, out float incSign);
            GetFaceQuad(incC, incM, incE, incAxis, incSign, out float3 v0, out float3 v1, out float3 v2, out float3 v3);

            Span8 poly = default;
            poly.Count = 4;
            poly.V0 = v0; poly.V1 = v1; poly.V2 = v2; poly.V3 = v3;

            // Clip incident polygon against reference face side planes
            poly = ClipPoly(poly, side0);
            poly = ClipPoly(poly, side1);
            poly = ClipPoly(poly, side2);
            poly = ClipPoly(poly, side3);

            if (poly.Count == 0)
                return;

            // Build candidate points + depths
            Span8 cand = default;
            Span8 depths = default;
            int count = 0;

            for (int i = 0; i < poly.Count; i++)
            {
                float3 p = poly[i];

                // depth relative to reference plane (positive means penetrating)
                float d = -SignedDistance(refPlane, p);
                if (d < -kPlaneSlop) continue;

                // Project point onto reference plane for stability
                float3 cp = p + refN * d;

                cand[count] = cp;
                depths[count] = new float3(d, 0, 0);
                count++;
                if (count == 8) break;
            }

            if (count == 0)
                return;

            // Reduce to <= 4 spread points in contact plane (plane normal is manifold normal)
            // Using refN is OK here because refN is collinear with nAtoB in face case.
            uint fidBase = 100u + (uint)(refBoxIsA ? 0 : 10) + (uint)(refAxis * 2) + (uint)(faceSign > 0 ? 0 : 1);
            ReduceTo4_Spread(cand, depths, count, refN, fidBase, ref m);

            // If still nothing (extreme degeneracy), add a single support-midpoint.
            if (m.PointCount == 0)
            {
                float3 fallback = refFaceCenter - refN * (0.5f * math.max(depth, 0f));
                AddPoint(ref m, fallback, math.max(depth, 0f), 20u);
            }
        }

        // -------------------------
        // Edge-edge manifold (SAT-driven)
        // -------------------------
        private static void BuildEdgeEdge(float3 aC, in float3x3 A, float3 aE,
                                          float3 bC, in float3x3 B, float3 bE,
                                          int edgeAxisA, int edgeAxisB,
                                          float3 nAtoB,
                                          float depth,
                                          ref ContactManifold m)
        {
            // Pick the extreme edge on each box that is "most opposing" along the normal.
            GetEdgeSegmentOnBox(aC, A, aE, edgeAxisA, -nAtoB, out float3 ea0, out float3 ea1);
            GetEdgeSegmentOnBox(bC, B, bE, edgeAxisB, +nAtoB, out float3 eb0, out float3 eb1);

            ClosestPointsSegmentSegment(ea0, ea1, eb0, eb1, out float3 ca, out float3 cb);
            float3 cp = 0.5f * (ca + cb);

            AddPoint(ref m, cp, math.max(depth, 0f), 20u);
        }

        // -------------------------
        // SAT-case helpers
        // -------------------------
        private static void FindIncidentFace(in float3x3 incM, float3 refN, out int axis, out float sign)
        {
            float bestAbs = -1f;
            axis = 0;
            sign = 1f;

            for (int i = 0; i < 3; i++)
            {
                // incM[i] returns column i (axis i)
                float d = math.dot(incM[i], refN); // [-1..1]
                float ad = math.abs(d);
                if (ad > bestAbs)
                {
                    bestAbs = ad;
                    axis = i;
                    // Want incident face normal pointing opposite refN:
                    sign = (d > 0f) ? -1f : +1f;
                }
            }
        }

        private static void GetFaceQuad(float3 c, in float3x3 M, float3 e, int faceAxis, float faceSign,
                                        out float3 v0, out float3 v1, out float3 v2, out float3 v3)
        {
            float3 n = M[faceAxis] * faceSign;
            float3 fc = c + n * e[faceAxis];

            int t0i = (faceAxis + 1) % 3;
            int t1i = (faceAxis + 2) % 3;

            float3 t0 = M[t0i];
            float3 t1 = M[t1i];

            float ext0 = e[t0i];
            float ext1 = e[t1i];

            v0 = fc + t0 * ext0 + t1 * ext1;
            v1 = fc - t0 * ext0 + t1 * ext1;
            v2 = fc - t0 * ext0 - t1 * ext1;
            v3 = fc + t0 * ext0 - t1 * ext1;
        }

        // -------------------------
        // Clipping
        // -------------------------
        private struct Plane4
        {
            public float3 N;
            public float D; // dot(N, X) + D = 0
        }

        private static Plane4 PlaneFromPointNormal(float3 p, float3 n)
        {
            n = math.normalizesafe(n, new float3(0, 1, 0));
            return new Plane4 { N = n, D = -math.dot(n, p) };
        }

        private static float SignedDistance(in Plane4 pl, float3 p)
            => math.dot(pl.N, p) + pl.D;

        private struct Span8
        {
            public int Count;
            public float3 V0, V1, V2, V3, V4, V5, V6, V7;

            public float3 this[int i]
            {
                get => i switch
                {
                    0 => V0,
                    1 => V1,
                    2 => V2,
                    3 => V3,
                    4 => V4,
                    5 => V5,
                    6 => V6,
                    _ => V7
                };
                set
                {
                    if (i == 0) V0 = value;
                    else if (i == 1) V1 = value;
                    else if (i == 2) V2 = value;
                    else if (i == 3) V3 = value;
                    else if (i == 4) V4 = value;
                    else if (i == 5) V5 = value;
                    else if (i == 6) V6 = value;
                    else V7 = value;
                }
            }
        }

        private static Span8 ClipPoly(Span8 poly, in Plane4 pl)
        {
            if (poly.Count == 0) return poly;

            Span8 outPoly = default;

            float3 prev = poly[poly.Count - 1];
            float prevD = SignedDistance(pl, prev);
            bool prevIn = prevD >= 0f;

            for (int i = 0; i < poly.Count; i++)
            {
                float3 cur = poly[i];
                float curD = SignedDistance(pl, cur);
                bool curIn = curD >= 0f;

                if (curIn)
                {
                    if (!prevIn)
                    {
                        float3 inter = IntersectSegmentPlane(prev, cur, prevD, curD);
                        outPoly[outPoly.Count++] = inter;
                    }
                    outPoly[outPoly.Count++] = cur;
                }
                else if (prevIn)
                {
                    float3 inter = IntersectSegmentPlane(prev, cur, prevD, curD);
                    outPoly[outPoly.Count++] = inter;
                }

                prev = cur;
                prevD = curD;
                prevIn = curIn;
            }

            if (outPoly.Count > 8) outPoly.Count = 8;
            return outPoly;
        }

        private static float3 IntersectSegmentPlane(float3 a, float3 b, float da, float db)
        {
            float t = da / (da - db);
            t = math.clamp(t, 0f, 1f);
            return a + (b - a) * t;
        }

        // -------------------------
        // Reduction (spread to 4)
        // -------------------------
        private static void ReduceTo4_Spread(in Span8 pts, in Span8 depths, int count, float3 n, uint fidBase, ref ContactManifold m)
        {
            // 1) deepest
            int i0 = 0;
            float d0 = depths[0].x;
            for (int i = 1; i < count; i++)
            {
                float d = depths[i].x;
                if (d > d0) { d0 = d; i0 = i; }
            }

            BuildBasis(n, out float3 t0, out float3 t1);
            float2 p0 = Project2(pts[i0], t0, t1);

            // 2) farthest from deepest
            int i1 = i0;
            float best = -1f;
            for (int i = 0; i < count; i++)
            {
                if (i == i0) continue;
                float2 p = Project2(pts[i], t0, t1);
                float dsq = math.lengthsq(p - p0);
                if (dsq > best) { best = dsq; i1 = i; }
            }

            if (i1 == i0)
            {
                AddPoint(ref m, pts[i0], depths[i0].x, fidBase + 0u);
                return;
            }

            float2 p1 = Project2(pts[i1], t0, t1);

            // 3) max distance to line p0->p1
            int i2 = i0;
            best = -1f;
            for (int i = 0; i < count; i++)
            {
                if (i == i0 || i == i1) continue;
                float2 p = Project2(pts[i], t0, t1);
                float area2 = math.abs(Cross2(p1 - p0, p - p0));
                if (area2 > best) { best = area2; i2 = i; }
            }

            // 4) farthest from triangle edges
            int i3 = i0;
            best = -1f;
            float2 p2 = (i2 != i0) ? Project2(pts[i2], t0, t1) : p0;

            for (int i = 0; i < count; i++)
            {
                if (i == i0 || i == i1 || i == i2) continue;
                float2 p = Project2(pts[i], t0, t1);
                float dsq = DistSqToSegment2(p, p0, p1);
                if (i2 != i0)
                {
                    dsq = math.min(dsq, DistSqToSegment2(p, p1, p2));
                    dsq = math.min(dsq, DistSqToSegment2(p, p2, p0));
                }
                if (dsq > best) { best = dsq; i3 = i; }
            }

            AddPoint(ref m, pts[i0], depths[i0].x, fidBase + 0u);
            AddPoint(ref m, pts[i1], depths[i1].x, fidBase + 1u);
            if (i2 != i0) AddPoint(ref m, pts[i2], depths[i2].x, fidBase + 2u);
            if (i3 != i0 && i3 != i1 && i3 != i2) AddPoint(ref m, pts[i3], depths[i3].x, fidBase + 3u);
        }

        private static void BuildBasis(float3 n, out float3 t0, out float3 t1)
        {
            float3 a = (math.abs(n.y) < 0.99f) ? new float3(0, 1, 0) : new float3(1, 0, 0);
            t0 = math.normalizesafe(math.cross(a, n), new float3(1, 0, 0));
            t1 = math.cross(n, t0);
        }

        private static float2 Project2(float3 p, float3 t0, float3 t1) => new float2(math.dot(p, t0), math.dot(p, t1));
        private static float Cross2(float2 a, float2 b) => a.x * b.y - a.y * b.x;

        private static float DistSqToSegment2(float2 p, float2 a, float2 b)
        {
            float2 ab = b - a;
            float denom = math.max(math.dot(ab, ab), 1e-12f);
            float t = math.dot(p - a, ab) / denom;
            t = math.clamp(t, 0f, 1f);
            float2 q = a + ab * t;
            return math.lengthsq(p - q);
        }

        // -------------------------
        // Support / edge selection (edge-edge)
        // -------------------------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 SupportPoint(float3 c, in float3x3 M, float3 e, float3 dirW)
        {
            float sx = (math.dot(dirW, M.c0) >= 0f) ? 1f : -1f;
            float sy = (math.dot(dirW, M.c1) >= 0f) ? 1f : -1f;
            float sz = (math.dot(dirW, M.c2) >= 0f) ? 1f : -1f;
            return c + M.c0 * (sx * e.x) + M.c1 * (sy * e.y) + M.c2 * (sz * e.z);
        }

        private static void GetEdgeSegmentOnBox(float3 c, in float3x3 M, float3 e, int edgeAxis, float3 towardNormal,
                                                out float3 p0, out float3 p1)
        {
            int a0 = edgeAxis;
            int a1 = (edgeAxis + 1) % 3;
            int a2 = (edgeAxis + 2) % 3;

            float s1 = (math.dot(M[a1], towardNormal) >= 0f) ? 1f : -1f;
            float s2 = (math.dot(M[a2], towardNormal) >= 0f) ? 1f : -1f;

            float3 baseP = c + M[a1] * (s1 * e[a1]) + M[a2] * (s2 * e[a2]);
            float3 dir = M[a0];

            p0 = baseP - dir * e[a0];
            p1 = baseP + dir * e[a0];
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

        // -------------------------
        // Geometry helpers
        // -------------------------
        private static void GetBoxWorld(in BoxCollider b, in Pose pose, out float3 c, out quaternion r, out float3 he)
        {
            c = pose.Position + math.mul(pose.Rotation, b.Center);
            r = math.mul(pose.Rotation, b.Orientation);
            he = b.HalfExtents;
        }

        // -------------------------
        // Manifold helpers
        // -------------------------
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
    }
}
