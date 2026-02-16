using UnityEngine;

namespace Shard.Manifolds
{
    using Unity.Mathematics;
        
    internal static class BoxBoxManifold
    {
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

            float3x3 A = new float3x3(aR);
            float3x3 B = new float3x3(bR);

            float3 tW = bC - aC;

            float3 tA = new float3(math.dot(tW, A.c0), math.dot(tW, A.c1), math.dot(tW, A.c2));
            float3 tB = new float3(math.dot(tW, B.c0), math.dot(tW, B.c1), math.dot(tW, B.c2));

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

            // Drop this into your BoxBoxManifold (replace the "best axis" selection block).
            // It splits bestFace vs bestEdge, and biases toward face contacts when edge-edge only barely wins.

            AxisChoice bestFace = default; bestFace.MinPen = float.PositiveInfinity;
            AxisChoice bestEdge = default; bestEdge.MinPen = float.PositiveInfinity;

            // Face axes A
            for (int i = 0; i < 3; i++)
            {
                float ra = aE[i];
                float rb = bE.x * AbsR[i][0] + bE.y * AbsR[i][1] + bE.z * AbsR[i][2];

                float dist = math.abs(tA[i]);
                float pen = (ra + rb) - dist;
                if (pen < 0f) return false;

                float sign = (tA[i] < 0f) ? -1f : 1f;
                float3 axisW = A[i] * sign;

                Consider(ref bestFace, pen, axisW, AxisKind.FaceA, i, 0, sign, tW);
            }

            // Face axes B
            for (int i = 0; i < 3; i++)
            {
                float ra = aE.x * AbsR[0][i] + aE.y * AbsR[1][i] + aE.z * AbsR[2][i];
                float rb = bE[i];

                float dist = math.abs(tB[i]);
                float pen = (ra + rb) - dist;
                if (pen < 0f) return false;

                float sign = (tB[i] < 0f) ? -1f : 1f;
                float3 axisW = B[i] * sign;

                Consider(ref bestFace, pen, axisW, AxisKind.FaceB, i, 0, sign, tW);
            }

            // Cross (edge-edge) axes
            // Add a small "edge penalty" so face wins unless edge-edge is meaningfully better.
            float minHalfExtent = math.cmin(math.min(aE, bE));     // smallest half extent across both boxes
            float edgeBias = 1e-3f * (minHalfExtent * 2f);         // ~0.1% of smallest box dimension
                                                                   // You can bump to 2e-3f if you still see flips.

            for (int i = 0; i < 3; i++)
            {
                int i1 = (i + 1) % 3;
                int i2 = (i + 2) % 3;

                for (int j = 0; j < 3; j++)
                {
                    float3 axisWraw = math.cross(A[i], B[j]);
                    float axisLenSq = math.lengthsq(axisWraw);
                    if (axisLenSq < 1e-8f) continue;

                    int j1 = (j + 1) % 3;
                    int j2 = (j + 2) % 3;

                    float ra = aE[i1] * AbsR[i2][j] + aE[i2] * AbsR[i1][j];
                    float rb = bE[j1] * AbsR[i][j2] + bE[j2] * AbsR[i][j1];

                    float dist = math.abs(tA[i2] * R[i1][j] - tA[i1] * R[i2][j]);
                    float pen = (ra + rb) - dist;
                    if (pen < 0f) return false;

                    float3 axisW = axisWraw * math.rsqrt(axisLenSq);
                    float sign = (math.dot(axisW, tW) < 0f) ? -1f : 1f;
                    axisW *= sign;

                    // bias the penetration so edge-edge is less likely to win near face/face cases
                    float penBiased = pen + edgeBias;

                    Consider(ref bestEdge, penBiased, axisW, AxisKind.EdgeEdge, i, j, sign, tW);
                }
            }

            // Decide final best. Prefer face if close.
            AxisChoice best = (bestFace.MinPen <= bestEdge.MinPen) ? bestFace : bestEdge;

            // Optional extra tolerance guard (in addition to the bias)
            float tol = edgeBias;
            if (best.Kind == AxisKind.EdgeEdge && bestFace.MinPen <= bestEdge.MinPen + tol)
                best = bestFace;

            // Use `best` from here onward exactly like you already do:
            float3 n = math.normalizesafe(best.AxisW, new float3(0, 1, 0));
            m.Normal = n;


            if (best.Kind == AxisKind.EdgeEdge)
            {
                GetEdgeSegmentOnBox(aC, A, aE, best.I, -n, out float3 ea0, out float3 ea1);
                GetEdgeSegmentOnBox(bC, B, bE, best.J, +n, out float3 eb0, out float3 eb1);

                ClosestPointsSegmentSegment(ea0, ea1, eb0, eb1, out float3 ca, out float3 cb);
                float3 cp = 0.5f * (ca + cb);

                AddPoint(ref m, cp, best.MinPen, 20u);
                return true;
            }

            // Face contact (clip)
            bool refIsA = (best.Kind == AxisKind.FaceA);
            int refAxis = best.I;

            float3 refC = refIsA ? aC : bC;
            float3x3 refM = refIsA ? A : B;
            float3 refE = refIsA ? aE : bE;

            float3 incC = refIsA ? bC : aC;
            float3x3 incM = refIsA ? B : A;
            float3 incE = refIsA ? bE : aE;

            float3 refAxisW = refM[refAxis];

            // Pick the reference face normal that points the same way as the manifold normal (A->B)
            float faceSign = (math.dot(refAxisW, n) >= 0f) ? 1f : -1f;

            // CORRECT: reference planes use the actual reference face normal (axis-aligned in ref box space)
            float3 refN = math.normalizesafe(refAxisW * faceSign, n);

            // And the face center must be offset along THAT face normal, not along n
            float3 refFaceCenter = refC + refN * refE[refAxis];

            int t0i = (refAxis + 1) % 3;
            int t1i = (refAxis + 2) % 3;
            float3 t0 = refM[t0i];
            float3 t1 = refM[t1i];
            float ext0 = refE[t0i];
            float ext1 = refE[t1i];

            Plane4 side0 = PlaneFromPointNormal(refFaceCenter + t0 * ext0, -t0);
            Plane4 side1 = PlaneFromPointNormal(refFaceCenter - t0 * ext0, +t0);
            Plane4 side2 = PlaneFromPointNormal(refFaceCenter + t1 * ext1, -t1);
            Plane4 side3 = PlaneFromPointNormal(refFaceCenter - t1 * ext1, +t1);

            Plane4 refPlane = PlaneFromPointNormal(refFaceCenter, refN);

            FindIncidentFace(incM, refN, out int incAxis, out float incSign);
            GetFaceQuad(incC, incM, incE, incAxis, incSign, out float3 v0, out float3 v1, out float3 v2, out float3 v3);

            Span8 poly = default;
            poly.Count = 4;
            poly.V0 = v0; poly.V1 = v1; poly.V2 = v2; poly.V3 = v3;

            poly = ClipPoly(poly, side0);
            poly = ClipPoly(poly, side1);
            poly = ClipPoly(poly, side2);
            poly = ClipPoly(poly, side3);

            if (poly.Count == 0) return false;

            uint fidBase = 100u + (uint)(refIsA ? 0 : 10) + (uint)refAxis * 2u + (uint)(faceSign > 0 ? 0 : 1);

            Span8 cand = default;
            Span8 candDepth = default; // store depths in .V0.x etc (hacky) OR use separate float array; see below.

            int candCount = 0;

            for (int i = 0; i < poly.Count; i++)
            {
                float3 pW = poly[i];
                float depth = -SignedDistance(refPlane, pW);
                if (depth < 0f) continue;

                float3 cp = pW + refN * (0.5f * depth);

                cand[candCount] = cp;
                // store depth in a parallel float4-like packing using float3.x
                candDepth[candCount] = new float3(depth, 0, 0);
                candCount++;
            }

            if (candCount == 0) return false;

            ReduceTo4_Spread(cand, candDepth, candCount, refN, fidBase, ref m);

            if (m.PointCount == 0)
            {
                float3 fallback = 0.5f * (aC + bC) - n * (0.5f * best.MinPen);
                AddPoint(ref m, fallback, best.MinPen, 20u);
            }

            m.Normal = n;
            return m.PointCount > 0;
        }

        private static void ReduceTo4_Spread(in Span8 pts, in Span8 depths, int count, float3 n, uint fidBase, ref ContactManifold m)
        {
            // 1) pick deepest as anchor
            int i0 = 0;
            float d0 = depths[0].x;
            for (int i = 1; i < count; i++)
            {
                float d = depths[i].x;
                if (d > d0) { d0 = d; i0 = i; }
            }

            // Build tangent basis (any stable basis)
            BuildBasis(n, out float3 t0, out float3 t1);

            // Project to 2D in contact plane
            float2 p0 = Project2(pts[i0], t0, t1);

            // 2) pick farthest from anchor
            int i1 = i0;
            float best = -1f;
            for (int i = 0; i < count; i++)
            {
                if (i == i0) continue;
                float2 p = Project2(pts[i], t0, t1);
                float dsq = math.lengthsq(p - p0);
                if (dsq > best) { best = dsq; i1 = i; }
            }

            // If everything collapsed, just output the deepest
            if (i1 == i0)
            {
                AddPoint(ref m, pts[i0], depths[i0].x, fidBase);
                return;
            }

            float2 p1 = Project2(pts[i1], t0, t1);

            // 3) pick point with max distance to line (p0->p1) (maximize area)
            int i2 = i0;
            best = -1f;
            for (int i = 0; i < count; i++)
            {
                if (i == i0 || i == i1) continue;
                float2 p = Project2(pts[i], t0, t1);
                float area2 = math.abs(Cross2(p1 - p0, p - p0)); // 2*area
                if (area2 > best) { best = area2; i2 = i; }
            }

            // 4) pick point farthest from triangle (maximize coverage)
            int i3 = i0;
            best = -1f;
            for (int i = 0; i < count; i++)
            {
                if (i == i0 || i == i1 || i == i2) continue;
                float2 p = Project2(pts[i], t0, t1);
                float dsq = DistSqToSegment2(p, p0, p1);
                if (i2 != i0) dsq = math.min(dsq, DistSqToSegment2(p, p1, Project2(pts[i2], t0, t1)));
                if (i2 != i0) dsq = math.min(dsq, DistSqToSegment2(p, Project2(pts[i2], t0, t1), p0));
                if (dsq > best) { best = dsq; i3 = i; }
            }

            // Emit in a stable order: deepest first, then spread
            AddPoint(ref m, pts[i0], depths[i0].x, fidBase + 0u);
            AddPoint(ref m, pts[i1], depths[i1].x, fidBase + 1u);

            if (i2 != i0)
                AddPoint(ref m, pts[i2], depths[i2].x, fidBase + 2u);

            if (i3 != i0 && i3 != i1 && i3 != i2)
                AddPoint(ref m, pts[i3], depths[i3].x, fidBase + 3u);
        }

        private static void BuildBasis(float3 n, out float3 t0, out float3 t1)
        {
            // Pick a vector not parallel to n
            float3 a = (math.abs(n.y) < 0.99f) ? new float3(0, 1, 0) : new float3(1, 0, 0);
            t0 = math.normalizesafe(math.cross(a, n), new float3(1, 0, 0));
            t1 = math.cross(n, t0);
        }

        private static float2 Project2(float3 p, float3 t0, float3 t1)
        {
            return new float2(math.dot(p, t0), math.dot(p, t1));
        }

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
        // Types
        // -------------------------
        private enum AxisKind : byte { FaceA, FaceB, EdgeEdge }

        private struct AxisChoice
        {
            public float MinPen;
            public float3 AxisW;
            public AxisKind Kind;
            public int I;
            public int J;
            public float Sign;
        }

        private struct Plane4
        {
            public float3 N;
            public float D; // dot(N, X) + D = 0
        }

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

        // -------------------------
        // Axis selection helper
        // -------------------------
        private static void Consider(ref AxisChoice best, float pen, float3 axisW, AxisKind kind, int i, int j, float sign, float3 tW)
        {
            // keep axis pointing A->B
            if (math.dot(axisW, tW) < 0f) axisW = -axisW;

            if (pen < best.MinPen)
            {
                best.MinPen = pen;
                best.AxisW = axisW;
                best.Kind = kind;
                best.I = i;
                best.J = j;
                best.Sign = sign;
            }
        }

        // -------------------------
        // Face helpers (incident selection + quad)
        // -------------------------
        private static void FindIncidentFace(in float3x3 incM, float3 refN, out int axis, out float sign)
        {
            float bestAbs = -1f;
            axis = 0;
            sign = 1f;

            for (int i = 0; i < 3; i++)
            {
                float d = math.dot(incM[i], refN);   // [-1..1]
                float ad = math.abs(d);
                if (ad > bestAbs)
                {
                    bestAbs = ad;
                    axis = i;
                    // We want incident face normal pointing opposite refN:
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
        private static Plane4 PlaneFromPointNormal(float3 p, float3 n)
        {
            n = math.normalizesafe(n, new float3(0, 1, 0));
            return new Plane4 { N = n, D = -math.dot(n, p) };
        }

        private static float SignedDistance(in Plane4 pl, float3 p)
            => math.dot(pl.N, p) + pl.D;

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
        // Edge-edge helper: pick an extreme edge
        // -------------------------
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

        // -------------------------
        // Geometry helpers
        // -------------------------
        private static void GetBoxWorld(in BoxCollider b, in Pose pose, out float3 c, out quaternion r, out float3 he)
        {
            c = pose.Position + math.mul(pose.Rotation, b.Center);
            r = math.mul(pose.Rotation, b.Orientation);
            he = b.HalfExtents;
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
        // Manifold helpers (mirrors your Narrowphase ones)
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
