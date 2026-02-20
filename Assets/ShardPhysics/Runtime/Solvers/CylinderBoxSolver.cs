using System;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using UnityEngine;

namespace Shard.Runtime.Solvers
{
    public struct CylinderBoxSolver
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
        public static bool Solve(in Box box, in Cylinder cyl, out ContactPointManifold cbcp)
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

            GenerateManifold(in box, in cyl, in boxAx, cylAxis, in axes, in comp, out cbcp);

            cbcp.globalPenAxis = comp.penAxis;
            cbcp.globalPenDepth = comp.minOverlap;

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
             out ContactPointManifold cbcp)
        {
            cbcp = default;

            float3 globalN = comp.penAxis; // box -> cyl

            if (comp.minAxis >= 0 && comp.minAxis <= 2)
            {
                int faceAxis = comp.minAxis;

                // Oriented box face normal for that axis, pointing box -> cyl
                float3 faceN = boxAx[faceAxis];
                if (math.dot(faceN, globalN) < 0f)
                    faceN = -faceN;

                // Cap-on-face: ONLY if cylAxis is extremely parallel to that face normal.
                // (Otherwise you are NOT in a true cap contact scenario.)
                float align = math.abs(math.dot(faceN, cylAxis));
                if (align > 0.9995f)
                {
                    if (BuildCapOnFaceRimManifold(in box, in cyl, in boxAx, cylAxis, faceAxis, faceN, comp.minOverlap, out cbcp))
                        return;
                }

                BuildBoxFaceVsCylinderSideManifold(
                    in box, in cyl, in boxAx, cylAxis,
                    faceAxis, globalN, comp.minOverlap,
                    out cbcp
                );
                return;
            }

            BuildClosestFeatureManifold(in box, in cyl, in boxAx, cylAxis, globalN, comp.minOverlap, out cbcp);
        }

        private static float3 ComputeDeepestContactPoint_BoxFaceVsCylinderSide(
            in Box box,
            in Cylinder cyl,
            in BoxAxis boxAx,
            float3 cylAxis,
            int faceAxis,     // 0..2 box axis index (winning axis)
            float3 globalN)   // box -> cyl (oriented)
        {
            // Oriented face normal (box -> cyl) for the reference face plane
            float3 faceN = boxAx[faceAxis];
            if (math.dot(faceN, globalN) < 0f) faceN = -faceN;

            // Plane point: the actual box face center
            float faceHalf = GetHalf(box.halfExtents, faceAxis);
            float3 faceCenter = box.center + faceN * faceHalf;

            // Deepest point on the CYLINDER in direction INTO the box (i.e. -faceN)
            float3 cylDeep = SupportPointCylinder(in cyl, cylAxis, -faceN);

            // Project that deepest cylinder point onto the box face plane for a stable "on-box" contact.
            // (If you want the point ON the cylinder instead, just return cylDeep.)
            float signed = math.dot(faceN, cylDeep - faceCenter);
            float3 onPlane = cylDeep - faceN * signed;

            // For your "hard separation", you usually want the contact point on the box.
            // This is guaranteed to correspond to the deepest cylinder penetration direction.
            return onPlane;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 SupportPointCylinder(in Cylinder cyl, float3 cylAxis, float3 dirW)
        {
            // Normalize direction for stability
            float3 dW = math.normalizesafe(dirW, new float3(0f, 1f, 0f));

            // Convert dir to cylinder local (local axis is +Y)
            quaternion inv = math.inverse(cyl.rot);
            float3 dL = math.rotate(inv, dW);

            // Choose cap by sign of Y
            float y = (dL.y >= 0f) ? cyl.halfHeight : -cyl.halfHeight;

            // Choose radial direction in XZ
            float2 xz = new float2(dL.x, dL.z);
            float lenSq = math.dot(xz, xz);

            float3 pL;
            if (lenSq > 1e-12f)
            {
                float invLen = math.rsqrt(lenSq);
                float2 n = xz * invLen;
                pL = new float3(n.x * cyl.radius, y, n.y * cyl.radius);
            }
            else
            {
                // Pure axis direction -> cap center
                pL = new float3(0f, y, 0f);
            }

            return cyl.center + math.rotate(cyl.rot, pL);
        }

        private static void BuildBoxFaceVsCylinderSideManifold(
            in Box box,
            in Cylinder cyl,
            in BoxAxis boxAx,
            float3 cylAxis,
            int faceAxis,      // 0..2
            float3 globalN,    // box -> cyl (oriented)
            float globalDepth,
            out ContactPointManifold cbcp)
        {
            cbcp = default;

            // Oriented face normal (box -> cyl)
            float3 faceN = boxAx[faceAxis];
            if (math.dot(faceN, globalN) < 0f) faceN = -faceN;

            float faceHalf = GetHalf(box.halfExtents, faceAxis);
            float3 faceCenter = box.center + faceN * faceHalf;

            // Compute one deepest point on the cylinder side at each end (t = +/- halfHeight)
            float3 cylP0 = DeepestSidePointAtAxisOffset(in cyl, cylAxis, -cyl.halfHeight, faceN);
            float3 cylP1 = DeepestSidePointAtAxisOffset(in cyl, cylAxis, +cyl.halfHeight, faceN);

            // Signed distance to the face plane (positive = in front of plane along faceN)
            float d0 = math.dot(faceN, cylP0 - faceCenter);
            float d1 = math.dot(faceN, cylP1 - faceCenter);

            // "Penetrating" means behind the plane. Use a small slop so grazing doesn’t spam 2 points.
            const float planePenEps = 1e-4f;
            bool pen0 = d0 <= planePenEps;
            bool pen1 = d1 <= planePenEps;

            // Project cylinder witness points onto the box face plane (stable contact point on box)
            float3 onPlane0 = cylP0 - faceN * d0;
            float3 onPlane1 = cylP1 - faceN * d1;

            // If both ends penetrate => 2-point manifold (supports along the side)
            if (pen0 && pen1)
            {
                // De-dupe in case halfHeight ~ 0 or extreme degeneracy
                if (math.lengthsq(onPlane1 - onPlane0) <= 1e-8f)
                {
                    WriteContact(ref cbcp.p1, onPlane0, globalN, globalDepth);
                    cbcp.numContactPoints = 1;
                    return;
                }

                WriteContact(ref cbcp.p1, onPlane0, globalN, globalDepth);
                WriteContact(ref cbcp.p2, onPlane1, globalN, globalDepth);
                cbcp.numContactPoints = 2;
                return;
            }

            // Otherwise emit ONLY ONE: the deeper of the two (most negative distance)
            // If only one penetrates, this automatically picks it.
            if (d1 < d0)
            {
                WriteContact(ref cbcp.p1, onPlane1, globalN, globalDepth);
                cbcp.numContactPoints = 1;
            }
            else
            {
                WriteContact(ref cbcp.p1, onPlane0, globalN, globalDepth);
                cbcp.numContactPoints = 1;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 DeepestSidePointAtAxisOffset(
            in Cylinder cyl,
            float3 cylAxis,     // unit
            float t,            // along axis in [-halfHeight, +halfHeight]
            float3 faceN)       // oriented box face normal (box -> cyl)
        {
            // cap/end center on cylinder axis
            float3 endCenter = cyl.center + cylAxis * t;

            // We want the point on the circular rim (perpendicular to cylAxis) that is deepest behind the plane.
            // That is in direction -faceN projected into the rim plane.
            float3 d = -faceN;

            // Remove component along axis to get a rim-plane direction
            float3 radial = d - cylAxis * math.dot(d, cylAxis);
            float radialLenSq = math.lengthsq(radial);

            if (radialLenSq <= 1e-12f)
            {
                // Plane normal aligned with cylinder axis => deepest at end center (no unique rim direction)
                return endCenter;
            }

            radial *= math.rsqrt(radialLenSq);
            return endCenter + radial * cyl.radius;
        }

        // --- Cap-on-face: rim points on the box face ---
        private static bool BuildCapOnFaceRimManifold(
    in Box box, in Cylinder cyl,
    in BoxAxis boxAx, float3 cylAxis,
    int boxFaceAxisIndex, // 0..2
    float3 faceN,         // oriented box face normal (box -> cyl)
    float globalDepth,
    out ContactPointManifold cbcp)
        {
            cbcp = default;

            float faceHalfN = GetHalf(box.halfExtents, boxFaceAxisIndex);
            float3 faceCenter = box.center + faceN * faceHalfN;

            int uI = (boxFaceAxisIndex + 1) % 3;
            int vI = (boxFaceAxisIndex + 2) % 3;
            float3 u = boxAx[uI];
            float3 v = boxAx[vI];
            float uHalf = GetHalf(box.halfExtents, uI);
            float vHalf = GetHalf(box.halfExtents, vI);

            // Choose the cap that faces toward the box (along -faceN)
            float s = math.dot(cylAxis, faceN);
            float3 capCenter = (s > 0f)
                ? (cyl.center - cylAxis * cyl.halfHeight)
                : (cyl.center + cylAxis * cyl.halfHeight);

            // Project cap center onto the box face plane (disk center in that plane)
            float signedDist = math.dot(faceN, capCenter - faceCenter);
            float3 capProjOnPlane = capCenter - faceN * signedDist;

            // Overlap guard: projected disk must overlap face rect (cheap reject)
            float2 c2 = new float2(
                math.dot(capProjOnPlane - faceCenter, u),
                math.dot(capProjOnPlane - faceCenter, v));

            float2 q = new float2(
                math.max(math.abs(c2.x) - uHalf, 0f),
                math.max(math.abs(c2.y) - vHalf, 0f));

            float rGuard = cyl.radius + 1e-4f;
            if (math.dot(q, q) > rGuard * rGuard)
                return false;

            // 4 candidates: +/-u, +/-v from projected center (then clamp into rect)
            Span<float3> pts = stackalloc float3[4];
            pts[0] = capProjOnPlane + u * cyl.radius;
            pts[1] = capProjOnPlane - u * cyl.radius;
            pts[2] = capProjOnPlane + v * cyl.radius;
            pts[3] = capProjOnPlane - v * cyl.radius;

            int outCount = 0;
            Span<float3> outPts = stackalloc float3[4];

            const float eps = 1e-4f;

            for (int i = 0; i < 4; i++)
            {
                float3 p = pts[i];

                // Clamp to rectangle on the box face plane
                float du = math.dot(p - faceCenter, u);
                float dv = math.dot(p - faceCenter, v);

                du = math.clamp(du, -uHalf, uHalf);
                dv = math.clamp(dv, -vHalf, vHalf);

                float3 pClamped = faceCenter + u * du + v * dv;

                // Must be on/behind the face plane slightly
                float planeSigned = math.dot(faceN, pClamped - faceCenter);
                if (planeSigned > kContactSlop)
                    continue;

                // *** THE IMPORTANT FIX ***
                // Must lie inside the ACTUAL cylinder volume (in cylinder local space),
                // not just inside some projected disk on the box plane.
                if (!PointInsideCylinderWorld(in cyl, pClamped, eps))
                    continue;

                if (!IsNearExisting(outPts, outCount, pClamped, 1e-6f))
                    outPts[outCount++] = pClamped;
            }

            if (outCount == 0)
                return false;

            WriteContact(ref cbcp.p1, outPts[0], faceN, globalDepth);
            if (outCount > 1) WriteContact(ref cbcp.p2, outPts[1], faceN, globalDepth);
            if (outCount > 2) WriteContact(ref cbcp.p3, outPts[2], faceN, globalDepth);
            if (outCount > 3) WriteContact(ref cbcp.p4, outPts[3], faceN, globalDepth);
            cbcp.numContactPoints = outCount;

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool PointInsideCylinderWorld(in Cylinder cyl, float3 pWorld, float eps)
        {
            // Transform point into cylinder local space:
            // cylinder local axis is +Y, with caps at +/-halfHeight.
            float3 p = math.rotate(math.inverse(cyl.rot), pWorld - cyl.center);

            // Height check
            if (math.abs(p.y) > cyl.halfHeight + eps)
                return false;

            // Radial check in XZ
            float rr = p.x * p.x + p.z * p.z;
            float r2 = (cyl.radius + eps) * (cyl.radius + eps);
            return rr <= r2;
        }

        // --- Generic robust manifold from closest segment↔OBB features (1–2 points) ---
        // --- Generic robust manifold from closest segment↔OBB features (1–2 points) ---
        // --- Generic robust manifold from closest segment↔OBB features (1–2 points) ---
        private static void BuildClosestFeatureManifold(
            in Box box, in Cylinder cyl,
            in BoxAxis boxAx, float3 cylAxis,
            float3 globalN,
            float globalDepth,
            out ContactPointManifold cbcp)
        {
            cbcp = default;

            // Cylinder axis segment endpoints
            GetCylinderSegment(in cyl, cylAxis, out float3 A, out float3 B);

            // Closest axis point to the OBB (world) + closest point on box (world)
            ClosestSegmentOBB(in box, in boxAx, A, B, out float3 segPtW, out float3 boxPtW, out float distSq);

            float3 n = globalN;

            // --- IMPORTANT: classify the closest box feature ---
            // If the closest point on the OBB lies on an EDGE or VERTEX, do NOT attempt a 2-point band.
            // Those cases are exactly where your "two points projected onto the cube" becomes nonsense.
            float3 boxPtL = ToBoxLocal(in box, in boxAx, boxPtW);
            bool boxFeatureIsFace = IsLocalPointOnFaceOnly(boxPtL, box.halfExtents, 1e-5f, out int faceAxis);

            // --- Create a robust single contact point ---
            // boxPtW is inside/on the box. Project it to the cylinder to get a cylinder witness point.
            float3 cylPtW = ClosestPointOnCylinderToPointWorld(in cyl, boxPtW);

            // With a single contact point slot (no separate points-per-body),
            // the safest "contained-ish" point is the midpoint of witnesses.
            // This dramatically reduces "point is on cube face but outside cylinder" artifacts.
            float3 contactW = 0.5f * (boxPtW + cylPtW);

            WriteContact(ref cbcp.p1, contactW, n, globalDepth);
            cbcp.numContactPoints = 1;

            // If not a face contact, we are done. (Edge/vertex -> one point only.)
            if (!boxFeatureIsFace)
                return;

            // If this is cap-like (normal ~ cylinder axis), you also generally want 1 point unless
            // you're doing a dedicated cap clipping routine. Your cap path is separate already.
            const float kCapParallel = 0.999f;
            float align = math.abs(math.dot(cylAxis, n));
            if (align >= kCapParallel)
                return;

            // --- Face contact candidate: now we can TRY a 2-point band ---
            // But we must produce points that correspond to the cylinder, not just "closest on box".
            float t0 = math.dot(segPtW - cyl.center, cylAxis);
            t0 = math.clamp(t0, -cyl.halfHeight, cyl.halfHeight);

            const float bandSlop = 1e-4f;
            float r = cyl.radius + bandSlop;

            if (!FindAxisContactBand(in box, in boxAx, in cyl, cylAxis, t0, r, out float tL, out float tR))
                return;

            // If the band is tiny, it's effectively a single-point contact (common near edges)
            if (math.abs(tR - tL) <= 1e-3f)
                return;

            // Build two contacts at band ends, but each contact must be "witnessed" against cylinder too.
            float3 P1 = cyl.center + cylAxis * tL;
            float3 Q1 = ClosestPointOnOBB(in box, in boxAx, P1);
            float3 S1 = ClosestPointOnCylinderToPointWorld(in cyl, Q1);
            float3 C1 = 0.5f * (Q1 + S1);

            float3 P2 = cyl.center + cylAxis * tR;
            float3 Q2 = ClosestPointOnOBB(in box, in boxAx, P2);
            float3 S2 = ClosestPointOnCylinderToPointWorld(in cyl, Q2);
            float3 C2 = 0.5f * (Q2 + S2);

            // Reject degenerate / duplicate points
            if (math.lengthsq(C1 - cbcp.p1.point) <= 1e-6f && math.lengthsq(C2 - cbcp.p1.point) <= 1e-6f)
                return;

            // Pick the second point as the one farther from the first (more stable)
            float d1 = math.lengthsq(C1 - cbcp.p1.point);
            float d2 = math.lengthsq(C2 - cbcp.p1.point);
            float3 pick = (d2 > d1) ? C2 : C1;

            if (math.lengthsq(pick - cbcp.p1.point) > 1e-6f)
            {
                WriteContact(ref cbcp.p2, pick, n, globalDepth);
                cbcp.numContactPoints = 2;
            }

            // Final safety: if the chosen second point ends up basically coincident, keep 1.
            if (cbcp.numContactPoints == 2 && math.lengthsq(cbcp.p2.point - cbcp.p1.point) <= 1e-6f)
                cbcp.numContactPoints = 1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ClosestPointOnCylinderToPointWorld(in Cylinder cyl, float3 pWorld)
        {
            // Cylinder local axis is +Y. Convert point to cylinder local space.
            float3 pL = math.rotate(math.inverse(cyl.rot), pWorld - cyl.center);

            // Clamp height into [-hh, +hh]
            float y = math.clamp(pL.y, -cyl.halfHeight, cyl.halfHeight);

            // Radial clamp in XZ
            float2 xz = new float2(pL.x, pL.z);
            float lenSq = math.lengthsq(xz);
            float r = cyl.radius;

            float2 xzClamped;
            if (lenSq <= 1e-12f)
            {
                // On axis: choose any point on radius (doesn't matter much for overlap case)
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool IsLocalPointOnFaceOnly(float3 pL, float3 half, float eps, out int faceAxis)
        {
            faceAxis = -1;

            int onCount = 0;
            int axis = -1;

            float dx = math.abs(math.abs(pL.x) - half.x);
            float dy = math.abs(math.abs(pL.y) - half.y);
            float dz = math.abs(math.abs(pL.z) - half.z);

            if (dx <= eps) { onCount++; axis = 0; }
            if (dy <= eps) { onCount++; axis = 1; }
            if (dz <= eps) { onCount++; axis = 2; }

            if (onCount == 1)
            {
                faceAxis = axis;
                return true;
            }

            // onCount 0 => interior (shouldn't happen for ClosestPointOnOBB unless box is degenerate)
            // onCount 2 => edge
            // onCount 3 => vertex
            return false;
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
