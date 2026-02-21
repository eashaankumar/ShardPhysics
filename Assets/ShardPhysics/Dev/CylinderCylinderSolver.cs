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

        // ------------------------------------------------------------
        // Public API (placeholder narrowphase)
        // ------------------------------------------------------------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Solve(in Cylinder a, in Cylinder b, out CylinderCylinderContactPoints cc)
        {
            cc = default;

            float3 aAxis = GetAxis(in a);
            float3 bAxis = GetAxis(in b);

            // 1) Narrow-phase overlap test (SAT) -> gives MTV axis (A->B) and depth
            if (!SAT_CylinderCylinder(in a, in b, aAxis, bAxis, out float3 mtvAxis, out float mtvDepth))
                return false;

            // Fill globals (even if we return early via cap-cap)
            cc.globalPenAxis = mtvAxis;
            cc.globalPenDepth = mtvDepth;

            // 2) Classify CAP-CAP:
            // For cap-cap, MTV axis must be axial-ish for BOTH cylinders (axes parallel-ish too)
            float da = math.abs(math.dot(mtvAxis, aAxis));
            float db = math.abs(math.dot(mtvAxis, bAxis));
            bool capCap = (da > 0.999f) && (db > 0.999f);

            if (capCap)
            {
                // Builds up to 4 rim points, clamped in the intersecting plane
                if (BuildCapCapManifold(in a, in b, aAxis, bAxis, mtvAxis, mtvDepth, out cc))
                    return true;

                // If something degenerate happens, fall back to "colliding but no manifold"
                // (you can choose to return false here, but SAT already said overlap)
                return false;
            }

            // 3) Side-Side

            // 4) Side-Cap

            // TODO: side-side + cap-side (later)
            // For now, just report collision with no manifold:
            return false;
        }

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

            if (cc.p1.depth >= 0f) tmp[count++] = cc.p1;
            if (cc.p2.depth >= 0f) tmp[count++] = cc.p2;
            if (cc.p3.depth >= 0f) tmp[count++] = cc.p3;
            if (cc.p4.depth >= 0f) tmp[count++] = cc.p4;

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
    }
}
