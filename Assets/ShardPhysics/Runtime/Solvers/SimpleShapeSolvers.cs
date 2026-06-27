using System.Runtime.CompilerServices;
using Unity.Mathematics;
using UnityEngine;

namespace Shard.Runtime.Solvers
{
    public static class SimpleShapeSolvers
    {
        private const float EPSILON = 1e-6f;

        public struct Sphere
        {
            public float3 center;
            public float radius;

            public Sphere(float3 center, float radius)
            {
                this.center = center;
                this.radius = radius;
            }
        }

        public struct Capsule
        {
            public float3 center;
            public quaternion rotation;
            public float halfHeight;
            public float radius;

            public Capsule(float3 center, quaternion rotation, float halfHeight, float radius)
            {
                this.center = center;
                this.rotation = rotation;
                this.halfHeight = halfHeight;
                this.radius = radius;
            }

            public float3 Axis => math.mul(rotation, new float3(0f, 1f, 0f));

            public float3 PointA => center - Axis * halfHeight;
            public float3 PointB => center + Axis * halfHeight;
        }

        public struct Box
        {
            public float3 center;
            public quaternion rotation;
            public float3 halfExtents;

            public Box(float3 center, quaternion rotation, float3 halfExtents)
            {
                this.center = center;
                this.rotation = rotation;
                this.halfExtents = halfExtents;
            }
        }

        public struct Triangle
        {
            public float3 a;
            public float3 b;
            public float3 c;

            public Triangle(float3 a, float3 b, float3 c)
            {
                this.a = a;
                this.b = b;
                this.c = c;
            }
        }

        public static bool SolveSphereSphere(Sphere a, Sphere b, out ContactPointManifold manifold)
        {
            manifold = default;

            float3 delta = b.center - a.center;
            float distSq = math.lengthsq(delta);
            float radiusSum = a.radius + b.radius;

            if (distSq > radiusSum * radiusSum)
                return false;

            float dist = math.sqrt(math.max(distSq, EPSILON));
            float3 normal = distSq > EPSILON ? delta / dist : new float3(0f, 1f, 0f);
            float depth = radiusSum - dist;
            float3 point = a.center + normal * (a.radius - depth * 0.5f);

            SetSingleContact(ref manifold, point, normal, depth);
            return true;
        }

        public static bool SolveSphereBox(Sphere sphere, Box box, out ContactPointManifold manifold)
        {
            manifold = default;

            float3 localSphere = WorldToBoxLocal(sphere.center, box);
            float3 closestLocal = math.clamp(localSphere, -box.halfExtents, box.halfExtents);
            float3 closestWorld = BoxLocalToWorld(closestLocal, box);

            float3 delta = sphere.center - closestWorld;
            float distSq = math.lengthsq(delta);

            if (distSq > sphere.radius * sphere.radius)
                return false;

            float3 normal;
            float depth;
            float3 point;

            if (distSq > EPSILON)
            {
                float dist = math.sqrt(distSq);

                // normal is sphere -> box? We need A -> B.
                // A is sphere, B is box, so direction is from sphere into box.
                normal = -delta / dist;
                depth = sphere.radius - dist;
                point = closestWorld;
            }
            else
            {
                float3 absLocal = math.abs(localSphere);
                float3 distanceToFace = box.halfExtents - absLocal;

                int axis = 0;
                if (distanceToFace.y < distanceToFace.x) axis = 1;
                if (distanceToFace.z < distanceToFace[axis]) axis = 2;

                float sign = localSphere[axis] >= 0f ? 1f : -1f;
                float3 localNormal = float3.zero;
                localNormal[axis] = sign;

                float3 worldNormalBoxToSphere = math.mul(box.rotation, localNormal);

                normal = -worldNormalBoxToSphere;
                depth = sphere.radius + distanceToFace[axis];

                float3 faceLocal = localSphere;
                faceLocal[axis] = box.halfExtents[axis] * sign;
                point = BoxLocalToWorld(faceLocal, box);
            }

            SetSingleContact(ref manifold, point, normal, depth);
            return true;
        }

        public static bool SolveBoxSphere(Box box, Sphere sphere, out ContactPointManifold manifold)
        {
            if (!SolveSphereBox(sphere, box, out manifold))
                return false;

            FlipManifold(ref manifold);
            return true;
        }

        public static bool SolveSphereCapsule(Sphere sphere, Capsule capsule, out ContactPointManifold manifold)
        {
            manifold = default;

            float3 closest = ClosestPointOnSegment(sphere.center, capsule.PointA, capsule.PointB);
            float3 delta = closest - sphere.center;

            float radiusSum = sphere.radius + capsule.radius;
            float distSq = math.lengthsq(delta);

            if (distSq > radiusSum * radiusSum)
                return false;

            float dist = math.sqrt(math.max(distSq, EPSILON));
            float3 normal = distSq > EPSILON ? delta / dist : capsule.Axis;
            float depth = radiusSum - dist;
            float3 point = sphere.center + normal * (sphere.radius - depth * 0.5f);

            SetSingleContact(ref manifold, point, normal, depth);
            return true;
        }

        public static bool SolveCapsuleSphere(Capsule capsule, Sphere sphere, out ContactPointManifold manifold)
        {
            if (!SolveSphereCapsule(sphere, capsule, out manifold))
                return false;

            FlipManifold(ref manifold);
            return true;
        }

        public static bool SolveCapsuleCapsule(Capsule a, Capsule b, out ContactPointManifold manifold)
        {
            manifold = default;

            ClosestPointsSegmentSegment(
                a.PointA,
                a.PointB,
                b.PointA,
                b.PointB,
                out float3 pointA,
                out float3 pointB);

            float3 delta = pointB - pointA;
            float radiusSum = a.radius + b.radius;
            float distSq = math.lengthsq(delta);

            if (distSq > radiusSum * radiusSum)
                return false;

            float dist = math.sqrt(math.max(distSq, EPSILON));
            float3 normal = distSq > EPSILON ? delta / dist : a.Axis;
            float depth = radiusSum - dist;
            float3 point = pointA + normal * (a.radius - depth * 0.5f);

            SetSingleContact(ref manifold, point, normal, depth);
            return true;
        }

        public static bool SolveCapsuleBox(Capsule capsule, Box box, out ContactPointManifold manifold)
        {
            manifold = default;

            // First-pass approximation:
            // find closest point on capsule segment to box, then treat capsule as sphere at that point.
            float3 p0 = capsule.PointA;
            float3 p1 = capsule.PointB;

            float3 bestSegmentPoint = p0;
            float3 bestBoxPoint = ClosestPointOnBox(p0, box);
            float bestDistSq = math.lengthsq(bestBoxPoint - p0);

            const int samples = 5;
            for (int i = 1; i < samples; i++)
            {
                float t = i / (float)(samples - 1);
                float3 p = math.lerp(p0, p1, t);
                float3 q = ClosestPointOnBox(p, box);
                float dSq = math.lengthsq(q - p);

                if (dSq < bestDistSq)
                {
                    bestDistSq = dSq;
                    bestSegmentPoint = p;
                    bestBoxPoint = q;
                }
            }

            float3 delta = bestBoxPoint - bestSegmentPoint;

            if (bestDistSq > capsule.radius * capsule.radius)
                return false;

            float3 normal;
            float depth;

            if (bestDistSq > EPSILON)
            {
                float dist = math.sqrt(bestDistSq);
                normal = delta / dist;
                depth = capsule.radius - dist;
            }
            else
            {
                float3 local = WorldToBoxLocal(bestSegmentPoint, box);
                float3 absLocal = math.abs(local);
                float3 distanceToFace = box.halfExtents - absLocal;

                int axis = 0;
                if (distanceToFace.y < distanceToFace.x) axis = 1;
                if (distanceToFace.z < distanceToFace[axis]) axis = 2;

                float sign = local[axis] >= 0f ? 1f : -1f;
                float3 localNormal = float3.zero;
                localNormal[axis] = sign;

                // box outward normal; capsule -> box is opposite.
                normal = -math.mul(box.rotation, localNormal);
                depth = capsule.radius + distanceToFace[axis];
                bestBoxPoint = ClosestPointOnBox(bestSegmentPoint + normal * capsule.radius, box);
            }

            SetSingleContact(ref manifold, bestBoxPoint, normal, depth);
            return true;
        }

        public static bool SolveBoxCapsule(Box box, Capsule capsule, out ContactPointManifold manifold)
        {
            if (!SolveCapsuleBox(capsule, box, out manifold))
                return false;

            FlipManifold(ref manifold);
            return true;
        }

        public static bool SolveSphereTriangle(Sphere sphere, Triangle triangle, out ContactPointManifold manifold)
        {
            manifold = default;

            float3 closest = ClosestPointOnTriangle(sphere.center, triangle.a, triangle.b, triangle.c);
            float3 delta = closest - sphere.center;
            float distSq = math.lengthsq(delta);

            if (distSq > sphere.radius * sphere.radius)
                return false;

            float3 triNormal = TriangleNormal(triangle);
            float3 normal;
            float depth;

            if (distSq > EPSILON)
            {
                float dist = math.sqrt(distSq);
                normal = delta / dist;
                depth = sphere.radius - dist;
            }
            else
            {
                normal = math.dot(triNormal, sphere.center - triangle.a) >= 0f ? -triNormal : triNormal;
                depth = sphere.radius;
            }

            SetSingleContact(ref manifold, closest, normal, depth);
            return true;
        }

        public static bool SolveCapsuleTriangle(Capsule capsule, Triangle triangle, out ContactPointManifold manifold)
        {
            manifold = default;

            float3 segA = capsule.PointA;
            float3 segB = capsule.PointB;

            float3 bestCapsulePoint = segA;
            float3 bestTrianglePoint = ClosestPointOnTriangle(segA, triangle.a, triangle.b, triangle.c);
            float bestDistSq = math.lengthsq(bestTrianglePoint - bestCapsulePoint);

            CheckSegmentTriangleCandidate(segB, triangle, ref bestCapsulePoint, ref bestTrianglePoint, ref bestDistSq);

            CheckEdgePair(segA, segB, triangle.a, triangle.b, ref bestCapsulePoint, ref bestTrianglePoint, ref bestDistSq);
            CheckEdgePair(segA, segB, triangle.b, triangle.c, ref bestCapsulePoint, ref bestTrianglePoint, ref bestDistSq);
            CheckEdgePair(segA, segB, triangle.c, triangle.a, ref bestCapsulePoint, ref bestTrianglePoint, ref bestDistSq);

            if (bestDistSq > capsule.radius * capsule.radius)
                return false;

            float3 delta = bestTrianglePoint - bestCapsulePoint;
            float3 normal;
            float depth;

            if (bestDistSq > EPSILON)
            {
                float dist = math.sqrt(bestDistSq);
                normal = delta / dist;
                depth = capsule.radius - dist;
            }
            else
            {
                normal = TriangleNormal(triangle);
                if (math.dot(normal, capsule.center - triangle.a) > 0f)
                    normal = -normal;

                depth = capsule.radius;
            }

            SetSingleContact(ref manifold, bestTrianglePoint, normal, depth);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FlipManifold(ref ContactPointManifold manifold)
        {
            manifold.globalPenAxis = -manifold.globalPenAxis;

            for (int i = 0; i < manifold.numContactPoints; i++)
            {
                ContactPoint cp = manifold[i];
                cp.normal = -cp.normal;
                manifold[i] = cp;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void SetSingleContact(ref ContactPointManifold manifold, float3 point, float3 normal, float depth)
        {
            manifold.numContactPoints = 1;
            manifold.globalPenAxis = normal;
            manifold.globalPenDepth = depth;
            manifold.p1 = new ContactPoint
            {
                point = point,
                normal = normal,
                depth = depth
            };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 WorldToBoxLocal(float3 point, Box box)
        {
            return math.mul(math.inverse(box.rotation), point - box.center);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 BoxLocalToWorld(float3 point, Box box)
        {
            return box.center + math.mul(box.rotation, point);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ClosestPointOnBox(float3 point, Box box)
        {
            float3 local = WorldToBoxLocal(point, box);
            float3 clamped = math.clamp(local, -box.halfExtents, box.halfExtents);
            return BoxLocalToWorld(clamped, box);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 ClosestPointOnSegment(float3 point, float3 a, float3 b)
        {
            float3 ab = b - a;
            float denom = math.lengthsq(ab);

            if (denom < EPSILON)
                return a;

            float t = math.dot(point - a, ab) / denom;
            return a + math.saturate(t) * ab;
        }

        private static void ClosestPointsSegmentSegment(
            float3 p1,
            float3 q1,
            float3 p2,
            float3 q2,
            out float3 c1,
            out float3 c2)
        {
            float3 d1 = q1 - p1;
            float3 d2 = q2 - p2;
            float3 r = p1 - p2;

            float a = math.dot(d1, d1);
            float e = math.dot(d2, d2);
            float f = math.dot(d2, r);

            float s;
            float t;

            if (a <= EPSILON && e <= EPSILON)
            {
                c1 = p1;
                c2 = p2;
                return;
            }

            if (a <= EPSILON)
            {
                s = 0f;
                t = math.saturate(f / e);
            }
            else
            {
                float c = math.dot(d1, r);

                if (e <= EPSILON)
                {
                    t = 0f;
                    s = math.saturate(-c / a);
                }
                else
                {
                    float b = math.dot(d1, d2);
                    float denom = a * e - b * b;

                    if (denom != 0f)
                        s = math.saturate((b * f - c * e) / denom);
                    else
                        s = 0f;

                    t = (b * s + f) / e;

                    if (t < 0f)
                    {
                        t = 0f;
                        s = math.saturate(-c / a);
                    }
                    else if (t > 1f)
                    {
                        t = 1f;
                        s = math.saturate((b - c) / a);
                    }
                }
            }

            c1 = p1 + d1 * s;
            c2 = p2 + d2 * t;
        }

        private static float3 ClosestPointOnTriangle(float3 p, float3 a, float3 b, float3 c)
        {
            float3 ab = b - a;
            float3 ac = c - a;
            float3 ap = p - a;

            float d1 = math.dot(ab, ap);
            float d2 = math.dot(ac, ap);

            if (d1 <= 0f && d2 <= 0f)
                return a;

            float3 bp = p - b;
            float d3 = math.dot(ab, bp);
            float d4 = math.dot(ac, bp);

            if (d3 >= 0f && d4 <= d3)
                return b;

            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float v = d1 / (d1 - d3);
                return a + v * ab;
            }

            float3 cp = p - c;
            float d5 = math.dot(ab, cp);
            float d6 = math.dot(ac, cp);

            if (d6 >= 0f && d5 <= d6)
                return c;

            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float w = d2 / (d2 - d6);
                return a + w * ac;
            }

            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            {
                float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                return b + w * (c - b);
            }

            float denom = 1f / (va + vb + vc);
            float vFinal = vb * denom;
            float wFinal = vc * denom;

            return a + ab * vFinal + ac * wFinal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 TriangleNormal(Triangle triangle)
        {
            float3 n = math.cross(triangle.b - triangle.a, triangle.c - triangle.a);
            float lenSq = math.lengthsq(n);

            if (lenSq < EPSILON)
                return new float3(0f, 1f, 0f);

            return n * math.rsqrt(lenSq);
        }

        private static void CheckSegmentTriangleCandidate(
            float3 capsulePoint,
            Triangle triangle,
            ref float3 bestCapsulePoint,
            ref float3 bestTrianglePoint,
            ref float bestDistSq)
        {
            float3 triPoint = ClosestPointOnTriangle(capsulePoint, triangle.a, triangle.b, triangle.c);
            float dSq = math.lengthsq(triPoint - capsulePoint);

            if (dSq < bestDistSq)
            {
                bestDistSq = dSq;
                bestCapsulePoint = capsulePoint;
                bestTrianglePoint = triPoint;
            }
        }

        private static void CheckEdgePair(
            float3 segA,
            float3 segB,
            float3 edgeA,
            float3 edgeB,
            ref float3 bestCapsulePoint,
            ref float3 bestTrianglePoint,
            ref float bestDistSq)
        {
            ClosestPointsSegmentSegment(segA, segB, edgeA, edgeB, out float3 p, out float3 q);
            float dSq = math.lengthsq(q - p);

            if (dSq < bestDistSq)
            {
                bestDistSq = dSq;
                bestCapsulePoint = p;
                bestTrianglePoint = q;
            }
        }
    }
}