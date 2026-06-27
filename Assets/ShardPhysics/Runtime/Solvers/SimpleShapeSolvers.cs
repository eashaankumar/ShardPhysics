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
        
        public static bool SolveBoxTriangle(Box box, Triangle triangle, out ContactPointManifold manifold)
        {
            manifold = default;

            // First try a terrain-friendly face contact path. This catches the most common
            // box-vs-heightfield case: one or more box vertices pass through the triangle plane.
            // The old SAT-only path could report tiny/unstable depths for coplanar triangle faces,
            // which let cubes fall through terrain while spheres/capsules still worked.
            if (TrySolveBoxTriangleVertexPlaneContacts(box, triangle, out manifold))
                return true;

            // Work in box-local space: box is centered at origin, axes are world-aligned,
            // triangle is transformed into that space.
            Triangle localTriangle = new Triangle(
                WorldToBoxLocal(triangle.a, box),
                WorldToBoxLocal(triangle.b, box),
                WorldToBoxLocal(triangle.c, box));

            float3 triEdge0 = localTriangle.b - localTriangle.a;
            float3 triEdge1 = localTriangle.c - localTriangle.b;
            float3 triEdge2 = localTriangle.a - localTriangle.c;

            float3 triNormalLocal = math.cross(triEdge0, localTriangle.c - localTriangle.a);

            float bestOverlap = float.PositiveInfinity;
            float3 bestAxisLocal = new float3(0f, 1f, 0f);

            // Box face axes.
            if (!TestBoxTriangleAxis(new float3(1f, 0f, 0f), box.halfExtents, localTriangle, ref bestOverlap, ref bestAxisLocal))
                return false;

            if (!TestBoxTriangleAxis(new float3(0f, 1f, 0f), box.halfExtents, localTriangle, ref bestOverlap, ref bestAxisLocal))
                return false;

            if (!TestBoxTriangleAxis(new float3(0f, 0f, 1f), box.halfExtents, localTriangle, ref bestOverlap, ref bestAxisLocal))
                return false;

            // Triangle face axis.
            if (!TestBoxTriangleAxis(triNormalLocal, box.halfExtents, localTriangle, ref bestOverlap, ref bestAxisLocal))
                return false;

            // Cross axes: box axes x triangle edges.
            float3 boxAxisX = new float3(1f, 0f, 0f);
            float3 boxAxisY = new float3(0f, 1f, 0f);
            float3 boxAxisZ = new float3(0f, 0f, 1f);

            if (!TestBoxTriangleAxis(math.cross(boxAxisX, triEdge0), box.halfExtents, localTriangle, ref bestOverlap, ref bestAxisLocal))
                return false;
            if (!TestBoxTriangleAxis(math.cross(boxAxisX, triEdge1), box.halfExtents, localTriangle, ref bestOverlap, ref bestAxisLocal))
                return false;
            if (!TestBoxTriangleAxis(math.cross(boxAxisX, triEdge2), box.halfExtents, localTriangle, ref bestOverlap, ref bestAxisLocal))
                return false;

            if (!TestBoxTriangleAxis(math.cross(boxAxisY, triEdge0), box.halfExtents, localTriangle, ref bestOverlap, ref bestAxisLocal))
                return false;
            if (!TestBoxTriangleAxis(math.cross(boxAxisY, triEdge1), box.halfExtents, localTriangle, ref bestOverlap, ref bestAxisLocal))
                return false;
            if (!TestBoxTriangleAxis(math.cross(boxAxisY, triEdge2), box.halfExtents, localTriangle, ref bestOverlap, ref bestAxisLocal))
                return false;

            if (!TestBoxTriangleAxis(math.cross(boxAxisZ, triEdge0), box.halfExtents, localTriangle, ref bestOverlap, ref bestAxisLocal))
                return false;
            if (!TestBoxTriangleAxis(math.cross(boxAxisZ, triEdge1), box.halfExtents, localTriangle, ref bestOverlap, ref bestAxisLocal))
                return false;
            if (!TestBoxTriangleAxis(math.cross(boxAxisZ, triEdge2), box.halfExtents, localTriangle, ref bestOverlap, ref bestAxisLocal))
                return false;

            float3 triCentroidLocal = (localTriangle.a + localTriangle.b + localTriangle.c) / 3f;

            // Normal must point from A -> B. A is box, B is triangle.
            if (math.dot(bestAxisLocal, triCentroidLocal) < 0f)
                bestAxisLocal = -bestAxisLocal;

            float3 normalWorld = math.normalize(math.mul(box.rotation, bestAxisLocal));
            float depth = math.max(bestOverlap, 0.001f);

            BuildBoxTriangleContactManifold(box, triangle, normalWorld, depth, out manifold);
            return true;
        }

        private static bool TrySolveBoxTriangleVertexPlaneContacts(
            Box box,
            Triangle triangle,
            out ContactPointManifold manifold)
        {
            manifold = default;

            float3 triNormal = TriangleNormal(triangle);
            float3 triCenter = (triangle.a + triangle.b + triangle.c) / 3f;

            // Contact normal must point from box -> triangle.
            float3 normal = triNormal;
            if (math.dot(normal, triCenter - box.center) < 0f)
                normal = -normal;

            const float contactSlop = 0.025f;
            const float minDepth = 0.001f;

            float deepest = 0f;

            for (int i = 0; i < 8; i++)
            {
                float3 vertex = GetBoxVertexWorld(box, i);
                float signedDepth = math.dot(vertex - triangle.a, normal);

                // signedDepth > 0 means the box vertex has crossed the triangle plane
                // in the box->triangle direction. Allow a small slop to create a contact
                // before numerical integration tunnels fully through the zero-thickness face.
                if (signedDepth < -contactSlop)
                    continue;

                float3 projected = vertex - normal * signedDepth;

                if (!PointInTriangle(projected, triangle.a, triangle.b, triangle.c))
                    continue;

                float depth = math.max(signedDepth, minDepth);
                AddBoxTriangleContact(ref manifold, projected, normal, depth);
                deepest = math.max(deepest, depth);

                if (manifold.numContactPoints == 4)
                    break;
            }

            if (manifold.numContactPoints == 0)
                return false;

            manifold.globalPenAxis = normal;
            manifold.globalPenDepth = math.max(deepest, minDepth);

            for (int i = 0; i < manifold.numContactPoints; i++)
            {
                ContactPoint cp = manifold[i];
                cp.normal = normal;
                cp.depth = manifold.globalPenDepth;
                manifold[i] = cp;
            }

            return true;
        }

        private static void AddBoxTriangleContact(
            ref ContactPointManifold manifold,
            float3 point,
            float3 normal,
            float depth)
        {
            int index = manifold.numContactPoints;
            if (index >= 4)
                return;

            manifold[index] = new ContactPoint
            {
                point = point,
                normal = normal,
                depth = depth
            };

            manifold.numContactPoints = index + 1;
        }
        private static void BuildBoxTriangleContactManifold(
            Box box,
            Triangle triangle,
            float3 normal,
            float depth,
            out ContactPointManifold manifold)
        {
            manifold = default;
            manifold.globalPenAxis = normal;
            manifold.globalPenDepth = depth;

            // 1) Triangle vertices inside the box.
            // This is important for wall/corner cases where the mesh penetrates the box,
            // but no box vertex projects neatly onto the triangle face.
            TryAddTriangleVertexInsideBoxContact(ref manifold, triangle.a, box, normal, depth);
            TryAddTriangleVertexInsideBoxContact(ref manifold, triangle.b, box, normal, depth);
            TryAddTriangleVertexInsideBoxContact(ref manifold, triangle.c, box, normal, depth);

            // 2) Box vertices projected onto the triangle plane.
            // This keeps the stable terrain-style contact behavior, but also works as
            // part of the generic manifold instead of being the only success path.
            const float projectionSlop = 0.05f;
            for (int i = 0; i < 8; i++)
            {
                float3 vertex = GetBoxVertexWorld(box, i);
                float signedDistance = math.dot(vertex - triangle.a, normal);

                if (math.abs(signedDistance) > depth + projectionSlop)
                    continue;

                float3 projected = vertex - normal * signedDistance;

                if (!PointInTriangle(projected, triangle.a, triangle.b, triangle.c))
                    continue;

                AddBoxTriangleContactUnique(ref manifold, projected, normal, depth);
            }

            // 3) Edge-edge closest contacts.
            // This catches vertical walls, sharp edges, corners, and thin triangles where
            // neither shape contributes a clean face contact.
            AddBoxTriangleEdgeContacts(ref manifold, box, triangle, normal, depth);

            // 4) Final fallback: one closest-feature contact.
            // SAT already proved overlap, so never return an empty manifold here.
            if (manifold.numContactPoints == 0)
            {
                float3 point = FindBoxTriangleContactPoint(box, triangle);
                AddBoxTriangleContactUnique(ref manifold, point, normal, depth);
            }

            for (int i = 0; i < manifold.numContactPoints; i++)
            {
                ContactPoint cp = manifold[i];
                cp.normal = normal;
                cp.depth = depth;
                manifold[i] = cp;
            }
        }

        private static void TryAddTriangleVertexInsideBoxContact(
            ref ContactPointManifold manifold,
            float3 triangleVertex,
            Box box,
            float3 normal,
            float depth)
        {
            float3 local = WorldToBoxLocal(triangleVertex, box);
            const float insideSlop = 0.001f;

            if (local.x < -box.halfExtents.x - insideSlop || local.x > box.halfExtents.x + insideSlop)
                return;
            if (local.y < -box.halfExtents.y - insideSlop || local.y > box.halfExtents.y + insideSlop)
                return;
            if (local.z < -box.halfExtents.z - insideSlop || local.z > box.halfExtents.z + insideSlop)
                return;

            AddBoxTriangleContactUnique(ref manifold, triangleVertex, normal, depth);
        }

        private static void AddBoxTriangleEdgeContacts(
            ref ContactPointManifold manifold,
            Box box,
            Triangle triangle,
            float3 normal,
            float depth)
        {
            float maxDistance = math.max(depth + 0.03f, 0.04f);
            float maxDistanceSq = maxDistance * maxDistance;

            for (int boxEdge = 0; boxEdge < 12; boxEdge++)
            {
                GetBoxEdgeWorld(box, boxEdge, out float3 boxA, out float3 boxB);

                TryAddEdgeEdgeContact(ref manifold, boxA, boxB, triangle.a, triangle.b, normal, depth, maxDistanceSq);
                TryAddEdgeEdgeContact(ref manifold, boxA, boxB, triangle.b, triangle.c, normal, depth, maxDistanceSq);
                TryAddEdgeEdgeContact(ref manifold, boxA, boxB, triangle.c, triangle.a, normal, depth, maxDistanceSq);

                if (manifold.numContactPoints == 4)
                    return;
            }
        }

        private static void TryAddEdgeEdgeContact(
            ref ContactPointManifold manifold,
            float3 boxA,
            float3 boxB,
            float3 triA,
            float3 triB,
            float3 normal,
            float depth,
            float maxDistanceSq)
        {
            ClosestPointsSegmentSegment(boxA, boxB, triA, triB, out float3 p, out float3 q);
            float distSq = math.lengthsq(q - p);

            if (distSq > maxDistanceSq)
                return;

            float3 point = (p + q) * 0.5f;
            AddBoxTriangleContactUnique(ref manifold, point, normal, depth);
        }

        private static void AddBoxTriangleContactUnique(
            ref ContactPointManifold manifold,
            float3 point,
            float3 normal,
            float depth)
        {
            const float duplicateDistanceSq = 0.0001f;

            for (int i = 0; i < manifold.numContactPoints; i++)
            {
                ContactPoint existing = manifold[i];

                if (math.lengthsq(existing.point - point) < duplicateDistanceSq)
                {
                    if (depth > existing.depth)
                    {
                        existing.point = point;
                        existing.normal = normal;
                        existing.depth = depth;
                        manifold[i] = existing;
                    }

                    return;
                }
            }

            if (manifold.numContactPoints < 4)
            {
                AddBoxTriangleContact(ref manifold, point, normal, depth);
                return;
            }

            // Keep the contact set spread out by replacing the point nearest to the new one.
            int nearestIndex = 0;
            float nearestDistanceSq = math.lengthsq(manifold[0].point - point);

            for (int i = 1; i < 4; i++)
            {
                float dSq = math.lengthsq(manifold[i].point - point);
                if (dSq < nearestDistanceSq)
                {
                    nearestDistanceSq = dSq;
                    nearestIndex = i;
                }
            }

            manifold[nearestIndex] = new ContactPoint
            {
                point = point,
                normal = normal,
                depth = depth
            };
        }


        private static bool PointInTriangle(float3 p, float3 a, float3 b, float3 c)
        {
            float3 v0 = c - a;
            float3 v1 = b - a;
            float3 v2 = p - a;

            float dot00 = math.dot(v0, v0);
            float dot01 = math.dot(v0, v1);
            float dot02 = math.dot(v0, v2);
            float dot11 = math.dot(v1, v1);
            float dot12 = math.dot(v1, v2);

            float denom = dot00 * dot11 - dot01 * dot01;
            if (math.abs(denom) < EPSILON)
                return false;

            float invDenom = 1f / denom;
            float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            const float eps = 1e-4f;
            return u >= -eps && v >= -eps && u + v <= 1f + eps;
        }

        public static bool SolveTriangleBox(Triangle triangle, Box box, out ContactPointManifold manifold)
        {
            if (!SolveBoxTriangle(box, triangle, out manifold))
                return false;

            FlipManifold(ref manifold);
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
        
        private static bool TestBoxTriangleAxis(
            float3 axis,
            float3 boxHalfExtents,
            Triangle localTriangle,
            ref float bestOverlap,
            ref float3 bestAxis)
        {
            float axisLenSq = math.lengthsq(axis);

            if (axisLenSq < EPSILON)
                return true;

            axis *= math.rsqrt(axisLenSq);

            ProjectBoxOnAxis(boxHalfExtents, axis, out float boxMin, out float boxMax);
            ProjectTriangleOnAxis(localTriangle, axis, out float triMin, out float triMax);

            float overlap = math.min(boxMax, triMax) - math.max(boxMin, triMin);

            if (overlap < 0f)
                return false;

            if (overlap < bestOverlap)
            {
                bestOverlap = overlap;
                bestAxis = axis;
            }

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ProjectBoxOnAxis(float3 halfExtents, float3 axis, out float min, out float max)
        {
            float r =
                halfExtents.x * math.abs(axis.x) +
                halfExtents.y * math.abs(axis.y) +
                halfExtents.z * math.abs(axis.z);

            min = -r;
            max = r;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ProjectTriangleOnAxis(Triangle triangle, float3 axis, out float min, out float max)
        {
            float p0 = math.dot(triangle.a, axis);
            float p1 = math.dot(triangle.b, axis);
            float p2 = math.dot(triangle.c, axis);

            min = math.min(p0, math.min(p1, p2));
            max = math.max(p0, math.max(p1, p2));
        }

        private static float3 FindBoxTriangleContactPoint(Box box, Triangle triangle)
        {
            float bestDistSq = float.PositiveInfinity;
            float3 bestPoint = triangle.a;

            // Triangle vertices against box.
            CheckTriangleVertexBoxPoint(triangle.a, box, ref bestPoint, ref bestDistSq);
            CheckTriangleVertexBoxPoint(triangle.b, box, ref bestPoint, ref bestDistSq);
            CheckTriangleVertexBoxPoint(triangle.c, box, ref bestPoint, ref bestDistSq);

            // Box vertices against triangle.
            for (int i = 0; i < 8; i++)
            {
                float3 boxVertex = GetBoxVertexWorld(box, i);
                float3 triPoint = ClosestPointOnTriangle(boxVertex, triangle.a, triangle.b, triangle.c);
                float dSq = math.lengthsq(triPoint - boxVertex);

                if (dSq < bestDistSq)
                {
                    bestDistSq = dSq;
                    bestPoint = (boxVertex + triPoint) * 0.5f;
                }
            }

            return bestPoint;
        }

        private static void CheckTriangleVertexBoxPoint(
            float3 triangleVertex,
            Box box,
            ref float3 bestPoint,
            ref float bestDistSq)
        {
            float3 boxPoint = ClosestPointOnBox(triangleVertex, box);
            float dSq = math.lengthsq(boxPoint - triangleVertex);

            if (dSq < bestDistSq)
            {
                bestDistSq = dSq;
                bestPoint = (triangleVertex + boxPoint) * 0.5f;
            }
        }

        private static void GetBoxEdgeWorld(Box box, int edgeIndex, out float3 a, out float3 b)
        {
            switch (edgeIndex)
            {
                case 0:  a = GetBoxVertexWorld(box, 0); b = GetBoxVertexWorld(box, 1); return;
                case 1:  a = GetBoxVertexWorld(box, 2); b = GetBoxVertexWorld(box, 3); return;
                case 2:  a = GetBoxVertexWorld(box, 4); b = GetBoxVertexWorld(box, 5); return;
                case 3:  a = GetBoxVertexWorld(box, 6); b = GetBoxVertexWorld(box, 7); return;

                case 4:  a = GetBoxVertexWorld(box, 0); b = GetBoxVertexWorld(box, 2); return;
                case 5:  a = GetBoxVertexWorld(box, 1); b = GetBoxVertexWorld(box, 3); return;
                case 6:  a = GetBoxVertexWorld(box, 4); b = GetBoxVertexWorld(box, 6); return;
                case 7:  a = GetBoxVertexWorld(box, 5); b = GetBoxVertexWorld(box, 7); return;

                case 8:  a = GetBoxVertexWorld(box, 0); b = GetBoxVertexWorld(box, 4); return;
                case 9:  a = GetBoxVertexWorld(box, 1); b = GetBoxVertexWorld(box, 5); return;
                case 10: a = GetBoxVertexWorld(box, 2); b = GetBoxVertexWorld(box, 6); return;
                default: a = GetBoxVertexWorld(box, 3); b = GetBoxVertexWorld(box, 7); return;
            }
        }

        private static float3 GetBoxVertexWorld(Box box, int index)
        {
            float3 local = new float3(
                (index & 1) == 0 ? -box.halfExtents.x : box.halfExtents.x,
                (index & 2) == 0 ? -box.halfExtents.y : box.halfExtents.y,
                (index & 4) == 0 ? -box.halfExtents.z : box.halfExtents.z);

            return BoxLocalToWorld(local, box);
        }
    }
}