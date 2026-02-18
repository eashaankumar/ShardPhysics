using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Shard.Runtime
{
    public static class ShardCreateBody
    {
       
        public static void ComputeMassPropertiesFromColliders(
            NativeArray<ShardCollider> colliders,
            float desiredTotalMass,
            out float3x3 inertiaBody)
        {
            // Reuse span impl
            inertiaBody = ComputeMassPropertiesFromColliders((ReadOnlySpan<ShardCollider>)colliders, desiredTotalMass);
        }

        public static void ComputeMassPropertiesFromColliders(
            ReadOnlySpan<ShardCollider> colliders,
            float desiredTotalMass,
            out float3x3 inertiaBody)
        {
            inertiaBody = ComputeMassPropertiesFromColliders(colliders, desiredTotalMass);
        }

        private static float3x3 ComputeMassPropertiesFromColliders(ReadOnlySpan<ShardCollider> colliders, float desiredTotalMass)
        {
            // Degenerate: no colliders -> pick a tiny isotropic inertia so angular motion isn’t NaN.
            if (colliders.Length == 0)
            {
                float m = math.max(desiredTotalMass, 1e-8f);
                float k = 1e-3f; // arbitrary small radius^2 scale
                return Diagonal((2f / 5f) * m * k);
            }

            // Step 1: compute "raw" masses from density * volume (relative weights).
            // If all densities/volumes are zero, fall back to equal weights.
            float rawMassSum = 0f;

            // We don’t want to allocate; do two passes.
            for (int i = 0; i < colliders.Length; i++)
                rawMassSum += ComputeRawMass(colliders[i]);

            bool allZero = rawMassSum <= 1e-12f;
            float invCount = allZero ? (1f / math.max(1, colliders.Length)) : 0f;

            // Step 2: scale raw masses so total mass == desiredTotalMass
            float scale = allZero ? (desiredTotalMass * invCount) : (desiredTotalMass / rawMassSum);

            // Step 3: sum inertia in BODY frame, including local rotations + offsets (parallel axis).
            float3x3 I = float3x3.zero;

            for (int i = 0; i < colliders.Length; i++)
            {
                ShardCollider c = colliders[i];

                float mi = allZero ? scale : (ComputeRawMass(c) * scale);
                if (mi <= 0f) continue;

                // inertia about collider's own local COM (shape frame)
                float3x3 Ishape = ComputeShapeInertiaLocal(c, mi);

                // rotate inertia from collider local orientation into body frame
                float3x3 Rc = new float3x3(c.localPose.rotation);
                float3x3 Irot = math.mul(Rc, math.mul(Ishape, math.transpose(Rc)));

                // parallel axis: I += m * ( (d^2)I - outer(d,d) )
                float3 d = c.localPose.position; // collider COM offset in body frame
                float3x3 Ipa = ParallelAxis(mi, d);

                I += Irot + Ipa;
            }

            // Safety: if it went singular for some reason, pad diagonal.
            // (Helps if user gives zero sizes, etc.)
            I = EnsureSPDish(I, desiredTotalMass);

            return I;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float ComputeRawMass(in ShardCollider c)
        {
            float dens = math.max(c.density, 0f);

            // If density is 0, treat as 0 weight (caller can set all to 0 to force equal weights fallback).
            if (dens == 0f)
                return 0f;

            float vol = ComputeVolume(c);
            return dens * vol;
        }

        private static float ComputeVolume(in ShardCollider c)
        {
            switch (c.type)
            {
                case ShardColliderType.Sphere:
                    {
                        float r = math.max(c.radius, 0f);
                        return (4f / 3f) * math.PI * r * r * r;
                    }
                case ShardColliderType.Box:
                    {
                        float3 e = math.max(c.halfExtents, 0f);
                        float3 s = 2f * e;
                        return s.x * s.y * s.z;
                    }
                case ShardColliderType.Cylinder:
                    {
                        float r = math.max(c.radius, 0f);
                        float h = math.max(c.height, 0f);
                        return math.PI * r * r * h;
                    }
                case ShardColliderType.Capsule:
                    {
                        // Convention: c.height = cylinder height excluding hemispheres
                        float r = math.max(c.radius, 0f);
                        float h = math.max(c.height, 0f);
                        float volCyl = math.PI * r * r * h;
                        float volSphere = (4f / 3f) * math.PI * r * r * r;
                        return volCyl + volSphere; // two hemispheres = one sphere
                    }

                // Not supported yet -> return 0 so it doesn’t influence weighting
                default:
                    return 0f;
            }
        }

        private static float3x3 ComputeShapeInertiaLocal(in ShardCollider c, float mass)
        {
            switch (c.type)
            {
                case ShardColliderType.Sphere:
                    {
                        float r = math.max(c.radius, 0f);
                        float i = (2f / 5f) * mass * (r * r);
                        return Diagonal(i);
                    }

                case ShardColliderType.Box:
                    {
                        // box dims: w,h,d = 2*halfExtents
                        float3 he = math.max(c.halfExtents, 0f);
                        float w = 2f * he.x;
                        float h = 2f * he.y;
                        float d = 2f * he.z;

                        float ix = (1f / 12f) * mass * (h * h + d * d);
                        float iy = (1f / 12f) * mass * (w * w + d * d);
                        float iz = (1f / 12f) * mass * (w * w + h * h);
                        return Diagonal(ix, iy, iz);
                    }

                case ShardColliderType.Cylinder:
                    {
                        // Assume cylinder axis is local Y.
                        float r = math.max(c.radius, 0f);
                        float h = math.max(c.height, 0f);

                        float ix = (1f / 12f) * mass * (3f * r * r + h * h);
                        float iy = (1f / 2f) * mass * (r * r);
                        float iz = ix;
                        return Diagonal(ix, iy, iz);
                    }

                case ShardColliderType.Capsule:
                    {
                        // Approximate as cylinder + sphere (same radius), both centered at origin.
                        // Good enough for now; if you later want exact capsule inertia you can upgrade.
                        float r = math.max(c.radius, 0f);
                        float h = math.max(c.height, 0f);

                        float volCyl = math.PI * r * r * h;
                        float volSphere = (4f / 3f) * math.PI * r * r * r;

                        float totalVol = volCyl + volSphere;
                        if (totalVol <= 1e-12f)
                            return Diagonal(1e-6f * mass);

                        float mc = mass * (volCyl / totalVol);
                        float ms = mass * (volSphere / totalVol);

                        // Cylinder inertia about its COM
                        float3x3 Ic = Diagonal(
                            (1f / 12f) * mc * (3f * r * r + h * h),
                            (1f / 2f) * mc * (r * r),
                            (1f / 12f) * mc * (3f * r * r + h * h));

                        // Sphere inertia about its COM
                        float isph = (2f / 5f) * ms * (r * r);
                        float3x3 Is = Diagonal(isph);

                        return Ic + Is;
                    }

                default:
                    {
                        // Unsupported shapes: tiny inertia so it doesn’t explode, but won’t contribute meaningfully.
                        return Diagonal(1e-6f * mass);
                    }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3x3 ParallelAxis(float mass, float3 d)
        {
            // m * ( (d·d)I - outer(d,d) )
            float dd = math.dot(d, d);

            float3x3 I = Diagonal(dd);
            float3x3 outer = new float3x3(
                d.x * d.x, d.x * d.y, d.x * d.z,
                d.y * d.x, d.y * d.y, d.y * d.z,
                d.z * d.x, d.z * d.y, d.z * d.z);

            return mass * (I - outer);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3x3 EnsureSPDish(float3x3 I, float mass)
        {
            // Add a small diagonal pad relative to mass to avoid singular matrices.
            float pad = math.max(1e-6f * math.max(mass, 1e-8f), 1e-9f);
            I.c0.x += pad;
            I.c1.y += pad;
            I.c2.z += pad;
            return I;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3x3 Diagonal(float v) => new float3x3(
            v, 0f, 0f,
            0f, v, 0f,
            0f, 0f, v);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3x3 Diagonal(float x, float y, float z) => new float3x3(
            x, 0f, 0f,
            0f, y, 0f,
            0f, 0f, z);
    }
}
