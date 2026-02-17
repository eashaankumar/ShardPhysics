using Shard.Manifolds;
using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Shard
{
    public sealed class PhysicsWorld : IDisposable
    {
        public readonly Allocator Allocator;
        public int BodyCount => m_bodyCount;

        // Optional: float32 local-origin trick for server
        public double3 WorldOrigin; // global origin in double space
        public uint WorldId;

        // ---- Bodies (SoA) ----
        internal NativeList<Body> Bodies;
        public NativeList<Pose> Poses;
        internal NativeList<Velocity> Velocities;
        internal NativeList<MassProperties> Masses;
        internal NativeList<Damping> Dampings;
        internal NativeList<byte> BodyAlive; // 0 = free, 1 = alive

        // Sleeping
        internal NativeList<float> SleepTime;   // seconds under thresholds
        internal NativeList<byte> IsSleeping;   // 0/1

        // Warm-start (normal impulses by feature id)
        private NativeParallelHashMap<ulong, float> _warmN; // key=(min<<32)^max^(fid<<1)

        // Bookkeeping
        internal NativeList<int> FreeBodySlots; // free-list for BodyId reuse
        internal int m_bodyCount;

        // ---- Colliders ----
        public ColliderStore Colliders;

        // ---- Broadphase / Narrowphase / Solver ----
        internal Broadphase Broadphase;
        internal Narrowphase Narrowphase;
        internal ConstraintGraph Constraints;
        internal Solver Solver;

        // ---- Scratch (per-world) ----
        internal RewindableAllocator Scratch; // per-step temp allocations

        public PhysicsWorld(int bodyCapacity, int colliderCapacity, Allocator allocator, uint worldId = 1)
        {
            Allocator = allocator;
            WorldId = worldId;

            Bodies = new NativeList<Body>(bodyCapacity, allocator);
            Poses = new NativeList<Pose>(bodyCapacity, allocator);
            Velocities = new NativeList<Velocity>(bodyCapacity, allocator);
            Masses = new NativeList<MassProperties>(bodyCapacity, allocator);
            Dampings = new NativeList<Damping>(bodyCapacity, allocator);
            FreeBodySlots = new NativeList<int>(allocator);

            BodyAlive = new NativeList<byte>(bodyCapacity, allocator);
            SleepTime = new NativeList<float>(bodyCapacity, allocator);
            IsSleeping = new NativeList<byte>(bodyCapacity, allocator);

            Colliders = new ColliderStore(colliderCapacity, allocator);

            Broadphase = new Broadphase(allocator);
            Narrowphase = new Narrowphase(allocator);
            Constraints = new ConstraintGraph(allocator);
            Solver = new Solver(allocator);

            _warmN = new NativeParallelHashMap<ulong, float>(1024, allocator);

            Scratch = new RewindableAllocator();
            const int scratchBytes = 256 * 1024;
            Scratch.Initialize(scratchBytes);
        }

        public void Dispose()
        {
            Solver.Dispose();
            Constraints.Dispose();
            Narrowphase.Dispose();
            Broadphase.Dispose();
            Colliders.Dispose();

            Scratch.Dispose();

            if (_warmN.IsCreated) _warmN.Dispose();

            FreeBodySlots.Dispose();
            Dampings.Dispose();
            Masses.Dispose();
            Velocities.Dispose();
            Poses.Dispose();
            Bodies.Dispose();

            BodyAlive.Dispose();
            SleepTime.Dispose();
            IsSleeping.Dispose();
        }

        // -------------------------
        // Bodies: allocation + API
        // -------------------------

        private BodyId AllocateBody()
        {
            int index;
            if (FreeBodySlots.Length > 0)
            {
                index = FreeBodySlots[^1];
                FreeBodySlots.RemoveAtSwapBack(FreeBodySlots.Length - 1);
                BodyAlive[index] = 1;

                // reset sleeping state
                IsSleeping[index] = 0;
                SleepTime[index] = 0f;
            }
            else
            {
                index = Bodies.Length;

                Bodies.Add(default);
                Poses.Add(default);
                Velocities.Add(default);
                Masses.Add(default);
                Dampings.Add(default);
                BodyAlive.Add(1);

                IsSleeping.Add(0);
                SleepTime.Add(0f);

                m_bodyCount++;
            }

            EnsureBroadphaseCapacity(index + 1);
            return new BodyId(index);
        }

        public BodyId AddBody(
            MotionType motionType,
            Pose pose,
            Velocity velocity,
            MassProperties mass,
            Damping damping,
            ColliderHandle collider,
            ushort materialId = 0,
            ushort flags = 0)
        {
            var id = AllocateBody();

            Bodies[id.Value] = new Body
            {
                MotionType = motionType,
                Collider = collider,
                MaterialId = materialId,
                Flags = flags
            };

            Poses[id.Value] = pose;
            Velocities[id.Value] = velocity;
            Masses[id.Value] = mass;
            Dampings[id.Value] = damping;

            // New dynamic bodies start awake
            IsSleeping[id.Value] = 0;
            SleepTime[id.Value] = 0f;

            // Placeholder: keep BodyToProxy in sync (real tree comes in step 4)
            SyncProxyForBody(id);

            return id;
        }

        public void RemoveBody(BodyId id)
        {
            if (!id.IsValid || (uint)id.Value >= (uint)Bodies.Length)
                return;

            BodyAlive[id.Value] = 0;

            // Placeholder proxy removal
            EnsureBroadphaseCapacity(id.Value + 1);
            Broadphase.BodyToProxy[id.Value] = -1;

            Bodies[id.Value] = default;
            Poses[id.Value] = default;
            Velocities[id.Value] = default;
            Masses[id.Value] = default;
            Dampings[id.Value] = default;

            IsSleeping[id.Value] = 0;
            SleepTime[id.Value] = 0f;

            FreeBodySlots.Add(id.Value);
        }

        public void SetPose(BodyId id, Pose pose)
        {
            Poses[id.Value] = pose;
            SyncProxyForBody(id);

            WakeBody(id.Value);
        }

        public void SetCollider(BodyId id, ColliderHandle collider)
        {
            var b = Bodies[id.Value];
            b.Collider = collider;
            Bodies[id.Value] = b;

            SyncProxyForBody(id);

            WakeBody(id.Value);
        }

        // -------------------------
        // Broadphase placeholder plumbing
        // -------------------------

        private void EnsureBroadphaseCapacity(int bodyCount)
        {
            Broadphase.EnsureBodyCapacity(bodyCount);
        }

        private void SyncProxyForBody(BodyId id)
        {
            Broadphase.EnsureBodyCapacity(id.Value + 1);

            var body = Bodies[id.Value];

            int proxy = Broadphase.BodyToProxy[id.Value];

            // If collider invalid, remove proxy if it exists
            if (!Colliders.IsValid(body.Collider))
            {
                if (proxy != -1)
                {
                    Broadphase.RemoveProxy(proxy);
                    Broadphase.BodyToProxy[id.Value] = -1;
                }
                return;
            }

            ref var header = ref Colliders.Resolve(body.Collider);
            Aabb worldAabb = WorldAabb.FromBodyPose(header.LocalAabb, Poses[id.Value]);

            if (proxy == -1)
                Broadphase.CreateProxy(id, worldAabb);
            else
                Broadphase.UpdateProxy(proxy, worldAabb);
        }

        // =========================
        // PhysicsWorld.Step (rewritten)
        // Key change:
        // 1) Integrate velocities
        // 2) Integrate poses (so overlaps actually exist THIS frame)
        // 3) Solve contacts using the UPDATED poses (no 1-frame-late contacts)
        // 4) Sleeping update
        // 5) Update proxies (job)
        // =========================
        public JobHandle Step(float dt, float3 gravity, JobHandle deps = default)
        {
            Broadphase.EnsureBodyCapacity(Bodies.Length);

            // 1) Integrate velocities
            var integrateV = new IntegrateVelocitiesJob
            {
                Dt = dt,
                Gravity = gravity,
                BodyAlive = BodyAlive.AsArray(),
                IsSleeping = IsSleeping.AsArray(),
                Bodies = Bodies.AsArray(),
                Velocities = Velocities.AsArray(),
                Masses = Masses.AsArray(),
                Dampings = Dampings.AsArray()
            };

            JobHandle hV = Unity.Jobs.IJobForExtensions.ScheduleParallel(integrateV, Bodies.Length, 64, deps);

            // 2) Integrate poses (using current velocities)
            var integrateP = new IntegratePosesJob
            {
                Dt = dt,
                BodyAlive = BodyAlive.AsArray(),
                IsSleeping = IsSleeping.AsArray(),
                Bodies = Bodies.AsArray(),
                Poses = Poses.AsArray(),
                Velocities = Velocities.AsArray(),
            };

            JobHandle hP = Unity.Jobs.IJobForExtensions.ScheduleParallel(integrateP, Bodies.Length, 64, hV);

            // 3) Robust contact correction (needs current poses)
            hP.Complete();
            SolvePostStabilize_BoxBoxOnly(dt, iterations: 4);

            // 4) Sleeping after stabilization
            UpdateSleeping(dt);

            // 5) Update proxies
            var update = new UpdateProxiesJob
            {
                BodyAlive = BodyAlive.AsArray(),
                Bodies = Bodies.AsArray(),
                Poses = Poses.AsArray(),
                ColliderSlots = Colliders.Slots.AsArray(),
                BodyToProxy = Broadphase.BodyToProxy.AsArray(),
                ProxyAabbs = Broadphase.ProxyAabbs.AsArray()
            };

            JobHandle hU = update.ScheduleParallel(Bodies.Length, 64, default);
            return hU;
        }

        // =========================
        // SolveGreedy_BoxBoxOnly (rewritten)
        // Key changes:
        // - Uses manifold contract directly: n = m.Normal (already A->B)
        // - Sequential impulses (accumN/accumT per contact point)
        // - Baumgarte velocity bias for resting stability
        // - Restitution only on impact-like contacts (and shallow penetration)
        // - PositionProject only AFTER velocity iterations (small) as a safety net
        // =========================
        private void SolvePostStabilize_BoxBoxOnly(float dt, int iterations)
        {
            const float slop = 0.001f;           // meters
            const float percent = 0.80f;         // strong separation
            const float maxCorr = 0.08f;         // meters per iteration cap

            const float bounceThreshold = 0.25f; // m/s (only bounce on real impacts)

            for (int it = 0; it < iterations; it++)
            {
                for (int ia = 0; ia < Bodies.Length; ia++)
                {
                    if (BodyAlive[ia] == 0) continue;

                    var bodyA = Bodies[ia];
                    if (!Colliders.IsValid(bodyA.Collider)) continue;

                    ref var ha = ref Colliders.Resolve(bodyA.Collider);
                    if (ha.Type != ColliderType.Box) continue;

                    bool dynA = bodyA.MotionType == MotionType.Dynamic;
                    float invMassA = dynA ? Masses[ia].InverseMass : 0f;
                    BoxCollider boxA = Colliders.Boxes[ha.PayloadIndex];

                    for (int ib = ia + 1; ib < Bodies.Length; ib++)
                    {
                        if (BodyAlive[ib] == 0) continue;

                        var bodyB = Bodies[ib];
                        if (!Colliders.IsValid(bodyB.Collider)) continue;

                        ref var hb = ref Colliders.Resolve(bodyB.Collider);
                        if (hb.Type != ColliderType.Box) continue;

                        if (bodyA.MotionType == MotionType.Static && bodyB.MotionType == MotionType.Static)
                            continue;

                        bool dynB = bodyB.MotionType == MotionType.Dynamic;
                        float invMassB = dynB ? Masses[ib].InverseMass : 0f;

                        float invMassSum = invMassA + invMassB;
                        if (invMassSum <= 0f) continue;

                        BoxCollider boxB = Colliders.Boxes[hb.PayloadIndex];

                        Pose poseA = Poses[ia];
                        Pose poseB = Poses[ib];

                        ContactManifold m = default;
                        if (!Shard.Manifolds.BoxBoxManifold.Generate(in boxA, in poseA, in boxB, in poseB, ref m))
                            continue;

                        if (m.PointCount <= 0) continue;

                        float3 n = m.Normal; // your manifold guarantees A->B

                        // Use max penetration across contacts
                        float bestPen = 0f;
                        for (int pi = 0; pi < m.PointCount; pi++)
                        {
                            float pen = pi switch
                            {
                                0 => m.P0.Penetration,
                                1 => m.P1.Penetration,
                                2 => m.P2.Penetration,
                                _ => m.P3.Penetration
                            };
                            if (pen > bestPen) bestPen = pen;
                        }

                        float depth = bestPen - slop;
                        if (depth <= 0f) continue;

                        // --- POSITION SEPARATION (robust, prevents "falls thru") ---
                        float corrMag = math.min(depth * percent, maxCorr);
                        float3 corr = n * (corrMag / invMassSum);

                        if (dynA)
                        {
                            poseA.Position -= corr * invMassA;
                            Poses[ia] = poseA;
                            WakeBody(ia);
                        }

                        if (dynB)
                        {
                            poseB.Position += corr * invMassB;
                            Poses[ib] = poseB;
                            WakeBody(ib);
                        }

                        // --- VELOCITY FIX (kill closing normal velocity + optional bounce) ---
                        var vA = Velocities[ia];
                        var vB = Velocities[ib];

                        float3 vRel = vB.Linear - vA.Linear;
                        float vn = math.dot(vRel, n);

                        PhysicsMaterial matA = Colliders.Materials[ha.MaterialId];
                        PhysicsMaterial matB = Colliders.Materials[hb.MaterialId];
                        float e = math.clamp(math.max(matA.Restitution, matB.Restitution), 0f, 1f);
                        float mu = math.max(matA.Friction, matB.Friction);

                        // If they are moving into each other, cancel it (and bounce if fast enough)
                        if (vn < 0f)
                        {
                            float targetVn = 0f;
                            if (-vn > bounceThreshold)
                                targetVn = -e * vn; // positive

                            // We want vn' = targetVn, so delta = (targetVn - vn)
                            float dvn = (targetVn - vn);

                            float3 dv = n * (dvn / invMassSum);

                            if (dynA) vA.Linear -= dv * invMassA;
                            if (dynB) vB.Linear += dv * invMassB;

                            // --- Simple friction: damp tangential relative velocity ---
                            if (mu > 0f)
                            {
                                vRel = (vB.Linear - vA.Linear);
                                float3 vt = vRel - n * math.dot(vRel, n);

                                // clamp tangential speed proportional to normal correction this step
                                // (not physically perfect, but stable + stops "slides off")
                                float vtLen = math.length(vt);
                                if (vtLen > 1e-6f)
                                {
                                    float maxVt = mu * math.abs(dvn);
                                    float newVtLen = math.min(vtLen, maxVt);
                                    float3 vtNew = vt * (newVtLen / vtLen);

                                    float3 vRelNew = n * math.dot(vRel, n) + vtNew;
                                    float3 dvRel = vRelNew - vRel;

                                    float3 dvF = dvRel / invMassSum;

                                    if (dynA) vA.Linear -= dvF * invMassA;
                                    if (dynB) vB.Linear += dvF * invMassB;
                                }
                            }

                            Velocities[ia] = vA;
                            Velocities[ib] = vB;
                        }
                    }
                }
            }
        }



        // -------------------------
        // Sleeping
        // -------------------------
        private void UpdateSleeping(float dt)
        {
            // Tweak these to taste.
            const float linThresh = 0.02f;          // m/s
            const float angThresh = 0.10f;          // rad/s
            const float linThreshSq = linThresh * linThresh;
            const float angThreshSq = angThresh * angThresh;
            const float timeToSleep = 0.5f;         // seconds

            for (int i = 0; i < Bodies.Length; i++)
            {
                if (BodyAlive[i] == 0) continue;
                if (Bodies[i].MotionType != MotionType.Dynamic) continue;

                if (IsSleeping[i] != 0)
                {
                    // keep asleep unless something external wakes it
                    Velocities[i] = default;
                    continue;
                }

                var v = Velocities[i];

                if (math.lengthsq(v.Linear) < linThreshSq && math.lengthsq(v.Angular) < angThreshSq)
                {
                    SleepTime[i] += dt;
                    if (SleepTime[i] >= timeToSleep)
                    {
                        IsSleeping[i] = 1;
                        SleepTime[i] = timeToSleep;
                        v.Linear = 0f;
                        v.Angular = 0f;
                        Velocities[i] = v;
                    }
                }
                else
                {
                    SleepTime[i] = 0f;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void WakeBody(int i)
        {
            if ((uint)i >= (uint)Bodies.Length) return;
            if (Bodies[i].MotionType != MotionType.Dynamic) return;
            IsSleeping[i] = 0;
            SleepTime[i] = 0f;
        }

        // -------------------------
        // Helpers
        // -------------------------
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 FixNormalAtoB(float3 n, float3 posA, float3 posB)
        {
            float3 ab = posB - posA;
            if (math.dot(n, ab) < 0f) n = -n;
            return n;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ref ContactPoint GetPointRef(ref ContactManifold m, int index)
        {
            switch (index)
            {
                case 0: return ref m.P0;
                case 1: return ref m.P1;
                case 2: return ref m.P2;
                default: return ref m.P3;
            }
        }

        // -------------------------
        // Position projection (your exact signature)
        // -------------------------
        private void PositionProject(
            int ia, int ib,
            in ContactManifold m,
            float3 normal,
            float slop,
            float percent,
            float maxCorr)
        {
            var bodyA = Bodies[ia];
            var bodyB = Bodies[ib];

            bool dynA = bodyA.MotionType == MotionType.Dynamic;
            bool dynB = bodyB.MotionType == MotionType.Dynamic;

            float invMassA = dynA ? Masses[ia].InverseMass : 0f;
            float invMassB = dynB ? Masses[ib].InverseMass : 0f;

            float invMassSum = invMassA + invMassB;
            if (invMassSum <= 0f) return;

            float bestPen = 0f;
            for (int pi = 0; pi < m.PointCount; pi++)
            {
                float pen = pi switch
                {
                    0 => m.P0.Penetration,
                    1 => m.P1.Penetration,
                    2 => m.P2.Penetration,
                    _ => m.P3.Penetration
                };
                if (pen > bestPen) bestPen = pen;
            }

            float depth = bestPen - slop;
            if (depth <= 0f) return;

            float corrMag = math.min(depth * percent, maxCorr);
            float3 corr = normal * (corrMag / invMassSum);

            if (dynA)
            {
                var pA = Poses[ia];
                pA.Position -= corr * invMassA;
                Poses[ia] = pA;
                WakeBody(ia);
            }

            if (dynB)
            {
                var pB = Poses[ib];
                pB.Position += corr * invMassB;
                Poses[ib] = pB;
                WakeBody(ib);
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float3 MulInvInertiaWorld(float3 invInertiaLocal, quaternion q, float3 v)
        {
            // World -> local
            float3 vLocal = math.rotate(math.conjugate(q), v);
            // diagonal inv inertia in local space
            float3 wLocal = vLocal * invInertiaLocal;
            // Local -> world
            return math.rotate(q, wLocal);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ulong MakeContactKey(int ia, int ib, uint fid)
        {
            uint a = (uint)math.min(ia, ib);
            uint b = (uint)math.max(ia, ib);
            return ((ulong)a << 32) ^ (ulong)b ^ ((ulong)fid << 1);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void WarmSet(ulong key, float value)
        {
            // Never do indexer GET. Safe add/update/remove.
            if (value <= 0f)
            {
                _warmN.Remove(key);
                return;
            }

            if (!_warmN.TryAdd(key, value))
                _warmN[key] = value; // update existing
        }
    }

    // -------------------------
    // Jobs
    // -------------------------

    [BurstCompile]
    internal struct IntegrateVelocitiesJob : IJobFor
    {
        public float Dt;
        public float3 Gravity;

        [ReadOnly] public NativeArray<byte> BodyAlive;
        [ReadOnly] public NativeArray<byte> IsSleeping;
        [ReadOnly] public NativeArray<Body> Bodies;

        public NativeArray<Velocity> Velocities;
        [ReadOnly] public NativeArray<MassProperties> Masses;
        [ReadOnly] public NativeArray<Damping> Dampings;

        public void Execute(int i)
        {
            if (BodyAlive[i] == 0) return;

            var body = Bodies[i];
            if (body.MotionType != MotionType.Dynamic) return;

            if (IsSleeping[i] != 0)
            {
                Velocities[i] = default;
                return;
            }

            var v = Velocities[i];
            var m = Masses[i];
            var d = Dampings[i];

            if (m.InverseMass > 0f)
                v.Linear += Gravity * Dt;

            float linDamp = math.max(0f, 1f - d.Linear * Dt);
            float angDamp = math.max(0f, 1f - d.Angular * Dt);
            v.Linear *= linDamp;
            v.Angular *= angDamp;

            Velocities[i] = v;
        }
    }

    [BurstCompile]
    internal struct IntegratePosesJob : IJobFor
    {
        public float Dt;

        [ReadOnly] public NativeArray<byte> BodyAlive;
        [ReadOnly] public NativeArray<byte> IsSleeping;
        [ReadOnly] public NativeArray<Body> Bodies;

        public NativeArray<Pose> Poses;
        [ReadOnly] public NativeArray<Velocity> Velocities;

        public void Execute(int i)
        {
            if (BodyAlive[i] == 0) return;

            var body = Bodies[i];
            if (body.MotionType != MotionType.Dynamic) return;
            if (IsSleeping[i] != 0) return;

            var v = Velocities[i];

            var p = Poses[i];
            p.Position += v.Linear * Dt;

            float3 w = v.Angular;
            float wLen = math.length(w);
            if (wLen > 0f)
            {
                float3 axis = w / wLen;
                quaternion dq = quaternion.AxisAngle(axis, wLen * Dt);
                p.Rotation = math.normalize(math.mul(p.Rotation, dq)); // quat*quat OK
            }

            Poses[i] = p;
        }
    }

    [BurstCompile]
    internal struct IntegrateJob : IJobFor
    {
        public float Dt;
        public float3 Gravity;

        [ReadOnly] public NativeArray<byte> BodyAlive;
        [ReadOnly] public NativeArray<byte> IsSleeping;
        [ReadOnly] public NativeArray<Body> Bodies;

        public NativeArray<Pose> Poses;
        public NativeArray<Velocity> Velocities;
        [ReadOnly] public NativeArray<MassProperties> Masses;
        [ReadOnly] public NativeArray<Damping> Dampings;

        public void Execute(int i)
        {
            if (BodyAlive[i] == 0) return;

            var body = Bodies[i];
            if (body.MotionType != MotionType.Dynamic) return;
            if (IsSleeping[i] != 0)
            {
                Velocities[i] = default;
                return;
            }

            var v = Velocities[i];
            var m = Masses[i];
            var d = Dampings[i];

            if (m.InverseMass > 0f)
                v.Linear += Gravity * Dt;

            float linDamp = math.max(0f, 1f - d.Linear * Dt);
            float angDamp = math.max(0f, 1f - d.Angular * Dt);
            v.Linear *= linDamp;
            v.Angular *= angDamp;

            Velocities[i] = v;

            var p = Poses[i];
            p.Position += v.Linear * Dt;

            float3 w = v.Angular;
            float wLen = math.length(w);
            if (wLen > 0f)
            {
                float3 axis = w / wLen;
                quaternion dq = quaternion.AxisAngle(axis, wLen * Dt);
                p.Rotation = math.normalize(math.mul(p.Rotation, dq));
            }

            Poses[i] = p;
        }
    }

    [BurstCompile]
    internal struct UpdateProxiesJob : IJobFor
    {
        [ReadOnly] public NativeArray<byte> BodyAlive;
        [ReadOnly] public NativeArray<Body> Bodies;
        [ReadOnly] public NativeArray<Pose> Poses;

        [ReadOnly] public NativeArray<ColliderSlot> ColliderSlots;

        public NativeArray<int> BodyToProxy;
        public NativeArray<Aabb> ProxyAabbs;

        public void Execute(int i)
        {
            if (BodyAlive[i] == 0) return;

            var body = Bodies[i];
            int proxy = BodyToProxy[i];

            // Validate collider handle without calling ColliderStore methods (Burst-safe)
            if ((uint)body.Collider.Slot >= (uint)ColliderSlots.Length)
            {
                BodyToProxy[i] = -1;
                return;
            }

            var slot = ColliderSlots[body.Collider.Slot];
            if (slot.IsAlive == 0 || slot.Header.Version != body.Collider.Version)
            {
                if (proxy != -1) BodyToProxy[i] = -1;
                return;
            }

            Aabb worldAabb = WorldAabb.FromBodyPose(slot.Header.LocalAabb, Poses[i]);

            if (proxy == -1)
            {
                // placeholder proxy id == body id
                proxy = i;
                BodyToProxy[i] = proxy;
            }

            ProxyAabbs[proxy] = worldAabb;
        }
    }
}
