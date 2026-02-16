using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using UnityEngine;

namespace BepuTests
{
    public static class BepuBasicExample_2_0_0
    {
        public sealed class ParallelThreadDispatcher : IThreadDispatcher, IDisposable
        {
            public int ThreadCount { get; }

            private readonly BufferPool[] workerPools;

            public ParallelThreadDispatcher(int threadCount)
            {
                ThreadCount = Math.Max(1, threadCount);

                workerPools = new BufferPool[ThreadCount];

                for (int i = 0; i < ThreadCount; i++)
                    workerPools[i] = new BufferPool();
            }

            public void DispatchWorkers(Action<int> workerBody)
            {
                Parallel.For(0, ThreadCount, i =>
                {
                    workerBody(i);
                });
            }

            public BufferPool GetThreadMemoryPool(int workerIndex)
            {
                return workerPools[workerIndex];
            }

            public void Dispose()
            {
                for (int i = 0; i < workerPools.Length; i++)
                    workerPools[i]?.Clear();
            }
        }

        // --- Narrow phase callbacks (matches your unsafe interface) ---
        public unsafe struct SimpleNarrowPhaseCallbacks : INarrowPhaseCallbacks
        {
            public float Friction;
            public float MaxRecoveryVelocity;
            public SpringSettings SpringSettings;

            public void Initialize(Simulation simulation) { }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b) => true;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB) => true;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ConvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
            {
                pairMaterial = new PairMaterialProperties
                {
                    FrictionCoefficient = Friction,
                    MaximumRecoveryVelocity = MaxRecoveryVelocity,
                    SpringSettings = SpringSettings
                };
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, NonconvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
            {
                pairMaterial = new PairMaterialProperties
                {
                    FrictionCoefficient = Friction,
                    MaximumRecoveryVelocity = MaxRecoveryVelocity,
                    SpringSettings = SpringSettings
                };
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ConvexContactManifold* manifold) => true;

            public void Dispose() { }
        }

        // --- Pose integrator callbacks (matches your IPoseIntegratorCallbacks) ---
        public struct SimplePoseIntegratorCallbacks : IPoseIntegratorCallbacks
        {
            public System.Numerics.Vector3 Gravity;
            private float dt;

            public AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

            public SimplePoseIntegratorCallbacks(System.Numerics.Vector3 gravity)
            {
                Gravity = gravity;
                dt = 0;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void PrepareForIntegration(float dt) => this.dt = dt;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void IntegrateVelocity(int bodyIndex, in RigidPose pose, in BodyInertia localInertia, int workerIndex, ref BodyVelocity velocity)
            {
                // Apply gravity: v += g * dt
                velocity.Linear += Gravity * dt;
            }
        }
    }
}
