using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities.Memory;
using System;
using System.Collections;
using UnityEngine;
using static BepuTests.BepuBasicExample_2_0_0;

public class TestBepu : MonoBehaviour
{
    BufferPool pool;
    Simulation simulation;
    ParallelThreadDispatcher dispatcher;

    private void Awake()
    {
        pool = new BufferPool();

        int threadCount = Math.Max(1, System.Environment.ProcessorCount - 1);
        dispatcher = new ParallelThreadDispatcher(threadCount);

        var narrow = new SimpleNarrowPhaseCallbacks
        {
            Friction = 1.0f,
            MaxRecoveryVelocity = 2.0f,
            SpringSettings = new SpringSettings(30f, 1f)
        };

        var pose = new SimplePoseIntegratorCallbacks(new System.Numerics.Vector3(0, -9.81f, 0));

        // Create simulation (timestepper null => PositionFirstTimestepper)
        simulation = Simulation.Create(pool, narrow, pose, timestepper: null,
            solverIterationCount: 8,
            solverFallbackBatchThreshold: 64,
            initialAllocationSizes: null);

        // -----------------------
        // Ground: big static box
        // -----------------------
        var groundShape = new Box(200f, 1f, 200f);
        var groundShapeIndex = simulation.Shapes.Add(groundShape);
        simulation.Statics.Add(new StaticDescription(
            position: new System.Numerics.Vector3(0, -0.5f, 0),
            orientation: default,
            collidable: new CollidableDescription(groundShapeIndex, 0.1f)));

        // -----------------------
        // Shared dynamic shapes
        // -----------------------
        var box = new Box(1f, 1f, 1f);
        var boxShapeIndex = simulation.Shapes.Add(box);
        box.ComputeInertia(1f, out var boxInertia);

        var cylinder = new Cylinder(0.5f, 1.5f);
        var cylShapeIndex = simulation.Shapes.Add(cylinder);
        cylinder.ComputeInertia(1f, out var cylInertia);

        var rng = new System.Random(1234);

        // 100 boxes in 10x10
        int side = 10;
        for (int i = 0; i < 100; i++)
        {
            int x = i % side;
            int z = i / side;

            float px = (x - (side - 1) * 0.5f) * 1.2f;
            float pz = (z - (side - 1) * 0.5f) * 1.2f;
            float py = 2f + (float)rng.NextDouble() * 2f;

            var poseDesc = new RigidPose(new System.Numerics.Vector3(px, py, pz), default);

            // Speculative margin is the 2nd arg of CollidableDescription in this era.
            var collidable = new CollidableDescription(boxShapeIndex, 0.1f);

            // Sleep tuning (0.01 is common in demos)
            var activity = new BodyActivityDescription(0.01f);

            var bodyDesc = BodyDescription.CreateDynamic(poseDesc, boxInertia, collidable, activity);
            simulation.Bodies.Add(bodyDesc);
        }

        // 100 cylinders offset to the side
        for (int i = 0; i < 100; i++)
        {
            int x = i % side;
            int z = i / side;

            float px = 15f + (x - (side - 1) * 0.5f) * 1.5f;
            float pz = (z - (side - 1) * 0.5f) * 1.5f;
            float py = 3f + (float)rng.NextDouble() * 2f;

            var poseDesc = new RigidPose(new System.Numerics.Vector3(px, py, pz), default);
            var collidable = new CollidableDescription(cylShapeIndex, 0.1f);
            var activity = new BodyActivityDescription(0.01f);

            var bodyDesc = BodyDescription.CreateDynamic(poseDesc, cylInertia, collidable, activity);
            simulation.Bodies.Add(bodyDesc);
        }

        StartCoroutine(PhysicsStep());
    }

    private void OnDestroy()
    {
        simulation.Dispose();
        pool.Clear();
    }

    IEnumerator PhysicsStep()
    {
        while (true)
        {
            simulation.Timestep(Time.fixedDeltaTime, dispatcher);
            yield return new WaitForSeconds(Time.fixedDeltaTime);
        }
    }
}
