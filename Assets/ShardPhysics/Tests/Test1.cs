using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Shard.Tests
{
    public class Test1 : MonoBehaviour
    {
        [SerializeField] Transform boxATransform;
        [SerializeField] Transform boxBTransform;
        PhysicsWorld world;

        BodyId aId;
        BodyId bId;

        void Awake()
        {
            // World (small capacities are fine)
            world = new PhysicsWorld(
                bodyCapacity: 16,
                colliderCapacity: 16,
                allocator: Allocator.Persistent,
                worldId: 1
            );

            ushort rubber = world.Colliders.CreateMaterial(new PhysicsMaterial { Restitution = 0.0f, Friction = 0.9f });

            // Create a box collider in the ColliderStore
            // (Half extents, local center, local orientation)
            float3 he = new float3(0.5f, 0.5f, 0.5f);
            ColliderHandle boxCol = world.Colliders.CreateBox(
                halfExtents: he,
                center: float3.zero,
                orientation: quaternion.identity,
                materialId:rubber
            );

            // Mass props for this box at density=1 (or choose your mass directly)
            // Here: give each box mass=1
            MassProperties mp = MakeBoxMass(mass: 1f, halfExtents: he);

            // Common damping
            Damping damping = new Damping { Linear = 0.02f, Angular = 0.02f };

            // Body A
            aId = world.AddBody(
                motionType: MotionType.Dynamic,
                pose: new Pose { Position = boxATransform.position, Rotation = boxATransform.rotation },
                velocity: new Velocity { Linear = new float3(0, 0, 0), Angular = float3.zero },
                mass: mp,
                damping: damping,
                collider: boxCol
            );

            // Body B (slightly rotated, near A)
            bId = world.AddBody(
                motionType: MotionType.Kinematic,
                pose: new Pose
                {
                    Position = boxBTransform.position,
                    Rotation = boxBTransform.rotation
                },
                velocity: new Velocity { Linear = new float3(0, 0, 0), Angular = float3.zero },
                mass: mp,
                damping: damping,
                collider: boxCol
            );
        }

        void FixedUpdate()
        {
            if (world == null) return;

            float dt = Time.fixedDeltaTime;
            float3 gravity = new float3(0, -9.81f, 0);

            // Step returns a JobHandle. Complete before reading poses.
            JobHandle h = world.Step(dt, gravity);
            h.Complete();

            // Read back poses for debugging
            Pose pa = world.Poses[aId.Value];
            Pose pb = world.Poses[bId.Value];

            // Draw simple debug lines
            //Debug.DrawLine((Vector3)pa.Position, (Vector3)(pa.Position + new float3(0, 0.5f, 0)), Color.green);
            //Debug.DrawLine((Vector3)pb.Position, (Vector3)(pb.Position + new float3(0, 0.5f, 0)), Color.cyan);

            // (Optional) log occasionally
            // if (Time.frameCount % 30 == 0) Debug.Log($"A:{pa.Position}  B:{pb.Position}");

            boxATransform.position = pa.Position;
            boxATransform.rotation = pa.Rotation;

            boxBTransform.position = pb.Position;
            boxBTransform.rotation = pb.Rotation;


        }

        void OnDestroy()
        {
            if (world != null)
            {
                world.Dispose();
                world = null;
            }
        }

        // --- Mass helper matching your engine's MassProperties assumptions ---
        // Inertia of a solid box about its center:
        // Ixx = (1/12) m (h^2 + d^2)
        // where h,d are full extents (size), NOT half extents.
        static MassProperties MakeBoxMass(float mass, float3 halfExtents)
        {
            float3 size = halfExtents * 2f;

            float x2 = size.x * size.x;
            float y2 = size.y * size.y;
            float z2 = size.z * size.z;

            float Ixx = (mass / 12f) * (y2 + z2);
            float Iyy = (mass / 12f) * (x2 + z2);
            float Izz = (mass / 12f) * (x2 + y2);

            float invMass = (mass > 0f) ? (1f / mass) : 0f;

            float3 invInertiaLocal = new float3(
                Ixx > 0f ? 1f / Ixx : 0f,
                Iyy > 0f ? 1f / Iyy : 0f,
                Izz > 0f ? 1f / Izz : 0f
            );

            return new MassProperties
            {
                InverseMass = invMass,
                InverseInertiaLocal = invInertiaLocal,
                CenterOfMassLocal = float3.zero
            };
        }
    }
}
