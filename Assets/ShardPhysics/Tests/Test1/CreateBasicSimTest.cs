using Shard.Runtime;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Shard.Tests.Test1
{
    public class CreateBasicSimTest : MonoBehaviour
    {
        [SerializeField] Transform box1Trans;
        [SerializeField] Transform box2Trans;

        ShardPhysicsWorld world;
        ShardBodyHandle box1, box2;

        private void Awake()
        {
            world = new ShardPhysicsWorld();
            world.gravity = math.down() * 9.81f;

            NativeList<ShardCollider> colliders = new NativeList<ShardCollider>(Allocator.Temp);
            colliders.Add(new ShardCollider
            {
                localPose = new Runtime.Pose { position = float3.zero, rotation = quaternion.identity },
                type = ShardColliderType.Box,
                density = 1.0f,
                halfExtents = box1Trans.localScale / 2,
                material = new ShardColliderMaterial
                {
                    bounciness = 0.2f,
                    frictionDynamic = 1.0f,
                    frictionStatic = 0.5f,
                    frictionRolling = 0.2f
                }
            });

            box1 = world.CreateBody(new Runtime.Pose { position = box1Trans.position, rotation = box1Trans.rotation }, BodyType.Dynamic, 1, colliders.AsArray(), new Velocity { linearVelocity=math.up() * 5, angularVelocity = new float3(0, 90 * math.TORADIANS, 0)});
            world.SetBodyType(box1, BodyType.Static);



            colliders.Clear();
            colliders.Add(new ShardCollider
            {
                localPose = new Runtime.Pose { position = float3.zero, rotation = quaternion.identity },
                type = ShardColliderType.Box,
                density = 1.0f,
                halfExtents = box2Trans.localScale / 2,
            });
            box2 = world.CreateBody(new Runtime.Pose { position = box2Trans.position, rotation = box2Trans.rotation }, BodyType.Dynamic, 1, colliders.AsArray());


            StartCoroutine(Tick());
        }

        private void OnDestroy()
        {
            world.Dispose();
        }

        private void Update()
        {
            if (Input.GetKeyDown(KeyCode.Space))
            {
                if (world.TryGetBodyType(box1, out var t))
                {
                    Debug.Log(t);
                    BodyType newType;
                    if (t == BodyType.Static) newType = BodyType.Kinematic;
                    else
                    {
                        newType = BodyType.Static;
                    }

                    world.SetBodyType(box1, newType);
                    world.SetVelocity(box1, new Velocity { linearVelocity = math.up() * 5, angularVelocity = math.up() * 5 });
                }
            }
        }

        IEnumerator Tick()
        {
            float dt = 1.0f / 60f;
            while(true)
            {
                yield return new WaitForSeconds(dt);

                world.Simulate(dt);

                if(world.TryGetPose(box1, out var pose))
                {
                    box1Trans.position = pose.position;
                    box1Trans.rotation = pose.rotation;
                }

                if (world.TryGetPose(box2, out pose))
                {
                    box2Trans.position = pose.position;
                    box2Trans.rotation = pose.rotation;
                }
            }
        }
    }
}
