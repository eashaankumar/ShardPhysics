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

        ShardPhysicsWorld world;
        ShardBodyHandle box1;

        private void Awake()
        {
            world = new ShardPhysicsWorld();
            world.gravity = math.down() * 9.81f;

            NativeList<ShardCollider> colliders = new NativeList<ShardCollider>(Allocator.Temp);
            colliders.Add(new ShardCollider
            {
                localPose=new Runtime.Pose { position=float3.zero, rotation=quaternion.identity },
                type = ShardColliderType.Box,
                density = 1.0f,
                halfExtents = box1Trans.localScale/2,
            });

            box1 = world.CreateBody(new Runtime.Pose { position = box1Trans.position, rotation = box1Trans.rotation }, BodyType.Dynamic, 1, colliders.AsArray(), new Velocity { linearVelocity=math.up() * 5, angularVelocity = new float3(0, 90 * math.TORADIANS, 0)});

            StartCoroutine(Tick());
        }

        private void OnDestroy()
        {
            world.Dispose();
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
            }
        }
    }
}
