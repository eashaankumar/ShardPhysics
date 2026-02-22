using Shard.Runtime;
using System.Collections;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Shard.Tests.Test1
{
    public class CylinderCylinderTest : MonoBehaviour
    {
        [SerializeField] Transform[] cylsTrans;
        [SerializeField] Transform floorBoxTrans;

        ShardPhysicsWorld world;
        ShardBodyHandle[] cylinderRbs;
        ShardBodyHandle floorBoxRb;

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
                halfExtents = floorBoxTrans.localScale / 2,
                material = new ShardColliderMaterial
                {
                    bounciness = 0.2f,
                    frictionDynamic = 1.0f,
                    frictionStatic = 0.5f,
                    frictionRolling = 0.2f
                }
            });

            floorBoxRb = world.CreateBody(new Runtime.Pose { position = floorBoxTrans.position, rotation = floorBoxTrans.rotation }, BodyType.Static, 1, colliders.AsArray(), new Velocity { linearVelocity = math.up() * 5, angularVelocity = new float3(0, 90 * math.TORADIANS, 0) });

            cylinderRbs = new ShardBodyHandle[cylsTrans.Length]; 

            for(int i =0; i <  cylsTrans.Length; i++)
            {
                var cylTransform = cylsTrans[i];
                colliders.Clear();
                colliders.Add(new ShardCollider
                {
                    localPose = new Runtime.Pose { position = float3.zero, rotation = quaternion.identity },
                    type = ShardColliderType.Cylinder,
                    height = 1.0f,
                    radius = 0.5f,
                    density = 1.0f,
                    //material = new ShardColliderMaterial
                    //{
                    //    bounciness = 0.2f,
                    //    frictionDynamic = 1.0f,
                    //    frictionStatic = 0.5f,
                    //    frictionRolling = 0.2f
                    //}
                });
                cylinderRbs[i] = world.CreateBody(new Runtime.Pose { position = cylTransform.position, rotation = cylTransform.rotation }, BodyType.Dynamic, 1, colliders.AsArray());
            }

           
            
            StartCoroutine(Tick());
        }

        private void OnDestroy()
        {
            world.Dispose();
        }

        IEnumerator Tick()
        {
            float dt = 1.0f / 60f;
            while (true)
            {
                yield return new WaitForSeconds(dt);

                world.Simulate(dt);

                UpdatePose(floorBoxRb, floorBoxTrans);

                for (int i = 0; i < cylsTrans.Length; i++)
                {
                    UpdatePose(cylinderRbs[i], cylsTrans[i]);
                }

            }
        }

        void UpdatePose(ShardBodyHandle body, Transform trans)
        {
            if (world.TryGetPose(body, out var pose))
            {
                trans.position = pose.position;
                trans.rotation = pose.rotation;
                trans.gameObject.SetActive(true);
            }
            else
            {
                trans.gameObject.SetActive(false);
            }
        }
    }
}
