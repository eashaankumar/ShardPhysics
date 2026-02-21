using Shard.Runtime;
using Shard.Runtime.Solvers;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Shard.Tests.Test1
{
    public class CreateBasicSimTest : MonoBehaviour
    {
        [SerializeField] Transform box1Trans;
        [SerializeField] Transform box2Trans;
        [SerializeField] Transform cyl1Trans;
        [SerializeField] Transform cyl2Trans;

        ShardPhysicsWorld world;
        ShardBodyHandle box1, box2, cylinder1, cylinder2;

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
                //material = new ShardColliderMaterial
                //{
                //    bounciness = 0.2f,
                //    frictionDynamic = 1.0f,
                //    frictionStatic = 0.5f,
                //    frictionRolling = 0.2f
                //}
            });
            box2 = world.CreateBody(new Runtime.Pose { position = box2Trans.position, rotation = box2Trans.rotation }, BodyType.Dynamic, 1, colliders.AsArray());

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
            cylinder1 = world.CreateBody(new Runtime.Pose { position = cyl1Trans.position, rotation = cyl1Trans.rotation }, BodyType.Dynamic, 1, colliders.AsArray());

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
            cylinder2 = world.CreateBody(new Runtime.Pose { position = cyl2Trans.position, rotation = cyl2Trans.rotation }, BodyType.Dynamic, 1, colliders.AsArray());

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

                //hit = CylinderBoxSolver.Solve(new CylinderBoxSolver.Box(boxTrans.position, boxTrans.rotation, boxTrans.localScale / 2),
                //                    new CylinderBoxSolver.Cylinder(cylinderTrans.position, cylinderTrans.rotation, cylinderTrans.localScale.y / 2, cylinderTrans.localScale.x / 2),
                //                    out cps);

                //if (hit)
                //{
                //    Debug.Log("Cylinder box hit! " + Time.time);
                //}
                //print(hit);

                UpdatePose(box1, box1Trans);

                UpdatePose(box2, box2Trans);


                UpdatePose(cylinder1, cyl1Trans);
                UpdatePose(cylinder2, cyl2Trans);

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
