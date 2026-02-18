using Shard.Runtime;
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

            //box1 = world.CreateBody(new Runtime.Pose { position= box1Trans.position, rotation=box1Trans.rotation}, 1, )
        }

        private void OnDestroy()
        {
            world.Dispose();
        }
    }
}
