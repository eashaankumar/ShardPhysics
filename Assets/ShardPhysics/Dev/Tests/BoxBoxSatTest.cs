using UnityEngine;

namespace Shard.Dev.Tests
{
    public class BoxBoxSatTest : MonoBehaviour
    {
        [SerializeField] Transform box1;
        [SerializeField] Transform box2;
        

        void Update()
        {
            bool collision = BoxBoxSolver.Solve(new BoxBoxSolver.Box(box1.position, box1.rotation, box1.localScale / 2), new BoxBoxSolver.Box(box2.position, box2.rotation, box2.localScale / 2), out var cps);

            if (collision)
            {
                Debug.Log($"Collision! {Time.time}");
            }
        }
    }
}
