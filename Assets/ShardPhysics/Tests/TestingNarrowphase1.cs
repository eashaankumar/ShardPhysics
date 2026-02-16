using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

namespace Shard.Tests
{
    public class TestingNarrowphase1 : MonoBehaviour
    {
        [SerializeField] Box[] boxes;

        [System.Serializable]
        class Box
        {
            public Transform transform;

            public BoxCollider Collider => new BoxCollider { HalfExtents = transform.localScale / 2, Center = 0, Orientation = Quaternion.identity };
            public Pose Pose => new Pose { Position=transform.position, Rotation=transform.rotation };
        }

        [System.Serializable]
        class Sphere
        {
            public Transform transform;

            public SphereCollider Collider => new SphereCollider { Radius = transform.localScale.x / 2, Center = 0};
            public Pose Pose => new Pose { Position = transform.position, Rotation = transform.rotation };
        }

        [System.Serializable]
        class Cylinder
        {
            public Transform transform;

            public CylinderCollider Collider => new CylinderCollider { Radius = transform.localScale.x / 2, HalfHeight=transform.localScale.y/2, Orientation=Quaternion.identity, Center = 0 };
            public Pose Pose => new Pose { Position = transform.position, Rotation = transform.rotation };
        }

        List<ContactManifold> contactManifols = new();

        private void Update()
        {
            contactManifols.Clear();
            for (int i = 0; i < boxes.Length; i++)
            {
                var boxA = boxes[i];
                for(int j = i + 1; j < boxes.Length; j++)
                {
                    var boxB = boxes[j];
                    ContactManifold contactManifold = default;
                    bool hit = Narrowphase.BoxBox(boxA.Collider, boxA.Pose, boxB.Collider, boxB.Pose, ref contactManifold);
                    //Debug.Log($"{hit}");
                    contactManifols.Add(contactManifold);
                }
            }
            
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            Gizmos.matrix = Matrix4x4.identity;

            foreach(var cm in contactManifols)
            {
                
                if (cm.PointCount >= 1)
                {
                    Gizmos.color = Color.green;
                    Gizmos.DrawSphere(cm.P0.Position, 0.02f);
                }
                if (cm.PointCount >= 2)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(cm.P1.Position, 0.02f);
                }
                if (cm.PointCount >= 3)
                {
                    Gizmos.color = Color.blue;
                    Gizmos.DrawSphere(cm.P2.Position, 0.02f);
                }
                if (cm.PointCount >= 4)
                {
                    Gizmos.color = Color.magenta;
                    Gizmos.DrawSphere(cm.P3.Position, 0.02f);
                }
            }
        }
#endif
    }
}