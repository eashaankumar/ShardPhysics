using Unity.Mathematics;
using UnityEngine;

namespace Shard.Runtime
{
    public struct ContactPoint
    {
        public float3 point;
        public float3 normal;
        public float depth;
    }

    public struct ContactPointManifold
    {
        public ContactPoint p1;
        public ContactPoint p2;
        public ContactPoint p3;
        public ContactPoint p4;
        public int numContactPoints;

        public float3 globalPenAxis;   // MTV axis (box -> cylinder)
        public float globalPenDepth;   // MTV depth

        public ContactPoint this[int index]
        {
            get
            {
                if (index == 0) return p1;
                if (index == 1) return p2;
                if (index == 2) return p3;
                if (index == 3) return p4;
                return default;
            }
            set
            {
                if (index == 0) p1 = value;
                if (index == 1) p2 = value;
                if (index == 2) p3 = value;
                if (index == 3) p4 = value;
            }
        }
    }


}
