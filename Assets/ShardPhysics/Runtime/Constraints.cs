using Unity.Mathematics;

namespace Shard
{
    public struct BallSocketJoint
    {
        public float3 LocalAnchorA;
        public float3 LocalAnchorB;
        public float Softness; // ERP/CFM-like tuning
    }

    public struct HingeJoint
    {
        public float3 LocalAnchorA;
        public float3 LocalAnchorB;
        public float3 LocalAxisA;
        public float3 LocalAxisB;

        public float MinAngle;
        public float MaxAngle;
        public float MotorSpeed;
        public float MotorMaxTorque;
    }

    public struct PrismaticJoint
    {
        public float3 LocalAnchorA;
        public float3 LocalAnchorB;
        public float3 LocalAxisA;

        public float MinDistance;
        public float MaxDistance;
        public float MotorSpeed;
        public float MotorMaxForce;
    }
}
