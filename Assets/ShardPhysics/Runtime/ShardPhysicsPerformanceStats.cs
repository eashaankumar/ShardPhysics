namespace Shard.Runtime
{
    public struct ShardTriangleMeshQueryStats
    {
        public int nodesVisited;
        public int leafNodesVisited;
        public int triangleCandidatesReturned;

        public void Reset()
        {
            nodesVisited = 0;
            leafNodesVisited = 0;
            triangleCandidatesReturned = 0;
        }
    }

    public struct ShardPhysicsPerformanceStats
    {
        public int substeps;
        public int collisionIterations;

        public int bodyPairsFromBroadphase;
        public int contactManifoldsGenerated;
        public int contactPointsGenerated;

        public int meshBvhQueries;
        public int meshBvhNodesVisited;
        public int meshBvhLeafNodesVisited;
        public int meshBvhCandidateTrianglesReturned;
        public int meshTriangleAabbTests;
        public int meshTriangleNarrowphaseTests;
        public int meshFullScanFallbacks;

        public void Reset()
        {
            substeps = 0;
            collisionIterations = 0;

            bodyPairsFromBroadphase = 0;
            contactManifoldsGenerated = 0;
            contactPointsGenerated = 0;

            meshBvhQueries = 0;
            meshBvhNodesVisited = 0;
            meshBvhLeafNodesVisited = 0;
            meshBvhCandidateTrianglesReturned = 0;
            meshTriangleAabbTests = 0;
            meshTriangleNarrowphaseTests = 0;
            meshFullScanFallbacks = 0;
        }
    }
}
