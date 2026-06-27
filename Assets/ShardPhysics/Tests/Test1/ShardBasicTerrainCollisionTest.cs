using System.Collections;
using System.Collections.Generic;
using Shard.Runtime;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using Pose = Shard.Runtime.Pose;

namespace Shard.Tests.Test1
{
    public sealed class ShardBasicTerrainCollisionTest : MonoBehaviour
    {
        [Header("Materials")]
        [SerializeField] Material terrainMaterial;
        [SerializeField] Material sphereMaterial;
        [SerializeField] Material boxMaterial;
        [SerializeField] Material capsuleMaterial;
        [SerializeField] Material triangleMaterial;

        [Header("Terrain")]
        [SerializeField] int chunksX = 3;
        [SerializeField] int chunksZ = 3;
        [SerializeField] int chunkResolution = 8;
        [SerializeField] float chunkSize = 8f;
        [SerializeField] float heightScale = 1.5f;
        [SerializeField] float noiseScale = 0.12f;

        [Header("Bodies")]
        [SerializeField] int sphereCount = 6;
        [SerializeField] int boxCount = 6;
        [SerializeField] int capsuleCount = 6;
        [SerializeField] float spawnHeight = 8f;

        [Header("Simulation")]
        [SerializeField] float fixedDt = 1f / 60f;

        ShardPhysicsWorld world;
        readonly List<VisualBody> visualBodies = new List<VisualBody>();

        Mesh sphereMesh;
        Mesh boxMesh;
        Mesh capsuleMesh;

        struct VisualBody
        {
            public ShardBodyHandle body;
            public Transform transform;
        }

        void Awake()
        {
            world = new ShardPhysicsWorld();
            world.gravity = math.down() * 9.81f;

            sphereMesh = CreatePrimitiveMesh(PrimitiveType.Sphere);
            boxMesh = CreatePrimitiveMesh(PrimitiveType.Cube);
            capsuleMesh = CreatePrimitiveMesh(PrimitiveType.Capsule);

            CreateTerrainChunks();
            CreateDynamicBodies();
            CreateSingleTriangleCollider();

            StartCoroutine(Tick());
        }

        void OnDestroy()
        {
            if (world != null)
                world.Dispose();
        }

        IEnumerator Tick()
        {
            while (true)
            {
                yield return new WaitForSeconds(fixedDt);

                world.Simulate(fixedDt);

                for (int i = 0; i < visualBodies.Count; i++)
                    SyncVisual(visualBodies[i]);
            }
        }

        void CreateTerrainChunks()
        {
            float startX = -chunksX * chunkSize * 0.5f;
            float startZ = -chunksZ * chunkSize * 0.5f;

            for (int x = 0; x < chunksX; x++)
            {
                for (int z = 0; z < chunksZ; z++)
                {
                    float3 origin = new float3(
                        startX + x * chunkSize,
                        0f,
                        startZ + z * chunkSize);

                    Mesh mesh = BuildHeightfieldChunkMesh(origin, x, z);

                    GameObject go = new GameObject($"Shard Terrain Chunk {x},{z}");
                    go.transform.SetParent(transform, false);

                    MeshFilter filter = go.AddComponent<MeshFilter>();
                    MeshRenderer renderer = go.AddComponent<MeshRenderer>();

                    filter.sharedMesh = mesh;
                    renderer.sharedMaterial = terrainMaterial;

                    ShardTriangleMeshHandle meshHandle = UnityMeshToShard.AddUnityMesh(world, mesh);

                    ShardCollider collider = ShardCreateBody.CreateTriangleMesh(
                        meshHandle,
                        density: 0f,
                        localPose: IdentityPose(),
                        material: DefaultMaterial());

                    CreateStaticBody(new Pose
                    {
                        position = float3.zero,
                        rotation = quaternion.identity
                    }, collider);
                }
            }
        }

        Mesh BuildHeightfieldChunkMesh(float3 origin, int chunkX, int chunkZ)
        {
            int vertsPerSide = chunkResolution + 1;
            int vertexCount = vertsPerSide * vertsPerSide;
            int indexCount = chunkResolution * chunkResolution * 6;

            Vector3[] vertices = new Vector3[vertexCount];
            int[] triangles = new int[indexCount];

            for (int z = 0; z < vertsPerSide; z++)
            {
                for (int x = 0; x < vertsPerSide; x++)
                {
                    float u = x / (float)chunkResolution;
                    float v = z / (float)chunkResolution;

                    float worldX = origin.x + u * chunkSize;
                    float worldZ = origin.z + v * chunkSize;

                    float h = noise.snoise(new float2(worldX, worldZ) * noiseScale) * heightScale;

                    int index = z * vertsPerSide + x;
                    vertices[index] = new Vector3(worldX, h, worldZ);
                }
            }

            int t = 0;

            for (int z = 0; z < chunkResolution; z++)
            {
                for (int x = 0; x < chunkResolution; x++)
                {
                    int i0 = z * vertsPerSide + x;
                    int i1 = z * vertsPerSide + x + 1;
                    int i2 = (z + 1) * vertsPerSide + x;
                    int i3 = (z + 1) * vertsPerSide + x + 1;

                    triangles[t++] = i0;
                    triangles[t++] = i2;
                    triangles[t++] = i1;

                    triangles[t++] = i1;
                    triangles[t++] = i2;
                    triangles[t++] = i3;
                }
            }

            Mesh mesh = new Mesh
            {
                name = $"ShardHeightfieldChunk_{chunkX}_{chunkZ}"
            };

            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            return mesh;
        }

        void CreateDynamicBodies()
        {
            for (int i = 0; i < sphereCount; i++)
            {
                float radius = 0.5f;

                float3 position = new float3(
                    -6f + i * 1.5f,
                    spawnHeight + i * 0.4f,
                    -3f);

                ShardCollider collider = ShardCreateBody.CreateSphere(
                    radius,
                    density: 1f,
                    localPose: IdentityPose(),
                    material: DefaultMaterial());

                ShardBodyHandle body = CreateDynamicBody(position, quaternion.identity, 1f, collider);

                CreateVisualBody(
                    $"Shard Sphere {i}",
                    body,
                    sphereMesh,
                    sphereMaterial,
                    new float3(radius * 2f));
            }

            for (int i = 0; i < boxCount; i++)
            {
                float3 halfExtents = new float3(0.5f, 0.5f, 0.5f);

                float3 position = new float3(
                    -6f + i * 1.5f,
                    spawnHeight + 2f + i * 0.4f,
                    0f);

                quaternion rotation = quaternion.EulerXYZ(
                    math.radians(12f * i),
                    math.radians(18f * i),
                    math.radians(7f * i));

                ShardCollider collider = ShardCreateBody.CreateBox(
                    halfExtents,
                    density: 1f,
                    localPose: IdentityPose(),
                    material: DefaultMaterial());

                ShardBodyHandle body = CreateDynamicBody(position, rotation, 1f, collider);

                CreateVisualBody(
                    $"Shard Box {i}",
                    body,
                    boxMesh,
                    boxMaterial,
                    halfExtents * 2f);
            }

            for (int i = 0; i < capsuleCount; i++)
            {
                float radius = 0.35f;
                float height = 1.4f;

                float3 position = new float3(
                    -6f + i * 1.5f,
                    spawnHeight + 4f + i * 0.4f,
                    3f);

                quaternion rotation = quaternion.EulerXYZ(
                    math.radians(0f),
                    math.radians(20f * i),
                    math.radians(35f));

                ShardCollider collider = ShardCreateBody.CreateCapsule(
                    radius,
                    height,
                    density: 1f,
                    localPose: IdentityPose(),
                    material: DefaultMaterial());

                ShardBodyHandle body = CreateDynamicBody(position, rotation, 1f, collider);

                CreateVisualBody(
                    $"Shard Capsule {i}",
                    body,
                    capsuleMesh,
                    capsuleMaterial,
                    new float3(radius * 2f, height + radius * 2f, radius * 2f));
            }
        }

        void CreateSingleTriangleCollider()
        {
            float3 a = new float3(4f, 1.5f, -3f);
            float3 b = new float3(8f, 1.5f, -3f);
            float3 c = new float3(6f, 3.5f, 1f);

            ShardCollider collider = ShardCreateBody.CreateTriangle(
                a,
                b,
                c,
                density: 0f,
                localPose: IdentityPose(),
                material: DefaultMaterial());

            CreateStaticBody(new Pose
            {
                position = float3.zero,
                rotation = quaternion.identity
            }, collider);

            Mesh mesh = new Mesh
            {
                name = "ShardSingleTriangleVisual"
            };

            mesh.vertices = new[]
            {
                (Vector3)a,
                (Vector3)b,
                (Vector3)c
            };

            mesh.triangles = new[] { 0, 1, 2 };
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            GameObject go = new GameObject("Shard Single Triangle Collider");
            go.transform.SetParent(transform, false);

            MeshFilter filter = go.AddComponent<MeshFilter>();
            MeshRenderer renderer = go.AddComponent<MeshRenderer>();

            filter.sharedMesh = mesh;
            renderer.sharedMaterial = triangleMaterial;
        }

        private ShardBodyHandle CreateDynamicBody(
            float3 position,
            quaternion rotation,
            float mass,
            ShardCollider collider)
        {
            NativeArray<ShardCollider> colliders =
                new NativeArray<ShardCollider>(1, Allocator.Temp);

            colliders[0] = collider;

            ShardBodyHandle body = world.CreateBody(
                new Pose
                {
                    position = position,
                    rotation = rotation
                },
                BodyType.Dynamic,
                mass,
                colliders,
                default);

            colliders.Dispose();

            return body;
        }

        private ShardBodyHandle CreateStaticBody(
            Pose pose,
            ShardCollider collider)
        {
            NativeArray<ShardCollider> colliders =
                new NativeArray<ShardCollider>(1, Allocator.Temp);

            colliders[0] = collider;

            ShardBodyHandle body = world.CreateBody(
                pose,
                BodyType.Static,
                0f,
                colliders,
                default);

            colliders.Dispose();

            return body;
        }
        
        void CreateVisualBody(
            string name,
            ShardBodyHandle body,
            Mesh mesh,
            Material material,
            float3 scale)
        {
            GameObject go = new GameObject(name);
            go.transform.SetParent(transform, false);
            go.transform.localScale = scale;

            MeshFilter filter = go.AddComponent<MeshFilter>();
            MeshRenderer renderer = go.AddComponent<MeshRenderer>();

            filter.sharedMesh = mesh;
            renderer.sharedMaterial = material;

            visualBodies.Add(new VisualBody
            {
                body = body,
                transform = go.transform
            });
        }

        void SyncVisual(VisualBody visual)
        {
            if (!world.TryGetPose(visual.body, out Pose pose))
            {
                visual.transform.gameObject.SetActive(false);
                return;
            }

            visual.transform.position = pose.position;
            visual.transform.rotation = pose.rotation;
            visual.transform.gameObject.SetActive(true);
        }

        static Mesh CreatePrimitiveMesh(PrimitiveType primitiveType)
        {
            GameObject go = GameObject.CreatePrimitive(primitiveType);
            Mesh mesh = go.GetComponent<MeshFilter>().sharedMesh;
            Destroy(go);
            return mesh;
        }

        static Pose IdentityPose()
        {
            return new Pose
            {
                position = float3.zero,
                rotation = quaternion.identity
            };
        }

        static ShardColliderMaterial DefaultMaterial()
        {
            return new ShardColliderMaterial
            {
                bounciness = 0.05f,
                frictionStatic = 0.8f,
                frictionDynamic = 0.6f,
                frictionRolling = 0.05f
            };
        }
    }
}