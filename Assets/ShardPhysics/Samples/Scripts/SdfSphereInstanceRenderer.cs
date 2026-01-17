using System;
using UnityEngine;
using UnityEngine.Rendering;

namespace Shard.Samples
{
    /// <summary>
    /// Instanced renderer for raymarched SDF spheres.
    ///
    /// Shader assumptions:
    /// - Proxy mesh is a unit cube in object space [-0.5 .. 0.5]
    /// - SDF is a canonical sphere at origin with radius = 0.5 (object space)
    /// - World-space radius is achieved by UNIFORM scaling of the instance matrix
    ///     objectScale = worldRadius / 0.5 = worldRadius * 2
    /// </summary>
    public sealed class SdfSphereInstancedRenderer
    {
        public const int MaxBatchSize = 1023;

        readonly Material _material;
        readonly Mesh _proxyMesh;
        readonly int _layer;

        readonly MaterialPropertyBlock _mpb = new MaterialPropertyBlock();

        readonly Matrix4x4[] _matrices = new Matrix4x4[MaxBatchSize];
        readonly Vector4[] _tints = new Vector4[MaxBatchSize];
        readonly float[] _metallic = new float[MaxBatchSize];
        readonly float[] _smoothness = new float[MaxBatchSize];

        // Canonical SDF radius in object space (shader constant)
        const float SdfRadiusObjectSpace = 0.5f;

        public SdfSphereInstancedRenderer(Material material, Mesh proxyMesh, int layer = 0)
        {
            _material = material ? material : throw new ArgumentNullException(nameof(material));
            _proxyMesh = proxyMesh ? proxyMesh : throw new ArgumentNullException(nameof(proxyMesh));
            _layer = layer;
        }

        public void Render(
            Vector3[] positions,
            Quaternion[] rotations,
            float[] worldRadii,
            Color[] colors = null,
            float[] metallic = null,
            float[] smoothness = null,
            ShadowCastingMode shadows = ShadowCastingMode.On,
            bool receiveShadows = true,
            Camera camera = null,
            LightProbeUsage lightProbes = LightProbeUsage.BlendProbes)
        {
            if (positions == null) throw new ArgumentNullException(nameof(positions));
            if (rotations == null) throw new ArgumentNullException(nameof(rotations));
            if (worldRadii == null) throw new ArgumentNullException(nameof(worldRadii));

            int countTotal = positions.Length;
            if (rotations.Length != countTotal) throw new ArgumentException("rotations.Length must match positions.Length");
            if (worldRadii.Length != countTotal) throw new ArgumentException("worldRadii.Length must match positions.Length");
            if (colors != null && colors.Length != countTotal) throw new ArgumentException("colors.Length must match positions.Length");
            if (metallic != null && metallic.Length != countTotal) throw new ArgumentException("metallic.Length must match positions.Length");
            if (smoothness != null && smoothness.Length != countTotal) throw new ArgumentException("smoothness.Length must match positions.Length");

            if (countTotal == 0)
                return;

            int offset = 0;
            while (offset < countTotal)
            {
                int batchCount = Mathf.Min(MaxBatchSize, countTotal - offset);

                for (int i = 0; i < batchCount; i++)
                {
                    int idx = offset + i;

                    float worldRadius = Mathf.Max(0.0001f, worldRadii[idx]);

                    // objectScale = worldRadius / 0.5 = worldRadius * 2
                    float objectScale = worldRadius / SdfRadiusObjectSpace;

                    _matrices[i] = Matrix4x4.TRS(
                        positions[idx],
                        rotations[idx],
                        new Vector3(objectScale, objectScale, objectScale)
                    );

                    Color c = (colors != null) ? colors[idx] : Color.white;
                    _tints[i] = new Vector4(c.r, c.g, c.b, c.a);

                    _metallic[i] = (metallic != null) ? Mathf.Clamp01(metallic[idx]) : 0f;
                    _smoothness[i] = (smoothness != null) ? Mathf.Clamp01(smoothness[idx]) : 0.5f;
                }

                _mpb.Clear();
                _mpb.SetVectorArray("_Tint", _tints);
                _mpb.SetFloatArray("_Metallic", _metallic);
                _mpb.SetFloatArray("_Smoothness", _smoothness);

                Graphics.DrawMeshInstanced(
                    _proxyMesh,
                    0,
                    _material,
                    _matrices,
                    batchCount,
                    _mpb,
                    shadows,
                    receiveShadows,
                    _layer,
                    camera,
                    lightProbes
                );

                offset += batchCount;
            }
        }
    }
}
