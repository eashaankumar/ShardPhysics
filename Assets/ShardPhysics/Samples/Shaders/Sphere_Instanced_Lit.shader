Shader "ShardPhysics/SDF/Sphere_Instanced_Lit"
{
    Properties
    {
        _MaxSteps("Max Steps", Range(8,256)) = 128
        _MaxDist("Max Dist (Object Space)", Float) = 5.0
        _HitEps("Hit Epsilon", Range(0.00001, 0.01)) = 0.001
    }

    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" }

        Pass
        {
            Name "FORWARD"
            Tags { "LightMode"="ForwardBase" }

            Cull Back
            ZWrite On
            ZTest LEqual

            CGPROGRAM
            #pragma target 3.0
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_fwdbase
            #pragma multi_compile_instancing

            #include "UnityCG.cginc"
            #include "Lighting.cginc"
            #include "AutoLight.cginc"

            float _MaxSteps;
            float _MaxDist;
            float _HitEps;

            // ---- INSTANCED ONLY ----
            UNITY_INSTANCING_BUFFER_START(Props)
                UNITY_DEFINE_INSTANCED_PROP(float4, _Tint)
                UNITY_DEFINE_INSTANCED_PROP(float,  _Metallic)
                UNITY_DEFINE_INSTANCED_PROP(float,  _Smoothness)
            UNITY_INSTANCING_BUFFER_END(Props)

            struct appdata
            {
                float4 vertex : POSITION;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float4 pos       : SV_POSITION;
                float4 screenPos : TEXCOORD0;
                float3 posWS     : TEXCOORD1;

                UNITY_LIGHTING_COORDS(2,3)
                UNITY_VERTEX_OUTPUT_STEREO
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            // Unit cube [-0.5..0.5], inscribed sphere radius = 0.5
            inline float sdSphere(float3 pOS)
            {
                return length(pOS) - 0.5;
            }

            inline float3 estimateNormalOS(float3 pOS)
            {
                const float e = 0.002;
                return normalize(float3(
                    sdSphere(pOS + float3(e,0,0)) - sdSphere(pOS - float3(e,0,0)),
                    sdSphere(pOS + float3(0,e,0)) - sdSphere(pOS - float3(0,e,0)),
                    sdSphere(pOS + float3(0,0,e)) - sdSphere(pOS - float3(0,0,e))
                ));
            }

            v2f vert(appdata v)
            {
                v2f o;
                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_TRANSFER_INSTANCE_ID(v, o);

                o.pos = UnityObjectToClipPos(v.vertex);
                o.screenPos = ComputeScreenPos(o.pos);
                o.posWS = mul(unity_ObjectToWorld, v.vertex).xyz;

                TRANSFER_VERTEX_TO_FRAGMENT(o);
                return o;
            }

            struct FragOut { float4 color : SV_Target; };

            FragOut frag(v2f i)
            {
                UNITY_SETUP_INSTANCE_ID(i);

                float4 tint = UNITY_ACCESS_INSTANCED_PROP(Props, _Tint);
                float metallic = saturate(UNITY_ACCESS_INSTANCED_PROP(Props, _Metallic));
                float smoothness = saturate(UNITY_ACCESS_INSTANCED_PROP(Props, _Smoothness));

                // ---- CORRECT BUILT-IN RAY CONSTRUCTION ----
                float2 uv = i.screenPos.xy / i.screenPos.w;
                float2 ndc = uv * 2.0 - 1.0;

                float4 rayVS4 = mul(unity_CameraInvProjection, float4(ndc, 1, 1));
                float3 rayVS = normalize(rayVS4.xyz / rayVS4.w);

                float3 rayWS = normalize(mul((float3x3)UNITY_MATRIX_I_V, rayVS));
                float3 rdOS = normalize(mul((float3x3)unity_WorldToObject, rayWS));
                float3 roOS = mul(unity_WorldToObject, float4(_WorldSpaceCameraPos, 1)).xyz;

                // ---- RAYMARCH ----
                float t = 0;
                float3 pOS;
                bool hit = false;

                [loop]
                for (int s = 0; s < (int)_MaxSteps; s++)
                {
                    pOS = roOS + rdOS * t;
                    float d = sdSphere(pOS);
                    if (d < _HitEps) { hit = true; break; }
                    t += max(d, 1e-4);
                    if (t > _MaxDist) break;
                }

                if (!hit) discard;

                float3 pWS = mul(unity_ObjectToWorld, float4(pOS,1)).xyz;
                float3 nWS = UnityObjectToWorldNormal(estimateNormalOS(pOS));

                float3 V = normalize(_WorldSpaceCameraPos - pWS);
                float3 L = normalize(UnityWorldSpaceLightDir(pWS));

                UNITY_LIGHT_ATTENUATION(atten, i, pWS);

                float3 albedo = tint.rgb;
                float3 diffuse = albedo * saturate(dot(nWS, L));
                float3 ambient = UNITY_LIGHTMODEL_AMBIENT.rgb * albedo;

                FragOut o;
                o.color = float4((diffuse + ambient) * atten, tint.a);
                return o;
            }
            ENDCG
        }
    }

    FallBack Off
}
