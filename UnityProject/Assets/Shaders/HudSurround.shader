// The first-person view's surroundings: an inward-facing sphere around the head with a vertical gradient
// (dark above and below, a slightly lighter band at eye level) and a very faint horizon line, drawn behind
// everything (queue Background, no depth write) so it never hides the floor grid or the model.
// Opaque; used only through the HudAssets material (no Shader.Find).
Shader "Hud/Surround"
{
    Properties
    {
        _Top ("Top colour", Color) = (0.040, 0.050, 0.070, 1)
        _Horizon ("Horizon band colour", Color) = (0.085, 0.105, 0.140, 1)
        _Bottom ("Bottom colour", Color) = (0.035, 0.043, 0.060, 1)
        _BandWidth ("Band width (sine of elevation)", Float) = 0.5
        _LineAlpha ("Horizon line alpha", Float) = 0.08
        _LineWidth ("Horizon line width (sine of elevation)", Float) = 0.0015
    }
    SubShader
    {
        Tags { "RenderType" = "Opaque" "Queue" = "Background" "RenderPipeline" = "UniversalPipeline" "IgnoreProjector" = "True" }
        Pass
        {
            Name "Surround"
            Cull Front
            ZWrite Off
            ZTest LEqual

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            CBUFFER_START(UnityPerMaterial)
                half4 _Top, _Horizon, _Bottom;
                float _BandWidth, _LineAlpha, _LineWidth;
            CBUFFER_END

            struct Attributes { float4 positionOS : POSITION; };
            struct Varyings { float4 positionCS : SV_POSITION; float3 dir : TEXCOORD0; };

            Varyings vert(Attributes v)
            {
                Varyings o;
                o.positionCS = TransformObjectToHClip(v.positionOS.xyz);
                o.dir = v.positionOS.xyz;   // the sphere is centred on the head and not rotated: object y = up
                return o;
            }

            half4 frag(Varyings i) : SV_Target
            {
                float y = normalize(i.dir).y;
                float a = abs(y);
                half3 end = y > 0 ? _Top.rgb : _Bottom.rgb;
                float band = 1.0 - smoothstep(0.0, _BandWidth, a);
                half3 col = lerp(end, _Horizon.rgb, band * band);
                float w = max(fwidth(y), 1e-5);
                float hl = 1.0 - smoothstep(_LineWidth * 0.5, _LineWidth * 0.5 + w, a);
                col = lerp(col, half3(1, 1, 1), _LineAlpha * hl);
                return half4(col, 1);
            }
            ENDHLSL
        }
    }
}
