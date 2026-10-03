// UI shader (UI/Default plus lens undistortion) for the first-person camera image. For each output pixel,
// the ideal (pinhole) ray is K^-1 * pixel; the plumb_bob model (k1 k2 p1 p2 k3) turns it into the pixel of
// the distorted camera image, which is sampled. UV flips keep working: they are in the texture coordinates
// the shader receives. Used only through the HudAssets material (no Shader.Find); the sim publishes D = 0,
// so it only runs for cameras that report a distortion.
Shader "Hud/UndistortImage"
{
    Properties
    {
        [PerRendererData] _MainTex ("Sprite Texture", 2D) = "white" {}
        _Color ("Tint", Color) = (1,1,1,1)

        _StencilComp ("Stencil Comparison", Float) = 8
        _Stencil ("Stencil ID", Float) = 0
        _StencilOp ("Stencil Operation", Float) = 0
        _StencilWriteMask ("Stencil Write Mask", Float) = 255
        _StencilReadMask ("Stencil Read Mask", Float) = 255
        _ColorMask ("Color Mask", Float) = 15
        [Toggle(UNITY_UI_ALPHACLIP)] _UseUIAlphaClip ("Use Alpha Clip", Float) = 0

        _K ("fx fy cx cy (pixels)", Vector) = (500, 500, 320, 240)
        _Dist ("k1 k2 p1 p2", Vector) = (0, 0, 0, 0)
        _Dist2 ("k3 width height", Vector) = (0, 640, 480, 0)
    }
    SubShader
    {
        Tags { "Queue" = "Transparent" "IgnoreProjector" = "True" "RenderType" = "Transparent" "PreviewType" = "Plane" "CanUseSpriteAtlas" = "True" }
        Stencil
        {
            Ref [_Stencil]
            Comp [_StencilComp]
            Pass [_StencilOp]
            ReadMask [_StencilReadMask]
            WriteMask [_StencilWriteMask]
        }
        Cull Off
        Lighting Off
        ZWrite Off
        ZTest [unity_GUIZTestMode]
        Blend SrcAlpha OneMinusSrcAlpha
        ColorMask [_ColorMask]

        Pass
        {
            Name "Default"
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 2.0
            #include "UnityCG.cginc"
            #include "UnityUI.cginc"
            #pragma multi_compile_local _ UNITY_UI_CLIP_RECT
            #pragma multi_compile_local _ UNITY_UI_ALPHACLIP

            struct appdata_t { float4 vertex : POSITION; float4 color : COLOR; float2 texcoord : TEXCOORD0; UNITY_VERTEX_INPUT_INSTANCE_ID };
            struct v2f
            {
                float4 vertex : SV_POSITION; fixed4 color : COLOR; float2 texcoord : TEXCOORD0;
                float4 worldPosition : TEXCOORD1; UNITY_VERTEX_OUTPUT_STEREO
            };

            sampler2D _MainTex;
            fixed4 _Color;
            fixed4 _TextureSampleAdd;
            float4 _ClipRect;
            float4 _K, _Dist, _Dist2;

            v2f vert(appdata_t v)
            {
                v2f OUT;
                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(OUT);
                OUT.worldPosition = v.vertex;
                OUT.vertex = UnityObjectToClipPos(OUT.worldPosition);
                OUT.texcoord = v.texcoord;
                OUT.color = v.color * _Color;
                return OUT;
            }

            // Source (distorted) texture coordinates for the ideal image coordinates `uv`.
            float2 Distort(float2 uv)
            {
                float2 size = _Dist2.yz;
                float2 n = (float2(uv.x * size.x, (1.0 - uv.y) * size.y) - _K.zw) / _K.xy;
                float r2 = dot(n, n);
                float radial = 1.0 + r2 * (_Dist.x + r2 * (_Dist.y + r2 * _Dist2.x));
                float2 d = n * radial + float2(2.0 * _Dist.z * n.x * n.y + _Dist.w * (r2 + 2.0 * n.x * n.x),
                                               _Dist.z * (r2 + 2.0 * n.y * n.y) + 2.0 * _Dist.w * n.x * n.y);
                float2 p = d * _K.xy + _K.zw;
                return float2(p.x / size.x, 1.0 - p.y / size.y);
            }

            fixed4 frag(v2f IN) : SV_Target
            {
                float2 s = Distort(IN.texcoord);
                half4 color = (tex2D(_MainTex, s) + _TextureSampleAdd) * IN.color;
                color.a *= step(0.0, s.x) * step(s.x, 1.0) * step(0.0, s.y) * step(s.y, 1.0);   // outside the camera image: transparent
                #ifdef UNITY_UI_CLIP_RECT
                color.a *= UnityGet2DClipping(IN.worldPosition.xy, _ClipRect);
                #endif
                #ifdef UNITY_UI_ALPHACLIP
                clip(color.a - 0.001);
                #endif
                return color;
            }
            ENDCG
        }
    }
}
