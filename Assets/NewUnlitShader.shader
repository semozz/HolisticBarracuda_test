Shader "Unlit/NewUnlitShader"
{
    Properties
    {
        //_Color ("Color", Color) = (1,1,1,1)
        _MainTex ("Albedo (RGB)", 2D) = "white" {}
       
    }
    SubShader
    {
        Tags { "RenderType" = "Transparent" "Queue" = "Transparent" }    //! 쉐이더 타입을 Transparent로 변경, Render Queue도 같이
        blend SrcAlpha OneMinusSrcAlpha    //! Blending 옵션 설정
 
        CGPROGRAM
      
        #pragma surface surf Lambert keepalpha    //! keepalpha는 Unity 5.0부터  SurfaceShader는 기본적으로 알파에 1.0값이 입력되는데 그것을 막아줍니다.
 
    
        sampler2D _MainTex;
 
        struct Input
        {
            float2 uv_MainTex;
			float4 color : Color;
        };
 
        fixed4 _Color;
 
        void surf (Input IN, inout SurfaceOutput o)
        {
            fixed4 c = tex2D (_MainTex, IN.uv_MainTex);
            o.Albedo = c.rgb * IN.color.rgb;
            o.Alpha = c.a * IN.color.a;
        }
        ENDCG
    }
    FallBack "Diffuse"
}
