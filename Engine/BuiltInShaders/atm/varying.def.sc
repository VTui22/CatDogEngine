vec2  v_texcoord0        : TEXCOORD0 = vec2(0.0, 0.0);
vec3  v_worldPos         : TEXCOORD1 = vec3(0.0, 0.0, 0.0);
vec3  v_skyboxDir        : TEXCOORD2 = vec3(0.0, 0.0, 0.0);
vec3  v_bc               : TEXCOORD3 = vec3(0.0, 0.0, 0.0);
vec3  v_normal           : NORMAL    = vec3(0.0, 0.0, 1.0);
mat3  v_TBN              : TEXCOORD4;
vec4  v_color0           : COLOR0    = vec4(1.0, 0.0, 0.0, 1.0);
vec4  v_color1           : COLOR1    = vec4(1.0, 0.0, 0.0, 1.0);
flat  ivec4 v_indices    : BLENDINDICES = vec4(0, 0, 0, 0);
vec4  v_weight           : BLENDWEIGHT = vec4(1.0, 1.0, 1.0, 1.0);
vec2  v_alphaMapTexCoord : TEXCOORD5 = vec2(0.0, 0.0);

vec3  a_position         : POSITION;
vec3  a_normal           : NORMAL;
vec3  a_tangent          : TANGENT;
vec3  a_bitangent        : BITANGENT;
vec2  a_texcoord0        : TEXCOORD0;
vec3  a_color0           : COLOR0;
vec3  a_color1           : COLOR1;
ivec4 a_indices          : BLENDINDICES;
vec4  a_weight           : BLENDWEIGHT;