#version 400

#define SG_REQUEST_POSVS
#define SG_REQUEST_TEXCOORD
#define SG_REQUEST_NORMALVS

uniform int tessAmount;
uniform vec3 campos; // TODO take a deeper look if this camera position is correct for the calculations

uniform sampler2D controlPointTex;

layout(vertices = 3) out;

//out vec3 position[];

// TODO
// sq, lin, const
const vec3 atten = vec3(0.001, 0.08, 0.3);
//const vec3 atten = vec3(0.001, 0.1, 0.5);

// http://ogldev.org/www/tutorial30/tutorial30.html
#ifdef TESS_DISTANCE
float GetTessLevel(float Distance0, float Distance1)
{
    float AvgDistance = (Distance0 + Distance1) / 2.0;
	return tessAmount / max(AvgDistance, 1);
}
#endif

// https://developer.nvidia.com/gpugems/GPUGems2/gpugems2_chapter07.html
#ifdef TESS_FLATNESS
float GetTessLevel(vec3 start, vec3 end, vec3 cpTest)
{
	float dist1 = max(0.1, length(end - start));
	float dist2 = length(cpTest - (start + end) / 2.0);
	float ratio = dist2 / dist1;

	//return min(dist, tessAmount);
	return max(1.0, ratio * 64.0);
}
#endif

// TODO writen an mixed test where camera and flatness influence the tess amount
void main()
{
	sg_MapIO(gl_InvocationID);

#ifdef TESS_CONST
	// Calculate the tessellation levels
    gl_TessLevelOuter[0] = tessAmount;
    gl_TessLevelOuter[1] = tessAmount;
    gl_TessLevelOuter[2] = tessAmount;
	// TODO this could be changed
    gl_TessLevelInner[0] = gl_TessLevelOuter[2];
#endif
#ifdef TESS_DISTANCE
	// Calculate the distance from the camera to the three control points
    float EyeToVertexDistance0 = distance(vec4(campos, 1.0), SG_INPUT_POSOS[0]);
    float EyeToVertexDistance1 = distance(vec4(campos, 1.0), SG_INPUT_POSOS[1]);
    float EyeToVertexDistance2 = distance(vec4(campos, 1.0), SG_INPUT_POSOS[2]);

    // Calculate the tessellation levels
    gl_TessLevelOuter[0] = GetTessLevel(EyeToVertexDistance1, EyeToVertexDistance2);
    gl_TessLevelOuter[1] = GetTessLevel(EyeToVertexDistance2, EyeToVertexDistance0);
    gl_TessLevelOuter[2] = GetTessLevel(EyeToVertexDistance0, EyeToVertexDistance1);
    gl_TessLevelInner[0] = (gl_TessLevelOuter[0] + gl_TessLevelOuter[1] + gl_TessLevelOuter[2]) / 3.0;
#endif
#ifdef TESS_FLATNESS
	// TODO enable this for higher degree
	vec3 cp1 = texelFetch(controlPointTex, ivec2(1, gl_PrimitiveID), 0).xyz;
	vec3 cp2 = texelFetch(controlPointTex, ivec2(3, gl_PrimitiveID), 0).xyz;
	vec3 cp3 = texelFetch(controlPointTex, ivec2(4, gl_PrimitiveID), 0).xyz;

    // Calculate the tessellation levels
    gl_TessLevelOuter[0] = GetTessLevel(SG_INPUT_POSOS[1].xyz, SG_INPUT_POSOS[2].xyz, cp1);
    gl_TessLevelOuter[1] = GetTessLevel(SG_INPUT_POSOS[2].xyz, SG_INPUT_POSOS[0].xyz, cp2);
    gl_TessLevelOuter[2] = GetTessLevel(SG_INPUT_POSOS[0].xyz, SG_INPUT_POSOS[1].xyz, cp3);
    gl_TessLevelInner[0] = (gl_TessLevelOuter[0] + gl_TessLevelOuter[1] + gl_TessLevelOuter[2]) / 3.0;
#endif
}
