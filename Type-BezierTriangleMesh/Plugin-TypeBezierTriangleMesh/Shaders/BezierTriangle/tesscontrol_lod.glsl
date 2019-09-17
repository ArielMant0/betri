#version 400

#define SG_REQUEST_POSVS

uniform int tessAmount;

layout(vertices = 3) out;

// sq, lin, const
const vec3 atten = vec3(0.001, 0.08, 0.3);
//const vec3 atten = vec3(0.001, 0.1, 0.5);

// id between 0-2
float getTessLevel(vec3 pos)
{
	// squared distance
	float d2 = dot(pos, pos);
	float d = sqrt(d2);

	//float lod = 1.0 / (dot(atten, vec3(d2, d, 1.0)));

	//return 1.0 / (dot(atten, vec3(d2, d, 1.0)));
	return tessAmount / max(d, 1);
}

void main()
{
	sg_MapIO(gl_InvocationID);

  	// lod based on distance to viewer
	vec3 edgePos_1 = (SG_INPUT_POSVS[0].xyz
		+ SG_INPUT_POSVS[0 + 1].xyz) / 2.0;
	vec3 edgePos_2 = (SG_INPUT_POSVS[0 + 1].xyz
		+ SG_INPUT_POSVS[0 + 2].xyz) / 2.0;
	vec3 edgePos_3 = (SG_INPUT_POSVS[0 + 2].xyz
		+ SG_INPUT_POSVS[0].xyz) / 2.0;
	vec3 center = (edgePos_1 + edgePos_2 + edgePos_3) / 3.0;

	vec3 edgePos = (SG_INPUT_POSVS[gl_InvocationID].xyz
		+ SG_INPUT_POSVS[(gl_InvocationID + 1) % 3].xyz) / 2.0;

	float p = getTessLevel(SG_INPUT_POSVS[gl_InvocationID].xyz);

	// Write only for the first call
	if (gl_InvocationID == 0)
	{

		//gl_TessLevelInner[0] = 3;
		//gl_TessLevelOuter[gl_InvocationID] = getTessLevel(edgePos);
		//gl_TessLevelOuter[0] = gl_PrimitiveID % 2 == 0 ? getTessLevel(edgePos_3) : getTessLevel(edgePos_2);
		//gl_TessLevelOuter[1] = 3;
		//gl_TessLevelOuter[2] = 3;


		gl_TessLevelInner[0] = p;
		gl_TessLevelOuter[0] = p;
		gl_TessLevelOuter[1] = p;
		gl_TessLevelOuter[2] = p;

	}
}
