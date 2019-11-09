#version 400

#define SG_REQUEST_POSVS
#define SG_REQUEST_TEXCOORD
#define SG_REQUEST_NORMALVS
  
uniform int tessAmount;
uniform vec3 campos; // TODO take a deeper look if this camera position is correct for the calculations

layout(vertices = 3) out;

//out vec3 position[];

// sq, lin, const
const vec3 atten = vec3(0.001, 0.08, 0.3);
//const vec3 atten = vec3(0.001, 0.1, 0.5);

// TODO rename
// http://ogldev.org/www/tutorial30/tutorial30.html
float GetTessLevel(float Distance0, float Distance1)
{
    float AvgDistance = (Distance0 + Distance1) / 2.0;
	return tessAmount / max(AvgDistance, 1);
}

void main()
{
	sg_MapIO(gl_InvocationID);

	// Calculate the distance from the camera to the three control points
    float EyeToVertexDistance0 = distance(vec4(campos, 1.0), SG_INPUT_POSOS[0]);
    float EyeToVertexDistance1 = distance(vec4(campos, 1.0), SG_INPUT_POSOS[1]);
    float EyeToVertexDistance2 = distance(vec4(campos, 1.0), SG_INPUT_POSOS[2]);

    // Calculate the tessellation levels
    gl_TessLevelOuter[0] = GetTessLevel(EyeToVertexDistance1, EyeToVertexDistance2);
    gl_TessLevelOuter[1] = GetTessLevel(EyeToVertexDistance2, EyeToVertexDistance0);
    gl_TessLevelOuter[2] = GetTessLevel(EyeToVertexDistance0, EyeToVertexDistance1);
	// TODO this could be changed
    gl_TessLevelInner[0] = gl_TessLevelOuter[2];
}
