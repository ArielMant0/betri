///////////////////////////////////////////////////////////////////////////////
// Defines
///////////////////////////////////////////////////////////////////////////////
#define SG_REQUEST_TEXCOORD

///////////////////////////////////////////////////////////////////////////////
// Header
///////////////////////////////////////////////////////////////////////////////
// Out -------------------------------------------------------------------------
out vec3 vRayOrigin;
out vec3 vRayDirection;
flat out int index;

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////
void main(void)
{
	//gl_Position = inPosition * g_mWVP;
	// TODO sollte das hier nicht nur die modelmatrix sein?
	//vec4 vertexPosition =  g_mWV * inPosition;
	vec4 vertexPosition = g_mWV * inPosition;
    gl_Position = g_mP * vertexPosition;

	//outVertexColor = g_mWV * inPosition;
	//outVertexPosCS = g_mWV * inPosition;
	//outVertexPosVS = g_mWV * inPosition;

	vec3 pos = vec3(0.0);
	//vec3 pos = g_vCamPos;
	// TODO not nessessary if correct value
	vRayOrigin = pos;
	// TODO das hier im fragmentshader tun
	//vRayDirection = vertexPosition.xyz;
	vRayDirection = inPosition.xyz;
	index = int(inTexCoord.x);
}
