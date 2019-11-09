
out vec3 vRayOrigin;
out vec3 vRayDirection;
flat out int index;

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
	index = int(inNormal.x);

/*
#ifdef SG_OUTPUT_POSOS
	SG_OUTPUT_POSOS = vec4(pos, 1);
#endif
#ifdef SG_OUTPUT_POSVS
	SG_OUTPUT_POSVS = g_mWV * vec4(pos, 1);
#endif
#ifdef SG_OUTPUT_POSCS
	SG_OUTPUT_POSCS = gl_Position;
#endif
#ifdef SG_OUTPUT_NORMALOS
	SG_OUTPUT_NORMALOS = surfaceNormal;
#endif
#ifdef SG_OUTPUT_NORMALVS
	SG_OUTPUT_NORMALVS = g_mWVIT * surfaceNormal;
#endif
*/
}
