#version 150

#define OPTION 1

in vec4 inPosition;

//uniform float u_viewportWidth; // TODO not needed we could simple use the aspect ratio _glState->aspect()
//uniform float u_viewportHeight;
//uniform float u_fovy;

uniform mat4 u_invproj;
uniform mat4 u_invmodelview;
uniform float u_near;
uniform float u_far;


#if OPTION == 0
uniform mat4 u_proj;
uniform mat4 u_modelview;
uniform vec3 g_vCamPos2;
#endif


//out vec3 vPosition;
//out vec2 vTexCoord;
out vec3 vOrigin;
out vec3 vRay;

void main()
{
	//vPosition = inPosition.xyz;
	vec2 pos = inPosition.xy;

#if OPTION == 0
	mat4 inv_modelviewproj = inverse(u_modelview * u_proj);
	vOrigin = vec3(3.0, 2.0, 7.0);
	vOrigin = g_vCamPos2;

	// TODO das geht besser
	vec4 tmp = vec4(0.0, 0.0, 0.0, 1.0);
	if (gl_VertexID == 0)
		tmp.xy = vec2(-1.0, 1.0);
	else if (gl_VertexID == 1)
		tmp.xy = vec2(-1.0, -1.0);
	else if (gl_VertexID == 2)
		tmp.xy = vec2(1.0, 1.0);
	else 
		tmp.xy = vec2(1.0, -1.0);

	tmp = tmp * inv_modelviewproj;
	tmp /= tmp.w;
	tmp -= vOrigin;

	vRay = tmp.xyz;
#else
	/*
	mat4 i_proj = mat4(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 1
	);
	*/

	// https://stackoverflow.com/questions/2354821/raycasting-how-to-properly-apply-a-projection-matrix?rq=1
	// Convert to Worldspace and project Origin on nearplane
	vOrigin = (u_invmodelview * u_invproj * vec4(pos, -1.0, 1.0) * u_near).xyz;
	vRay = (u_invmodelview * u_invproj * vec4(pos * (u_far - u_near), u_far + u_near, u_far - u_near)).xyz;
#endif

	gl_Position = inPosition;
	//gl_Position = vec4(pos, 0.0, 1.0);
}