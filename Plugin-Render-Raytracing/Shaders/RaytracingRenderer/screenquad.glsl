#version 150

#define M_PI 3.1415926535897932384626433832795
#define OPTION 1

in vec4 inPosition;

uniform float u_viewportWidth; // TODO not needed we could simple use the aspect ratio _glState->aspect()
uniform float u_viewportHeight;
uniform float u_fovy;

uniform mat4 u_invproj;
uniform mat4 u_invmodelview;
uniform mat4 u_proj;
uniform mat4 u_modelview;
uniform float u_near;
uniform float u_far;

uniform vec3 g_vCamPos2;

out vec3 vPosition;
out vec2 vTexCoord;
out vec3 vOrigin;
out vec3 vRay;

void main()
{
	mat4 inv_modelviewproj = inverse(u_modelview * u_proj);
	vPosition = inPosition.xyz;
	vec2 pos = inPosition.xy;

	if (OPTION == 0)
	{
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
	}
	else if (OPTION == 1) 
	{
		// TODO probably not nessessary because inPosition contains the ones already
		/*
		if (gl_VertexID == 0)
			pos = vec2(-1.0, 1.0);
		else if (gl_VertexID == 1)
			pos = vec2(-1.0, -1.0);
		else if (gl_VertexID == 2)
			pos = vec2(1.0, 1.0);
		else 
			pos = vec2(1.0, -1.0);
		*/

		mat4 i_proj = mat4(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 1
		);

		// https://stackoverflow.com/questions/2354821/raycasting-how-to-properly-apply-a-projection-matrix?rq=1
		//vOrigin = (u_invmodelview * u_invproj * vec4(inPosition.xy, -1.0, 1.0) * u_near).xyz;
		// Convert to Worldspace and project Origin on nearplane
		vOrigin = (u_invmodelview * u_invproj * vec4(pos, -1.0, 1.0) * u_near).xyz;
		//vRay = (u_invmodelview * u_invproj * vec4(inPosition.xy * (u_far - u_near), u_far + u_near, u_far - u_near)).xyz;
		vRay = (u_invmodelview * u_invproj * vec4(pos * (u_far - u_near), u_far + u_near, u_far - u_near)).xyz;
	}

	gl_Position = inPosition;
	//gl_Position = vec4(pos, 0.0, 1.0);
	//gl_Position = tmp;
	vTexCoord = inPosition.xy * 0.5 + vec2(0.5, 0.5);
}