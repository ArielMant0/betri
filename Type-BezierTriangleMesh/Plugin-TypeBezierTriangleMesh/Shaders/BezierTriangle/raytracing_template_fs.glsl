///////////////////////////////////////////////////////////////////////////////
// Define Types
///////////////////////////////////////////////////////////////////////////////
#define MAX_SCENE_BOUNDS 1000.0
#define POSITIONS 4

///////////////////////////////////////////////////////////////////////////////
// Header
///////////////////////////////////////////////////////////////////////////////
// In -------------------------------------------------------------------------
in vec3 vRayOrigin;
in vec3 vRayDirection;
flat in int index; // TODO name

// Uniforms--------------------------------------------------------------------
uniform sampler2D btriangles;
uniform mat4 viewMatrix;
uniform vec3 campos;

uniform float b_error;
uniform float d_error;

///////////////////////////////////////////////////////////////////////////////
// Structs
///////////////////////////////////////////////////////////////////////////////
struct btriangle {
	vec3[CPSUM] cps;
} bt;

// TODO remove unnessessary informations
// lambda - distances to the intersection on the ray
// id - object index
struct hitinfo {
	vec2 lambda;
	int id;
	vec4 color;
};

///////////////////////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////////////////////
vec3 g_color = vec3(0.0);
vec3 g_position = vec3(0.0);

vec2 baryCoords[POSITIONS] = vec2[] ( vec2(1.0, 0.0), vec2(0.0, 1.0), vec2(0.0, 0.0), vec2(1.0/3.0) );
//vec2 baryCoords[POSITIONS] = vec2[] ( vec2(0.0, 0.0) );

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////
void reorder(vec3 ray_origin, vec3 ray_direction)
{
	float dist = 0.0;

	// TODO why is the order like this
	float tmp[POSITIONS] = float[] (
		dot(bt.cps[0], ray_direction),
		dot(bt.cps[CPSUM-1], ray_direction),
		dot(bt.cps[GRAD], ray_direction),
		dot((bt.cps[1] + bt.cps[3] + bt.cps[4]) / 3.0, ray_direction)
	);

	/*
	float tmp[POSITIONS] = float[] (
		length(bt.cp0 - ray_origin),
		length(bt.cp5 - ray_origin),
		length(bt.cp2 - ray_origin),
		100.0
		//length((bt.cp1 + bt.cp3 + bt.cp4) / 3.0 - ray_origin)
	);
	*/

	for (int j = 1; j < baryCoords.length(); ++j)
	{
		vec2 key = baryCoords[j];
		float k2 = tmp[j];
		int i = j - 1;
		while (i >= 0 && tmp[i] > k2)
		{
			baryCoords[i+1] = baryCoords[i];
			tmp[i+1] = tmp[i];
			--i;
		}
		baryCoords[i+1] = key;
		tmp[i+1] = k2;
	}
}

///////////////////////////////////////////////////////////////////////////////
// Beziertriangle Intersect
///////////////////////////////////////////////////////////////////////////////

// TODO naming is inconsistent - ray_origin vs origin
/**
 * Test one beziertriangle againt a ray.
 *
 * @in origin Origin of the ray
 * @in dir Direction of the ray
 * @in box the btriangle to test against
 * @return A three component vector with the nearest intersection lambda (t) and u, v
 */
vec3 intersectBTriangle(vec3 ray_origin, vec3 ray_direction)
{
	// TODO heavy on performance maybe not ordering but rather search for i min
	reorder(ray_origin, ray_direction);

	// TODO
	// Get perpendicular ray by switching coords
	//vec3 normal_1 = ray_direction.zyx;
	vec3 normal_1 = cross(ray_direction, vec3(-1.0));
	vec3 normal_2 = cross(ray_direction, normal_1);
	// TODO warum tun wir das?
	float d_1 = -dot(normal_1, ray_origin);
	float d_2 = -dot(normal_2, ray_origin);

	// BEGIN QS
	// END QS

	// N1 * B(u, v) + d1 = 0
	// N2 * B(u, v) + d2 = 0

	mat2 inv_jacobi = mat2(0.0);
	// TODO is this nessessary?
	float inv_constant = 0;
	// TODO was das?
	vec2 R = vec2(0.0);

	vec3 dBs;
	vec3 dBt;
	vec3 B_uv;
	vec3 B_uv_old;
	vec2 result;
	vec3 result_2;

	bool found = false;

	for (int j = 0; j < baryCoords.length(); j++) {
		// Initial guess
		result = baryCoords[j];

		float s = 0.0;
		float t = 0.0;

		for (int i = 0; i < NEWTON_IT_COUNT; i++)
		{
			s = result.x;
			t = result.y;

			// BEGIN DBS
			// END DBS

			// BEGIN DBT
			// END DBT

			// BEGIN BUV
			// END BUV

			R = vec2(dot(normal_1, B_uv) + d_1, dot(normal_2, B_uv) + d_2);

			float dotN_1B_s = dot(normal_1, dBs);
			float dotN_1B_t = dot(normal_1, dBt);
			float dotN_2B_s = dot(normal_2, dBs);
			float dotN_2B_t = dot(normal_2, dBt);

			inv_constant = 1 / (dotN_1B_s * dotN_2B_t - dotN_1B_t * dotN_2B_s);

			inv_jacobi = mat2(
				dotN_2B_t * inv_constant, -dotN_2B_s * inv_constant,
				-dotN_1B_t * inv_constant, dotN_1B_s * inv_constant
			);

			// Newton iteration
			result = result - inv_jacobi * R;
		}

		float z = 1.0 - result.x - result.y;
		if (result.x >= -b_error && result.x <= 1.0 + b_error &&
			result.y >= -b_error && result.y <= 1.0 + b_error &&
			z >= -b_error &&
			abs(dot(normal_1, B_uv) + d_1) < d_error &&
		    abs(dot(normal_2, B_uv) + d_2) < d_error
		)
		{
			//outFragment = vec4(vec3(float(j)/4.0), 1.0);
			g_color = B_uv;
			g_position = B_uv;

		    return vec3(result, z);
			/*
			if (!found)
			{
				result_2 = vec3(result, z);
				found = true;
			}
			else if (dot(ray_direction, result.x * bt.cp0 + result.y * bt.cp2 + z * bt.cp5) <
				dot(ray_direction, result_2.x * bt.cp0 + result_2.y * bt.cp2 + result_2.z * bt.cp5)
			)
			{
				//result_2 = vec3(result, z);
				result_2 = vec3(-1.0);
			}
			*/
		}
	}

	//if (found)
	//	return vec3(result_2);

	// discard;
	//outFragment = vec4(0.0, 0.0, 1.0, 1.0);
    return vec3(-1.0);
}

/**
 * Test all Boxes of the cubes texture against the ray that is given.
 *
 * @in origin Origin of the ray
 * @in dir Direction of the ray
 * @out hitinfo Outvariable - Information about the nearest and selected hit
 */
bool intersectBTriangles(vec3 origin, vec3 dir, inout hitinfo info)
{
	float smallest = info.lambda.x;
	bool found = false;
	int y = index; // TODO

	// TODO direkt construktor nehmen und mit texture stuff befüllen
	for (int i = 0; i < bt.cps.length(); i++)
	{
		bt.cps[i] = texelFetch(btriangles, ivec2(i, y), 0).xyz;
	}

	vec3 lambda = intersectBTriangle(origin, dir);
	/*if (lambda.x >= 0.0 && lambda.x <= 1.0 &&
		lambda.y >= 0.0 && lambda.y <= 1.0 &&
		lambda.z >= 0.0 && lambda.z <= 1.0
	)*/
	if (lambda.x > -0.5)
	{
		// TODO this are the barycentric coord (used for normal) but this should be the distance on the ray
		info.lambda = lambda.xy;
		info.id = y;
		//smallest = lambda;
		found = true;
	}
	return found;
}

///////////////////////////////////////////////////////////////////////////////
// Tracing
///////////////////////////////////////////////////////////////////////////////

/**
 * Trace a specific ray against all objects of the scene and color them
 * according to the Objectattributes.
 *
 * @in origin Origin of the ray
 * @in dir Direction of the ray
 * @return the color for this ray (pixel)
 */
hitinfo trace(vec3 origin, vec3 dir)
{
	hitinfo hit = hitinfo(vec2(MAX_SCENE_BOUNDS), -1, vec4(0.0, 0.0, 0.0, 1.0));
	if (intersectBTriangles(origin, dir, hit))
	{
		// TODO
		//hit.color.rgb = texture(btriangles, vec2(0.0, hit.id)).rgb;
		hit.color.rgb = vec3(1.0, 1.0, 1.0);
		//hit.color.rgb = vec3(g_color);
	}

	return hit;
}

///////////////////////////////////////////////////////////////////////////////
// Normals
///////////////////////////////////////////////////////////////////////////////

vec3 calcNormal(vec3 ray_origin, vec3 ray_direction, hitinfo hit)
{
	// TODO refactor everything, this is a stupid doublication

	int y = index; // TODO



	vec3 q_1 = bt.cps[0] + bt.cps[2] - 2 * bt.cps[1];
	vec3 q_2 = 2 * bt.cps[3] - 2 * bt.cps[1] - 2 * bt.cps[4] + 2 * bt.cps[2];
	vec3 q_3 = bt.cps[5] - 2 * bt.cps[4] + bt.cps[2];
	vec3 q_4 = 2 * bt.cps[4] - 2 * bt.cps[2];
	vec3 q_5 = 2 * bt.cps[1] - 2 * bt.cps[2];
	vec3 q_6 = bt.cps[2];


	float s = hit.lambda.x;
	float t = hit.lambda.y;

	// Partial derivate by s
	vec3 dBs = 2 * q_1 * s +
		q_2 * t +
		q_5;

	// Partial derivate by t
	vec3 dBt = q_2 * s +
		2 * q_3 * t +
		q_4;

	return normalize(cross(dBs, dBt));
}

///////////////////////////////////////////////////////////////////////////////
// Curvature
///////////////////////////////////////////////////////////////////////////////

// http://www.math.harvard.edu/archive/21b_fall_04/exhibits/2dmatrices/index.html
// http://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node29.html
vec3 calcCurvature(vec3 ray_origin, vec3 ray_direction, hitinfo hit, vec3 normal)
{
	// TODO refactor everything, this is a stupid doublication

	int y = index; // TODO

	vec3 q_1 = bt.cps[0] + bt.cps[2] - 2 * bt.cps[1];
	vec3 q_2 = 2 * bt.cps[3] - 2 * bt.cps[1] - 2 * bt.cps[4] + 2 * bt.cps[2];
	vec3 q_3 = bt.cps[5] - 2 * bt.cps[4] + bt.cps[2];
	vec3 q_4 = 2 * bt.cps[4] - 2 * bt.cps[2];
	vec3 q_5 = 2 * bt.cps[1] - 2 * bt.cps[2];
	vec3 q_6 = bt.cps[2];


	float s = hit.lambda.x;
	float t = hit.lambda.y;

	// second partial derivate by s and s
	vec3 dBss = 2 * q_1;

	// second partial derivate by s and t
	vec3 dBst = q_2;

	// second partial derivate by t and t
	vec3 dBtt = 2 * q_3;

	mat2 secFund = mat2(
		dot(dBss, normal), dot(dBst, normal),
		dot(dBst, normal), dot(dBtt, normal)
	);

	float trace = secFund[0][0] + secFund[1][1];
	float det = secFund[0][0] * secFund[1][1] - secFund[1][0] * secFund[1][0];

	float l1 = trace * 0.5 + sqrt(trace * trace * 0.25 - det);
	float l2 = trace * 0.5 - sqrt(trace * trace * 0.25 - det);

	return vec3(-l1, -l2, 0.0);
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

void main(void)
{
	// TODO var names
	//vec3 tmp = (inverse(viewMatrix) * vec4(g_vCamPos, 1.0)).xyz;
	//vec3 tmp = g_vCamPos;
	//vec3 ray_direction = normalize((viewMatrix * vec4(vRayDirection,1.0)).xyz - tmp);
	//vec3 ray_direction = normalize((inverse(viewMatrix) * g_mWV * vec4(vRayDirection,1.0)).xyz - tmp);
	//vec3 ray_direction = normalize(g_mWV * vec4(vRayDirection,1.0)).xyz - tmp);

	vec3 tmp = campos;
	vec3 ray_direction = normalize(vec4(vRayDirection, 1.0).xyz - campos);
	vec3 ray_origin = tmp;

	hitinfo hit = trace(ray_origin, ray_direction);
	vec4 color = hit.color;
	vec4 sg_cColor = vec4(0.0);
	vec3 sg_vPosOS = g_position; // TODO

	if (hit.id > -1)
	{
		// https://community.khronos.org/t/playing-with-gl-fragdepth/57016/7
		// https://stackoverflow.com/questions/10264949/glsl-gl-fragcoord-z-calculation-and-setting-gl-fragdepth
		vec4 n_pos = g_mWVP * vec4(g_position, 1.0);

		float ndc_depth = n_pos.z / n_pos.w;

		float far = gl_DepthRange.far; 
		float near = gl_DepthRange.near;
		float depth = (((far-near) * ndc_depth) + near + far) / 2.0;
// TODO why is this needed?
#ifndef SHOWBVOLUME
		gl_FragDepth = depth;
#endif

		vec3 sg_vNormalOS = calcNormal(ray_origin, ray_direction, hit);
#ifdef SG_OUTPUT_NORMAL
		color.rgb = sg_vNormalOS;
#endif
#ifdef SG_OUTPUT_CURVATURE
		color.rgb = calcCurvature(ray_origin, ray_direction, hit, sg_vNormalOS);
#endif
#ifdef SG_OUTPUT_PHONGCOLOR
		vec3 sg_vNormalVS = (g_mWV * vec4(sg_vNormalOS, 0.0)).xyz;
		vec3 sg_vPosVS = (g_mWV * vec4(sg_vPosOS, 0.0)).xyz;
		SG_FRAGMENT_LIGHTING
		color *= sg_cColor;
#endif
#ifdef SG_OUTPUT_DEPTH
		color.rgb = vec3(gl_FragDepth);
#endif

		outFragment = vec4(color.rgb, 1.0);
	} else {
#ifdef SHOWBVOLUME
		outFragment = vec4(0.0, 0.0, 1.0, 1.0);
#endif
#ifndef SHOWBVOLUME
		discard;
#endif
	}
}
