///////////////////////////////////////////////////////////////////////////////
// Define Types
///////////////////////////////////////////////////////////////////////////////
#define MAX_SCENE_BOUNDS 1000.0

///////////////////////////////////////////////////////////////////////////////
// Header
///////////////////////////////////////////////////////////////////////////////
// In -------------------------------------------------------------------------
in vec3 vRayOrigin;
in vec3 vRayDirection;
flat in int index; // TODO name

// Uniforms--------------------------------------------------------------------
uniform sampler2D controlPointTex;
uniform sampler2D uvCoordTex;
uniform sampler2D exampleTex;
uniform mat4 viewMatrix;
uniform vec3 campos;

uniform float b_error;
uniform float d_error;
uniform float uCurvatureScale;

///////////////////////////////////////////////////////////////////////////////
// Structs
///////////////////////////////////////////////////////////////////////////////
struct btriangle {
	vec3[CPSUM] cps;
} bt;

struct hitinfo {
	vec3 baryCoords;
	vec3 normal;
	vec3 position;
	vec3 curve;
} hit;

///////////////////////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////////////////////
#if GRAD > 1
	vec2 baryCoords[4] = vec2[] ( vec2(1.0, 0.0), vec2(0.0, 1.0), vec2(0.0, 0.0), vec2(1.0/3.0) );
#else
	vec2 baryCoords[1] = vec2[] ( vec2(0.0, 0.0) );
#endif

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////
#if GRAD > 1
void reorder(vec3 ray_origin, vec3 ray_direction)
{
	float dist = 0.0;

	// TODO why is the order like this
	float tmp[4] = float[] (
		dot(bt.cps[0], ray_direction),
		dot(bt.cps[CPSUM-1], ray_direction),
		dot(bt.cps[GRAD], ray_direction),
		dot((bt.cps[1] + bt.cps[3] + bt.cps[4]) / 3.0, ray_direction)
	);

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
#endif

///////////////////////////////////////////////////////////////////////////////
// Beziertriangle Intersect
///////////////////////////////////////////////////////////////////////////////

// TODO naming is inconsistent - ray_origin vs origin
/**
 * Test one beziertriangle againt a ray.
 *
 * @in origin Origin of the ray
 * @in dir Direction of the ray
 */
void intersectBTriangle(vec3 ray_origin, vec3 ray_direction)
{
#if GRAD > 1
	// TODO heavy on performance maybe not ordering but rather search for i min
	reorder(ray_origin, ray_direction);
#endif

	// TODO
	// Get perpendicular ray by switching coords
	//vec3 normal_1 = ray_direction.zyx;
	// TODO normalize
	vec3 normal_1 = normalize(cross(ray_direction, vec3(-1.0)));
	vec3 normal_2 = normalize(cross(ray_direction, normal_1));
	// TODO warum tun wir das?
	// distance to startpoint
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
			hit.baryCoords = vec3(result, z);
			hit.position = B_uv;
			hit.normal = normalize(cross(dBs, dBt));

			// Curvature calculation
			// http://www.math.harvard.edu/archive/21b_fall_04/exhibits/2dmatrices/index.html
			// http://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node29.html
			// http://u.math.biu.ac.il/~katzmik/goldman05.pdf
#ifdef SG_OUTPUT_GAUSS_CURVATURE
			// BEGIN dsdsB
			// END dsdsB

			// BEGIN dsdtB
			// END dsdtB

			// BEGIN dtdtB
			// END dtdtB

			float tDots = dot(dBt, dBs);
			mat2 fi = mat2(
				dot(dBs, dBs), tDots,
				tDots, dot(dBt, dBt)
			);
			float stDotNormal = dot(dsdtB, hit.normal);
			mat2 fii = mat2(
				dot(dsdsB, hit.normal), stDotNormal,
				stDotNormal, dot(dtdtB, hit.normal)
			);
			float gauss = (fii[0][0] * fii[1][1] - fii[1][0] * fii[0][1]) / (fi[0][0] * fi[1][1] - fi[1][0] * fi[0][1]);

			// set gauss curvature
			hit.curve = vec3(gauss, 0.0, -gauss) / uCurvatureScale;
#endif
#ifdef SG_OUTPUT_MEAN_CURVATURE
			// BEGIN dsdsB
			// END dsdsB

			// BEGIN dsdtB
			// END dsdtB

			// BEGIN dtdtB
			// END dtdtB

			float tDots = dot(dBt, dBs);
			mat2 fi = mat2(
				dot(dBs, dBs), tDots,
				tDots, dot(dBt, dBt)
			);
			float stDotNormal = -dot(dsdtB, hit.normal);
			mat2 combine =  fi * mat2(
				dot(dtdtB, hit.normal), stDotNormal,
				stDotNormal, dot(dsdsB, hit.normal)
			);
			float mean = (combine[0][0] + combine[1][1]) / (2.0 * (fi[0][0] * fi[1][1] - fi[1][0] * fi[0][1]));

			// set mean curvature
			hit.curve = vec3(mean, 0.0, -mean) / uCurvatureScale;
#endif

			return;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////
void main(void)
{
	// TODO var names
	vec3 tmp = campos;
	vec3 ray_direction = normalize(vec4(vRayDirection, 1.0).xyz - campos);
	vec3 ray_origin = tmp;

	///////////
	// Setup //
	///////////
	hit = hitinfo(vec3(-1), vec3(0.0), vec3(0.0), vec3(0.0));

	for (int i = 0; i < bt.cps.length(); i++)
	{
		bt.cps[i] = texelFetch(controlPointTex, ivec2(i, index), 0).xyz;
	}

	//////////////////////
	// Get Intersection //
	//////////////////////
	intersectBTriangle(ray_origin, ray_direction);

	if (hit.baryCoords.x > -0.5)
	{
		///////////////
		// Get Color //
		///////////////
#ifdef TEXTURED
		vec2 uv = texelFetch(uvCoordTex, ivec2(0, index), 0).xy * hit.baryCoords.y +
			texelFetch(uvCoordTex, ivec2(1, index), 0).xy * hit.baryCoords.z +
			texelFetch(uvCoordTex, ivec2(2, index), 0).xy * hit.baryCoords.x;
		vec4 color = texture(exampleTex, uv);
#else
		vec2 uv = vec2(0.5);
		vec4 color = vec4(0.0, 0.0, 0.0, 1.0);
#endif
		vec4 sg_cColor = color;

		///////////////////////
		// Reset Depth Value //
		///////////////////////
		// https://community.khronos.org/t/playing-with-gl-fragdepth/57016/7
		// https://stackoverflow.com/questions/10264949/glsl-gl-fragcoord-z-calculation-and-setting-gl-fragdepth
		vec4 n_pos = g_mWVP * vec4(hit.position, 1.0);

		float ndc_depth = n_pos.z / n_pos.w;

		float far = gl_DepthRange.far;
		float near = gl_DepthRange.near;
		float depth = (((far-near) * ndc_depth) + near + far) / 2.0;
// TODO why is this needed?
#ifndef SHOWBVOLUME
		gl_FragDepth = depth;
#endif

		////////////////
		// Set Output //
		////////////////
#ifdef SG_OUTPUT_PHONGCOLOR
		vec3 sg_vPosOS = hit.position;
		vec3 sg_vNormalOS = hit.normal;
		vec3 sg_vNormalVS = (g_mWV * vec4(sg_vNormalOS, 0.0)).xyz;
		vec3 sg_vPosVS = (g_mWV * vec4(sg_vPosOS, 0.0)).xyz;
		SG_FRAGMENT_LIGHTING
		color = sg_cColor;
#endif
#ifdef SG_OUTPUT_NORMAL
		color.rgb = hit.normal;
#endif
#ifdef SG_OUTPUT_DEPTH
		color.rgb = vec3(gl_FragDepth);
#endif
#ifdef SG_OUTPUT_UV
		color.rgb = vec3(uv, 0.0);
#endif
#ifdef SG_OUTPUT_MEAN_CURVATURE
		color.rgb = hit.curve;
#endif
#ifdef SG_OUTPUT_GAUSS_CURVATURE
		color.rgb = hit.curve;
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
