///////////////////////////////////////////////////////////////////////////////
// Define Types
///////////////////////////////////////////////////////////////////////////////

#define TYPE_CUBE 0
#define TYPE_SPHERE 1
#define TYPE_TRIANGLE 2
#define TYPE_BTRIANGLE 3

///////////////////////////////////////////////////////////////////////////////
// Define Count
///////////////////////////////////////////////////////////////////////////////

#define NUM_BOXES 2
#define NUM_SPHERES 2
#define NUM_TRIANGLES 2
#define NUM_BTRIANGLES 1

///////////////////////////////////////////////////////////////////////////////
// Define Max distance
///////////////////////////////////////////////////////////////////////////////

#define MAX_SCENE_BOUNDS 1000.0
#define POSITIONS 4
#define NEWTON_IT_COUNT 4

in vec3 vRayOrigin;
in vec3 vRayDirection;
in float index; // TODO name

uniform sampler2D btriangles;
uniform mat4 viewMatrix;
uniform vec3 campos;

// TODO there are already lightuniform slots
uniform vec3 lig;

// TODO how to get a variable amout of cp's
// cp0 - controllpoint 0 and vertex 0
// cp1 - controllpoint 1
// cp2 - controllpoint 2 and vertex 1
// cp3 - controllpoint 3
// cp4 - controllpoint 4
// cp5 - controllpoint 5 and vertex 2
struct btriangle {
	vec3 cp0;
	vec3 cp1;
	vec3 cp2;
	vec3 cp3;
	vec3 cp4;
	vec3 cp5;
};

// lambda - distances to the intersection on the ray
// id - object index
struct hitinfo {
	vec2 lambda;
	int id;
	int type;
	vec4 color;
};

vec2 baryCoords[POSITIONS] = vec2[] ( vec2(1.0, 0.0), vec2(0.0, 1.0), vec2(0.0, 0.0), vec2(1.0/3.0) );
//vec2 baryCoords[POSITIONS] = vec2[] ( vec2(0.0, 0.0) );

void reorder(vec3 ray_origin, vec3 ray_direction, const btriangle bt)
{
	float dist = 0.0;
	
	// TODO why is the order like this
	float tmp[POSITIONS] = float[] ( 
		dot(bt.cp0, ray_direction),
		dot(bt.cp5, ray_direction),
		dot(bt.cp2, ray_direction),
		dot((bt.cp1 + bt.cp3 + bt.cp4) / 3.0, ray_direction)
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
vec3 intersectBTriangle(vec3 ray_origin, vec3 ray_direction, const btriangle bt)
{	
	// TODO heavy on performance maybe not ordering but rather search for i min
	reorder(ray_origin, ray_direction, bt);

	// TODO
	// Get perpendicular ray by switching coords
	//vec3 normal_1 = ray_direction.zyx;
	vec3 normal_1 = cross(ray_direction, vec3(-1.0));
	vec3 normal_2 = cross(ray_direction, normal_1);
	// TODO warum tun wir das?
	float d_1 = -dot(normal_1, ray_origin);
	float d_2 = -dot(normal_2, ray_origin);

	// TODO test if MAD (multiply then add) makes a difference in performance - see B_uv also
	vec3 q_1 = bt.cp0 + bt.cp2 - 2 * bt.cp1;
	vec3 q_2 = 2 * bt.cp3 - 2 * bt.cp1 - 2 * bt.cp4 + 2 * bt.cp2;
	vec3 q_3 = bt.cp5 - 2 * bt.cp4 + bt.cp2;
	vec3 q_4 = 2 * bt.cp4 - 2 * bt.cp2;
	vec3 q_5 = 2 * bt.cp1 - 2 * bt.cp2;
	vec3 q_6 = bt.cp2;

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
	float b_error = 0.01; // TODO as uniform
	float d_error = 0.01; // TODO as uniform

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

			// Partial derivate by s
			dBs = 2 * q_1 * s +
				q_2 * t +
				q_5;

			// Partial derivate by t
			dBt = q_2 * s +
				2 * q_3 * t +
				q_4;

			// Original TODO should not be a derivate even if said so in the paper
			B_uv = q_1 * pow(s, 2) +
				q_2 * s * t +
				q_3 * pow(t, 2) +
				q_4 * t +
				q_5 * s +
				q_6;


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
	btriangle bt; // TODO direkt construktor nehmen und mit texture stuff befüllen
	int y = int(index); // TODO

	bt.cp0 = texelFetch(btriangles, ivec2(0, y), 0).xyz;
	bt.cp1 = texelFetch(btriangles, ivec2(1, y), 0).xyz;
	bt.cp2 = texelFetch(btriangles, ivec2(2, y), 0).xyz;
	bt.cp3 = texelFetch(btriangles, ivec2(3, y), 0).xyz;
	bt.cp4 = texelFetch(btriangles, ivec2(4, y), 0).xyz;
	bt.cp5 = texelFetch(btriangles, ivec2(5, y), 0).xyz;

	vec3 lambda = intersectBTriangle(origin, dir, bt);
	if (lambda.x >= 0.0 && lambda.x <= 1.0 &&
		lambda.y >= 0.0 && lambda.y <= 1.0 &&
		lambda.z >= 0.0 && lambda.z <= 1.0
	)
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
	hitinfo hit = hitinfo(vec2(MAX_SCENE_BOUNDS), -1, -1, vec4(0.0, 0.0, 0.0, 1.0));
	if (intersectBTriangles(origin, dir, hit))
	{
		// TODO
		//hit.color.rgb = texture(btriangles, vec2(0.0, hit.id)).rgb;
		hit.color.rgb = vec3(1.0, 0.0, 0.0);
		hit.type = TYPE_BTRIANGLE;
	}

	return hit;
}

///////////////////////////////////////////////////////////////////////////////
// Normals
///////////////////////////////////////////////////////////////////////////////

vec3 calcNormal(vec3 ray_origin, vec3 ray_direction, hitinfo hit) 
{
	// TODO refactor everything, this is a stupid doublication

	int y = int(index); // TODO

	btriangle bt; // TODO direkt construktor nehmen und mit texture stuff befüllen

	bt.cp0 = texelFetch(btriangles, ivec2(0, y), 0).xyz;
	bt.cp1 = texelFetch(btriangles, ivec2(1, y), 0).xyz;
	bt.cp2 = texelFetch(btriangles, ivec2(2, y), 0).xyz;
	bt.cp3 = texelFetch(btriangles, ivec2(3, y), 0).xyz;
	bt.cp4 = texelFetch(btriangles, ivec2(4, y), 0).xyz;
	bt.cp5 = texelFetch(btriangles, ivec2(5, y), 0).xyz;

	vec3 q_1 = bt.cp0 + bt.cp2 - 2 * bt.cp1;
	vec3 q_2 = 2 * bt.cp3 - 2 * bt.cp1 - 2 * bt.cp4 + 2 * bt.cp2;
	vec3 q_3 = bt.cp5 - 2 * bt.cp4 + bt.cp2;
	vec3 q_4 = 2 * bt.cp4 - 2 * bt.cp2;
	vec3 q_5 = 2 * bt.cp1 - 2 * bt.cp2;
	vec3 q_6 = bt.cp2;

	
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

	return cross(dBs, dBt);
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
	vec3 ray_direction = normalize(vec4(vRayDirection,1.0).xyz - tmp);
	vec3 ray_origin = tmp;

	hitinfo hit = trace(ray_origin, ray_direction);
	vec4 oColor = hit.color;

	if (hit.id != -1)
	{
		vec3 normal = calcNormal(ray_origin, ray_direction, hit);
		oColor.rgb *= clamp(dot(normal, lig), 0.0, 1.0);
		//oColor.rgb = normal;
	}

	//gl_FragColor = vec4(ray_direction, 1.0);
	outFragment = vec4(oColor.rgb, 1.0);

	if (hit.id == -1)
		discard;

	//outFragment = vec4(normalize(ray_direction), 1.0);
	//outFragment.xyz = normalize(abs(campos));
}
