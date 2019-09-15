#version 150

#define NUM_BOXES 2
#define MAX_SCENE_BOUNDS 1000.0

in vec2 vTexCoord;
in vec3 vPosition;

in vec3 vOrigin;
in vec3 vRay;

uniform sampler2D sceneObject;
// The inverse of the texture dimensions along X and Y
uniform vec2 texcoordOffset;

uniform mat4 g_mWVP;
uniform vec3 g_vCamPos;

out vec4 oColor;

// min - left lower corner
// max - right upper corner
struct box {
	vec3 min;
	vec3 max;
};

const box boxes[] = {
	/* The ground */
	{ vec3(-5.0, -3.0, -5.0), vec3(5.0, -2.9, 5.0) },
	/* Box in the middle */
	{ vec3(-1.5, -1.5, 3), vec3(1.5, 1.5, 4.5) }

	
	/* The ground */
	//{vec3(-5.0, -0.1, -5.0), vec3(5.0, 0.0, 5.0)},
	/* Box in the middle */
	//{vec3(-0.5, 0.0, -0.5), vec3(0.5, 1.0, 0.5)}
};

// lambda - distances to the intersection on the ray
// bi - box index
struct hitinfo {
	vec2 lambda;
	int bi;
};

// https://gamedev.stackexchange.com/questions/96459/fast-ray-sphere-collision-code
bool intesectSphere(vec3 ray_origin, vec3 ray_direction)
{
	//vec4 sphere_center = vec4(texture(sceneObject, vec2(0.0, 0.0)).xyz, 1.0);
	vec4 sphere_center = vec4(0.0, 10.0, -10.0, 1.0); // TODO vec3
	//float sphere_radius = texture(sceneObject, vec2(0.0, 0.0)).x;
	float sphere_radius = 0.5;

    //vec3 oc = vec3(g_mWVP * vec4(g_vCamPos, 1.0)) - sphere_center.xyz;
    vec3 oc = ray_origin - sphere_center.xyz;
    float a = dot(ray_direction, ray_direction);
    float b = 2.0 * dot(oc, ray_direction);
    float c = dot(oc, oc) - sphere_radius * sphere_radius;
    float discriminant = b*b - 4*a*c;

	// A negative discriminant corresponds to ray missing sphere 
    if (discriminant < 0) 
        return false;
    else
	{
		// Ray now found to intersect sphere, compute smallest t value of intersection
		// If it is negative, ray started inside sphere
		if (-b - sqrt(discriminant) < 0.0)
			return false;
        return true;
	}

	return true;
}

/**
 * Test one box againt a ray.
 * @param origin Origin of the ray
 * @param dir Direction of the ray
 * @param box the box to test against
 * @return A two component vector with the nearest and furthest intersection
 */
vec2 intersectBox(vec3 origin, vec3 dir, const box b) 
{
	vec3 tMin = (b.min - origin) / dir;
	vec3 tMax = (b.max - origin) / dir;
	vec3 t1 = min(tMin, tMax);
	vec3 t2 = max(tMin, tMax);
	float tNear = max(max(t1.x, t1.y), t1.z);
	float tFar = min(min(t2.x, t2.y), t2.z);
	return vec2(tNear, tFar);
}

/**
 * Test all Boxes of the boxes array against the ray that is given.
 *
 * @param origin Origin of the ray
 * @param dir Direction of the ray
 * @param hitinfo Outvariable - Information about the nearest and selected hit 
 */
bool intersectBoxes(vec3 origin, vec3 dir, out hitinfo info)
{
	float smallest = MAX_SCENE_BOUNDS;
	bool found = false;
	for (int i = 0; i < NUM_BOXES; i++) 
	{
		box b;
		//b.min = boxes[i].min;
		b.min = texture(sceneObject, vec2(i == 0 ? 0 : 1, 0.0)).xyz;
		//b.max = boxes[i].max;
		b.max = texture(sceneObject, vec2(i == 0 ? 0 : 1, 1.0)).xyz;
		vec2 lambda = intersectBox(origin, dir, b);
		if (lambda.x > 0.0 && lambda.x < lambda.y && lambda.x < smallest) 
		{
			info.lambda = lambda;
			info.bi = i;
			smallest = lambda.x;
			found = true;
		}	
	}
	return found;
}

/**
 * Trace a specific ray against all objects of the scene and color them 
 * according to the Objectattributes.
 * 
 * @param origin Origin of the ray
 * @param dir Direction of the ray
 * @return the color for this ray (pixel)
 */
vec4 trace(vec3 origin, vec3 dir) 
{
	hitinfo hit;
	if (intersectBoxes(origin, dir, hit))
	{
		vec4 gray = vec4(hit.bi / 10.0 + 0.8);
		return vec4(gray.rgb, 1.0);
	}
	return vec4(0.0, 0.0, 0.0, 1.0);
}

void main() 
{
	vec3 ray_direction = normalize(vRay);
	//vec3 ray_direction = vec3(g_mWVP * vec4(normalize(vPosition - g_vCamPos), 1.0));
	//vec3 ray_direction = normalize(vec3(-1.0, 0.0, 0.0));
	
	vec3 ray_origin = vOrigin;
	//vec3 ray_origin = g_vCamPos;
	//vec3 ray_origin = vec3(3.0, 2.0, 7.0);

	/*
	if (intesectSphere(ray_origin, ray_direction))
		//oColor = vec4(texture(sceneObject, vec2(0.0, 0.0)).xyz, 1.0);
		oColor = vec4(1.0, 0.0, 0.0, 1.0);
	else
		oColor = vec4(0.0, 0.0, 0.0, 1.0);
	*/

	oColor = trace(ray_origin, ray_direction);

	//oColor = vec4(normalize(vRay), 1.0);
	//oColor = vec4(vOrigin, 1.0);
	//oColor = vec4(texture(sceneObject, vec2(1.0, 0.0)).xyz ,1.0);
}
