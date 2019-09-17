#version 150

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

in vec2 vTexCoord;
in vec3 vPosition;

in vec3 vOrigin;
in vec3 vRay;

uniform sampler2D cubes;
uniform sampler2D spheres;
uniform sampler2D triangles;
uniform sampler2D btriangles;

//uniform mat4 g_mWVP;
//uniform vec3 g_vCamPos;

out vec4 oColor;

// v0 - vertex 0
// v1 - vertex 1
// v2 - vertex 2
struct triangle {
	vec3 v0;
	vec3 v1;
	vec3 v2;
};

// min - left lower corner
// max - right upper corner
struct box {
	vec3 min;
	vec3 max;
};

// pos - sphere center
// radius - sphere radius
struct sphere {
	vec3 pos;
	float radius;
};

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
// id - box index
struct hitinfo {
	vec2 lambda;
	int id;
	int type;
	vec4 color;
};

///////////////////////////////////////////////////////////////////////////////
// Link to multiple examples for ray - object intersections
// http://www.iquilezles.org/www/articles/intersectors/intersectors.htm
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Triangle Intersect
///////////////////////////////////////////////////////////////////////////////

/**
 * Test one triangle againt a ray.
 *
 * @in origin Origin of the ray
 * @in dir Direction of the ray
 * @in sphere the sphere to test against
 * @return A three component vector with the nearest intersection lambda (t) and u, v
 */
vec3 intesectTriangle(vec3 ray_origin, vec3 ray_direction, triangle tri)
{
    vec3 v1v0 = tri.v1 - tri.v0;
    vec3 v2v0 = tri.v2 - tri.v0;
    vec3 rov0 = ray_origin - tri.v0;
    vec3  n = cross(v1v0, v2v0);
    vec3  q = cross(rov0, ray_direction);
    float d = 1.0 / dot(ray_direction, n);
    float u = d * dot(-q, v2v0);
    float v = d * dot( q, v1v0);
    float t = d * dot(-n, rov0);
    if (u < 0.0 || u > 1.0 || v < 0.0 || (u+v) > 1.0) 
		t = -1.0;
    return vec3(t, u, v);
}

/**
 * Test all Triangles of the triangle texture against the ray that is given.
 *
 * @in origin Origin of the ray
 * @in dir Direction of the ray
 * @inout hitinfo Outvariable - Information about the nearest and selected hit 
 */
bool intersectTriangles(vec3 origin, vec3 dir, inout hitinfo info)
{
	//float smallest = MAX_SCENE_BOUNDS;
	float smallest = info.lambda.x;
	bool found = false;
	for (int i = 0; i < NUM_TRIANGLES; i++)
	{
		float y = i * 1.0 / NUM_TRIANGLES + 0.5 / NUM_TRIANGLES; // TODO
		//float y = 0.5; // TODO
		triangle t;
		t.v0 = texture(triangles, vec2(0.25, y)).xyz;
		t.v1 = texture(triangles, vec2(0.5, y)).xyz;
		t.v2 = texture(triangles, vec2(0.75, y)).xyz;
		float lambda = intesectTriangle(origin, dir, t).x;
		if (lambda > 0.0 && lambda < smallest)
		{
			info.lambda.x = lambda; // TODO vec3
			info.id = i;
			smallest = lambda;
			found = true;
		}
	}
	return found;
}

///////////////////////////////////////////////////////////////////////////////
// Sphere Intersect
///////////////////////////////////////////////////////////////////////////////

/**
 * Test one sphere againt a ray.
 *
 * @in origin Origin of the ray
 * @in dir Direction of the ray
 * @in sphere the sphere to test against
 * @return A two component vector with the nearest and furthest intersection lambda (t)
 */
vec2 intesectSphere(vec3 ray_origin, vec3 ray_direction, sphere s)
{
    vec3 oc = ray_origin - s.pos;
    float b = dot(oc, ray_direction);
    float c = dot(oc, oc) - s.radius * s.radius;
    float h = b*b - c;
    if (h < 0.0) 
		return vec2(-1.0); // no intersection
    h = sqrt(h);
    return vec2(-b-h, -b+h);
}

/**
 * Test all Spheres of the sphere texture against the ray that is given.
 *
 * @in origin Origin of the ray
 * @in dir Direction of the ray
 * @inout hitinfo Outvariable - Information about the nearest and selected hit 
 */
bool intersectSpheres(vec3 origin, vec3 dir, inout hitinfo info)
{
	//float smallest = MAX_SCENE_BOUNDS;
	float smallest = info.lambda.x;
	bool found = false;
	for (int i = 0; i < NUM_SPHERES; i++)
	{
		float y = i * 1.0 / NUM_SPHERES + 0.5 / NUM_SPHERES; // TODO
		sphere s;
		s.pos = texture(spheres, vec2(0.0, y)).xyz;
		s.radius = texture(spheres, vec2(0.0, y)).w;
		float lambda = intesectSphere(origin, dir, s).x;
		if (lambda > 0.0 && lambda < smallest)
		{
			info.lambda.x = lambda; // TODO vec2
			info.id = i;
			smallest = lambda;
			found = true;
		}
	}
	return found;
}

///////////////////////////////////////////////////////////////////////////////
// Box Intersect
///////////////////////////////////////////////////////////////////////////////

/**
 * Test one box againt a ray.
 *
 * @in origin Origin of the ray
 * @in dir Direction of the ray
 * @in box the box to test against
 * @return A two component vector with the nearest and furthest intersection lambda (t)
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
 * Test all Boxes of the cubes texture against the ray that is given.
 *
 * @in origin Origin of the ray
 * @in dir Direction of the ray
 * @out hitinfo Outvariable - Information about the nearest and selected hit 
 */
bool intersectBoxes(vec3 origin, vec3 dir, out hitinfo info)
{
	float smallest = MAX_SCENE_BOUNDS;
	bool found = false;
	for (int i = 0; i < NUM_BOXES; i++) 
	{
		box b;
		float y = i * 1.0 / NUM_BOXES + 0.5 / NUM_BOXES; // TODO
		b.min = texture(cubes, vec2(0.0, y)).xyz;
		b.max = texture(cubes, vec2(1.0, y)).xyz;
		vec2 lambda = intersectBox(origin, dir, b);
		if (lambda.x > 0.0 && lambda.x < lambda.y && lambda.x < smallest) 
		{
			info.lambda = lambda;
			info.id = i;
			smallest = lambda.x;
			found = true;
		}	
	}
	return found;
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
vec2 intersectBTriangle(vec3 ray_origin, vec3 ray_direction, const btriangle bt) 
{
    vec3 v1v0 = bt.cp2 - bt.cp0;
    vec3 v2v0 = bt.cp5 - bt.cp0;
    vec3 rov0 = ray_origin - bt.cp0;
    vec3  n = cross(v1v0, v2v0);
    vec3  q = cross(rov0, ray_direction);
    float d = 1.0 / dot(ray_direction, n);
    float u = d * dot(-q, v2v0);
    float v = d * dot( q, v1v0);
    float t = d * dot(-n, rov0);
    if (u < 0.0 || u > 1.0 || v < 0.0 || (u+v) > 1.0) 
		t = -1.0;
    return vec3(t, u, v);
}

/**
 * Test all Boxes of the cubes texture against the ray that is given.
 *
 * @in origin Origin of the ray
 * @in dir Direction of the ray
 * @out hitinfo Outvariable - Information about the nearest and selected hit 
 */
bool intersectBTriangles(vec3 origin, vec3 dir, out hitinfo info)
{
	float smallest = info.lambda.x;
	bool found = false;
	for (int i = 0; i < NUM_BOXES; i++) 
	{
		btriangle bt;
		float y = i * 1.0 / NUM_BOXES + 0.5 / NUM_BTRIANGLES; // TODO
		const float padding = 1; // TODO all as defines? from the cpu?
		const float cp_count = 6;
		const float mid = 2 / (cp_count + padding); // TODO is this nessessary?
		const float x = 1 / (cp_count + padding);
		bt.cp0 = texture(btriangles, vec2((padding + 0) * x + mid, y)).xyz;
		bt.cp2 = texture(btriangles, vec2((padding + 2) * x + mid, y)).xyz;
		bt.cp5 = texture(btriangles, vec2((padding + 5) * x + mid, y)).xyz;
		float lambda = intersectBTriangle(origin, dir, bt).x;
		if (lambda > 0.0 && lambda < smallest)
		{
			info.lambda.x = lambda;
			info.id = i;
			smallest = lambda;
			found = true;
		}	
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
	hitinfo hit;
	hit.id = -1;
	hit.color = vec4(0.0, 0.0, 0.0, 1.0);
	hit.lambda = vec2(MAX_SCENE_BOUNDS); // TODO generell verwenden? auch für boxes
	vec4 r_color = vec4(0.0, 0.0, 0.0, 1.0); 
	if (intersectBoxes(origin, dir, hit))
	{
		hit.color.rgb = vec3(hit.id / 10.0 + 0.8);
		hit.type = TYPE_CUBE;
	}
	
	if (intersectSpheres(origin, dir, hit))
	{
		hit.color.rgb = texture(spheres, vec2(1.0, hit.id)).rgb;
		hit.type = TYPE_SPHERE;
	}

	if (intersectTriangles(origin, dir, hit))
	{
		hit.color.rgb = texture(triangles, vec2(0.0, hit.id)).rgb;
		hit.type = TYPE_TRIANGLE;
	}

	if (intersectBTriangles(origin, dir, hit))
	{
		hit.color.rgb = texture(btriangles, vec2(0.0, hit.id)).rgb;
		hit.type = TYPE_BTRIANGLE;
	}

	return hit;
}

///////////////////////////////////////////////////////////////////////////////
// Normals
///////////////////////////////////////////////////////////////////////////////

vec3 sphNormal(vec3 pos, vec3 sphere_pos)
{
    return normalize(pos - sphere_pos);
}

vec3 calcNormal(vec3 ray_origin, vec3 ray_direction, in hitinfo hit)
{
	switch (hit.type) 
	{
		case TYPE_CUBE: 
		{
			return vec3(0.0, 1.0, 0.0); // TODO
		}
		case TYPE_SPHERE: 
		{
			vec3 sphere_pos = texture(spheres, vec2(0.0, hit.id)).xyz;
			vec3 pos = ray_origin + hit.lambda.x * ray_direction;
			return sphNormal(pos, sphere_pos);
		}
		case TYPE_TRIANGLE: 
		{
			return vec3(0.0, 1.0, 0.0); // TODO
		}
		default: return vec3(0.0, 1.0, 0.0);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

void main() 
{
	vec3 lig = normalize(vec3(0.6, 0.3, 0.4)); // TODO as texture/uniform

	vec3 ray_direction = normalize(vRay);
	vec3 ray_origin = vOrigin;

	hitinfo hit = trace(ray_origin, ray_direction);
	oColor = hit.color;

	if (hit.id != -1)
	{
		vec3 normal = calcNormal(ray_origin, ray_direction, hit);
		oColor.rgb *= clamp(dot(normal, lig), 0.0, 1.0);
	}

	//oColor = vec4(texture(triangles, vec2(0.5, 0)).xyz, 1.0);

	//oColor = vec4(normalize(vRay), 1.0);
	//oColor = vec4(vOrigin, 1.0);
}
