#version 400

#define SG_REQUEST_POSVS
#define SG_REQUEST_TEXCOORD
#define SG_REQUEST_NORMALVS

#define EPSILON 0.0001

uniform int tessAmount;

uniform sampler2D controlPointTex;

layout(vertices = 3) out;

///////////////////////////////////////////////////////////////////////////////
// Viewpos dependent
// http://ogldev.org/www/tutorial30/tutorial30.html
///////////////////////////////////////////////////////////////////////////////
// TODO try out different attenuations for the distance
// sq, lin, const ...
// const vec3 atten = vec3(0.001, 0.08, 0.3);
#ifdef TESS_DISTANCE
float getTessLevel(float Distance0, float Distance1)
{
    float AvgDistance = (Distance0 + Distance1) / 2.0;
	return tessAmount / max(AvgDistance, 1);
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Point to plane distance based
// https://developer.nvidia.com/gpugems/GPUGems2/gpugems2_chapter07.html
///////////////////////////////////////////////////////////////////////////////
#ifdef TESS_FLATNESS
float getAbsDistance(vec3 baryCoords, vec3 cpTest)
{
	vec3 planarPos = SG_INPUT_POSOS[0].xyz * baryCoords.x
		+ SG_INPUT_POSOS[1].xyz * baryCoords.y
		+ SG_INPUT_POSOS[2].xyz * baryCoords.z;

	return length(cpTest - planarPos);
}

/**
 * Calculate the distance of each point on the lines, this should be used, 
 * to calculate the outer tessellation level, since the here calculated
 * values directly correlate to the amount each edge needs to be divided
 */
vec3 sumDistForLines()
{
	// Init distance with 0 since it could be flat
	vec3 sumDist = vec3(0.0);

	int start = DEGREE + 1;
	int indexR = DEGREE * 2;
	int indexL = start; 

	// Iterate over the triangle to get the index in the texture 
	// and the barycentric coordinates of this control point
	for (int i = DEGREE; i < DEGREE * DEGREE; i += DEGREE)
	{
		float stepSize = float(i)/float(DEGREE*DEGREE);

		//    100
		// 001 - 010
		// bot edge
		sumDist.x += getAbsDistance(
			vec3(0.0, stepSize, 1.0 - stepSize),
			texelFetch(controlPointTex, ivec2(i/DEGREE, gl_PrimitiveID), 0).xyz
		);
		// left edge
		sumDist.y += getAbsDistance(
			//vec3(0.0, 1.0 - stepSize, stepSize),
			vec3(stepSize, 0.0, 1.0 - stepSize),
			texelFetch(controlPointTex, ivec2(indexL, gl_PrimitiveID), 0).xyz
		);
		// right edge
		sumDist.z += getAbsDistance(
			vec3(stepSize, 1.0 - stepSize, 0.0),
			texelFetch(controlPointTex, ivec2(indexR, gl_PrimitiveID), 0).xyz
		);

		// Move along the edge indices
		start -= 1;
		indexL += start;
		indexR += start - 1;
	}

	return sumDist;
}

/**
 * TODO
 * Calculate inner barycoords and use them to compare
 * the control point positions to these
 */
float sumDistForInside()
{
	float dist = 0.0;

	return dist;
}

/**
 * Sets the tesselation level by calculating the closest distance 
 * of each point to the plane of the triangle.
 */
void setTessLevel()
{
	// Area of 3d triangle
	// http://geomalgorithms.com/a01-_area.html
	vec3 AB = SG_INPUT_POSOS[0].xyz - SG_INPUT_POSOS[1].xyz;
	vec3 AC = SG_INPUT_POSOS[0].xyz - SG_INPUT_POSOS[2].xyz;
	float area = length(cross(AB, AC)) * 0.5;

	// Collect distances
	vec3 lineDists = sumDistForLines();
	float centerDist = sumDistForInside();
	lineDists /= max(EPSILON, area);

	gl_TessLevelOuter[0] = max(1.0, lineDists.x * tessAmount);
    gl_TessLevelOuter[1] = max(1.0, lineDists.y * tessAmount);
    gl_TessLevelOuter[2] = max(1.0, lineDists.z * tessAmount);
    gl_TessLevelInner[0] = 
		(lineDists.x + lineDists.y + lineDists.z) / 3.0 + centerDist;
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////
void main()
{
	sg_MapIO(gl_InvocationID);

#ifdef TESS_CONST
	// Calculate the tessellation levels
    gl_TessLevelOuter[0] = tessAmount;
    gl_TessLevelOuter[1] = tessAmount;
    gl_TessLevelOuter[2] = tessAmount;
    gl_TessLevelInner[0] = gl_TessLevelOuter[2];
#endif
#ifdef TESS_DISTANCE
	// Calculate the distance from the camera to the three control points
    float EyeToVertexDistance0 = distance(vec4(campos, 1.0), SG_INPUT_POSOS[0]);
    float EyeToVertexDistance1 = distance(vec4(campos, 1.0), SG_INPUT_POSOS[1]);
    float EyeToVertexDistance2 = distance(vec4(campos, 1.0), SG_INPUT_POSOS[2]);

    // Calculate the tessellation levels
    gl_TessLevelOuter[0] = getTessLevel(EyeToVertexDistance1, EyeToVertexDistance2);
    gl_TessLevelOuter[1] = getTessLevel(EyeToVertexDistance2, EyeToVertexDistance0);
    gl_TessLevelOuter[2] = getTessLevel(EyeToVertexDistance0, EyeToVertexDistance1);
    gl_TessLevelInner[0] = (gl_TessLevelOuter[0] + gl_TessLevelOuter[1] + gl_TessLevelOuter[2]) / 3.0;
#endif
#ifdef TESS_FLATNESS
	setTessLevel();
#endif
}
