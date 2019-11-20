#version 400

#define SG_REQUEST_POSOS
#define SG_REQUEST_TEXCOORD
#define SG_REQUEST_NORMALVS

layout(triangles, equal_spacing, ccw) in;

uniform sampler2D controlPointTex;
uniform int tessAmount;

//uniform vec4 uvRange;

// more then 12! is not possible because signed integer overflow
// and therefore compilation errors
uniform float FACTORIALS[13] = {
	1.0, 1.0, 2.0, 6, 24, 120, // 0, 1, 2, 3, 4, 5
	720, 5040, 40320, 362880, 3628800, // 6, 7, 8, 9, 10
	39916800, 479001600 // 11, 12
};

/**
 * Function to calculate Exponentiation.
 * This Function is nessessary, because the default pow() does not 
 * give the results that the calculation formular for the beziertriangles
 * needs. Therefore the special cases are treated and otherwise the default 
 * function is called.
 * @param base The base which should be raised
 * @param expo The exponent which is used to raise the first argument
 * @return The resulting number
 */
float myPow(float base, float expo) 
{
	if (base == 0.0 && expo == 0.0)
	{
		if (expo == 0.0)
			return 1.0;
		return 0.0;
	}
	return pow(base, expo);
}

/**
 * Get the amount of points that are in the levels below the parameter-level.
 * @param level the level in which the point is
 */
int pointsBefore(int level)
{
	// TODO das ist langsam?!
	// TODO gaus summe ?
	int sum = 0;
	for (int i = 0; i < level; i++)
	{
		sum += DEGREE + 1 - i;
	}
	return sum;
}

/**
 * Get the element (Control Point) from the texture
 * @param i row-index of the elem
 * @param j column-index of the elem in row i
 * @param k not used TODO remove?
 */
vec3 getCP(int i, int j, int k)
{
	// calculate which cp from this triangle should be fetched
	int cpIndex = pointsBefore(i) + j;

	// gl_PrimitiveID contains the Triangle-ID (used for the texture lookup)
	ivec2 controlPtID = ivec2(cpIndex, gl_PrimitiveID);
	return texelFetch(controlPointTex, controlPtID, 0).xyz;
}

vec3 offset = vec3(0.0);

/**
 * Get one Entry for a secific set of i,j,k (control points)
 * multiply these with the barycentric coords
 */
vec3 oneEntry(int i, int j, int k)
{
	// TODO float nessessary?
	vec3 entry = (FACTORIALS[DEGREE] / (FACTORIALS[i] * FACTORIALS[j] * FACTORIALS[k]))
		* myPow(gl_TessCoord.x + offset.x, float(i)) 
		* myPow(gl_TessCoord.y + offset.y, float(j)) 
		* myPow(gl_TessCoord.z + offset.z, float(k))
		* getCP(i, j, k);

	return entry;
}

/**
 * Sum over all combinations of ControlPoints
 * To achive this we iterate the Points row-wise from left to right
 * and from bottom to top.
 */
vec3 newPosition()
{
	vec3 sum = vec3(0.0);

	// TODO should there be a special treatment for the points that are on the edges or corners? because it should in generell be easier to evaluate them

	// 0 0 2
	// 0 1 1
	// 0 2 0
	// 1 0 1
	// 1 1 0
	// 2 0 0
	// i is the row-index
	for (int i = 0; i <= DEGREE; i++)
	{
		// j is the column-index
		for (int j = 0; j + i <= DEGREE; j++)
		{
			/*
			TODO Both variant should be the same, which is more readable?
			int k = DEGREE - i - j;
			if (k >= 0) 
			*/
			// k is directly dependent from i and j
			for (int k = DEGREE - i - j; k + j + i == DEGREE && k >= 0; k++)
			{
				sum += oneEntry(i, j, k);
			}
		}
	}

	return sum;
}

void main()
{
	sg_MapIOBarycentric();
	//outTeColor = vec4(1.0, 0.0, 0.0, 1.0);

	// Set Vertex-position
	vec3 pos = newPosition();
	gl_Position = g_mWVP * vec4(pos, 1.0);

	/////////////
	// Normale //
	/////////////
	vec3 toEval;
	vec3 pos1;
	vec3 pos2;

	float stepSize = 1.0 / tessAmount;

	// It is nessessary to distinguish between vertices that are
	// in the face and those which are on a border, because for 
	// these the vector needs to show in the other direction and 
	// sometimes the order for the cross-product needs to be changed
	// Otherwise the result is wrong i.e. flat triangle
	if (gl_TessCoord.z < stepSize) {
		if (gl_TessCoord.x < stepSize) {
			offset = vec3(stepSize, -stepSize, 0.0);
			pos2 = newPosition();
			offset = vec3(0.0, -stepSize, stepSize);
			pos1 = newPosition();
		} else if (gl_TessCoord.y < stepSize) {
			offset = vec3(-stepSize, stepSize, 0.0);
			pos1 = newPosition();
			offset = vec3(-stepSize, 0.0, stepSize);
			pos2 = newPosition();
		} else {
			offset = vec3(-stepSize, 0.0, stepSize);
			pos1 = newPosition();
			offset = vec3(0.0, -stepSize, stepSize);
			pos2 = newPosition();
		}
	} else {
		offset = vec3(stepSize, 0.0, -stepSize);
		pos1 = newPosition();
		offset = vec3(0.0, stepSize, -stepSize);
		pos2 = newPosition();
	}

	vec3 surfaceNormal = cross(normalize(pos1 - pos), normalize(pos2 - pos));

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
}
