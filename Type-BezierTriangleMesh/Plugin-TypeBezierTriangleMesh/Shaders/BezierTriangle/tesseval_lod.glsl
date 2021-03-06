#version 400

#define SG_REQUEST_POSOS
#define SG_REQUEST_TEXCOORD
#define SG_REQUEST_NORMALVS

layout(triangles, equal_spacing, ccw) in;

uniform int tessAmount;
uniform sampler2D controlPointTex;

///////////////////////////////////////////////////////////////////////////////
// Utility
///////////////////////////////////////////////////////////////////////////////
// TODO
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
	if (expo == 0.0)
		return 1.0;
	if (base == 0.0)
		return 0.0;

	return pow(base, expo);
}

///////////////////////////////////////////////////////////////////////////////
// Position
///////////////////////////////////////////////////////////////////////////////
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

/**
 * Get one Entry for a secific set of i,j,k (control points)
 * multiply these with the barycentric coords
 */
vec3 oneEntry(int i, int j, int k)
{
	// TODO float nessessary?
	vec3 entry = (FACTORIALS[DEGREE] / (FACTORIALS[i] * FACTORIALS[j] * FACTORIALS[k]))
		* myPow(gl_TessCoord.x, float(i))
		* myPow(gl_TessCoord.y, float(j))
		* myPow(gl_TessCoord.z, float(k))
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
	// TODO should there be a special treatment for the points that are on
	// the edges or corners? because it should in generell be easier to
	// evaluate them
	// i is the row-index
	for (int i = 0; i <= DEGREE; i++)
	{
		// j is the column-index
		for (int j = 0; j + i <= DEGREE; j++)
		{
			// TODO should not be needed anyway
			// TODO Both variant should be the same, which is more readable?
			//int k = DEGREE - i - j;
			//if (k >= 0)

			// k is directly dependent from i and j
			for (int k = DEGREE - i - j; k + j + i == DEGREE && k >= 0; k++)
			{
				sum += oneEntry(i, j, k);
			}
		}
	}

	return sum;
}

///////////////////////////////////////////////////////////////////////////////
// Normal
///////////////////////////////////////////////////////////////////////////////
float derives(int i, int j, int k)
{
	float deriv = float(i) * myPow(gl_TessCoord.x, float(i-1))
		* myPow(gl_TessCoord.y, float(j))
		* myPow(gl_TessCoord.z, float(k));

	return deriv;
}

float derivet(int i, int j, int k)
{
	float deriv = myPow(gl_TessCoord.x, float(i))
		* float(j) * myPow(gl_TessCoord.y, float(j-1))
		* myPow(gl_TessCoord.z, float(k));

	return deriv;
}

vec3 newNormal()
{

	vec3 ds = vec3(0.0);
	vec3 dt = vec3(0.0);

	// i is the row-index
	for (int i = 0; i <= DEGREE; i++)
	{
		// j is the column-index
		for (int j = 0; j + i <= DEGREE; j++)
		{
			int k = DEGREE - i - j;
			vec3 entry = (FACTORIALS[DEGREE] / (FACTORIALS[i] * FACTORIALS[j] * FACTORIALS[k]))
				* getCP(i, j, k);
			float minusTerm = - float(k) * myPow(gl_TessCoord.x, float(i))
				* myPow(gl_TessCoord.y, float(j))
				* myPow(gl_TessCoord.z, float(k-1));
			float sDeriv = derives(i, j, k) + minusTerm;
			float tDeriv = derivet(i, j, k) + minusTerm;

			ds += entry * sDeriv;
			dt += entry * tDeriv;
		}
	}

	return normalize(cross(ds, dt));
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////
void main()
{
	sg_MapIOBarycentric();
	//outTeColor = vec4(1.0, 0.0, 0.0, 1.0);

	// Set Vertex-position
	vec3 pos = newPosition();
	gl_Position = g_mWVP * vec4(pos, 1.0);

	vec3 surfaceNormal = newNormal();

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
#ifdef SG_OUTPUT_TEXCOORD
	SG_OUTPUT_TEXCOORD = SG_INPUT_TEXCOORD[0] * gl_TessCoord.x
		+ SG_INPUT_TEXCOORD[1] * gl_TessCoord.y
		+ SG_INPUT_TEXCOORD[2] * gl_TessCoord.z;
#endif
}
