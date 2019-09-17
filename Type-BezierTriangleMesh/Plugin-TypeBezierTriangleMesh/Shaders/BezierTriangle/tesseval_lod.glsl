#version 400

#define SG_REQUEST_POSOS
#define SG_REQUEST_TEXCOORD
#define SG_REQUEST_NORMALVS

layout(triangles, equal_spacing, ccw) in;


//uniform samplerBuffer knotBufferU;
//uniform samplerBuffer knotBufferV;

uniform sampler2D controlPointTex;

//uniform vec4 uvRange;

// more then 12! is not possible because signed integer overflow
// and therefore compilation errors
uniform float FACTORIALS[13] = {
	1, 1, 2, 6, 24, 120, // 0, 1, 2, 3, 4, 5
	720, 5040, 40320, 362880, 3628800, // 6, 7, 8, 9, 10
	39916800, 479001600 // 11, 12
};

const int GRAD = 2;

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

/*
float factorial(int f)
{
	if (f == 1)
		return 1;
	return f * factorial(f-1);
}*/

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
		sum += GRAD + 1 - i;
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
	vec3 entry = (FACTORIALS[GRAD] / (FACTORIALS[i] * FACTORIALS[j] * FACTORIALS[k]))
		* myPow(gl_TessCoord.x, float(i)) * myPow(gl_TessCoord.y, float(j)) * myPow(gl_TessCoord.z, float(k))
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
	for (int i = 0; i <= GRAD; i++)
	{
		// j is the column-index
		for (int j = 0; j + i <= GRAD; j++)
		{
			/*
			TODO Both variant should be the same, which is more readable?
			int k = GRAD - i - j;
			if (k >= 0) 
			*/
			// k is directly dependent from i and j
			for (int k = GRAD - i - j; k + j + i == GRAD && k >= 0; k++)
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
	//outTeNormalOS = vec3(1.0, 0.0, 0.0);
	//outTeColor = vec4(gl_PrimitiveID, 0.0, 0.0, 1.0);

	// Set Vertex-position
	vec3 pos = newPosition();

	//if (gl_TessCoord.x != 0 && gl_TessCoord.y != 0 && gl_TessCoord.z != 0)
	gl_Position = g_mWVP * vec4(pos, 1.0);

	//outTeNormalVS = gl_TessCoord.x * outTcNormalVS[0] + gl_TessCoord.y * outTcNormalVS[1] + gl_TessCoord.z * outTcNormalVS[2];
	//outTeNormalOS = vec3(1.0, 1.0, 0.0);
	// TODO er sagt hier, dass er nicht compilieren konnte das Ergebnis sagt aber was anderes - außerdem ist die reihenfolge anders rum
	// aber das geht nur wenn man keine subdivision macht
	//outTeNormalOS = gl_TessCoord.x * outTcNormalOS[2] + gl_TessCoord.y * outTcNormalOS[1] + gl_TessCoord.z * outTcNormalOS[0];

	vec3 surfaceNormal = normalize(cross(getCP(GRAD, 0, 0) - getCP(0, GRAD, 0), getCP(0, 0, GRAD) - getCP(0, GRAD, 0)));

	// DEBUG ELEMENTS
	// Just forward the Position
	//gl_Position = g_mWVP * vec4(gl_TessCoord.x * getCP(GRAD, 0, 0) + gl_TessCoord.y * getCP(0, 0, GRAD) + gl_TessCoord.z * getCP(0, GRAD, 0), 1.0);
	// Shows which vertex is processed first, by visualisation of the barycentric coords
	// TODO does this mean, that the order in the texture in not correct for every vertex?
	//vec3 surfaceNormal = vec3(gl_TessCoord.x, gl_TessCoord.y, gl_TessCoord.z);

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
