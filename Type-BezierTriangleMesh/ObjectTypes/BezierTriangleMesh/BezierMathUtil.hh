#pragma once

#include <cmath>
#include <iostream> // TODO weg damit
#include <set> // TODO weg damit

namespace betri
{

///////////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////////

/**
 * @param n The number for which the calculation should be taken
 */
static inline int gaussSum(int n)
{
	return (n*n + n) / 2;
}

/**
 * Expected Result for
 * n = 0 -> 0
 * n = 1 -> 1
 * n = 2 -> 3
 * n = 3 -> 7
 * n = 4 -> 15
 * n = 5 -> 31
 * https://en.wikipedia.org/wiki/Mersenne_prime
 *
 * @param n The number for which the calculation should be taken
 */
static inline int mersennePrime(int n)
{
	return pow(2.0, n) - 1;
}

/**
 * Returns the third barycentric coordinates for two values between 0 and 1
 * Order: 100 - 010 - 001
 *
 * @param u First Coordinate
 * @param v Secound Coordinate
 * @returns A full point with w = 1-u-v
 */
static inline BezierTMesh::Point getBaryCoords(double u, double v)
{
	return BezierTMesh::Point(u, v, 1.0 - u - v);
}

static double inline length(BezierTMesh::Point p)
{
	return sqrt(dot(p, p));
}



///////////////////////////////////////////////////////////////////////////////
// Unused (was das?)
///////////////////////////////////////////////////////////////////////////////
static void createSimplex(std::vector<BezierTMesh::Point> &faceControlP)
{
	float minX = 0, maxX = 0;
	float minY = 0, maxY = 0;
	float minZ = 0, maxZ = 0;

	for (int i = 0; i < faceControlP.size(); i++) {
		if (faceControlP[i][0] < faceControlP[minX][0])
			minX = i;
		if (faceControlP[i][0] > faceControlP[maxX][0])
			maxX = i;
		if (faceControlP[i][1] < faceControlP[minY][1])
			minY = i;
		if (faceControlP[i][1] > faceControlP[maxY][1])
			maxY = i;
		if (faceControlP[i][2] < faceControlP[minZ][2])
			minZ = i;
		if (faceControlP[i][2] > faceControlP[maxZ][2])
			maxZ = i;
	}
}
}