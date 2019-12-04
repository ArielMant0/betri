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

// https://rosettacode.org/wiki/Compile-time_calculation#C.2B.2B
template<int i> struct Factorial
{
	static const int result = i * Factorial<i - 1>::result;
};

template<> struct Factorial<0>
{
	static const int result = 1;
};

// TODO array should contain result of calculation
const int FACTORIALS[13] = {
	Factorial<0>::result, Factorial<1>::result,
	Factorial<2>::result, Factorial<3>::result,
	Factorial<4>::result, Factorial<5>::result,
	Factorial<6>::result, Factorial<7>::result,
	Factorial<8>::result, Factorial<9>::result,
	Factorial<10>::result, Factorial<11>::result,
	Factorial<12>::result
};

///////////////////////////////////////////////////////////////////////////////
// evaluate bezier functions or surface
///////////////////////////////////////////////////////////////////////////////

static inline BezierTMesh::Scalar eval(int i, int j, int k, BezierTMesh::Point &bary, unsigned int degree)
{
	return FACTORIALS[degree] / (FACTORIALS[i] * FACTORIALS[j] * FACTORIALS[k])
		* std::pow(bary[0], i) * std::pow(bary[1], j) * std::pow(bary[2], k);
}

static inline int cpIndex(int i, int j, unsigned int degree)
{
	int sum = 0;
	int grad = degree + 1;

	for (int k = 0; k < i; ++k) {
		sum += grad - k;
	}

	return sum + j;
}

// TODO: remove inline
static inline BezierTMesh::Point evalSurface(std::vector<BezierTMesh::Point> &cps, BezierTMesh::Point &bary, unsigned int degree)
{
	BezierTMesh::Point point(0.0);

	for (int i = 0; i <= degree; i++) {
		for (int j = 0; j + i <= degree; j++) {
			point += cps[cpIndex(i, j, degree)] * eval(i, j, degree - i - j, bary, degree);
		}
	}
	return point;
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