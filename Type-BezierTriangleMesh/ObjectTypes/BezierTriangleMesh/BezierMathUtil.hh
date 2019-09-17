#pragma once

#include <cmath>

namespace betri
{

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
 * @param u First Coordinate
 * @param v Secound Coordinate
 * @returns A full point with w = 1-u-v
 */
static inline BezierTMesh::Point getBaryCoords(double u, double v)
{
	return BezierTMesh::Point(u, v, 1.0 - u - v);
};

}