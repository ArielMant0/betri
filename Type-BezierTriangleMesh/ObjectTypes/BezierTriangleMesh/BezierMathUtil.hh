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

enum boundingVolumeType
{
	AABB = 0, 
	PrismVolume = 1
};

static void addBoundingBoxFromPoints(
	const int controlPointsPerFace,
	int &vboIndex,
	int &iboIndex,
	int face_index,
	std::vector<float> &vboData,
	std::vector<int> &iboData,
	std::vector<BezierTMesh::Point> &faceControlP
)
{
	const int boundingVolumeVCount = 8;

	BezierTMesh::Point cp;
	BezierTMesh::Point min(INFINITY);
	BezierTMesh::Point max(-INFINITY); // TODO minus
	for (int i = 0; i < controlPointsPerFace; i++) {
		cp = faceControlP[i];
		for (int m = 0; m < 3; ++m) {
			cp[m] < min[m] ? min[m] = cp[m] : -1;
			cp[m] > max[m] ? max[m] = cp[m] : -1;
		}
	}

	std::array<BezierTMesh::Point, boundingVolumeVCount> pointArray = {
		// first quad
		BezierTMesh::Point(min[0], min[1], min[2]),
		BezierTMesh::Point(min[0], max[1], min[2]),
		BezierTMesh::Point(min[0], max[1], max[2]),
		BezierTMesh::Point(min[0], min[1], max[2]),
		// second quad
		BezierTMesh::Point(max[0], min[1], min[2]),
		BezierTMesh::Point(max[0], max[1], min[2]),
		BezierTMesh::Point(max[0], max[1], max[2]),
		BezierTMesh::Point(max[0], min[1], max[2]),
	};

	for (auto p : pointArray) {
		// store pos
		for (int m = 0; m < 3; ++m)
			vboData[vboIndex++] = float(p[m]);

		// TODO not nessessary
		// store normal
		//for (int m = 0; m < 3; ++m)
		vboData[vboIndex++] = float(face_index);
		//vboData[vboIndex++] = float(1.0);
		vboData[vboIndex++] = float(0.0);
		vboData[vboIndex++] = float(0.0);

		// store texcoord
		//texCoord = bezierTriangleMesh_.texcoord2D(v); // TODO
		//vboData[vboIndex++] = texCoord[0];
		//vboData[vboIndex++] = texCoord[1];
		vboData[vboIndex++] = 1.0;
		vboData[vboIndex++] = 0.0;
	}

	// first face - front
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 0;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 3;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 1;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 3;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 2;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 1;
	// second face - top
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 1;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 2;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 5;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 2;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 6;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 5;
	// third face - back
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 5;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 6;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 4;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 6;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 7;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 4;
	// forth face - bottom
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 4;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 7;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 0;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 7;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 3;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 0;
	// fifth face - left
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 4;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 0;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 5;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 0;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 1;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 5;
	// sixth face -rigth
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 3;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 7;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 2;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 7;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 6;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 2;
};

static void addPrismVolumeFromPoints(
	const int controlPointsPerFace,
	int &vboIndex,
	int &iboIndex,
	int face_index,
	std::vector<float> &vboData,
	std::vector<int> &iboData,
	std::vector<BezierTMesh::Point> &faceControlP
)
{
	const int boundingVolumeVCount = 6;

	BezierTMesh::Point cp;
	BezierTMesh::Point p1 = faceControlP[0];
	BezierTMesh::Point p2 = faceControlP[2];
	BezierTMesh::Point p3 = faceControlP[5];
	BezierTMesh::Point l1 = p2 - p1;
	BezierTMesh::Point l2 = p3 - p2;
	BezierTMesh::Point l3 = p1 - p3;
	BezierTMesh::Point normal = cross(p2 - p1, p3 - p1);
	float l1_min = INFINITY;
	float l2_min = INFINITY;
	float l3_min = INFINITY;
	float n_min = INFINITY;
	float l1_max = -INFINITY;
	float l2_max = -INFINITY;
	float l3_max = -INFINITY;
	float n_max = -INFINITY;


	// TODO berechnungen noch nicht richtig, unklar was genau das problem ist, oder wie es aussehen sollte
	// vmtl dot() schon falsch
	for (int i = 0; i < controlPointsPerFace; i++) {
		cp = faceControlP[i];

		dot(cp - p1, l1) < l1_min ? l1_min = dot(cp - p1, l1) : -1;
		dot(cp - p1, l1) > l1_max ? l1_max = dot(cp - p1, l1) : -1;

		dot(cp - p2, l2) < l2_min ? l2_min = dot(cp - p2, l2) : -1;
		dot(cp - p2, l2) > l2_max ? l2_max = dot(cp - p2, l2) : -1;

		dot(cp - p3, l3) < l3_min ? l3_min = dot(cp - p3, l3) : -1;
		dot(cp - p3, l3) > l3_max ? l3_max = dot(cp - p3, l3) : -1;

		dot(cp - p1, normal) < n_min ? n_min = dot(cp - p1, normal) : -1;
		dot(cp - p1, normal) > n_max ? n_max = dot(cp - p1, normal) : -1;
	}

	std::array<BezierTMesh::Point, boundingVolumeVCount> pointArray = {
		// top triangle
		p1 + normal * n_max + l1 * l1_min + l3 * l3_max,
		p2 + normal * n_max + l1 * l1_max + l2 * l2_min,
		p3 + normal * n_max + l2 * l2_max + l3 * l3_min,
		// bot triangle
		p1 + normal * n_min + l1 * l1_min + l3 * l3_max,
		p2 + normal * n_min + l1 * l1_max + l2 * l2_min,
		p3 + normal * n_min + l2 * l2_max + l3 * l3_min,
	};

	for (auto p : pointArray) {
		// store pos
		for (int m = 0; m < 3; ++m)
			vboData[vboIndex++] = float(p[m]);

		// TODO not nessessary
		// store normal
		//for (int m = 0; m < 3; ++m)
		vboData[vboIndex++] = float(face_index);
		//vboData[vboIndex++] = float(1.0);
		vboData[vboIndex++] = float(0.0);
		vboData[vboIndex++] = float(0.0);

		// store texcoord
		//texCoord = bezierTriangleMesh_.texcoord2D(v); // TODO
		//vboData[vboIndex++] = texCoord[0];
		//vboData[vboIndex++] = texCoord[1];
		vboData[vboIndex++] = 1.0;
		vboData[vboIndex++] = 0.0;
	}

	// first face - top
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 0;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 1;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 2;
	// second face - bot
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 3;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 5;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 4;
	// third face - front
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 3;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 4;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 0;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 4;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 1;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 0;
	// forth face - right
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 4;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 5;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 1;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 5;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 2;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 1;
	// fifth face - left
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 5;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 3;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 2;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 3;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 0;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 2;
};

}