#pragma once

#include <cmath>
#include <iostream> // TODO weg damit

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
}

// TODO this is accesseable without by betri::AABB
enum boundingVolumeType
{
	AABB = 0,
	PrismVolume = 1
};

static void getVertexIndexCounts(int bVolume, int &numVerts, int &numIndices)
{

	constexpr int indicesPerTriangle = 3;
	constexpr int AABBTriangles = 12;
	constexpr int prismTriangles = 6 + 2;

	switch (bVolume) {
		case boundingVolumeType::AABB :
			numVerts = 8;
			numIndices = AABBTriangles * indicesPerTriangle;
			break;
		case boundingVolumeType::PrismVolume:
			numVerts = 6;
			numIndices = prismTriangles * indicesPerTriangle;
			break;
		default:
			numVerts = 0;
			numIndices = 0;
	}
}

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
}

static double inline length(BezierTMesh::Point p)
{
	return sqrt(dot(p, p));
}

// http://geomalgorithms.com/a05-_intersect-1.html
static bool inline linePlaneIntersect(
	BezierTMesh::Point line_p0, BezierTMesh::Point line_p1,
	BezierTMesh::Point plane_p, BezierTMesh::Point plane_n,
	BezierTMesh::Point &intersection
)
{
	BezierTMesh::Point dir_vec = normalize(line_p1 - line_p0);
	float s = length(line_p1 - line_p0);

	// if parallel
	if (dot(plane_n, dir_vec) == 0)
		return false;

	// TODO test if line lies in plane?

	// length value
	float intersect_s = dot(plane_n, plane_p - line_p0) / dot(plane_n, dir_vec);

	std::cerr << "intersect_s: " << intersect_s << " s: " << s << std::endl;
	//std::cerr << " plane_n " << plane_n << " plane_p " << plane_p << " line_p0 " << line_p0 << " line_p1 " << line_p1 << " dir_vec " << dir_vec << std::endl;
	//std::cerr << "dot(plane_n, dir_vec) " << dot(plane_n, dir_vec) << std::endl;
	//std::cerr << "dot(plane_n, plane_p - line_p0) " << dot(plane_n, plane_p - line_p0) << std::endl;

	if (s > 0.6) {
		std::cerr << "line_p0: " << line_p0 << " line_p1: " << line_p1 << " result: " << (line_p0 + intersect_s * dir_vec) << std::endl;
	}

	if (intersect_s < s && intersect_s > 0) {
		intersection = line_p0 + intersect_s * dir_vec;
		return true;
	}

	return false;
}

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
	BezierTMesh::Point normal = normalize(cross(p2 - p1, p3 - p1));
	std::cerr << normal << std::endl;
	float l1_min = 0;
	float l2_min = 0;
	float l3_min = 0;
	float n_min = 0;
	float l1_len = sqrt(dot(l1, l1));
	float l2_len = sqrt(dot(l2, l2));
	float l3_len = sqrt(dot(l3, l3));
	float l1_max = 0;
	float l2_max = 0;
	float l3_max = 0;
	float n_max = 0.0;
	//l1 = normalize(l1);
	//l2 = normalize(l2);
	//l3 = normalize(l3);

	std::cerr << "P2: " << p2 << " p3: " << p3 << "\n" << std::endl;

	// TODO Not yet sure if this is totally correct
	for (int i = 0; i < controlPointsPerFace; i++) {
		cp = faceControlP[i];

		if (i == 0 || i == 2 || i == 5)
			continue;

		// TODO is das gut so
		BezierTMesh::Point intersection;

		// TODO boolean ins if
		// Calculate intersection between p1-cp and p2-p3
		bool intersect = linePlaneIntersect(p1, cp, p2, normalize(cross(normal, p3 - p2)), intersection);
		
		if (intersect) {
			// Calculate ratio between intersect-cp and p1-intersect
			double ratio = length(cp - intersection) / length(intersection - p1);

			if (ratio > l1_max) {
				l1_max = ratio;
				l3_min = -ratio;
			}
		}	


		// Calculate intersection between p2-cp and p1-p3
		intersect = linePlaneIntersect(p2, cp, p1, normalize(cross(normal, p1 - p3)), intersection);
		std::cerr << "Lenght: " << length(intersection - p1) << std::endl;

		if (intersect) {
			// Calculate ratio between intersect-cp and p2-intersect
			double ratio = length(cp - intersection) / length(intersection - p2);

			std::cerr << "Ratio: " << ratio << " intersect: " << intersection << "\n" << std::endl;

			if (ratio > l2_max) {
				l2_max = ratio;
				l1_min = -ratio;
			}
		}
		
		// Calculate intersection between p3-cp and p1-p2
		intersect = linePlaneIntersect(p3, cp, p1, normalize(cross(normal, p2 - p1)), intersection);

		if (intersect) {
			// Calculate ratio between intersect-cp and p3-intersect
			double ratio = length(cp - intersection) / length(intersection - p3);

			if (ratio > l3_max) {
				l3_max = ratio;
				l2_min = -ratio;
			}
		}

		// Calculate ratio between p3-intersect and intersect-cp

		/*
		dot(cp - p1, l1) < l1_min ? l1_min = dot(cp - p1, l1) : -1;
		dot(cp - p1, l1) > l1_max ? l1_max = dot(cp - p1, l1) : -1;

		dot(cp - p2, l2) < l2_min ? l2_min = dot(cp - p2, l2) : -1;
		dot(cp - p2, l2) > l2_max ? l2_max = dot(cp - p2, l2) : -1;

		dot(cp - p3, l3) < l3_min ? l3_min = dot(cp - p3, l3) : -1;
		dot(cp - p3, l3) > l3_max ? l3_max = dot(cp - p3, l3) : -1;
		*/

		// TODO double dotP = dot(cp - p1, normal) 
		dot(cp - p1, normal) < n_min ? n_min = dot(cp - p1, normal) : -1;
		dot(cp - p1, normal) > n_max ? n_max = dot(cp - p1, normal) : -1;
	}

	std::cerr << p1 << " " << p2 << " " << (p2 - p1) << std::endl;
	std::cerr << (p2 + l1 * l1_max) << " " << (l1 * l1_max) << " " << l1 << " " << l1_max << std::endl;

	/*
	std::cerr << "l1_min " << l1_min << std::endl;
	std::cerr << "l2_min " << l2_min << std::endl;
	std::cerr << "l3_min " << l3_min << std::endl;

	std::cerr << "l1_max " << l1_max << " l1_len " << l1_len << std::endl;
	std::cerr << "12_max " << l2_max << " l2_len " << l2_len << std::endl;
	std::cerr << "l3_max " << l3_max << " l3_len " << l3_len << std::endl;*/

	std::array<BezierTMesh::Point, boundingVolumeVCount> pointArray = {
		// top triangle
		p1 + normal * n_max + l1 * l1_min + l3 * l3_max,
		p2 + normal * n_max + l1 * l1_max + l2 * l2_min,
		//p2 + normal * n_max + l2 * l2_min,
		//p2 + l1 * l1_max,
		p3 + normal * n_max + l2 * l2_max + l3 * l3_min,
		//p3 + normal * n_max + l2 * l2_max,
		//p3 + l3 * l3_min,

		// bot triangle
		p1 + normal * n_min + l1 * l1_min + l3 * l3_max,
		p2 + normal * n_min + l1 * l1_max + l2 * l2_min,
		//p2 + normal * n_min + l2 * l2_min,
		//p2 + l1 * l1_max,
		p3 + normal * n_min + l2 * l2_max + l3 * l3_min
		//p3 + normal * n_min + l2 * l2_max
		//p3 + l3 * l3_min
	};

	/*
	std::array<BezierTMesh::Point, boundingVolumeVCount> pointArray = {
		// top triangle
		p1 + normal * n_max + l1 * l1_min + l3 * (l3_max - l3_len),
		p2 + normal * n_max + l1 * (l1_max - l1_len) + l2 * l2_min,
		p3 + normal * n_max + l2 * (l2_max - l2_len) + l3 * l3_min,
		// bot triangle
		p1 + normal * n_min + l1 * l1_min + l3 * (l3_max - l3_len),
		p2 + normal * n_min + l1 * (l1_max - l1_len) + l2 * l2_min,
		p3 + normal * n_min + l2 * (l2_max - l2_len) + l3 * l3_min,
	};
	*/

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
}

}