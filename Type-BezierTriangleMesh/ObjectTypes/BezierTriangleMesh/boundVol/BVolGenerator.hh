#pragma once

#include <cmath>
#include <iostream>
#include <set>
#include <queue>

#include "../BezierTMesh.hh"
#include "../BezierMathUtil.hh"
#include "../quickhull/QuickHull.hpp"
#include "../boundVol/Decimator.hh"

namespace betri
{

///////////////////////////////////////////////////////////////////////////////
// BoundingVolume Helper Functions
///////////////////////////////////////////////////////////////////////////////
enum boundingVolumeType
{
	BoundingTetraeder = 0,
	AABB = 1,
	PrismVolume = 2,
	ConvexHull = 3,
	BoundingMesh = 4,
	BoundingBillboard = 5
};

template <typename T, typename I>
inline void setValue(std::vector<T> &data, T value, I &index)
{
	data.push_back(value);
	//data[index++] = value;
}

/**
 * Estimate how much vertices and indices need to be used for the 
 * chosen bounding volume
 */
static void estimateVertexIndexCounts(int bVolume, int &numVerts, int &numIndices)
{
	constexpr int indicesPerTriangle = 3;
	constexpr int AABBTriangles = 12;
	constexpr int prismTriangles = 6 + 2;
	constexpr int tetraederTriangles = 4;
	constexpr int hullTriangles = 1;
	constexpr int bMeshTriangles = 1;
	constexpr int bBoardTriangles = 1;

	switch (bVolume) {
		case boundingVolumeType::BoundingTetraeder:
			numVerts = 4;
			numIndices = tetraederTriangles * indicesPerTriangle;
			break;
		case boundingVolumeType::AABB:
			numVerts = 8;
			numIndices = AABBTriangles * indicesPerTriangle;
			break;
		case boundingVolumeType::PrismVolume:
			numVerts = 6;
			numIndices = prismTriangles * indicesPerTriangle;
			break;
		case boundingVolumeType::ConvexHull:
			numVerts = 3;
			numIndices = hullTriangles * indicesPerTriangle;
			break;
		case boundingVolumeType::BoundingMesh:
			numVerts = 3;
			numIndices = bMeshTriangles * indicesPerTriangle;
			break;
		case boundingVolumeType::BoundingBillboard:
			numVerts = 3;
			numIndices = bBoardTriangles * indicesPerTriangle;
			break;
		default:
			numVerts = 0;
			numIndices = 0;
	}
}


///////////////////////////////////////////////////////////////////////////////
// BoundingTetraeder
// TODO improve tetrahedron build up
///////////////////////////////////////////////////////////////////////////////
/**
 * Construct a bounding tetrahedron
 * @param controlPointsPerFace
 * @param grad
 * @param vboIndex last index for the vertices
 * @param iboIndex last index for the indexbuffer
 * @param face_index
 * @param vboData data vector to save to
 * @param iboData data vector to save to
 * @param faceControlP control points
 */
static void addBoundingTetraederFromPoints(
	const int controlPointsPerFace,
	const int grad,
	int &vboIndex,
	int &iboIndex,
	int face_index,
	std::vector<float> &vboData,
	std::vector<int> &iboData,
	std::vector<BezierTMesh::Point> &faceControlP
)
{
	quickhull::QuickHull<float> qh;

	std::vector<float> points;
	points.reserve(controlPointsPerFace * 3);
	for (auto p : faceControlP) {
		points.push_back(p[0]);
		points.push_back(p[1]);
		points.push_back(p[2]);
	}
	auto hull = qh.getConvexHullAsMesh(points.data(), controlPointsPerFace, true);

	/////////////////////
	// Build Tetraeder //
	/////////////////////
	// Find biggest face
	// https://www.mathsisfun.com/geometry/herons-formula.html
	int i = 0;
	int maxIndex = 0;
	double maxA = 0.0;
	for (auto f : hull.m_faces) {
		auto he1 = hull.m_halfEdges[f.m_halfEdgeIndex];
		auto he2 = hull.m_halfEdges[he1.m_next];
		auto he3 = hull.m_halfEdges[he2.m_next];
		auto v1 = hull.m_vertices[he1.m_endVertex];
		auto v2 = hull.m_vertices[he2.m_endVertex];
		auto v3 = hull.m_vertices[he3.m_endVertex];
		auto l1 = (v2 - v1).getLength();
		auto l2 = (v3 - v2).getLength();
		auto l3 = (v1 - v3).getLength();

		auto s = (l1 + l2 + l3) / 2.0;
		auto A = sqrt(s * (s - l1) * (s - l2) * (s - l3));
		if (A > maxA) {
			maxIndex = i;
			maxA = A;
		}
		i++;
	}

	auto myFace = hull.m_faces[maxIndex];
	auto he1 = hull.m_halfEdges[myFace.m_halfEdgeIndex];
	auto he2 = hull.m_halfEdges[he1.m_next];
	auto he3 = hull.m_halfEdges[he2.m_next];
	auto v1 = hull.m_vertices[he1.m_endVertex];
	auto v2 = hull.m_vertices[he2.m_endVertex];
	auto v3 = hull.m_vertices[he3.m_endVertex];
	auto v4 = (v1 + v2 + v3) / 3.0 + (v3 - v1).crossProduct(v2 - v1) * 1.0;

	// Shift Vertices
	// we only need to test the three top-side faces since the last one
	// was in the convex hull
	for (auto v : hull.m_vertices) {
		auto normal = (v2 - v4).crossProduct(v1 - v4);
		normal.normalize();
		auto dist = (v - v4).dotProduct(normal);
		if (dist > 0.0) {
			auto dir1 = v1 - v3;
			auto dir1_n = dir1;
			dir1_n.normalize();
			auto dir2 = v2 - v3;
			auto dir2_n = dir2;
			dir2_n.normalize();
			auto dir4 = v4 - v3;
			auto dir4_n = dir4;
			dir4_n.normalize();

			auto dDiv1 = dir1_n.dotProduct(normal);
			auto dDiv2 = dir2_n.dotProduct(normal);
			auto dDiv4 = dir4_n.dotProduct(normal);

			v1 += (dist / dDiv1) * dir1_n;
			v2 += (dist / dDiv2) * dir2_n;
			v4 += (dist / dDiv4) * dir4_n;
		}
	}

	for (auto v : hull.m_vertices) {
		auto normal = (v3 - v4).crossProduct(v2 - v4);
		normal.normalize();
		auto dist = (v - v4).dotProduct(normal);
		if (dist > 0.0) {
			auto dir2 = v2 - v1;
			auto dir2_n = dir2;
			dir2_n.normalize();
			auto dir3 = v3 - v1;
			auto dir3_n = dir3;
			dir3_n.normalize();
			auto dir4 = v4 - v1;
			auto dir4_n = dir4;
			dir4_n.normalize();

			auto dDiv2 = dir2_n.dotProduct(normal);
			auto dDiv3 = dir3_n.dotProduct(normal);
			auto dDiv4 = dir4_n.dotProduct(normal);

			v2 += (dist / dDiv2) * dir2_n;
			v3 += (dist / dDiv3) * dir3_n;
			v4 += (dist / dDiv4) * dir4_n;
		}
	}

	for (auto v : hull.m_vertices) {
		auto normal = (v1 - v4).crossProduct(v3 - v4);
		normal.normalize();
		auto dist = (v - v4).dotProduct(normal);
		if (dist > 0.0) {
			auto dir3 = v3 - v2;
			auto dir3_n = dir3;
			dir3_n.normalize();
			auto dir1 = v1 - v2;
			auto dir1_n = dir1;
			dir1_n.normalize();
			auto dir4 = v4 - v2;
			auto dir4_n = dir4;
			dir4_n.normalize();

			auto dDiv3 = dir3_n.dotProduct(normal);
			auto dDiv1 = dir1_n.dotProduct(normal);
			auto dDiv4 = dir4_n.dotProduct(normal);

			v3 += (dist / dDiv3) * dir3_n;
			v1 += (dist / dDiv1) * dir1_n;
			v4 += (dist / dDiv4) * dir4_n;
		}
	}

	std::vector<BezierTMesh::Point> newPoints;
	newPoints.push_back(BezierTMesh::Point(v1.x, v1.y, v1.z));
	newPoints.push_back(BezierTMesh::Point(v2.x, v2.y, v2.z));
	newPoints.push_back(BezierTMesh::Point(v3.x, v3.y, v3.z));
	newPoints.push_back(BezierTMesh::Point(v4.x, v4.y, v4.z));

	//////////////////
	// Setup Buffer //
	//////////////////
	const size_t boundingVolumeVCount = 4;

	for (auto v : newPoints) {
		setValue(vboData, float(v[0]), vboIndex);
		setValue(vboData, float(v[1]), vboIndex);
		setValue(vboData, float(v[2]), vboIndex);

		setValue(vboData, float(face_index), vboIndex);
		setValue(vboData, 0.0f, vboIndex);
	}

	for (int i = 0; i <= 3; ++i) {
		for (int j = i; j <= i + 2; ++j) {
			setValue(
				iboData,
				int(face_index * boundingVolumeVCount + (j % 4)),
				iboIndex
			);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// BoundingBox
///////////////////////////////////////////////////////////////////////////////
/**
 * Construct a bounding box
 * @param controlPointsPerFace
 * @param grad
 * @param vboIndex last index for the vertices
 * @param iboIndex last index for the indexbuffer
 * @param face_index
 * @param vboData data vector to save to
 * @param iboData data vector to save to
 * @param faceControlP control points
 */
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
	BezierTMesh::Point max(-INFINITY);
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
		for (int m = 0; m < 3; ++m) {
			setValue(vboData, float(p[m]), vboIndex);
		}

		setValue(vboData, float(face_index), vboIndex);
		setValue(vboData, 0.f, vboIndex);
	}

	// TODO loop instead of list
	// first face - front
	setValue(iboData, face_index*boundingVolumeVCount + 0, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 3, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 1, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 3, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 2, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 1, iboIndex);
	// second face - top
	setValue(iboData, face_index*boundingVolumeVCount + 1, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 2, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 5, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 2, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 6, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 5, iboIndex);
	// third face - back
	setValue(iboData, face_index*boundingVolumeVCount + 5, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 6, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 4, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 6, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 7, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 4, iboIndex);
	// forth face - bottom
	setValue(iboData, face_index*boundingVolumeVCount + 4, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 7, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 0, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 7, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 3, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 0, iboIndex);
	// fifth face - left
	setValue(iboData, face_index*boundingVolumeVCount + 4, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 0, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 5, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 0, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 1, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 5, iboIndex);
	// sixth face - right
	setValue(iboData, face_index*boundingVolumeVCount + 3, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 7, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 2, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 7, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 6, iboIndex);
	setValue(iboData, face_index*boundingVolumeVCount + 2, iboIndex);
}

///////////////////////////////////////////////////////////////////////////////
// BoundingPrism
// http://geomalgorithms.com/a05-_intersect-1.html
///////////////////////////////////////////////////////////////////////////////
/**
 * Test for intersection point of a line and a plane
 * @param line_p0
 * @param line_p1
 * @param plane_p
 * @param plane_n
 * @param intersection 
 */
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

	// length value
	float intersect_s = dot(plane_n, plane_p - line_p0) / dot(plane_n, dir_vec);

	if (intersect_s < s && intersect_s > 0) {
		intersection = line_p0 + intersect_s * dir_vec;
		return true;
	}

	return false;
}

/**
 * Construct a bounding prism
 * @param controlPointsPerFace
 * @param grad
 * @param vboIndex last index for the vertices
 * @param iboIndex last index for the indexbuffer
 * @param face_index
 * @param vboData data vector to save to
 * @param iboData data vector to save to
 * @param faceControlP control points
 */
static void addPrismVolumeFromPoints(
	const int controlPointsPerFace,
	const int grad,
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
	BezierTMesh::Point p2 = faceControlP[grad];
	BezierTMesh::Point p3 = faceControlP[controlPointsPerFace - 1];
	BezierTMesh::Point l1 = p2 - p1;
	BezierTMesh::Point l2 = p3 - p2;
	BezierTMesh::Point l3 = p1 - p3;
	BezierTMesh::Point normal = normalize(cross(p2 - p1, p3 - p1));
	float l1_min = 0;
	float l2_min = 0;
	float l3_min = 0;
	float n_min = 0;
	float l1_max = 0;
	float l2_max = 0;
	float l3_max = 0;
	float n_max = 0;

	for (int i = 0; i < controlPointsPerFace; i++) {
		cp = faceControlP[i];

		if (i == 0 || i == grad || i == controlPointsPerFace - 1)
			continue;

		// TODO: dont make intersection test, calculate distance to plane?
		BezierTMesh::Point intersection;

		// Calculate intersection between p1-cp and p2-p3
		if (linePlaneIntersect(p1, cp, p2, normalize(cross(normal, p3 - p2)), intersection)) {
			// Calculate ratio between intersect-cp and p1-intersect
			double ratio = length(cp - intersection) / length(intersection - p1);

			if (ratio > l1_max) {
				l1_max = ratio;
				l3_min = -ratio;
			}
		}

		// Calculate intersection between p2-cp and p1-p3
		if (linePlaneIntersect(p2, cp, p1, normalize(cross(normal, p1 - p3)), intersection)) {
			// Calculate ratio between intersect-cp and p2-intersect
			double ratio = length(cp - intersection) / length(intersection - p2);

			if (ratio > l2_max) {
				l2_max = ratio;
				l1_min = -ratio;
			}
		}

		// Calculate intersection between p3-cp and p1-p2
		if (linePlaneIntersect(p3, cp, p1, normalize(cross(normal, p2 - p1)), intersection)) {
			// Calculate ratio between intersect-cp and p3-intersect
			double ratio = length(cp - intersection) / length(intersection - p3);

			if (ratio > l3_max) {
				l3_max = ratio;
				l2_min = -ratio;
			}
		}

		// Calculate ratio between p3-intersect and intersect-cp
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
		p3 + normal * n_min + l2 * l2_max + l3 * l3_min
	};

	for (auto p : pointArray) {
		// store pos
		for (int m = 0; m < 3; ++m) {
			setValue(vboData, float(p[m]), vboIndex);
		}

		// store texcoord
		setValue(vboData, float(face_index), vboIndex);
		setValue(vboData, 0.0f, vboIndex);
	}

	std::array<int, 24> indexArray = {
		0, 1, 2,
		3, 5, 4,
		3, 4, 0, 4, 1, 0,
		4, 5, 1, 5, 2, 1,
		5, 3, 2, 3, 0, 2
	};

	for (int i : indexArray) {
		setValue(iboData, face_index * boundingVolumeVCount + i, iboIndex);
	}
}

///////////////////////////////////////////////////////////////////////////////
// ConvexHull
// https://www.cs.jhu.edu/~misha/Spring16/06.pdf
// Angle calculation
// https://math.stackexchange.com/questions/878785/how-to-find-an-angle-in-range0-360-between-2-vectors
// https://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors
///////////////////////////////////////////////////////////////////////////////
/**
 * Construct a convex hull for a point set using Gift wrapping
 *
 * @param faceControlP control points
 */
static std::vector<size_t> findConvexHull2D(std::vector<BezierTMesh::Point> &faceControlP)
{
	std::vector<size_t> convexHull;

	// Search for biggest x as one of the points of the convex hull
	auto start = BezierTMesh::Point(-INFINITY);
	auto last = BezierTMesh::Point(-INFINITY);

	for (auto p : faceControlP) {
		if (p[0] > start[0] || (p[0] >= start[0] && p[1] < start[1])) {
			start = p;
		}
	}

	for (auto p : faceControlP) {
		if (p[0] > last[0] && (p[0] < start[0] || p[1] > start[1])) {
			last = p;
		}
	}

	int itCount = 0;
	size_t lastIndex = 0;
	BezierTMesh::Point tmp;
	BezierTMesh::Point pos = start;
	// Search as long as there is no loop
	do {
		auto angleMax = 1.0;
		auto vector1 = last - pos;
		auto normVec1 = vector1;

		for (size_t j = 0; j < faceControlP.size(); j++) {
			auto pointNow = faceControlP.at(j);
			auto vector2 = pointNow - pos;
			auto normVec2 = vector2;

			// If it is the same point continue
			if (length(vector2) == 0.0)
				continue;
			auto angle = dot(normalize(normVec1), normalize(normVec2));
			auto vector3 = tmp - pos;
			if (angle < angleMax || (angle == angleMax && length(vector2) > length(vector3))) {
				angleMax = angle;
				tmp = pointNow;
				lastIndex = j;
			}
		}
		convexHull.push_back(lastIndex);
		last = pos;
		pos = tmp;
		assert(itCount++ < faceControlP.size());
	} while (pos != start);

	assert(convexHull.size() <= faceControlP.size());

	return convexHull;
}

/**
 * Construct a convex hull using quick hull
 * http://algolist.ru/maths/geom/convhull/qhull3d.php
 * http://www.cogsci.rpi.edu/~destem/gamearch/quickhull.pdf
 * Source: https://github.com/akuukka/quickhull/blob/master/QuickHull.hpp
 *
 * @param controlPointsPerFace
 * @param grad
 * @param vboIndex last index for the vertices
 * @param iboIndex last index for the indexbuffer
 * @param face_index
 * @param vboData data vector to save to
 * @param iboData data vector to save to
 * @param faceControlP control points
 */
static void addConvexHullFromPoints(
	const int controlPointsPerFace,
	const int grad,
	int &vboIndex,
	int &iboIndex,
	int face_index,
	std::vector<float> &vboData,
	std::vector<int> &iboData,
	std::vector<BezierTMesh::Point> &faceControlP
)
{
	quickhull::QuickHull<double> qh;

	std::vector<double> points;
	points.reserve(controlPointsPerFace * 3);

	for (auto p : faceControlP) {
		points.push_back(p[0]);
		points.push_back(p[1]);
		points.push_back(p[2]);
	}

	auto hull = qh.getConvexHullAsMesh(points.data(), controlPointsPerFace, true);
	std::vector<size_t> hull2;
	if (qh.m_planar) {
		//return;
	}

	//////////////////
	// Convert Mesh //
	//////////////////
	TriMesh reducedMesh;
	reducedMesh.request_vertex_status();
	reducedMesh.request_halfedge_status();
	reducedMesh.request_edge_status();
	reducedMesh.request_face_status();

	for (auto &v : hull.m_vertices) {
		TriMesh::VertexHandle vh = reducedMesh.add_vertex({ v.x, v.y, v.z });
	}

	for (auto &f : hull.m_faces) {
		auto he1 = hull.m_halfEdges[f.m_halfEdgeIndex];
		auto he2 = hull.m_halfEdges[he1.m_next];
		auto he3 = hull.m_halfEdges[he2.m_next];

		TriMesh::VertexHandle vh1 = reducedMesh.vertex_handle(he1.m_endVertex);
		TriMesh::VertexHandle vh2 = reducedMesh.vertex_handle(he2.m_endVertex);
		TriMesh::VertexHandle vh3 = reducedMesh.vertex_handle(he3.m_endVertex);

		// Guarantee that no wrong faces are added
		if (he1.m_endVertex == he2.m_endVertex || 
			he2.m_endVertex == he3.m_endVertex || 
			he3.m_endVertex == he1.m_endVertex)
			continue;

		assert(f.m_halfEdgeIndex != he1.m_next);
		assert(he1.m_next != he2.m_next);
		assert(he2.m_next != f.m_halfEdgeIndex);

		assert(he1.m_endVertex != he2.m_endVertex);
		assert(he2.m_endVertex != he3.m_endVertex);
		assert(he3.m_endVertex != he1.m_endVertex);

		assert(vh1 != vh2 && vh2 != vh3 && vh3 != vh1);
		reducedMesh.add_face(vh1, vh2, vh3);
	}

	//////////////////
	// Setup Buffer //
	//////////////////

	// num vertices that came before in vbo (divide by 3 3D coords + 2 tex coords)
	const size_t beforeSize = vboData.size() / 5;

	for (auto v : reducedMesh.vertices()) {
		setValue(vboData, float(reducedMesh.point(v)[0]), vboIndex);
		setValue(vboData, float(reducedMesh.point(v)[1]), vboIndex);
		setValue(vboData, float(reducedMesh.point(v)[2]), vboIndex);

		setValue(vboData, float(face_index), vboIndex);
		setValue(vboData, 0.0f, vboIndex);
	}

	for (auto f : reducedMesh.faces()) {
		for (TriMesh::FaceVertexIter fv_it = reducedMesh.fv_iter(f); fv_it.is_valid(); ++fv_it) {
			setValue(iboData, int(beforeSize + fv_it->idx()), iboIndex);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// BoundingMesh
// https://github.com/gaschler/bounding-mesh/blob/master/src/boundingmesh/Decimator.cpp
///////////////////////////////////////////////////////////////////////////////
static void generateBoundingMesh(TriMesh &reducedMesh)
{
	boundingmesh::Decimator deci(boundingmesh::Outward);
	deci.setTargetVertices(100);

	std::cerr << "You sucessfully created a decimator" << std::endl;

	// TODO the metric_generator_ is the problem
	//deci.setMesh(reducedMesh);

	//std::cerr << "You sucessfully set the mesh" << std::endl;

	//deci.doContractions(1);

	//std::cerr << "You sucessfully did a Contraction" << std::endl;

}

/**
 * Construct a bounding mesh
 * TODO This is work in progress
 *
 * @param controlPointsPerFace
 * @param grad
 * @param vboIndex last index for the vertices
 * @param iboIndex last index for the indexbuffer
 * @param face_index
 * @param vboData data vector to save to
 * @param iboData data vector to save to
 * @param faceControlP control points
 */
static void addBoundingMeshFromPoints(
	const int controlPointsPerFace,
	const int grad,
	int &vboIndex,
	int &iboIndex,
	int face_index,
	std::vector<float> &vboData,
	std::vector<int> &iboData,
	std::vector<BezierTMesh::Point> &faceControlP
)
{
	quickhull::QuickHull<float> qh;

	//auto data = bezierTriangleMesh_.data(bezierTriangleMesh_.face_handle(0)).points();
	std::vector<float> points;
	points.reserve(controlPointsPerFace * 3);
	for (auto p : faceControlP) {
		points.push_back(p[0]);
		points.push_back(p[1]);
		points.push_back(p[2]);
	}
	auto hull = qh.getConvexHullAsMesh(points.data(), controlPointsPerFace, true);

	//////////////////
	// Convert Mesh //
	//////////////////
	TriMesh reducedMesh;
	for (auto v : hull.m_vertices) {
		reducedMesh.add_vertex({ v.x, v.y, v.z });
	}

	for (auto f : hull.m_faces) {
		auto he1 = hull.m_halfEdges[f.m_halfEdgeIndex];
		auto he2 = hull.m_halfEdges[he1.m_next];
		auto he3 = hull.m_halfEdges[he2.m_next];
		TriMesh::VertexHandle vh1 = reducedMesh.vertex_handle(he1.m_endVertex);
		TriMesh::VertexHandle vh2 = reducedMesh.vertex_handle(he2.m_endVertex);
		TriMesh::VertexHandle vh3 = reducedMesh.vertex_handle(he3.m_endVertex);
		reducedMesh.add_face(vh1, vh2, vh3);
	}

	// https://www.openflipper.org/media/Documentation/OpenFlipper-1.2/tutorial_05.html
	reducedMesh.request_vertex_status();
	reducedMesh.request_edge_status();
	reducedMesh.request_halfedge_status();
	reducedMesh.request_face_status();

	reducedMesh.request_face_normals();
	reducedMesh.request_vertex_normals();

	generateBoundingMesh(reducedMesh);
	/*
	///////////////////
	// Delete Vertex //
	///////////////////
	// TODO Select vertex
	TriMesh::VertexHandle vh = reducedMesh.vertex_handle(3);

	auto oldPos = reducedMesh.point(vh);
	auto oldNormal = reducedMesh.normal(vh);
	reducedMesh.delete_vertex(vh);
	reducedMesh.garbage_collection();

	// Search first border vertex
	TriMesh::HalfedgeHandle borderStart;
	for (auto h_it = reducedMesh.halfedges_begin(); h_it != reducedMesh.halfedges_end(); ++h_it) {
		if (reducedMesh.is_boundary(h_it)) {
			borderStart = h_it;
			break;
		}
	}

	TriMesh::HalfedgeHandle borderIt = borderStart;
	std::vector<TriMesh::VertexHandle> border;
	do {
		// Find next border-vertex
		border.push_back(reducedMesh.to_vertex_handle(borderIt));
		// increment iterator
		borderIt = reducedMesh.next_halfedge_handle(borderIt);
	} while (borderIt != borderStart);

	reducedMesh.add_face(border);

	//////////////////////////
	// Move border vertices //
	//////////////////////////
	std::vector<TriMesh::Point> directions;

	reducedMesh.update_normals();
	//reducedMesh.calc_face_normal;

	for (auto v_it = reducedMesh.vertices_begin(); v_it != reducedMesh.vertices_end(); ++v_it) {
		std::cerr << reducedMesh.point(v_it) << std::endl;
	}

	for (auto i = 0; i < border.size(); i++) {
		BezierTMesh::Point intersection;
		// TODO 100.0
		bool intersect = linePlaneIntersect(
			reducedMesh.point(border[i]),
			reducedMesh.normal(border[i])*100.0,
			oldPos, oldNormal, intersection
		);
		if (intersect)
			reducedMesh.point(border[i]) = intersection;
	}

	for (auto v_it = reducedMesh.vertices_begin(); v_it != reducedMesh.vertices_end(); ++v_it) {
		std::cerr << reducedMesh.point(v_it) << std::endl;
	}*/

	//////////////////
	// Setup Buffer //
	//////////////////

	// num vertices that came before in vbo (divide by 3 3D coords + 2 tex coords)
	const size_t beforeSize = vboData.size() / 5;

	if (false) {

		for (auto v : hull.m_vertices) {
			setValue(vboData, float(v.x), vboIndex);
			setValue(vboData, float(v.y), vboIndex);
			setValue(vboData, float(v.z), vboIndex);


			setValue(vboData, float(face_index), vboIndex);
			//vboData[vboIndex++] = float(1.0);
			setValue(vboData, 0.0f, vboIndex);
			setValue(vboData, 0.0f, vboIndex);

			setValue(vboData, 1.0f, vboIndex);
			setValue(vboData, 0.0f, vboIndex);
		}

		//int asd = 0;
		//int adf2 = 0;
		for (auto f : hull.m_faces) {
			//asd++;
			auto e = f.m_halfEdgeIndex;
			do {
				//adf2++;
				//std::cerr << asd << " " << adf2 << std::endl;
				//iboData[iboIndex++] = face_index * boundingVolumeVCount + hull.m_halfEdges[e].m_endVertex;
				setValue(iboData, int(beforeSize + hull.m_halfEdges[e].m_endVertex), iboIndex);
				e = hull.m_halfEdges[e].m_next;
			} while (e != f.m_halfEdgeIndex);
		}
	} else {

		// num vertices that came before in vbo (divide by 3 3D coords + 2 tex coords)
		const size_t beforeSize = vboData.size() / 5;

		for (auto v : reducedMesh.vertices()) {
			setValue(vboData, float(reducedMesh.point(v)[0]), vboIndex);
			setValue(vboData, float(reducedMesh.point(v)[1]), vboIndex);
			setValue(vboData, float(reducedMesh.point(v)[2]), vboIndex);

			setValue(vboData, float(face_index), vboIndex);
			setValue(vboData, 0.0f, vboIndex);
		}

		for (auto f : reducedMesh.faces()) {
			for (TriMesh::FaceVertexIter fv_it = reducedMesh.fv_iter(f); fv_it.is_valid(); ++fv_it) {
				setValue(iboData, int(beforeSize + fv_it->idx()), iboIndex);
			}
		}

		//const size_t boundingVolumeVCount = hull.m_vertices.size();

		//for (auto v : reducedMesh.vertices()) {
		//	vboData[vboIndex++] = float(reducedMesh.point(v)[0]);
		//	vboData[vboIndex++] = float(reducedMesh.point(v)[1]);
		//	vboData[vboIndex++] = float(reducedMesh.point(v)[2]);

		//	/*
		//	vboData[vboIndex++] = float(face_index);
		//	//vboData[vboIndex++] = float(1.0);
		//	vboData[vboIndex++] = float(0.0);
		//	vboData[vboIndex++] = float(0.0);
		//	*/

		//	vboData[vboIndex++] = float(face_index);
		//	vboData[vboIndex++] = 0.0;
		//}

		//for (auto f : reducedMesh.faces()) {
		//	for (TriMesh::FaceVertexIter fv_it = reducedMesh.fv_iter(f); fv_it.is_valid(); ++fv_it) {
		//		iboData[iboIndex++] = face_index * boundingVolumeVCount + fv_it->idx();
		//	}
		//}
	}
}

///////////////////////////////////////////////////////////////////////////////
// Bounding Billboard
///////////////////////////////////////////////////////////////////////////////

/**
 * Construct a bounding billboard
 *
 * @param controlPointsPerFace
 * @param grad
 * @param vboIndex last index for the vertices
 * @param iboIndex last index for the indexbuffer
 * @param face_index
 * @param origin
 * @param normal
 * @param nearPlane
 * @param vboData data vector to save to
 * @param iboData data vector to save to
 * @param faceControlP control points
 */
static void addBoundingBillboardFromPoints(
	const int controlPointsPerFace,
	const int grad,
	int &vboIndex,
	int &iboIndex,
	int face_index,
	ACG::Vec3d origin,
	ACG::Vec3d normal,
	double nearPlane,
	std::vector<float> &vboData,
	std::vector<int> &iboData,
	std::vector<BezierTMesh::Point> &faceControlP
)
{
	std::vector<BezierTMesh::Point> movedP;

	double nearP = nearPlane;
	// Perspective Projection
	// http://www.cse.psu.edu/~rtc12/CSE486/lecture12.pdf
	for (auto p : faceControlP) {
		auto cpDir = p - origin;
		auto nDist = dot(normal, cpDir);
		auto npDir = p - (origin + (nDist * normal));

		double offset = (length(npDir) / nDist) * nearP;
		auto projPoint = (origin + nearP * normal) + offset * normalize(npDir);
		movedP.push_back(projPoint);
	}

	auto hull = findConvexHull2D(movedP);

	//////////////////
	// Convert Mesh //
	//////////////////
	TriMesh reducedMesh;
	std::vector<BezierTMesh::VertexHandle> vh;
	for (auto index : hull) {
		auto v = faceControlP[index];
		vh.push_back(reducedMesh.add_vertex({ v[0], v[1], v[2] }));
	}
	reducedMesh.add_face(vh);

	//////////////////
	// Setup Buffer //
	//////////////////
	// num vertices that came before in vbo (divide by 3 3D coords + 2 tex coords)
	const size_t beforeSize = vboData.size() / 5;

	for (auto v : reducedMesh.vertices()) {
		setValue(vboData, float(reducedMesh.point(v)[0]), vboIndex);
		setValue(vboData, float(reducedMesh.point(v)[1]), vboIndex);
		setValue(vboData, float(reducedMesh.point(v)[2]), vboIndex);

		setValue(vboData, float(face_index), vboIndex);
		setValue(vboData, 0.0f, vboIndex);
	}

	for (auto f : reducedMesh.faces()) {
		for (TriMesh::FaceVertexIter fv_it = reducedMesh.fv_iter(f); fv_it.is_valid(); ++fv_it) {
			setValue(iboData, int(beforeSize + fv_it->idx()), iboIndex);
		}
	}
}

}
