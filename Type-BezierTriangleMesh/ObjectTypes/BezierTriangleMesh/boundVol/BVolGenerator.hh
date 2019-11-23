#pragma once

#include <cmath>
#include <iostream> // TODO weg damit
#include <set>

#include "../BezierTMesh.hh"
#include "../BezierMathUtil.hh"
#include "../quickhull/QuickHull.hpp"
#include "../boundVol/Decimator.hh"

namespace betri
{

///////////////////////////////////////////////////////////////////////////////
// BoundingVolume Helper Functions
///////////////////////////////////////////////////////////////////////////////

// TODO this is accesseable without by betri::AABB
enum boundingVolumeType
{
	AABB = 0,
	PrismVolume = 1,
	ConvexHull = 2,
	BoundingMesh = 3,
	BoundingBillboard = 4
};

static void getVertexIndexCounts(int bVolume, int &numVerts, int &numIndices)
{
	constexpr int indicesPerTriangle = 3;
	constexpr int AABBTriangles = 12;
	constexpr int prismTriangles = 6 + 2;
	constexpr int hullTriangles = 8;
	constexpr int bMeshTriangles = 8; // TODO
	constexpr int bBoardTriangles = 4; // TODO

	switch (bVolume) {
		case boundingVolumeType::AABB:
			numVerts = 8;
			numIndices = AABBTriangles * indicesPerTriangle;
			break;
		case boundingVolumeType::PrismVolume:
			numVerts = 6;
			numIndices = prismTriangles * indicesPerTriangle;
			break;
		case boundingVolumeType::ConvexHull:
			numVerts = 6; // TODO 
			numIndices = hullTriangles * indicesPerTriangle; // BIG TODO
			break;
		case boundingVolumeType::BoundingMesh:
			numVerts = 6; // TODO 
			numIndices = bMeshTriangles * indicesPerTriangle; // BIG TODO
			break;
		case boundingVolumeType::BoundingBillboard:
			numVerts = 6; // TODO 
			numIndices = bBoardTriangles * indicesPerTriangle; // BIG TODO
			break;
		default:
			numVerts = 0;
			numIndices = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////
// BoundingBox
///////////////////////////////////////////////////////////////////////////////
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

		/*
		// TODO not nessessary
		// store normal
		//for (int m = 0; m < 3; ++m)
		vboData[vboIndex++] = float(face_index);
		//vboData[vboIndex++] = float(1.0);
		vboData[vboIndex++] = float(0.0);
		vboData[vboIndex++] = float(0.0);
		*/

		// store texcoord
		//texCoord = bezierTriangleMesh_.texcoord2D(v); // TODO
		//vboData[vboIndex++] = texCoord[0];
		//vboData[vboIndex++] = texCoord[1];
		vboData[vboIndex++] = float(face_index);
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
	// sixth face - rigth
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 3;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 7;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 2;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 7;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 6;
	iboData[iboIndex++] = face_index * boundingVolumeVCount + 2;
}

///////////////////////////////////////////////////////////////////////////////
// BoundingPrism
///////////////////////////////////////////////////////////////////////////////
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

	//std::cerr << "intersect_s: " << intersect_s << " s: " << s << std::endl;
	//std::cerr << " plane_n " << plane_n << " plane_p " << plane_p << " line_p0 " << line_p0 << " line_p1 " << line_p1 << " dir_vec " << dir_vec << std::endl;
	//std::cerr << "dot(plane_n, dir_vec) " << dot(plane_n, dir_vec) << std::endl;
	//std::cerr << "dot(plane_n, plane_p - line_p0) " << dot(plane_n, plane_p - line_p0) << std::endl;

	if (intersect_s < s && intersect_s > 0) {
		intersection = line_p0 + intersect_s * dir_vec;
		return true;
	}

	return false;
}

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

		// TODO does not work for higher degrees but should not be nessessary in generell
		if (i == 0 || i == grad || i == controlPointsPerFace - 1)
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

		if (intersect) {
			// Calculate ratio between intersect-cp and p2-intersect
			double ratio = length(cp - intersection) / length(intersection - p2);

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
		// TODO double dotP = dot(cp - p1, normal) 
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
		for (int m = 0; m < 3; ++m)
			vboData[vboIndex++] = float(p[m]);

		/*
		// TODO not nessessary
		// store normal
		//for (int m = 0; m < 3; ++m)
		vboData[vboIndex++] = float(face_index);
		//vboData[vboIndex++] = float(1.0);
		vboData[vboIndex++] = float(0.0);
		vboData[vboIndex++] = float(0.0);
		*/

		// store texcoord
		vboData[vboIndex++] = float(face_index);
		vboData[vboIndex++] = 0.0;
	}

	// TODO ask franziska if this is better
	std::array<int, 24> indexArray = {
		0, 1, 2,
		3, 5, 4,
		3, 4, 0, 4, 1, 0,
		4, 5, 1, 5, 2, 1,
		5, 3, 2, 3, 0, 2
	};

	for (int i : indexArray) {
		iboData[iboIndex++] = face_index * boundingVolumeVCount + i;
	}
}

///////////////////////////////////////////////////////////////////////////////
// ConvexHull
///////////////////////////////////////////////////////////////////////////////
// Quick Hull
// http://algolist.ru/maths/geom/convhull/qhull3d.php
// http://www.cogsci.rpi.edu/~destem/gamearch/quickhull.pdf
/*
static std::vector<BezierTMesh::Point> findConvexHull(std::vector<BezierTMesh::Point> &faceControlP)
{
	// Pick start by finding point with lowest y
	auto p1 = faceControlP[0];
	//auto p0 = faceControlP[0];
	int m = 0;

	auto C = std::vector<BezierTMesh::Point>();
	C.push_back(p1);

	for (int t = 1; t < log(log(faceControlP.size())); t++) {
		m = int(pow(2, pow(2, t)));
	}

	return C;
}
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
	quickhull::QuickHull<float> qh;

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

	//////////////////
	// Setup Buffer //
	//////////////////
	const size_t boundingVolumeVCount = hull.m_vertices.size();

	for (auto v : reducedMesh.vertices()) {
		vboData[vboIndex++] = float(reducedMesh.point(v)[0]);
		vboData[vboIndex++] = float(reducedMesh.point(v)[1]);
		vboData[vboIndex++] = float(reducedMesh.point(v)[2]);

		/*
		vboData[vboIndex++] = float(face_index);
		//vboData[vboIndex++] = float(1.0);
		vboData[vboIndex++] = float(0.0);
		vboData[vboIndex++] = float(0.0);
		*/

		vboData[vboIndex++] = float(face_index);
		vboData[vboIndex++] = 0.0;
	}

	for (auto f : reducedMesh.faces()) {
		for (TriMesh::FaceVertexIter fv_it = reducedMesh.fv_iter(f); fv_it.is_valid(); ++fv_it) {
			iboData[iboIndex++] = face_index * boundingVolumeVCount + fv_it->idx();
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

	// TODO im metric_generator_ initialize gehts kaputt (endlosschleife)
	//deci.setMesh(reducedMesh);

	//std::cerr << "You sucessfully set the mesh" << std::endl;

	//deci.doContractions(1);

	//std::cerr << "You sucessfully did a Contraction" << std::endl;

}

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
	/*
std::cerr << "controlP:\n";
for (auto vec : faceControlP) {
	std::cerr << vec << '\n';
}
std::cerr << std::endl;
*/
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
	if (false) {
		const size_t boundingVolumeVCount = hull.m_vertices.size();

		for (auto v : hull.m_vertices) {
			vboData[vboIndex++] = float(v.x);
			vboData[vboIndex++] = float(v.y);
			vboData[vboIndex++] = float(v.z);


			vboData[vboIndex++] = float(face_index);
			//vboData[vboIndex++] = float(1.0);
			vboData[vboIndex++] = float(0.0);
			vboData[vboIndex++] = float(0.0);

			vboData[vboIndex++] = 1.0;
			vboData[vboIndex++] = 0.0;
		}

		//int asd = 0;
		//int adf2 = 0;
		for (auto f : hull.m_faces) {
			//asd++;
			auto e = f.m_halfEdgeIndex;
			do {
				//adf2++;
				//std::cerr << asd << " " << adf2 << std::endl;
				iboData[iboIndex++] = face_index * boundingVolumeVCount + hull.m_halfEdges[e].m_endVertex;
				e = hull.m_halfEdges[e].m_next;
			} while (e != f.m_halfEdgeIndex);
		}
	} else {
		const size_t boundingVolumeVCount = hull.m_vertices.size();

		for (auto v : reducedMesh.vertices()) {
			vboData[vboIndex++] = float(reducedMesh.point(v)[0]);
			vboData[vboIndex++] = float(reducedMesh.point(v)[1]);
			vboData[vboIndex++] = float(reducedMesh.point(v)[2]);

			/*
			vboData[vboIndex++] = float(face_index);
			//vboData[vboIndex++] = float(1.0);
			vboData[vboIndex++] = float(0.0);
			vboData[vboIndex++] = float(0.0);
			*/

			vboData[vboIndex++] = float(face_index);
			vboData[vboIndex++] = 0.0;
		}

		for (auto f : reducedMesh.faces()) {
			for (TriMesh::FaceVertexIter fv_it = reducedMesh.fv_iter(f); fv_it.is_valid(); ++fv_it) {
				iboData[iboIndex++] = face_index * boundingVolumeVCount + fv_it->idx();
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// Bounding Billboard
///////////////////////////////////////////////////////////////////////////////
// https://www.cs.jhu.edu/~misha/Spring16/06.pdf
// Angle calculation
// https://math.stackexchange.com/questions/878785/how-to-find-an-angle-in-range0-360-between-2-vectors
// https://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors
// since the points lie on a plane we can ignore one component?!
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

	assert(start != last);
	assert(BezierTMesh::Point(-INFINITY) != last);

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
			if (length(vector2) < 0.001)
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

static void addBoundingBillboardFromPoints(
	const int controlPointsPerFace,
	const int grad,
	int &vboIndex,
	int &iboIndex,
	int face_index,
	ACG::Vec3d origin,
	ACG::Vec3d normal,
	double nearPlane,
	ACG::GLMatrixd projection,
	ACG::GLMatrixd modelview,
	std::vector<float> &vboData,
	std::vector<int> &iboData,
	std::vector<BezierTMesh::Point> &faceControlP
)
{
	std::vector<BezierTMesh::Point> movedP;

	// Parallel Projection
	// https://stackoverflow.com/questions/9605556/how-to-project-a-point-onto-a-plane-in-3d
	/*
	for (auto p : faceControlP) {
		// project
		auto dir = p - origin;
		auto dist = dot(normal, dir);
		auto projPoint = p - normal * dist;

		//std::cerr << "Original: " << p << " projPoint " << projPoint << std::endl;
		movedP.push_back(projPoint);
	}*/

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
	const size_t boundingVolumeVCount = faceControlP.size();

	for (auto v : reducedMesh.vertices()) {
		vboData[vboIndex++] = float(reducedMesh.point(v)[0]);
		vboData[vboIndex++] = float(reducedMesh.point(v)[1]);
		vboData[vboIndex++] = float(reducedMesh.point(v)[2]);

		/*
		vboData[vboIndex++] = float(face_index);
		//vboData[vboIndex++] = float(1.0);
		vboData[vboIndex++] = float(0.0);
		vboData[vboIndex++] = float(0.0);
		*/

		vboData[vboIndex++] = float(face_index);
		vboData[vboIndex++] = 0.0;
	}

	// TODO das ist dämlich
	vboIndex += 5 * (boundingVolumeVCount - reducedMesh.n_vertices());

	for (auto f : reducedMesh.faces()) {
		for (TriMesh::FaceVertexIter fv_it = reducedMesh.fv_iter(f); fv_it.is_valid(); ++fv_it) {
			iboData[iboIndex++] = face_index * boundingVolumeVCount + fv_it->idx();
		}
	}
}

}