#pragma once

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <queue>
#include <unordered_set>
#include <fstream>

#define PRINT

namespace betri
{

using VH = BezierTMesh::VertexHandle;
using EH = BezierTMesh::EdgeHandle;
using HH = BezierTMesh::HalfedgeHandle;
using FH = BezierTMesh::FaceHandle;
using P = BezierTMesh::Point;

using ID = int;
static const VH INVALID_V = BezierTMesh::InvalidVertexHandle;
static const HH INVALID_H = BezierTMesh::InvalidHalfedgeHandle;
static const EH INVALID_E = BezierTMesh::InvalidEdgeHandle;
static const FH INVALID_F = BezierTMesh::InvalidFaceHandle;

struct Dijkstra
{
	static constexpr char *REGION = "region";
	static constexpr char *PREDECESSOR = "pred";
	static constexpr char *DISTANCE = "dist";
	static constexpr char *CROSSED = "crossed";
};

template <class Container>
void dijkstra(BezierTMesh &mesh, Container &seeds, bool useColors=true) {
	// queues used for dijkstra (one for each region)
	std::queue<FH> queue;
	// color map
	std::vector<BezierTMesh::Color> colors;

	// temporary property to easily access region from vertex
	auto id = OpenMesh::getOrMakeProperty<FH, ID>(mesh, Dijkstra::REGION);
	auto pred = OpenMesh::getOrMakeProperty<FH, FH>(mesh, Dijkstra::PREDECESSOR);
	auto dist = OpenMesh::getOrMakeProperty<FH, double>(mesh, Dijkstra::DISTANCE);
	auto crossed = OpenMesh::getOrMakeProperty<EH, bool>(mesh, Dijkstra::CROSSED);

	const float COLOR_STEP = 1.f / seeds.size();
	const double INF = std::numeric_limits<double>::max();
	// initialize face properties
	for (auto &face : mesh.faces()) {
		id[face] = -1;
		pred[face] = INVALID_F;
		dist[face] = INF;
	}

#ifdef PRINT
	std::ofstream out("dijkstra-log.txt", std::ios::out);
#endif

	if (useColors) {
		if (!mesh.has_face_colors())
			mesh.request_face_colors();
		colors.reserve(seeds.size());
	}

	const auto addToRegion = [&](FH &face, ID region) {
		id[face] = region;
		queue.push(face);
		if (useColors) {
			mesh.set_color(face, colors[region]);
		}
	};

	ID i = 0;
	BezierTMesh::Color regionColor(0.f, 0.f, 0.f, 1.f);
	for (auto &face : seeds) {
		dist[face] = 0;
		addToRegion(face, i++);
		colors.push_back(regionColor);
		regionColor[0] += COLOR_STEP;
		regionColor[1] += COLOR_STEP;
		regionColor[2] += COLOR_STEP;
	}


	while (!queue.empty()) {
		// do one dijkstra iteration for each region/seed

		auto face = queue.front();
		queue.pop();

#ifdef PRINT
		out << "predecessor for " << face << " is " << pred[face] << std::endl;
#endif

		P p1 = mesh.calc_face_centroid(face);
		double d = std::numeric_limits<double>::max();
		FH nextF = INVALID_F;
		EH nextE = INVALID_E;

		// iterate over all incident halfedges of this face
		for (auto he = mesh.fh_begin(face); he != mesh.fh_end(face); ++he) {
			const auto &f = mesh.opposite_face_handle(*he);
			// if neighbor face has not yet been visited
			if (id[f] == -1) {
				const P p2 = mesh.calc_edge_midpoint(*he);
				const P p3 = mesh.calc_face_centroid(f);
				// distance to the next face
				const double update = dist[face] + (p1 - p2).norm() + (p2 - p3).norm();
				// update neighbor face distance if the value can be improved
				if (update < dist[f]) {
					dist[f] = update;
					pred[f] = face;
					// if distance is smaller than last smallest distance
					// store this face and incident edge as next to visit
					if (update < d) {
						d = update;
						nextF = f;
						nextE = mesh.edge_handle(*he);
					}
				}
			}
		}

		// add the neighboring face with the smallest distance to the queue
		if (nextF != INVALID_F) {
			crossed[nextE] = true;
			addToRegion(nextF, id[face]);
#ifdef PRINT
			out << "next face " << nextF << " (" << dist[nextF] << "), crossing edge ";
			out << nextE << '(' << crossed[nextE] << ")\n";
#endif
		}
	}

#ifdef PRINT
	for (const auto &f : mesh.faces()) {
		out << "face " << f << '(' << id[f] << ") has distance " << dist[f] << '\n';
	}
	out.close();
#endif
}

}
