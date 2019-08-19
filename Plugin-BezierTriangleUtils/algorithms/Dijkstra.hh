#pragma once

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <queue>
#include <unordered_set>
#include <fstream>

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
void dijkstra(BezierTMesh &mesh, Container &seeds) {
	// queues used for dijkstra (one for each region)
	std::vector<std::queue<FH>> queues;
	queues.reserve(seeds.size());

	// temporary property to easily access region from vertex
	auto id = OpenMesh::getOrMakeProperty<FH, ID>(mesh, Dijkstra::REGION);
	auto pred = OpenMesh::getOrMakeProperty<FH, FH>(mesh, Dijkstra::PREDECESSOR);
	auto dist = OpenMesh::getOrMakeProperty<FH, double>(mesh, Dijkstra::DISTANCE);
	auto crossed = OpenMesh::getOrMakeProperty<EH, bool>(mesh, Dijkstra::CROSSED);

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

	ID i = 0;
	for (auto &face : seeds) {
		id[face] = i++;
		dist[face] = 0;
		std::queue<FH> f;
		f.push(face);
		queues.push_back(f);
	}

	int notEmpty;
	do {
		// reset not empty counter
		notEmpty = 0;

		// do one dijkstra iteration for each region/seed
		for (auto &q : queues) {
			if (!q.empty()) {
				notEmpty++;

				auto face = q.front();
				q.pop();

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
					id[nextF] = id[face];
					q.push(nextF);
					crossed[nextE] = true;
#ifdef PRINT
					out << "next face " << nextF << " (" << dist[nextF] << "), crossing edge ";
					out << nextE << '(' << crossed[nextE] << ")\n";
#endif
				}
			}
		}
	} while (notEmpty > 0);
#ifdef PRINT
	for (const auto &f : mesh.faces()) {
		out << "face " << f << '(' << id[f] << ") has distance " << dist[f] << '\n';
	}
	out.close();
#endif
}

}

#define PRINT
