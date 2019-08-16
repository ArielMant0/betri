#pragma once

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <queue>
#include <unordered_set>

namespace betri
{

template <class Container>
void dijkstra(
	BezierTMesh &mesh,
	Container &seeds,
	const char *regionName="region",
	const char *predecessorName="predecessor",
	const char *distanceName="dist",
	const char *edgeCrossName="crossed"
) {
	using VH = BezierTMesh::VertexHandle;
	using EH = BezierTMesh::EdgeHandle;
	using HH = BezierTMesh::HalfedgeHandle;
	using FH = BezierTMesh::FaceHandle;
	using P = BezierTMesh::Point;

	using ID = int;
	const EH INVALID_E = BezierTMesh::InvalidEdgeHandle;
	const FH INVALID_F = BezierTMesh::InvalidFaceHandle;

	// queues used for dijkstra (one for each region)
	std::vector<std::queue<FH>> queues;
	queues.reserve(seeds.size());

	// temporary property to easily access region from vertex
	auto id = OpenMesh::getOrMakeProperty<FH, ID>(mesh, regionName);
	auto pred = OpenMesh::getOrMakeProperty<FH, FH>(mesh, predecessorName);
	auto dist = OpenMesh::getOrMakeProperty<FH, double>(mesh, distanceName);
	auto crossed = OpenMesh::getOrMakeProperty<EH, bool>(mesh, edgeCrossName);

	const double INF = std::numeric_limits<double>::infinity();
	// initialize face properties
	for (auto &face : mesh.faces()) {
		id[face] = -1;
		pred[face] = INVALID_F;
		dist[face] = INF;
	}

	ID i = 0;
	for (auto &face : seeds) {
		id[face] = i++;
		dist[face] = 0;
		std::queue<FH> f;
		f.push(face);
		queues.push_back(f);
	}

	int notEmpty = 0;
	while (notEmpty > 0) {
		// do one dijkstra iteration for each region/seed
		for (auto &q : queues) {
			if (!q.empty()) {
				notEmpty++;

				auto face = q.front();
				q.pop();

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
				}
			}
		}
		// reset not empty counter
		notEmpty = 0;
	}
}

}