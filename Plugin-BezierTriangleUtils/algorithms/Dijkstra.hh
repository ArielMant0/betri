#pragma once

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <OpenFlipper/libs_required/ACG/GL/ColorTranslator.hh>
#include <OpenFlipper/libs_required/ACG/Utils/HuePartitioningColors.hh>

#include <set>

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
void dijkstra(BezierTMesh &mesh, Container &seeds, bool useColors=true)
{
	// temporary property to easily access region from vertex
	auto id = OpenMesh::getOrMakeProperty<FH, ID>(mesh, Dijkstra::REGION);
	auto pred = OpenMesh::getOrMakeProperty<FH, FH>(mesh, Dijkstra::PREDECESSOR);
	auto dist = OpenMesh::getOrMakeProperty<FH, double>(mesh, Dijkstra::DISTANCE);
	auto crossed = OpenMesh::getOrMakeProperty<EH, bool>(mesh, Dijkstra::CROSSED);

	std::vector<BezierTMesh::Color> colors;
	if (useColors) {
		colors.reserve(seeds.size());
		ACG::HuePartitioningColors::generateNColors(seeds.size(), std::back_inserter(colors));
		if (!mesh.has_face_colors()) mesh.request_face_colors();
	}

	using QElem = std::pair<double, FH>;
	// queues used for dijkstra
	std::set<QElem> q;

	// add a face to a region
	const auto addToRegion = [&](FH &face, ID region, double distance) {
		id[face] = region;
		auto pair = QElem(dist[face], face);
		auto it = q.find(pair);
		q.erase(it);
		dist[face] = distance;
		pair.first = distance;
		q.insert(pair);
		// add correct face color
		if (useColors && region > 0) {
			mesh.set_color(face, colors[region]);
		}
	};

	const double INF = std::numeric_limits<double>::max();
	// initialize face properties
	for (auto &face : mesh.faces()) {
		pred[face] = INVALID_F;
		id[face] = -1;
		dist[face] = INF;
		q.insert({ INF, face });
	}

	// look at all seeds
	ID i = 0;
	for (auto &face : seeds) {
		addToRegion(face, i++, 0.0);
	}

	while (!q.empty()) {

		auto it = q.begin();
		auto face = (*it).second;
		q.erase(it);

		P p1 = mesh.calc_face_centroid(face);

		// iterate over all incident halfedges of this face
		for (auto he = mesh.fh_begin(face); he != mesh.fh_end(face); ++he) {
			auto &f = mesh.opposite_face_handle(*he);
			const P p2 = mesh.calc_face_centroid(f);
			// distance to the next face
			const double update = dist[face] + (p1 - p2).norm();
			// update neighbor face distance if the value can be improved
			if (update < dist[f]) {
				pred[f] = face;
				addToRegion(f, id[face], update);
			}
		}
	}

	if (useColors &&
		PluginFunctions::drawMode() != ACG::SceneGraph::DrawModes::SOLID_FACES_COLORED
	) {
		PluginFunctions::setDrawMode(ACG::SceneGraph::DrawModes::SOLID_FACES_COLORED);
	}
}

}
