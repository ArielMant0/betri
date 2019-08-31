#include "VoronoiRemesh.hh"

#include <queue>
#include <unordered_set>
#include <map>
#include <random>
#include <fstream>

//#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <OpenFlipper/BasePlugin/PluginFunctions.hh>
#include <OpenFlipper/libs_required/ACG/GL/ColorTranslator.hh>
#include <OpenFlipper/libs_required/ACG/Utils/HuePartitioningColors.hh>

#define PRINT

namespace betri
{

static const VoronoiRemesh::VH INVALID_V = BezierTMesh::InvalidVertexHandle;
static const VoronoiRemesh::HH INVALID_H = BezierTMesh::InvalidHalfedgeHandle;
static const VoronoiRemesh::EH INVALID_E = BezierTMesh::InvalidEdgeHandle;
static const VoronoiRemesh::FH INVALID_F = BezierTMesh::InvalidFaceHandle;

void VoronoiRemesh::prepare()
{
	// (temporary) property to easily access region from vertex
	if (!m_mesh.get_property_handle(m_region, Props::REGION))
		m_mesh.add_property(m_region, Props::REGION);

	if (!m_mesh.get_property_handle(m_pred, Props::PREDECESSOR))
		m_mesh.add_property(m_pred, Props::PREDECESSOR);

	if (!m_mesh.get_property_handle(m_distance, Props::DISTANCE))
		m_mesh.add_property(m_distance, Props::DISTANCE);

	if (!m_mesh.get_property_handle(m_crossed, Props::CROSSED))
		m_mesh.add_property(m_crossed, Props::CROSSED);

	if (!m_mesh.get_property_handle(m_ftt, Props::FACETOTRI))
		m_mesh.add_property(m_ftt, Props::FACETOTRI);
}

void VoronoiRemesh::cleanup()
{
	if (m_mesh.get_property_handle(m_region, Props::REGION))
		m_mesh.remove_property(m_region);

	if (m_mesh.get_property_handle(m_pred, Props::PREDECESSOR))
		m_mesh.remove_property(m_pred);

	if (m_mesh.get_property_handle(m_distance, Props::DISTANCE))
		m_mesh.remove_property(m_distance);

	if (m_mesh.get_property_handle(m_crossed, Props::CROSSED))
		m_mesh.remove_property(m_crossed);

	if (m_mesh.get_property_handle(m_ftt, Props::FACETOTRI))
		m_mesh.remove_property(m_ftt);
}

/**
 * Performs face-based dijkstra on the m_mesh (for the given seeds)
 */
void VoronoiRemesh::partition(std::vector<VoronoiRemesh::FH> &seeds, bool useColors)
{
#ifdef PRINT
	std::ofstream out("voronoi-log.txt", std::ios::out);
	out << seeds.size() << " seeds\n\n";
	out << "started dijkstra" << std::endl;
#endif
	//////////////////////////////////////////////////////////
	// perform dijkstra (creates voronoi regions)
	//////////////////////////////////////////////////////////
	dijkstra(seeds, useColors);

#ifdef PRINT
	out << "finished dijkstra" << std::endl;
#endif

	// new m_mesh
	BezierTMesh nmesh;

	// map regions IDs to new vertices
	std::vector<VH> vertices;
	vertices.reserve(seeds.size());

	// for each region: add a vertex to the new m_mesh (store in vector to find using region id)
	std::transform(seeds.begin(), seeds.end(), vertices.begin(), [&](FH &f) {
		return nmesh.add_vertex(m_mesh.calc_face_centroid(f));
	});

#ifdef PRINT
	out << "added vertices to new m_mesh" << std::endl;
#endif

	// find next edge of a region border
	// (always return the halfedge that is adjacent to face with id f1)
	auto findStartBorder = [&](const VH &v, ID f1, ID f2, EH forbidden) {
		for (auto h = m_mesh.voh_begin(v); h != m_mesh.voh_end(v); ++h) {
			const ID id1 = id(m_mesh.face_handle(*h));
			const ID id2 = id(m_mesh.opposite_face_handle(*h));
			const EH edge = m_mesh.edge_handle(*h);
			// if edge was not crossed and is adjacent to the given region
			if (!isCrossed(edge) && edge != forbidden) {
				if (id1 == f1 && id2 == f2) {
					return *h;
				} else  if (id1 == f2 && id2 == f1) {
					return m_mesh.opposite_halfedge_handle(*h);
				}
			}
		}
		return INVALID_H;
	};

	// find next edge of a region border
	// (always return the halfedge that is adjacent to face with id f1)
	auto findNextBorder = [&](const VH &v, ID f1, EH forbidden) {
		for (auto h = m_mesh.voh_begin(v); h != m_mesh.voh_end(v); ++h) {
			const ID id1 = id(m_mesh.face_handle(*h));
			const ID id2 = id(m_mesh.opposite_face_handle(*h));
			const EH edge = m_mesh.edge_handle(*h);
			// if edge was not crossed and is adjacent to the given region
			if (!isCrossed(edge) && edge != forbidden && id1 == f1 && id2 != f1) {
				return *h;
			} else if (!isCrossed(edge) && edge != forbidden && id1 == f1 && id2 != f1) {
				return m_mesh.opposite_halfedge_handle(*h);
			}
		}
		return INVALID_H;
	};

	auto findCrossedEdge = [&](FH &f, EH &forbidden, ID region) {
#ifdef PRINT
		out << "\tlooking for crossed edge not " << forbidden << std::endl;
#endif
		for (auto fh = m_mesh.fh_begin(f); fh != m_mesh.fh_end(f); ++fh) {
			const HH he = m_mesh.opposite_halfedge_handle(fh);
			const EH e = m_mesh.edge_handle(he);
#ifdef PRINT
			out << "\t\t\tedge (" << e << ") was" << (isCrossed(e) ? "" : " NOT");
			out << " crossed (opp face region = " << id(m_mesh.face_handle(he));
			out << ", face region = " << id(m_mesh.opposite_face_handle(he)) << ")\n";
#endif
			if (isCrossed(e) && e != forbidden && id(m_mesh.face_handle(he)) == region) {
				return e;
			}
		}
		return INVALID_E;
	};

	auto shortestPath = [&](const VH &v, const FH &f1, const FH &f2) {
		const ID id_1 = id(f1), id_2 = id(f2);

#ifdef PRINT
		out << "trying to find path between regions " << id_1 << " and " << id_2 << "\n";
#endif
		HH start = findStartBorder(v, id_1, id_2, INVALID_E);
		//auto b_it = borders[id_1].begin();
		//HH start = firstBorder(*b_it, id_1);
		HH he = start, begin = start;

		FH f11, f22;
		FH tmp1 = m_mesh.face_handle(he), tmp2 = m_mesh.opposite_face_handle(he);
		double sum = std::numeric_limits<double>::max();

		// find faces with shortest path to their respective seed points
		do {
			const double distSum = dist(tmp1) + dist(tmp2);
			if (distSum < sum && id(m_mesh.opposite_face_handle(he)) == id_2) {
#ifdef PRINT
				out << "\t\tnew distance " << distSum << " < " << sum << "\n";
#endif
				sum = distSum;
				f11 = tmp1;
				f22 = tmp2;
				start = he;
			}

			//b_it++;
			//if (b_it == borders[id_1].end()) b_it = borders[id_1].begin();

			he = findNextBorder(m_mesh.to_vertex_handle(he), id_1, m_mesh.edge_handle(he));
			tmp1 = m_mesh.face_handle(he);
			tmp2 = m_mesh.opposite_face_handle(he);
		} while (he != INVALID_H && he != begin && id(tmp1) == id_1 && id(tmp2) == id_2);

#ifdef PRINT
		if (sum == std::numeric_limits<double>::max() || start == INVALID_H) {
			out << "\tcould not find faces or starting edge !!! " << start << "\n";
		} else {
			out << "\tfound faces with shortest distance (" << sum << ") at border\n";
		}
#endif

		EH e1 = m_mesh.edge_handle(start), e2 = m_mesh.edge_handle(start), edge;

		std::deque<FH> path;
		path.push_front(f11);

		while (pred(f11) != INVALID_F) {
			// find next edge
			edge = findCrossedEdge(f11, e1, id_1);
			if (edge == INVALID_E) {
#ifdef PRINT
				out << "\tERROR 1: could not find next crossed edge";
				out << " (predecessor: " << pred(f11) << ", target: " << f1;
				out << ", region: " << id(f11) << ')' << std::endl;
#endif
				break;
			}
			e1 = edge;
			f11 = pred(f11);
			path.push_front(f11);
		}

#ifdef PRINT
		out << "\tadded path faces for region 1 (current path length: ";
		out << path.size() << ')' << std::endl;
#endif

		while (pred(f22) != INVALID_F) {
			edge = findCrossedEdge(f22, e2, id_2);
			if (edge == INVALID_E) {
#ifdef PRINT
				out << "\tERROR 2: could not find next crossed edge";
				out << " (predecessor: " << pred(f22) << ", target: " << f2;
				out << ", region: " << id(f22) << ')' << std::endl;
#endif
				break;
			}
			e2 = edge;
			f22 = pred(f22);
			path.push_back(f22);
		}

#ifdef PRINT
		out << "\tadded path faces for region 2 (total path length: ";
		out << path.size() << ')' << std::endl;
#endif

		return path;
	};

	if (useColors && !nmesh.has_face_colors()) {
		nmesh.request_face_colors();
	}
	if (!m_mesh.get_property_handle(m_ttf, Props::TRITOFACE))
		m_mesh.add_property(m_ttf, Props::TRITOFACE);

	//////////////////////////////////////////////////////////
	// create base m_mesh
	//////////////////////////////////////////////////////////
	std::unordered_set<FH> check;
	std::vector<FH> seedFaces, faceCollection; // need a vector to keep correct iterator order
	std::vector<VH> points;

	// for each vertex in the m_mesh
	for (const auto &v : m_mesh.vertices()) {
		// check how many different regions are around that vertex
		for (auto vf = m_mesh.vf_begin(v); vf != m_mesh.vf_end(v); ++vf) {
			if (check.insert(seeds[id(*vf)]).second) {
				seedFaces.push_back(seeds[id(*vf)]);
			}
		}

#ifdef PRINT
		out << "\nvertex adj to " << seedFaces.size() << " regions" << std::endl;
#endif
		// only do sth if we found at least 3 regions
		if (seedFaces.size() > 2) {
			//points.push_back(nmesh.add_vertex(m_mesh.point(v)));
			for (auto f = seedFaces.begin(); f != seedFaces.end(); ++f) {
				// get next face
				auto next = std::next(f, 1);
				if (next == seedFaces.end()) {
					next = seedFaces.begin();
				}
				points.push_back(vertices[id(*f)]);
				// add all points from shortest path between two regions
				auto path = shortestPath(v, *f, *next);
				faceCollection.push_back(*f);
				faceCollection.insert(faceCollection.end(), path.begin(), path.end());
			}
#ifdef PRINT
			out << "\tadding face with " << points.size() << " vertices" << std::endl;
#endif
			auto before = nmesh.n_faces();
			const auto fh = nmesh.add_face(points);
			ttf(fh).set(faceCollection);

			if (useColors) {
				if (fh.is_valid()) {
					nmesh.set_color(fh, m_mesh.color(*seedFaces.begin()));
				} else {
					auto col = m_mesh.color(*seedFaces.begin());
					for (size_t i = before; i < nmesh.n_faces(); ++i) {
						nmesh.set_color(m_mesh.face_handle(i), col);
					}
				}
			}
			points.clear();
			faceCollection.clear();
		}
		seedFaces.clear();
		check.clear();
	}

	//////////////////////////////////////////////////////////
	// parameterization (harmonic map)
	//////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////
	// replace original mesh
	//////////////////////////////////////////////////////////

	m_mesh.clean_keep_reservation();
	// does not work (also not with gc)
	//m_mesh.assign(nmesh);

	// does not work either (also not with gc)
	//m_mesh = nmesh;

	// works but seems really stupid
	for (const auto &v : nmesh.vertices()) {
		m_mesh.add_vertex_dirty(nmesh.point(v));
	}
	auto before = m_mesh.n_faces();
	for (const auto &f : nmesh.faces()) {
		std::vector<VH> faces(nmesh.fv_begin(f), nmesh.fv_end(f));
		const auto fh = m_mesh.add_face(faces);
		if (useColors) {
			if (fh.is_valid()) {
				m_mesh.set_color(fh, nmesh.color(f));
			} else {
				for (int i = before; i < m_mesh.n_faces(); ++i) {
					m_mesh.set_color(m_mesh.face_handle(i), nmesh.color(f));
				}
			}
			before = m_mesh.n_faces();
		}

	}

#ifdef PRINT
	out << "added faces to the new m_mesh" << std::endl;
	out.close();
#endif

	m_mesh.garbage_collection();
}

void VoronoiRemesh::dijkstra(std::vector<VoronoiRemesh::FH> &seeds, bool useColors)
{
	std::vector<BezierTMesh::Color> colors;
	if (useColors) {
		colors.reserve(seeds.size());
		ACG::HuePartitioningColors::generateNColors(seeds.size(), std::back_inserter(colors));
		if (!m_mesh.has_face_colors()) {
			m_mesh.request_face_colors();
		}
	}

#ifdef PRINT
	std::ofstream out("dijkstra-log.txt", std::ios::out);
	out << "using colors: " << (useColors ? "yes" : "no") << std::endl;
#endif

	using QElem = std::pair<double, FH>;
	// queues used for dijkstra
	std::set<QElem> q;

	// add a face to a region
	const auto addToRegion = [&](FH &face, ID region, double distance) {
		id(face) = region;
		auto pair = QElem(dist(face), face);
		auto it = q.find(pair);
		q.erase(it);
		dist(face) = distance;
		pair.first = distance;
		q.insert(pair);
		// add correct face color
		if (useColors && region >= 0) {
			m_mesh.set_color(face, colors[region]);
		}
	};

	const double INF = std::numeric_limits<double>::max();
	// initialize face properties
	for (auto &face : m_mesh.faces()) {
		pred(face) = INVALID_F;
		id(face) = -1;
		dist(face) = INF;
		q.insert({ INF, face });
	}

#ifdef PRINT
	out << "filled queue" << std::endl;
#endif

	// look at all seeds
	ID i = 0;
	for (auto &face : seeds) {
		addToRegion(face, i++, 0.0);
	}

	while (!q.empty()) {

		auto it = q.begin();
		auto face = (*it).second;
		q.erase(it);
#ifdef PRINT
		out << "relaxing face " << face << std::endl;
#endif

		P p1 = m_mesh.calc_face_centroid(face);

		// iterate over all incident halfedges of this face
		for (auto he = m_mesh.fh_begin(face); he != m_mesh.fh_end(face); ++he) {
			auto &f = m_mesh.opposite_face_handle(*he);
			const P p2 = m_mesh.calc_face_centroid(f);
			// distance to the next face
			const double update = dist(face) + (p1 - p2).norm();
			// update neighbor face distance if the value can be improved
			if (update < dist(f)) {
				pred(f) = face;
				crossed(*he) = true;
				addToRegion(f, id(face), update);
			}
		}
	}

#ifdef PRINT
	int count = 0;
	for (const auto &face : m_mesh.faces()) {
		if (id(face) == -1) count++;
	}
	out << "-- " << count << " faces have no region (" << seeds.size() << " regions, ";
	out << colors.size() << " colors)" << std::endl;
	for (int i = 0; i < colors.size(); ++i) {
		out << i << ": " << colors[i] << "\n";
	}
	out.close();
#endif

	if (useColors) {
		PluginFunctions::setDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_FACES_COLORED |
			ACG::SceneGraph::DrawModes::HIDDENLINE
		);
	}
}

void VoronoiRemesh::remesh(unsigned int size)
{
	if (size > m_mesh.n_faces()) {
		size = m_mesh.n_faces();
	}

	// source nodes
	std::vector<FH> seeds;
	seeds.reserve(size);

	// mostly used for debugging
	if (size == m_mesh.n_faces()) {
		return partition(std::vector<FH>(m_mesh.faces_begin(), m_mesh.faces_end()));
	}

	std::unordered_set<FH> check;
	check.reserve(size);

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);

	//auto neighbors = [&](const BezierTMesh::FaceHandle &f) {
	//	for (auto ff = m_mesh.cff_begin(f); ff != m_mesh.cff_end(f); ++ff) {
	//		if (seeds.find(*ff) != seeds.end()) {
	//			return true;
	//		}
	//	}
	//	return false;
	//};

	while (seeds.size() < size) {
		for (const auto &fh : m_mesh.faces()) {
			if (seeds.size() == size) {
				break;
			}
			if (dis(gen) > 0.5 && check.insert(fh).second) { // && !neighbors(fh)) {
				seeds.push_back(fh);
			}
		}
	}

	partition(seeds, true);
}

}