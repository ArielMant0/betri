#include "VoronoiRemesh.hh"

#include "Parametrization.hh"
#include "Fitting.hh"

#include <queue>
#include <unordered_set>
#include <map>
#include <random>
#include <fstream>

#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <OpenFlipper/BasePlugin/PluginFunctions.hh>
#include <OpenFlipper/libs_required/ACG/GL/ColorTranslator.hh>
#include <OpenFlipper/libs_required/ACG/Utils/HaltonColors.hh>

namespace betri
{

static const VoronoiRemesh::VH INVALID_V;
static const VoronoiRemesh::HH INVALID_H;
static const VoronoiRemesh::EH INVALID_E;
static const VoronoiRemesh::FH INVALID_F;

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

	if (!m_mesh.get_property_handle(m_vtt, Props::VERTEXTOTRI))
		m_mesh.add_property(m_vtt, Props::VERTEXTOTRI);

	//if (!m_mesh.get_property_handle(m_border, Props::BORDER))
	//	m_mesh.add_property(m_border, Props::BORDER);

	//if (!m_mesh.get_property_handle(m_split, "split"))
	//	m_mesh.add_property(m_split, "split");

	if (!m_ctrl.get_property_handle(m_ttv, Props::TRITOVERTEX)) {
		m_ctrl.add_property(m_ttv, Props::TRITOVERTEX);
	}

	/*for (const auto &he : m_mesh.halfedges()) {
		border(he) = -1;
	}*/

	if (m_useColors) {
		if (!m_mesh.has_face_colors()) {
			m_mesh.request_face_colors();
		}
		if (!m_mesh.has_edge_colors()) {
			m_mesh.request_edge_colors();
		}
		if (!m_mesh.has_vertex_colors()) {
			m_mesh.request_vertex_colors();
		}

		if (!m_ctrl.has_face_colors()) {
			m_ctrl.request_face_colors();
		}
	}
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

	if (m_mesh.get_property_handle(m_vtt, Props::VERTEXTOTRI))
		m_mesh.remove_property(m_vtt);

	//if (m_mesh.get_property_handle(m_border, Props::BORDER))
	//	m_mesh.remove_property(m_border);

	//if (m_mesh.get_property_handle(m_split, "split"))
	//	m_mesh.remove_property(m_split);

	if (m_ctrl.get_property_handle(m_ttv, Props::TRITOVERTEX))
		m_ctrl.remove_property(m_ttv);

	ShortestPath::clear();
}

/**
 * Preventively splits edges when adjacent tiles only share 1 edge
 * TODO: doesnt work with split faces?
 */
void VoronoiRemesh::preventiveEdgeSplits()
{
	std::vector<std::vector<std::pair<int, EH>>> counts;
	counts.reserve(m_seeds.size());
	for (int i = 0; i < m_seeds.size(); ++i) {
		auto arr = std::vector<std::pair<int, EH>>();
		arr.reserve(m_seeds.size() - i);
		for (int j = 0; j < m_seeds.size() - i; ++j) {
			arr.push_back({ 0, INVALID_E });
		}
		counts.push_back(arr);
	}

	for (EH edge : m_mesh.edges()) {
		if (isRegionBorderEdge(edge)) {
			HH h = m_mesh.halfedge_handle(edge, 0);
			FH f1 = m_mesh.face_handle(h), f2 = m_mesh.opposite_face_handle(h);
			// only store count once!
			if (id(f1) < id(f2)) {
				counts[id(f1)][id(f2)-id(f1)].first++;
				counts[id(f1)][id(f2)-id(f1)].second = edge;
			} else {
				counts[id(f2)][id(f1)-id(f2)].first++;
				counts[id(f2)][id(f1)-id(f2)].second = edge;
			}
		}
	}

	auto halfdist = [&](FH f1, FH f2) {
		P p1 = m_mesh.calc_face_centroid(f1);
		P p2 = m_mesh.calc_face_centroid(f2);
		return (p1 - p2).norm();
	};

	for (int i = 0; i < counts.size(); ++i) {
		for (int j = 0; j < counts[i].size(); ++j) {
			if (i != j && counts[i][j].first == 1) {
				auto e = counts[i][j].second;
				assert(e != INVALID_E);

				HH h1 = m_mesh.halfedge_handle(e, 0);
				HH h2 = m_mesh.halfedge_handle(e, 1);

				std::array<HH, 4> prevEdges;
				prevEdges[0] = m_mesh.next_halfedge_handle(h1);
				prevEdges[1] = m_mesh.next_halfedge_handle(prevEdges[0]);
				prevEdges[2] = m_mesh.next_halfedge_handle(h2);
				prevEdges[3] = m_mesh.next_halfedge_handle(prevEdges[2]);

				FH f1 = m_mesh.face_handle(h1);
				FH f2 = m_mesh.opposite_face_handle(h1);

				double f1dist = dist(f1), f2dist = dist(f2);
				m_mesh.splitFacesRivara(f1, f2, true);

				// new faces
				f1 = m_mesh.face_handle(prevEdges[0]);
				FH f3 = m_mesh.face_handle(prevEdges[1]);
				f2 = m_mesh.face_handle(prevEdges[2]);
				FH f4 = m_mesh.face_handle(prevEdges[3]);

				// update predecessor (always face next to old edge)
				pred(f1) = m_mesh.opposite_face_handle(prevEdges[0]);
				pred(f3) = m_mesh.opposite_face_handle(prevEdges[1]);
				pred(f2) = m_mesh.opposite_face_handle(prevEdges[2]);
				pred(f4) = m_mesh.opposite_face_handle(prevEdges[3]);

				// update distances
				dist(f1) = f1dist + halfdist(f1, pred(f1));
				dist(f3) = f1dist + halfdist(f3, pred(f3));
				dist(f2) = f2dist + halfdist(f2, pred(f2));
				dist(f4) = f2dist + halfdist(f4, pred(f4));
			}
		}
	}
}

/**
 * Performs face-based dijkstra on the m_mesh (for the given seeds)
 * TODO: make it so at least 3 regions are created
 */
void VoronoiRemesh::partition()
{
	ACG::HaltonColors cGenerator;
	// special "priority-queue" for dijkstra
	std::set<QElem> q;

	// add a face to a region
	const auto grow = [&](FH &face, FH predFace = INVALID_F, double distance = 0.0) {
		id(face) = predFace == INVALID_F ? m_seeds.size()-1 : id(predFace);
		if (m_useColors) {
			m_mesh.set_color(face, m_colors[id(face)]);
		}
		pred(face) = predFace;
		auto pair = QElem(dist(face), face);
		auto it = q.find(pair);
		if (it != q.end()) q.erase(it);
		dist(face) = distance;
		pair.first = distance;
		q.insert(pair);
	};
	const auto isSeedFace = [&](const FH &face) {
		return m_seeds.find(face) != m_seeds.end();
	};
	const auto addSeedFace = [&](FH &face) {
		assert(!isSeedFace(face));
		m_seeds.insert(face);
		m_boundary.push_back(m_mesh.edge_handle(m_mesh.halfedge_handle(face)));
		if (m_useColors) {
			m_colors.push_back(cGenerator.generateNextColor());
		}
		grow(face);

		// reduce alpha so seed faces are visible
		auto c = m_colors[id(face)];
		c[3] = 0.5f;
		m_mesh.set_color(face, c);

		P p1 = m_mesh.calc_face_centroid(face);
		for (auto he = m_mesh.fh_begin(face); he != m_mesh.fh_end(face); ++he) {
			auto ff = m_mesh.face_handle(he);
			P p2 = m_mesh.calc_face_centroid(ff);
			double distance = (p1 - p2).norm();
			if (distance < dist(ff)) {
				crossed(*he) = id(face);
				grow(ff, face, distance);
			} else {
				crossed(*he) = -1;
			}
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

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<unsigned int> dis(0, m_mesh.n_faces()-1);
	addSeedFace(m_mesh.face_handle(dis(gen)));

	const auto isAdjTo = [&](const VH &v, ID tile) {
		for (auto f = m_mesh.cvf_begin(v); f != m_mesh.cvf_end(v); ++f) {
			if (id(*f) == tile) return true;
		}
		return false;
	};

	const auto homeomorphicDisk = [&](FH &f, VH &v, ID tile) {
		int countEdge = 0;
		for (auto he = m_mesh.cfh_begin(f); he != m_mesh.cfh_end(f); ++he) {
			if (id(m_mesh.opposite_face_handle(*he)) != tile) countEdge++;
		}
		return !(isAdjTo(v, tile) && countEdge == 2);
	};

	const auto adjTiles = [&](const VH &v) {
		std::set<ID> tiles;
		for (auto f = m_mesh.cvf_begin(v); f != m_mesh.cvf_end(v); ++f) {
			if (id(*f) >= 0) tiles.insert(id(*f));
		}
		return tiles.size();
	};

	auto marked = OpenMesh::makeTemporaryProperty<EH, short>(m_mesh);

	const auto nextCutEdge = [&](const HH &h, ID id1, ID id2) {
		VH v = m_mesh.to_vertex_handle(h);
		for (auto voh = m_mesh.voh_begin(v); voh != m_mesh.voh_end(v); ++voh) {
			auto e = m_mesh.edge_handle(*voh);
			auto id11 = id(m_mesh.face_handle(*voh));
			auto id22 = id(m_mesh.opposite_face_handle(*voh));
			if (marked[e] == 0 && id11 == id1 && id22 == id2) {
				return *voh;
			}
		}
		return INVALID_H;
	};

	do {
		// grow tiles until M is covered or COND 1 is violated
		while (!q.empty()) {

			auto it = q.begin();
			auto face = (*it).second;
			q.erase(it);

			P p1 = m_mesh.calc_face_centroid(face);

			bool stop = false;
			// iterate over all incident halfedges of this face
			for (auto he = m_mesh.fh_begin(face); he != m_mesh.fh_end(face) && !stop; ++he) {
				auto &f = m_mesh.opposite_face_handle(*he);
				auto &edge = m_mesh.edge_handle(*he);
				const P p2 = m_mesh.calc_face_centroid(f);
				// distance to the next face
				const double update = dist(face) + (p1 - p2).norm();
				// update neighbor face distance if the value can be improved
				if (update < dist(f)) {
					auto opp = m_mesh.opposite_halfedge_handle(*he);
					auto next = m_mesh.next_halfedge_handle(opp);
					// unmark edges
					if (id(f) != id(face)) {
						for (auto he = m_mesh.fh_begin(f); he != m_mesh.fh_end(f); ++he) {
							crossed(*he) = -1;
						}
					}
					crossed(*he) = id(face);
					auto v = m_mesh.to_vertex_handle(next);
					// COND 1: if not homeomorphic to disk (new face is adj to boundary across
					// which lies a face of the same tile) add face as new seed
					if (!homeomorphicDisk(f, v, id(face))) {
						crossed(*he) = -1;
						addSeedFace(f);
						stop = true;
						break;
					}
					grow(f, face, update);
					// update boundary edge of this tile
					// TODO: does not work reliably
					/*if (id(m_mesh.opposite_face_handle(next)) == id(face))
						next = m_mesh.next_halfedge_handle(next);
					if (id(m_mesh.opposite_face_handle(next)) != id(face)) {
						m_mesh.set_color(m_boundary[id(face)], { 0.f, 0.f, 0.f, 1.f });
						m_boundary[id(face)] = m_mesh.edge_handle(next);
						m_mesh.set_color(m_boundary[id(face)], { 1.f, 0.f, 0.f, 1.f });
					}*/
				}
			}
		}

		// COND 2: if more than one shared boundary per pair of tiles exists, add one of the
		// faces adj to the cut as a new seed face
		std::vector<std::vector<short>> cuts;
		cuts.reserve(m_seeds.size());
		for (int i = 0; i < m_seeds.size(); ++i) {
			cuts.push_back(std::vector<short>(m_seeds.size()-i, 0));
		}

		for (auto &edge : m_mesh.edges()) {
			marked[edge] = 0;
		}

		for (auto &edge : m_mesh.edges()) {
			HH forward = m_mesh.halfedge_handle(edge, 0);
			HH backward = m_mesh.halfedge_handle(edge, 1);

			FH face = m_mesh.face_handle(forward);
			ID id1 = id(face);
			ID id2 = id(m_mesh.face_handle(backward));
			if (marked[edge] == 0) {
				// check id
				if (id1 >= cuts.size() || id2 >= cuts.size() ||
					id1 == id2 || id1 == -1 || id2 == -1) continue;

				while (forward != INVALID_H || backward != INVALID_H) {
					if (forward != INVALID_H) {
						if (isSeedFace(face)) {
							face = m_mesh.face_handle(forward);
						}
						marked[m_mesh.edge_handle(forward)] = 1;
						forward = nextCutEdge(forward, id1, id2);
					}
					if (backward != INVALID_H) {
						if (isSeedFace(face)) {
							face = m_mesh.face_handle(backward);
						}
						marked[m_mesh.edge_handle(backward)] = 1;
						backward = nextCutEdge(backward, id2, id1);
					}
				};
				// deal with cut if necessary
				if (id1 < id2) {
					cuts[id1][id2 - id1]++;
					if (cuts[id1][id2 - id1] > 1) {
						addSeedFace(face);
					}
				} else {
					cuts[id2][id1 - id2]++;
					if (cuts[id2][id1 - id2] > 1) {
						addSeedFace(face);
					}
				}
			}
		}

		// COND 3: if one vertex is adj to more than 3 regions, add one adj face as a new seed face
		for (auto &v : m_mesh.vertices()) {
			if  (adjTiles(v) > 3) {
				int before = m_seeds.size();
				for (auto f = m_mesh.cvf_begin(v); f != m_mesh.cvf_end(v); ++f) {
					if (!isSeedFace(*f)) {
						addSeedFace(*f);
						break;
					}
				}
				if (m_seeds.size() <= before) {
					std::cerr << "ERROR: vertex adj to more than 3 seeds\n";
					// TODO: return an use original mesh
				}
			}
		}

	} while (!q.empty());

	if (m_useColors) {
		BezierTMesh::Color boundaryColor(0.f, 0.f, 0.f, 1.f);
		for (auto &edge : m_mesh.edges()) {
			const auto he = m_mesh.halfedge_handle(edge, 0);
			const auto f1 = m_mesh.face_handle(he);
			const auto f2 = m_mesh.opposite_face_handle(he);
			if (id(f1) == id(f2)) {
				crossed(edge) = id(f1);
				m_mesh.set_color(edge, m_colors[id(f1)]);
			} else {
				m_mesh.set_color(edge, boundaryColor);
			}
		}

		for (auto &edge : m_boundary) {
			m_mesh.set_color(edge, { 1.f, 0.f, 0.f, 1.f });
		}
	}

	std::cerr << "used " << m_seeds.size() << '/' << m_mesh.n_faces();
	std::cerr << " faces as seeds" << std::endl;
}

void VoronoiRemesh::remesh()
{
	if (m_useColors) {
		if (!m_ctrl.has_face_colors()) m_ctrl.request_face_colors();

		if (!m_mesh.has_face_colors()) m_mesh.request_face_colors();
		if (!m_mesh.has_edge_colors()) m_mesh.request_edge_colors();
		if (!m_mesh.has_vertex_colors()) m_mesh.request_vertex_colors();
	}

	partition();

	//return; // EARLY DEBUG RETURN

	// do preventive edge spliiting wherever there are two regions only touching at 1 edge
	preventiveEdgeSplits();

	// return; // EARLY DEBUG RETURN
	size_t nvertices = m_mesh.n_vertices();
	size_t nfaces = m_mesh.n_faces();

	std::vector<VertexHandle> newverts(m_seeds.size());
	std::vector<VertexHandle> sv(m_seeds.size());

	// variable name scope
	{
		std::array<HH,3> neighbors;

		// for each region: add a vertex to the new m_mesh (store in vector to find using region id)
		for (auto f : m_seeds) {
			// remember original halfedges
			auto hit = m_mesh.cfh_begin(f);
			neighbors[0] = hit++;
			neighbors[1] = hit++;
			neighbors[2] = hit;

			sv[id(f)] = m_mesh.splitFaceBarycentric(f, true);
			newverts[id(f)] = m_ctrl.add_vertex(m_mesh.point(sv[id(f)]));

			// assign correct predecessor face
			for (size_t i = 0; i < 3; ++i) {
				if (pred(m_mesh.opposite_face_handle(neighbors[i])) == f) {
					pred(m_mesh.opposite_face_handle(neighbors[i])) = m_mesh.face_handle(neighbors[i]);
				}
			}
		}
	}

	for (auto e : m_mesh.edges()) {
		crossed(e) = -1;
	}

	//return; // EARLY DEBUG RETURN

	// find next edge of a region border
	// (always return the halfedge that is adjacent to face with id f1)
	const auto findStartBorder = [&](const VH &v, ID f1, ID f2) {
		for (auto h = m_mesh.voh_begin(v); h != m_mesh.voh_end(v); ++h) {
			const ID id1 = id(m_mesh.face_handle(*h));
			const ID id2 = id(m_mesh.opposite_face_handle(*h));
			// if edge was not crossed and is adjacent to the given region
			if (id1 == f1 && id2 == f2) {
				return *h;
			} else  if (id1 == f2 && id2 == f1) {
				return m_mesh.opposite_halfedge_handle(*h);
			}
		}
		return INVALID_H;
	};

	// find next edge of a region border
	// (always return the halfedge that is adjacent to face with id f1)
	const auto findNextBorder = [&](const VH &v, ID f1, ID f2, EH forbidden) {
		for (auto h = m_mesh.voh_begin(v); h != m_mesh.voh_end(v); ++h) {
			const ID id1 = id(m_mesh.face_handle(*h));
			const ID id2 = id(m_mesh.opposite_face_handle(*h));
			const EH edge = m_mesh.edge_handle(*h);
			// if edge was not crossed and is adjacent to the given region
			if (edge != forbidden && (id1 == f1 && id2 == f2 ||
				id1 == f2 && id2 == f1)) {
				return *h;
			}
		}
		return INVALID_H;
	};

	const auto connectingHalfedge = [&](const FH from, const FH to, const VH adj) {
		assert(m_mesh.adjToFace(from, to));
		for (auto he = m_mesh.cfh_begin(from); he != m_mesh.cfh_end(from); ++he) {
			if (m_mesh.to_vertex_handle(*he) == adj || m_mesh.from_vertex_handle(*he) == adj) {
				auto v = m_mesh.to_vertex_handle(*he) == adj ?
					m_mesh.from_vertex_handle(*he) :
					m_mesh.to_vertex_handle(*he);

				if (!isCrossed(*he) &&
					m_mesh.opposite_face_handle(*he) != to &&
					m_mesh.adjToFace(v, to)
				) {
					return *he;
				}
			}
		}
		return INVALID_H;
	};

	const auto notCrossed = [&](const FH from, const FH to, const VH node) {
		return connectingHalfedge(from, to, node).is_valid();
	};

	const auto shortestPath = [&](
		const VH v,
		const FH f1,
		const FH f2,
		const FH ctrlFace,
		ShortestPath &path
	) {
		const ID id_1 = id(f1), id_2 = id(f2);

		HH start = findStartBorder(v, id_1, id_2), he = start, bEdge;

		FH f11, f22, tmp1, tmp2;

		double sum = std::numeric_limits<double>::max();

		// find adj faces with shortest path to their respective seed points
		while (he.is_valid()) {
			tmp1 = m_mesh.face_handle(he);
			tmp2 = m_mesh.opposite_face_handle(he);

			const double distSum = dist(tmp1) + dist(tmp2);
			if (distSum <= sum) {
				sum = distSum;
				f11 = id(tmp1) == id_1 ? tmp1 : tmp2;
				f22 = id(tmp2) == id_2 ? tmp2 : tmp1;
				bEdge = he;
			}

			he = findNextBorder(m_mesh.to_vertex_handle(he), id_1, id_2, m_mesh.edge_handle(he));

			// two regions should not have a circular boundary
			assert(he != start);
		}

		std::cerr << "\t\tfound faces " << f11 << " and " << f22 << "\n";

		he = start;

		int count = 0;
		// add interior vertices
		while (he != bEdge) {
			tmp1 = m_mesh.face_handle(he);
			tmp2 = m_mesh.opposite_face_handle(he);
			if (id(tmp1) == id_2) {
				tmp1 = m_mesh.opposite_face_handle(he);
				tmp2 = m_mesh.face_handle(he);
			}
			ttv(ctrlFace).inner.push_back(m_mesh.to_vertex_handle(he));
			he = findNextBorder(m_mesh.to_vertex_handle(he), id_1, id_2, m_mesh.edge_handle(he));
			count++;
		}
		std::cerr << "\t\tadded " << count << " interior vertices\n";

		std::deque<FH> subpath;

		// make a queue containing all faces of the path in correct order
		FH working = f11;
		while (working.is_valid()) {
			subpath.push_front(working);
			std::cerr << "\t\tface " << working << " has pred " << pred(working) << "\n";
			working = pred(working);
		}
		working = f22;
		while (working.is_valid()) {
			subpath.push_back(working);
			std::cerr << "\t\tface " << working << " has pred " << pred(working) << "\n";
			working = pred(working);
		}

		VH node = sv[id(subpath[0])];

		assert(m_mesh.adjToFace(node, subpath[0]));

		HH way; EH edge;
		FH f_it, next;

		size_t sIndex = 0, eIndex = subpath.size(), mod = 1;
		if (!notCrossed(subpath[0], subpath[1], node)) {
			sIndex = eIndex-1;
			eIndex = -1;
			mod = -1;
			node = sv[id(subpath[sIndex])];
			assert(notCrossed(subpath[sIndex], subpath[sIndex - 1], node));
		}

		for (size_t i = sIndex; i != eIndex; i+=mod) {
			f_it = subpath[i];

			m_mesh.set_color(node, { 0.f, 0.f, 0.f, 1.f });
			vtt(node).setFace(INVALID_F);
			m_mesh.set_color(f_it, { 1.f, 1.f, 1.f, 1.f });

			size_t nextIndex = i == eIndex - mod ? i - mod : i + mod;
			next = subpath[nextIndex];

			way = connectingHalfedge(f_it, next, node);
			// if we dont need as many edges, we will get an invalid handle here
			if (way.is_valid()) {
				edge = m_mesh.edge_handle(way);
				node = m_mesh.to_vertex_handle(way) != node ?
					m_mesh.to_vertex_handle(way) :
					m_mesh.from_vertex_handle(way);

				// edge was already crossed
				if (false && i < subpath.size()-1 && isCrossed(node)) {
					std::cerr << "----------------> CROSSING PATHS (at node" << node << ") !!!\n";

					// TODO: check which face needs to be adjusted
					// - if next edge is already crossed: split this face and next
					// - else: split next face and its predecessor (next next face)

					if (!nextEdgeCrossed(edge, f_it)) {
						std::cerr << "\tsplitting current + next\n";
						VH newNode = m_mesh.splitFacesRivara(f_it, next, true);
						// set correct faces
						HH help = m_mesh.find_halfedge(node, newNode);
						subpath[i] = m_mesh.face_handle(help);
						subpath[nextIndex] = m_mesh.opposite_face_handle(help);
						f_it = subpath[i];
						next = subpath[nextIndex];

						way = connectingHalfedge(f_it, next, node);
						edge = m_mesh.edge_handle(way);
					} else {
						std::cerr << "\tsplitting next + nextNext\n";
						FH nextNext = subpath[nextIndex + mod];
						VH newNode = m_mesh.splitFacesRivara(next, nextNext, true);
						// TODO: correct pred for new faces?
						// replace edges in other path
						ShortestPath::replace(m_mesh, node, newNode, id_1, id_2,
							[&](EH e) { crossed(e) = -1; },
							[&](EH eNew, EH eOld) { crossed(eNew) = crossed(eOld); }
						);
					}
					assert(m_mesh.adjToFace(edge, f_it));
				}
				m_mesh.set_color(edge, { 0.f, 0.f, 0.f, 1.f });
				path.push(edge);
				crossed(edge) = ctrlFace.idx();
			}
		}
	};

	//////////////////////////////////////////////////////////
	// create base m_mesh
	//////////////////////////////////////////////////////////
	std::unordered_set<FH> check;
	std::vector<FH> seedFaces; // need a vector to keep correct iterator order
	std::vector<VH> points;

	std::queue<VH> q;
	std::unordered_set<EH> boundary;
	std::unordered_set<VH> inside;

	const auto adjToBorder = [&](const VH v) {
		for (auto he = m_mesh.cve_begin(v); he != m_mesh.cve_end(v); ++he) {
			if (boundary.find(*he) != boundary.end()) {
				return true;
			}
		}
		return false;
	};

	const auto getSeed = [&](unsigned int i) {
		auto f = std::find_if(m_seeds.begin(), m_seeds.end(), [&](const FH &f) {
			return id(f) == i;
		});

		return f != m_seeds.end() ? *f : INVALID_F;
	};

	const auto findInnerVertex = [&](ShortestPath &sp) {
		for (size_t i = 0; i < sp.edges().size(); ++i) {
			auto e = sp.edges()[i];
			HH h1 = m_mesh.next_halfedge_handle(m_mesh.halfedge_handle(e, 0));
			HH h2 = m_mesh.next_halfedge_handle(m_mesh.halfedge_handle(e, 1));

			if (!vtt(m_mesh.to_vertex_handle(h1)).border() && !isCrossed(h1)) {
				return m_mesh.to_vertex_handle(h1);
			} else if (!vtt(m_mesh.to_vertex_handle(h2)).border() && !isCrossed(h2)) {
				return m_mesh.to_vertex_handle(h2);
			}
		}
		return INVALID_V;
	};

	ACG::HaltonColors colgen;

	VH v;
	// for each vertex in the m_mesh
	for (size_t i = 0; i < nvertices; ++i) {
		v = m_mesh.vertex_handle(i);

		std::cerr << "checking vertex " << v << "\n";
		int count = 0;
		// check how many different regions are around that vertex
		for (auto vf = m_mesh.vf_begin(v); vf != m_mesh.vf_end(v); ++vf) {
			const auto f = getSeed(id(*vf));
			if (check.insert(f).second) {
				seedFaces.push_back(f);
				points.push_back(newverts[id(f)]);
			}
		}

		// only do sth if we found 3 regions
		if (seedFaces.size() > 2) {
			std::cerr << "\tVoronoi Vertex\n";

			assert(seedFaces.size() == 3);

			// add the face
			const auto fh = m_ctrl.add_face(points);
			auto col = colgen.generateNextColor();

			for (size_t j = 0; j < seedFaces.size(); ++j) {
				const FH f = seedFaces[j];
				const FH next = seedFaces[(j + 1) % seedFaces.size()];
				// tell face which regions it was generated from (to later find boundary)
				ttv(fh)[j] = id(f);

				// get next face
				ShortestPath bp;
				// collect border halfedges between these two regions
				if (!ShortestPath::has(id(f), id(next))) {
					std::cerr << "\tcalculating shortest path\n";
					bp = ShortestPath(id(f), id(next));
					shortestPath(v, f, next, fh, bp);
					ShortestPath::path(bp);
				} else {
					std::cerr << "\tgetting shortest path\n";
					bp = ShortestPath::path(id(f), id(next));
					auto vh = findInnerVertex(bp);
					if(vh.is_valid()) ttv(fh).inner.push_back(vh);
				}
				boundary.insert(bp.edges().begin(), bp.edges().end());
			}

			if (!adjToBorder(v)) {
				ttv(fh).inner.push_back(v);
			}

			if (!ttv(fh).inner.empty()) {
				// remove vertices falsy classified as being inside the triangle
				auto eraseStart = std::remove_if(ttv(fh).inner.begin(), ttv(fh).inner.end(),
					[&](const VH &vh) { return adjToBorder(vh); }
				);
				if (eraseStart != ttv(fh).inner.end()) {
					ttv(fh).inner.erase(eraseStart);
				}
				// add inner vertices to q
				for (VH vh : ttv(fh).inner) {
					q.push(vh);
					inside.insert(vh);
					//m_mesh.set_color(vh, col);
				}
			}

			assert(!q.empty());

			// find all inner vertices
			while (!q.empty()) {
				auto vert = q.front();
				q.pop();

				for (auto vh = m_mesh.cvv_begin(vert); vh != m_mesh.cvv_end(vert); ++vh) {
					if (!adjToBorder(*vh) &&inside.find(*vh) == inside.end()) {
						ttv(fh).inner.push_back(*vh);
						vtt(*vh).setFace(fh);
						//m_mesh.set_color(vh, col);
						inside.insert(*vh);
						// only add to q if this vertex is not a border vertex
						q.push(*vh);
					}
				}
			}

			assert(q.size() == 0);

			inside.clear();
			boundary.clear();
		}
		points.clear();
		seedFaces.clear();
		check.clear();
	}

	//////////////////////////////////////////////////////////
	// parameterization (harmonic map)
	//////////////////////////////////////////////////////////

	//Parametrization param(m_mesh, m_ctrl, m_ttv, m_vtt, m_pred);

	//param.solve();

	//////////////////////////////////////////////////////////
	// fitting
	//////////////////////////////////////////////////////////

	//Fitting fit(m_mesh, m_ctrl, m_ttv, m_vtt);

	//fit.solve();

	//////////////////////////////////////////////////////////
	// replace original mesh
	//////////////////////////////////////////////////////////

	//m_mesh.clean_keep_reservation();
	// does not work (also not with gc)
	//m_mesh.assign(nmesh);

	// does not work either (also not with gc)
	//m_mesh = nmesh;

	// works but seems really stupid
	//copyMesh(m_ctrl, m_mesh);

	m_mesh.garbage_collection();

	std::cerr << "done" << std::endl;
}

void VoronoiRemesh::copyMesh(BezierTMesh & src, BezierTMesh & dest)
{
	for (const auto &v : src.vertices()) {
		dest.add_vertex_dirty(src.point(v));
	}
	for (const auto &f : src.faces()) {
		std::vector<VH> faces(src.fv_begin(f), src.fv_end(f));
		const FaceHandle fh = dest.add_face(faces);
		dest.data(fh).points(src.data(f).points());
		dest.set_color(fh, src.color(f));
	}
}

}