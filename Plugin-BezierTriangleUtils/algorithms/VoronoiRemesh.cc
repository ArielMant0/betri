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

	for (auto &edge : m_mesh.edges()) {
		if (!isCrossed(edge)) {
			HH h = m_mesh.halfedge_handle(edge, 0);
			FH f1 = m_mesh.face_handle(h), f2 = m_mesh.opposite_face_handle(h);
			// only store count once!
			if (id(f1) < id(f2)) {
				counts[id(f1)][id(f2)-id(f1)].first++;
				counts[id(f1)][id(f2)-id(f1)].second = edge;
			} else {
				counts[id(f2)][id(f1) - id(f2)].first++;
				counts[id(f2)][id(f1) - id(f2)].second = edge;
			}
		}
	}

	auto halfdist = [&](FH &f1, FH &f2) {
		P p1 = m_mesh.calc_face_centroid(f1);
		P p2 = m_mesh.calc_face_centroid(f2);
		return (p1 - p2).norm() / 2;
	};

	for (int i = 0; i < counts.size(); ++i) {
		for (int j = 0; j < counts[i].size(); ++j) {
			if (i != j && counts[i][j].first == 1) {
				auto &e = counts[i][j].second;
				assert(e != INVALID_E);
				auto h1 = m_mesh.halfedge_handle(e, 0);
				auto f1 = m_mesh.face_handle(h1);
				auto f2 = m_mesh.opposite_face_handle(h1);

				P p1 = m_mesh.point(m_mesh.to_vertex_handle(h1));
				P p2 = m_mesh.point(m_mesh.from_vertex_handle(h1));

				auto v = m_mesh.add_vertex(p1 * 0.5f + p2 * 0.5f);
				auto beforeFace = m_mesh.n_faces();
				m_mesh.split_edge(e, v);

				// update faces that were already there
				dist(f1) = dist(pred(f1)) + (m_mesh.calc_face_centroid(f1) - m_mesh.calc_face_centroid(pred(f1))).norm();
				dist(f2) = dist(pred(f2)) + (m_mesh.calc_face_centroid(f2) - m_mesh.calc_face_centroid(pred(f2))).norm();

				for (auto he = m_mesh.voh_begin(v); he != m_mesh.voh_end(v); ++he) {
					auto ff1 = m_mesh.face_handle(*he);
					// TODO: check for shortest path to new face
					//if (ff1.idx() >= beforeFace) {
					//	dist(ff1) = std::numeric_limits<double>::max();
					//	P pp1 = m_mesh.calc_face_centroid(ff1);
					//	for (auto ff2 = m_mesh.cff_begin(ff1); ff2 != m_mesh.cff_end(ff1); ++ff2) {
					//		P pp2 = m_mesh.calc_face_centroid(*ff2);
					//		if (ff2->idx() < beforeFace) {

					//		}
					//	}
					//}
					auto ff2 = m_mesh.opposite_face_handle(*he);
					// switch variables so we only have one case
					if (ff2.idx() >= beforeFace && (ff1 == f1 || ff1 == f2)) {
						ff2 = m_mesh.face_handle(*he);
						ff1 = m_mesh.opposite_face_handle(*he);
					}
					// update this new face
					if (ff1.idx() >= beforeFace && (ff2 == f1 || ff2 == f2)) {
						P pp1 = m_mesh.calc_face_centroid(ff1);
						P pp2 = m_mesh.calc_face_centroid(ff2);
						id(ff1) = id(ff2);
						dist(ff1) = dist(ff2) + (pp1-pp2).norm();
						pred(ff1) = ff2;
						crossed(*he) = id(ff2);
						if (m_useColors) {
							m_mesh.set_color(ff1, m_colors[id(ff1)]);
							m_mesh.set_color(m_mesh.edge_handle(*he), m_colors[id(ff1)]);
						}
					} else {
						crossed(*he) = -1;
						if (m_useColors) {
							m_mesh.set_color(m_mesh.edge_handle(*he), { 0.f, 0.f, 0.f, 1.f });
						}
					}
				}
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

	auto newverts = OpenMesh::makeTemporaryProperty<FH, VH>(m_mesh);
	std::vector<VertexHandle> sv;
	sv.reserve(m_seeds.size());

	for (const auto &f : m_mesh.faces()) {
		newverts[f] = INVALID_V;
	}
	// variable name scope
	{
		std::array<HH,3> neighbors;

		// for each region: add a vertex to the new m_mesh (store in vector to find using region id)
		for (auto f : m_seeds) {
			newverts[f] = m_ctrl.add_vertex(m_mesh.calc_face_centroid(f));
			// remember original halfedges
			auto hit = m_mesh.cfh_begin(f);
			neighbors[0] = hit++;
			neighbors[1] = hit++;
			neighbors[2] = hit;

			sv.push_back(m_mesh.splitFaceBarycentric(f, true));
			// assign correct predecessor face
			for (size_t i = 0; i < 3; ++i) {
				pred(m_mesh.opposite_face_handle(neighbors[i])) = m_mesh.face_handle(neighbors[i]);
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
			if (edge != forbidden && id1 == f1 && id2 == f2) {
				return *h;
			}
		}
		return INVALID_H;
	};

	const auto connectingHalfedge = [&](const FH from, const FH to) {
		for (auto he = m_mesh.cfh_begin(from); he != m_mesh.cfh_end(from); ++he) {
			if (!m_mesh.adjToFace(*he, to) &&
				m_mesh.adjToFace(m_mesh.to_vertex_handle(*he), to)) return *he;
		}
		return INVALID_H;
	};

	const auto findSmallestFace = [&](
		const VH vh,
		const ID fid,
		const double value=std::numeric_limits<double>::max()
		) {
		double val = value;
		FaceHandle face;
		for (auto fit = m_mesh.cvf_begin(vh); fit != m_mesh.cvf_end(vh); ++fit) {
			if (id(*fit) == fid && dist(*fit) < val) {
				val = dist(*fit);
				face = *fit;
			}
		}
		return face;
	};

	const auto shortestPath = [&](
		const VH v,
		const FH f1,
		const FH f2,
		const FH ctrlFace,
		ShortestPath &path
	) {
		const ID id_1 = id(f1), id_2 = id(f2);

		HH start = findStartBorder(v, id_1, id_2), he = start, bEdge = start;

		FH f11, f22;
		VH vStart, vTemp = m_mesh.to_vertex_handle(start);
		FH tmp1 = findSmallestFace(vTemp, id_1);
		FH tmp2 = findSmallestFace(vTemp, id_2);

		double sum = std::numeric_limits<double>::max();

		// TODO: these faces must not be adj. they can also just have a vertex in common
		// so looking at vertices rather than edges should be considered

		// find faces with shortest path to their respective seed points
		while (he.is_valid()) {
			const double distSum = dist(tmp1) + dist(tmp2);
			if (distSum <= sum) {
				sum = distSum;
				f11 = tmp1;
				f22 = tmp2;
				vStart = vTemp;
				bEdge = he;
				if (ttv(ctrlFace).inner.empty()) {
					ttv(ctrlFace).inner.push_back(vStart);
				}
			}

			he = findNextBorder(m_mesh.to_vertex_handle(he), id_1, id_2, m_mesh.edge_handle(he));
			if (!he.is_valid()) break;
			// info: the nice thing about this approach is that we always use the vertex
			// furthest away from the center of the new face
			vTemp = m_mesh.to_vertex_handle(he);
			tmp1 = findSmallestFace(vTemp, id_1);
			tmp2 = findSmallestFace(vTemp, id_2);

			// two regions should not have a circular boundary
			assert(he != start);
		}

		std::cerr << "\t\tfound faces " << f11 << " and " << f22 << "\n";

		// if faces are adj, then we dont need another edge along the faces
		if (m_mesh.adjToFace(bEdge, f11) && m_mesh.adjToFace(bEdge, f22)) {
			f11 = pred(f11);
			f22 = pred(f22);
		}
		// find common vertex between border faces of the region as start vertex
		HH connect = connectingHalfedge(f11, pred(f11)), checkNext;
		EH edge;

		if (!pred(f11).is_valid()) {
			for (auto h_it = m_mesh.cfh_begin(f11); h_it != m_mesh.cfh_end(f11); ++h_it) {
				if (m_mesh.to_vertex_handle(*h_it) == sv[id_1]) {
					path.pushFront(edge);
					crossed(edge) = ctrlFace.idx();
					m_mesh.set_color(edge, { 0.f, 1.f, 0.9f, 1.f });
				}
			}
		}

		while (pred(f11) != INVALID_F) {
			assert(m_mesh.is_valid_handle(connect));
			checkNext = m_mesh.next_halfedge_handle(
				m_mesh.opposite_halfedge_handle(m_mesh.next_halfedge_handle(connect))
			);

			edge = m_mesh.edge_handle(connect);
			// to make sure we generate a correct (and nice?) path
			if (pred(f11) != INVALID_F && m_mesh.face_handle(checkNext) != pred(f11)) {
				connect = m_mesh.next_halfedge_handle(m_mesh.opposite_halfedge_handle(connect));
			} else {
				// edge was already used in a path
				if (isCrossed(edge)) {
					std::cerr << "--------------------> CROSSING PATHS (1) !!!\n";
					// TODO: can I just flip the edge here instead?
					// split the faces in question
					m_mesh.splitFacesRivara(f11, pred(f11), true);

					ShortestPath::replace(m_mesh.edge_handle(connect),
						m_mesh.edge_handle(m_mesh.next_halfedge_handle(connect))
					);
					HH prevReplace = m_mesh.prev_halfedge_handle(
						m_mesh.opposite_halfedge_handle(
							m_mesh.prev_halfedge_handle(connect)));
					ShortestPath::replace(m_mesh.edge_handle(prevReplace),
						m_mesh.edge_handle(
							m_mesh.prev_halfedge_handle(prevReplace))
					);
				}

				path.pushFront(edge);
				crossed(edge) = ctrlFace.idx();
				m_mesh.set_color(edge, { 0.f, 1.f, 0.9f, 1.f });
				if (m_mesh.to_vertex_handle(connect) == sv[id_1]) break;
				connect = checkNext;
			}
			// go to predecessor
			f11 = pred(f11);
		}

		connect = connectingHalfedge(f22, pred(f22));
		if (!pred(f22).is_valid()) {
			for (auto h_it = m_mesh.cfh_begin(f22); h_it != m_mesh.cfh_end(f22); ++h_it) {
				if (m_mesh.to_vertex_handle(*h_it) == sv[id_2]) {
					path.pushFront(edge);
					crossed(edge) = ctrlFace.idx();
					m_mesh.set_color(edge, { 0.f, 1.f, 0.9f, 1.f });
				}
			}
		}

		while (pred(f22) != INVALID_F) {
			assert(m_mesh.is_valid_handle(connect));
			checkNext = m_mesh.next_halfedge_handle(
				m_mesh.opposite_halfedge_handle(m_mesh.next_halfedge_handle(connect))
			);

			edge = m_mesh.edge_handle(connect);
			// to make sure we generate a correct (and nice?) path
			if (pred(f22) != INVALID_F && m_mesh.face_handle(checkNext) != pred(f22)) {
				connect = m_mesh.next_halfedge_handle(m_mesh.opposite_halfedge_handle(connect));
			} else {
				// edge was already used in a path
				if (isCrossed(edge)) {
					std::cerr << "--------------------> CROSSING PATHS (2) !!!\n";
					m_mesh.splitFacesRivara(f22, pred(f22), true);

					// TODO: adjust this so onle the new path is affected (saves lots of time)
					ShortestPath::replace(m_mesh.edge_handle(connect),
						m_mesh.edge_handle(m_mesh.next_halfedge_handle(connect))
					);
					HH prevReplace = m_mesh.prev_halfedge_handle(
						m_mesh.opposite_halfedge_handle(
							m_mesh.prev_halfedge_handle(connect)));
					ShortestPath::replace(m_mesh.edge_handle(prevReplace),
						m_mesh.edge_handle(
							m_mesh.prev_halfedge_handle(prevReplace))
					);
				}
				path.pushFront(edge);
				crossed(edge) = ctrlFace.idx();
				m_mesh.set_color(edge, { 0.f, 1.f, 0.9f, 1.f });
				if (m_mesh.to_vertex_handle(connect) == sv[id_2]) break;
				connect = checkNext;
			}
			// go to predecessor
			f22 = pred(f22);
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

	const auto adjToBorder = [&](const VH &v) {
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
				points.push_back(newverts[f]);
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
					ShortestPath prevPath;
					bp = ShortestPath(id(f), id(next));
					shortestPath(v, f, next, fh, bp);
					ShortestPath::path(bp);
				} else {
					std::cerr << "\tgetting shortest path\n";
					bp = ShortestPath::path(id(f), id(next));
				}
				boundary.insert(bp.edges().begin(), bp.edges().end());
			}

			// TODO: general case
			if (!ttv(fh).inner.empty()) {
				q.push(ttv(fh).inner[0]);
				m_mesh.set_color(ttv(fh).inner[0], col);
				inside.insert(ttv(fh).inner[0]);
			}

			if (!adjToBorder(v)) {
				ttv(fh).inner.push_back(v);
				inside.insert(v);
				q.push(v);
				m_mesh.set_color(v, col);
			} else {
				for (auto h_it = m_mesh.cvoh_begin(v); h_it != m_mesh.cvoh_end(v); ++h_it) {
					if (id(m_mesh.face_handle(*h_it)) != id(m_mesh.opposite_face_handle(*h_it))) {
						VH vh = m_mesh.to_vertex_handle(*h_it);
						if (!adjToBorder(vh)) {
							ttv(fh).inner.push_back(vh);
							inside.insert(vh);
							q.push(vh);
							m_mesh.set_color(vh, col);
							break;
						}
					}
				}
			}

			assert(!q.empty());

			// find all inner vertices
			while (!q.empty()) {
				auto vert = q.front();
				q.pop();

				for (auto vh = m_mesh.cvv_begin(vert); vh != m_mesh.cvv_end(vert); ++vh) {
					if (adjToBorder(*vh)) {
						vtt(*vh).face = INVALID_F;
						m_mesh.set_color(vh, { 0.f, 0.f, 0.f, 1.f });
					} else if (inside.find(*vh) == inside.end()) {
						ttv(fh).inner.push_back(*vh);
						vtt(*vh).face = fh;
						m_mesh.set_color(vh, col);
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