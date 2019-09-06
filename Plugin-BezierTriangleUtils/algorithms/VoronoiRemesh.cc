#include "VoronoiRemesh.hh"

#include "Parametrization.hh"
#include "Subdivision.hh"

#include <queue>
#include <unordered_set>
#include <map>
#include <random>
#include <fstream>

#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <OpenFlipper/BasePlugin/PluginFunctions.hh>
#include <OpenFlipper/libs_required/ACG/GL/ColorTranslator.hh>
#include <OpenFlipper/libs_required/ACG/Utils/HaltonColors.hh>

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

	if (!m_mesh.get_property_handle(m_vtt, Props::FACETOTRI))
		m_mesh.add_property(m_vtt, Props::FACETOTRI);

	if (!m_ctrl.get_property_handle(m_ttv, Props::TRITOFACE)) {
		m_ctrl.add_property(m_ttv, Props::TRITOFACE);
	}

	if (m_useColors) {
		if (!m_mesh.has_face_colors()) {
			m_mesh.request_face_colors();
		}
		if (!m_mesh.has_edge_colors()) {
			m_mesh.request_edge_colors();
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

	if (m_mesh.get_property_handle(m_vtt, Props::TRITOFACE))
		m_mesh.remove_property(m_vtt);

	if (m_ctrl.get_property_handle(m_ttv, Props::TRITOFACE))
		m_ctrl.remove_property(m_ttv);
}

/**
 * Preventively splits edges when adjacent tiles only share 1 edge
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

void VoronoiRemesh::remesh(unsigned int size)
{
	if (size > m_mesh.n_faces()) {
		size = m_mesh.n_faces();
	}

	partition();

	// return; // EARLY DEBUG RETURN

	// do preventive edge spliiting wherever there are two regions only touching at 1 edge
	preventiveEdgeSplits();

	// return; // EARLY DEBUG RETURN

	if (m_useColors && !m_ctrl.has_face_colors()) {
		m_ctrl.request_face_colors();
	}

	auto borders = OpenMesh::makeTemporaryProperty<FH, bool>(m_mesh);
	auto newverts = OpenMesh::makeTemporaryProperty<FH, VH>(m_mesh);

	for (const auto &f : m_mesh.faces()) {
		newverts[f] = INVALID_V;
	}
	// for each region: add a vertex to the new m_mesh (store in vector to find using region id)
	for (const auto &f : m_seeds) {
		newverts[f] = m_ctrl.add_vertex(m_mesh.calc_face_centroid(f));
	}

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
				}
				else  if (id1 == f2 && id2 == f1) {
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
			}
			else if (!isCrossed(edge) && edge != forbidden && id1 == f1 && id2 != f1) {
				return m_mesh.opposite_halfedge_handle(*h);
			}
		}
		return INVALID_H;
	};

	auto findCrossedEdge = [&](FH &f, EH &forbidden, ID region) {
		for (auto fh = m_mesh.fh_begin(f); fh != m_mesh.fh_end(f); ++fh) {
			const HH he = m_mesh.opposite_halfedge_handle(fh);
			const EH e = m_mesh.edge_handle(he);
			if (isCrossed(e) && e != forbidden && id(m_mesh.face_handle(he)) == region) {
				return e;
			}
		}
		return INVALID_E;
	};

	auto shortestPath = [&](const VH &v, const FH &f1, const FH &f2) {
		const ID id_1 = id(f1), id_2 = id(f2);

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
				sum = distSum;
				f11 = tmp1;
				f22 = tmp2;
				//start = he;
			}

			he = findNextBorder(m_mesh.to_vertex_handle(he), id_1, m_mesh.edge_handle(he));
			tmp1 = m_mesh.face_handle(he);
			tmp2 = m_mesh.opposite_face_handle(he);
		} while (he != INVALID_H && he != begin && id(tmp1) == id_1 && id(tmp2) == id_2);

		std::deque<FH> path;
		while (pred(f11) != INVALID_F) {
			path.push_front(f11);
			borders[f11] = true;
			if (m_useColors) {
				m_mesh.set_color(f11, { 1.f, 1.f, 1.f, 1.f });
			}
			f11 = pred(f11);
		}

		while (pred(f22) != INVALID_F) {
			path.push_back(f22);
			borders[f22] = true;
			if (m_useColors) {
				m_mesh.set_color(f11, { 1.f, 1.f, 1.f, 1.f });
			}
			f22 = pred(f22);
		}

		return path;
	};

	//////////////////////////////////////////////////////////
	// create base m_mesh
	//////////////////////////////////////////////////////////
	std::unordered_set<FH> check;
	std::vector<FH> seedFaces; // need a vector to keep correct iterator order
	std::vector<VH> points;


	std::queue<VH> q;
	std::deque<FH> border;
	std::set<VH> in,out;

	const auto adjFaces = [&](const VH &v) {
		int total = 0, count = 0;
		for (auto f = m_mesh.cvf_begin(v); f != m_mesh.cvf_end(v); ++f, total++) {
			if (!borders[f]) count++;
		}
		return std::pair<unsigned int, unsigned int>(total, count);
	};

	const auto getSeed = [&](unsigned int i) {
		auto f = std::find_if(m_seeds.begin(), m_seeds.end(), [&](const FH &f) {
			return id(f) == i;
		});

		return f != m_seeds.end() ? *f : INVALID_F;
	};

	ACG::HaltonColors colgen;

	// for each vertex in the m_mesh
	for (const auto &v : m_mesh.vertices()) {
		// check how many different regions are around that vertex
		for (auto vf = m_mesh.vf_begin(v); vf != m_mesh.vf_end(v); ++vf) {
			const auto f = getSeed(id(*vf));
			if (check.insert(f).second) {
				seedFaces.push_back(f);
			}
		}

		// only do sth if we found 3 regions
		if (seedFaces.size() > 2) {
			assert(seedFaces.size() == 3);

			for (auto f = seedFaces.begin(); f != seedFaces.end(); ++f) {
				points.push_back(newverts[*f]);
				// get next face
				auto next = std::next(f, 1);
				if (next == seedFaces.end()) {
					next = seedFaces.begin();
				}
				// add all points from shortest path between two regions to border
				auto path = shortestPath(v, *f, *next);
				border.insert(border.end(), path.begin(), path.end());
			}
			// add the face
			const auto fh = m_ctrl.add_face(points);

			for (const auto &f : border) {
				for (auto vv = m_mesh.cfv_begin(f); vv != m_mesh.cfv_end(f); ++vv) {
					out.insert(*vv);
					m_mesh.set_color(*vv, { 0.f, 0.f, 0.f, 1.f });
				}
			}

			auto col = colgen.generateNextColor();
			m_mesh.set_color(v, { 1.f, 1.f, 1.f, 1.f });
			q.push(v);
			if (out.find(v) == out.end()) {
				in.insert(v);
			}
			// TODO: else look for another more fitting vertex to start at?

			// IDEA: pass start vertex and boundary faces, then start from vertex looking at neighbors
			// and if they are not part of a boundary face, add as inner vertex, else as outer vertex
			while (!q.empty()) {
				auto vert = q.front();
				q.pop();

				for (auto vh = m_mesh.cvv_begin(vert); vh != m_mesh.cvv_end(vert); ++vh) {
					if (out.find(*vh) == out.end() && in.find(*vh) == in.end()) {
						in.insert(*vh);
						vtt(*vh).face = fh;
						m_mesh.set_color(*vh, col);
						q.push(*vh);
					}
				}
			}

			ttv(fh).set(in, out);
			border.clear();
			in.clear(); out.clear();

			points.clear();
		}
		seedFaces.clear();
		check.clear();
	}

	/*for (const auto &f : m_mesh.faces()) {
		if (borders[f]) {
			m_mesh.set_color(f, { 1.f, 1.f, 1.f, 1.f });
		}
	}*/

	PluginFunctions::setDrawMode(ACG::SceneGraph::DrawModes::SOLID_POINTS_COLORED);

	return;

	//////////////////////////////////////////////////////////
	// parameterization (harmonic map)
	//////////////////////////////////////////////////////////

	//Parametrization param(m_mesh);

	//param.solve();

	//////////////////////////////////////////////////////////
	// fitting
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
	copyMesh(m_ctrl, m_mesh);

	m_mesh.garbage_collection();

	if (m_useColors) {
		PluginFunctions::setDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_FACES_COLORED |
			ACG::SceneGraph::DrawModes::HIDDENLINE
		);
	}
}

void VoronoiRemesh::copyMesh(BezierTMesh & src, BezierTMesh & dest)
{
	for (const auto &v : src.vertices()) {
		dest.add_vertex_dirty(src.point(v));
	}
	auto before = src.n_faces();
	for (const auto &f : src.faces()) {
		std::vector<VH> faces(src.fv_begin(f), src.fv_end(f));
		dest.add_face(faces);
	}
}

}