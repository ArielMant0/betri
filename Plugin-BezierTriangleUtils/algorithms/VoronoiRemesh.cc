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

	if (!m_mesh.get_property_handle(m_ttf, Props::TRITOFACE))
		m_mesh.add_property(m_ttf, Props::TRITOFACE);

	if (!m_mesh.get_property_handle(m_tribound, Props::TRIBOUND))
		m_mesh.add_property(m_tribound, Props::TRIBOUND);
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

	if (m_mesh.get_property_handle(m_ttf, Props::TRITOFACE))
		m_mesh.remove_property(m_ttf);

	if (m_mesh.get_property_handle(m_tribound, Props::TRIBOUND))
		m_mesh.remove_property(m_tribound);
}

void VoronoiRemesh::preventiveEdgeSplits(unsigned int size)
{
	std::vector<std::vector<std::pair<int, EH>>> counts;
	counts.reserve(size);
	for (int i = 0; i < size; ++i) {
		auto arr = std::vector<std::pair<int, EH>>();
		arr.reserve(size - i);
		for (int j = 0; j < size - i; ++j) {
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

				for (auto vf = m_mesh.vf_begin(v); vf != m_mesh.vf_end(v); ++vf) {
					auto face = *vf;
					if (face.idx() >= beforeFace) {
						P point = m_mesh.calc_face_centroid(face);
						for (auto ff = m_mesh.ff_begin(face); ff != m_mesh.ff_end(face); ++ff) {
							if (*ff == f1 || *ff == f2) {
								id(face) = id(*ff);
								dist(face) = dist(*ff);// +halfdist(*vf, *ff);
								pred(face) = *ff;
								break;
							}
						}
					}
				}
				for (auto vhe = m_mesh.voh_begin(v); vhe != m_mesh.voh_end(v); ++vhe) {
					if (id(m_mesh.face_handle(*vhe)) != id(m_mesh.opposite_face_handle(*vhe))) {
						crossed(*vhe) = 0;
					} else {
						crossed(*vhe) = 1;
					}
				}
			}
		}
	}

	//for (auto &f : m_mesh.faces()) {
	//	std::cerr << f << ": " << id(f) << ", " << dist(f) << ", " << pred(f) << "\n";
	//}
}

/**
 * Performs face-based dijkstra on the m_mesh (for the given seeds)
 */
void VoronoiRemesh::partition()
{
	ACG::HaltonColors cGenerator;
	if (m_useColors) {
		if (!m_mesh.has_face_colors()) {
			m_mesh.request_face_colors();
		}
		if (!m_mesh.has_edge_colors()) {
			m_mesh.request_edge_colors();
		}
	}

	std::set<FH> seeds;
	// special "priority-queue" for dijkstra
	std::set<QElem> q;

	// add a face to a region
	const auto grow = [&](FH &face, FH predFace = INVALID_F, double distance = 0.0) {
		id(face) = predFace == INVALID_F ? seeds.size()-1 : id(predFace);
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
		return seeds.find(face) != seeds.end();
	};
	const auto addSeedFace = [&](FH &face) {
		assert(!isSeedFace(face));
		seeds.insert(face);
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
					auto before = id(f);
					// unmark edges
					if (before != id(face)) {
						for (auto he = m_mesh.fh_begin(f); he != m_mesh.fh_end(f); ++he) {
							crossed(*he) = -1;
						}
					}
					crossed(*he) = id(face);
					auto v = m_mesh.to_vertex_handle(m_mesh.next_halfedge_handle(opp));
					// COND 1: if not homeomorphic to disk (new face is adj to boundary across
					// which lies a face of the same tile) add face as new seed
					if (!homeomorphicDisk(f, v, id(face))) {
						crossed(*he) = -1;
						addSeedFace(f);
						stop = true;
						break;
					}
					grow(f, face, update);
				}
			}
		}
		// COND 2: if more than one shared boundary per pair of tiles exists, add one of the
		// faces adj to the cut as a new seed face
		std::vector<std::vector<short>> cuts;
		cuts.reserve(seeds.size());
		for (int i = 0; i < seeds.size(); ++i) {
			cuts.push_back(std::vector<short>(seeds.size()-i, 0));
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
				int before = seeds.size();
				for (auto f = m_mesh.cvf_begin(v); f != m_mesh.cvf_end(v); ++f) {
					if (!isSeedFace(*f)) {
						addSeedFace(*f);
						break;
					}
				}
				assert(seeds.size() > before);
			}
		}

	} while (!q.empty());


	
	if (m_useColors) {
		BezierTMesh::Color boundaryColor(0.f, 0.f, 0.f, 1.f);
		for (auto edge : m_mesh.edges()) {
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
	}

	std::cerr << "used " << seeds.size() << '/' << m_mesh.n_faces();
	std::cerr << " faces as seeds" << std::endl;
}

void VoronoiRemesh::remesh(unsigned int size)
{
	if (size > m_mesh.n_faces()) {
		size = m_mesh.n_faces();
	}

	// source nodes
	std::vector<FH> seeds;
	seeds.reserve(size);

	partition();

	return; // EARLY DEBUG RETURN

#ifdef PRINT
	std::ofstream out("voronoi-log.txt", std::ios::out);
	out << "finished partition with " << seeds.size() << " seeds" << std::endl;
#endif


	// do preventive edge spliiting wherever there are two regions only touching at 1 edge
	preventiveEdgeSplits(seeds.size());

	// new m_mesh
	BezierTMesh nmesh;

	if (m_useColors && !nmesh.has_face_colors()) {
		nmesh.request_face_colors();
	}
	if (!m_mesh.get_property_handle(m_ftt, Props::FACETOTRI))
		m_mesh.add_property(m_ftt, Props::FACETOTRI);

	auto ntri = OpenMesh::makeTemporaryProperty<VH, short>(nmesh, Props::TRIBOUND);
	auto newverts = OpenMesh::makeTemporaryProperty<FH, VH>(m_mesh);

	// for each region: add a vertex to the new m_mesh (store in vector to find using region id)
	for (auto &f : m_mesh.faces()) {
		newverts[f] = INVALID_V;
	}
	for (auto &f : seeds) {
		auto v = nmesh.add_vertex(m_mesh.calc_face_centroid(f));
		ntri[v] = 1;
		newverts[f] = v;
	}

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

			he = findNextBorder(m_mesh.to_vertex_handle(he), id_1, m_mesh.edge_handle(he));
			tmp1 = m_mesh.face_handle(he);
			tmp2 = m_mesh.opposite_face_handle(he);
		} while (he != INVALID_H && he != begin && id(tmp1) == id_1 && id(tmp2) == id_2);

#ifdef PRINT
		if (sum == std::numeric_limits<double>::max() || start == INVALID_H) {
			out << "\tcould not find faces or starting edge !!! " << start << "\n";
		}
		else {
			out << "\tfound faces with shortest distance (" << sum << ") at border\n";
		}
#endif

		EH e1 = m_mesh.edge_handle(start), e2 = m_mesh.edge_handle(start), edge;

		std::deque<VH> path;
#ifdef PRINT
		out << "\tstarting path finding with edge " << e1 << " (f1: " << id(m_mesh.face_handle(start));
		out << ", f1: " << id(m_mesh.opposite_face_handle(start)) << ")\n";
#endif

		while (pred(f11) != INVALID_F) {
			if (newverts[f11] == INVALID_V) {
				newverts[f11] = nmesh.add_vertex(m_mesh.calc_face_centroid(f11));
				ntri[newverts[f11]] = 0;
			}
			path.push_front(newverts[f11]);
			f11 = pred(f11);
		}

#ifdef PRINT
		out << "\tadded path faces for region 1 (length: ";
		out << path.size() << ')' << std::endl;
#endif

		while (pred(f22) != INVALID_F) {
			if (newverts[f22] == INVALID_V) {
				newverts[f22] = nmesh.add_vertex(m_mesh.calc_face_centroid(f22));
				ntri[newverts[f22]] = 0;
			}
			path.push_back(newverts[f22]);
			f22 = pred(f22);
		}

#ifdef PRINT
		out << "\tadded path faces for region 2 (total length: ";
		out << path.size() << ')' << std::endl;
#endif

		return path;
	};

	//////////////////////////////////////////////////////////
	// create base m_mesh
	//////////////////////////////////////////////////////////
	std::unordered_set<FH> check;
	std::vector<FH> seedFaces; // need a vector to keep correct iterator order
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
				points.push_back(newverts[*f]);
				// mark as corner vertex of a delaunay triangle
				ntri[newverts[*f]] = 1;

				// add all points from shortest path between two regions
				auto path = shortestPath(v, *f, *next);
				points.insert(points.end(), path.begin(), path.end());
			}
#ifdef PRINT
			out << "\tadding face with " << points.size() << " vertices :\n\t\t";
			for (const auto &vh : points) {
				out << vh << "   ";
			}
			out << std::endl;
#endif
			auto before = nmesh.n_faces();
			const auto fh = nmesh.add_face(points);

			if (m_useColors) {
				if (fh.is_valid()) {
					nmesh.set_color(fh, m_mesh.color(*seedFaces.begin()));
				}
				else {
					auto col = m_mesh.color(*seedFaces.begin());
					for (size_t i = before; i < nmesh.n_faces(); ++i) {
						nmesh.set_color(m_mesh.face_handle(i), col);
					}
				}
			}
			points.clear();
		}
		seedFaces.clear();
		check.clear();
	}

	//////////////////////////////////////////////////////////
	// parameterization (harmonic map)
	//////////////////////////////////////////////////////////

	//Parametrization param(m_mesh);

	//param.solve();

	//////////////////////////////////////////////////////////
	// control net subdivision and position optimization
	//////////////////////////////////////////////////////////

	// loop subdivision (once)
	//Subdivision<BezierTMesh> loop(nmesh);
	//loop.loop(1);

	// control point optimization
	//copyMesh(nmesh, m_ctrl);
	// || ctrl verts x bernstein - surface points ||� = 0
	//LLS::solve(ctrl);

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
		//if (v.is_valid() &!nmesh.is_isolated(v))
		m_mesh.add_vertex_dirty(nmesh.point(v));
	}
	auto before = m_mesh.n_faces();
	for (const auto &f : nmesh.faces()) {
		std::vector<VH> verts(nmesh.fv_begin(f), nmesh.fv_end(f));
		const auto fh = m_mesh.add_face(verts);
		if (m_useColors) {
			auto col = nmesh.color(f);
			col[3] = 1.f;
			m_mesh.set_color(fh, col);
		}

	}

#ifdef PRINT
	out << "added faces to the new m_mesh" << std::endl;
	out.close();
#endif

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