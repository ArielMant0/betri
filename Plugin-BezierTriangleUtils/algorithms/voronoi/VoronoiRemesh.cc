#include "VoronoiRemesh.hh"

#include "../common/Parametrization.hh"
#include "../common/Fitting.hh"

#include <queue>
#include <unordered_set>
#include <map>
#include <random>
#include <fstream>

#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <OpenFlipper/BasePlugin/PluginFunctions.hh>
//#include <OpenFlipper/libs_required/ACG/GL/ColorTranslator.hh>

namespace betri
{

void VoronoiRemesh::prepare()
{
	// (temporary) property to easily access region from vertex
	if (!m_mesh.get_property_handle(m_region, Props::REGION))
		m_mesh.add_property(m_region, Props::REGION);

	if (!m_mesh.get_property_handle(m_vid, "vseedid"))
		m_mesh.add_property(m_vid, "vseedid");

	if (!m_mesh.get_property_handle(m_vborder, "vborder"))
		m_mesh.add_property(m_vborder, "vborder");

	if (!m_mesh.get_property_handle(m_pred, Props::PREDECESSOR))
		m_mesh.add_property(m_pred, Props::PREDECESSOR);

	if (!m_mesh.get_property_handle(m_vpred, "vseedpred"))
		m_mesh.add_property(m_vpred, "vseedpred");

	if (!m_mesh.get_property_handle(m_distance, Props::DISTANCE))
		m_mesh.add_property(m_distance, Props::DISTANCE);

	if (!m_mesh.get_property_handle(m_vdist, "vseeddist"))
		m_mesh.add_property(m_vdist, "vseeddist");

	if (!m_mesh.get_property_handle(m_crossed, Props::CROSSED))
		m_mesh.add_property(m_crossed, Props::CROSSED);

	if (!m_mesh.get_property_handle(m_vtt, Props::VERTEXTOTRI))
		m_mesh.add_property(m_vtt, Props::VERTEXTOTRI);

	if (!m_ctrl.get_property_handle(m_ttv, Props::TRITOVERTEX))
		m_ctrl.add_property(m_ttv, Props::TRITOVERTEX);


	ShortestPath::clear();

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

	if (m_mesh.get_property_handle(m_vid, "vseedid"))
		m_mesh.remove_property(m_vid);

	if (m_mesh.get_property_handle(m_vborder, "vborder"))
		m_mesh.remove_property(m_vborder);

	if (m_mesh.get_property_handle(m_pred, Props::PREDECESSOR))
		m_mesh.remove_property(m_pred);

	if (m_mesh.get_property_handle(m_vpred, "vseedpred"))
		m_mesh.remove_property(m_vpred);

	if (m_mesh.get_property_handle(m_distance, Props::DISTANCE))
		m_mesh.remove_property(m_distance);

	if (m_mesh.get_property_handle(m_vdist, "vseeddist"))
		m_mesh.remove_property(m_vdist);

	if (m_mesh.get_property_handle(m_crossed, Props::CROSSED))
		m_mesh.remove_property(m_crossed);

	if (m_mesh.get_property_handle(m_vtt, Props::VERTEXTOTRI))
		m_mesh.remove_property(m_vtt);

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
	std::cerr << __FUNCTION__ << " START\n";
	size_t num = m_mesh.n_faces();

	std::vector<std::vector<std::pair<int, EH>>> counts;
	counts.reserve(m_seeds.size());
	for (int i = 0; i < m_seeds.size(); ++i) {
		auto arr = std::vector<std::pair<int, EH>>();
		arr.reserve(m_seeds.size() - i);
		for (int j = 0; j < m_seeds.size() - i; ++j) {
			arr.push_back({ 0, EH() });
		}
		counts.push_back(arr);
	}

	for (EH edge : m_mesh.edges()) {
		if (isRegionBorderEdge(edge)) {
			HH h = m_mesh.halfedge_handle(edge, 0);
			FH f1 = m_mesh.face_handle(h), f2 = m_mesh.opposite_face_handle(h);
			const ID i1 = std::min(id(f1), id(f2));
			const ID i2 = std::abs(id(f2) - id(f1));
			// only store count once!
			counts[i1][i2].first++;
			counts[i1][i2].second = edge;
		}
	}

	for (int i = 0; i < counts.size(); ++i) {
		for (int j = 0; j < counts[i].size(); ++j) {
			if (i != j && counts[i][j].first == 1) {
				auto e = counts[i][j].second;

				assert(e.is_valid());

				HH h1 = m_mesh.halfedge_handle(e, 0);
				HH h2 = m_mesh.halfedge_handle(e, 1);

				FH f1 = m_mesh.face_handle(h1);
				FH f2 = m_mesh.face_handle(h2);

				m_mesh.set_color(f1, { 1., 1., 1., 1. });
				m_mesh.set_color(f2, { 1., 1., 1., 1. });

				std::cerr << "splitting faces " << f1 << " and " << f2 << '\n';
				fixCrossing(f1, f2);

				FH f3 = m_mesh.face_handle(m_mesh.n_faces() - 2);
				FH f4 = m_mesh.face_handle(m_mesh.n_faces() - 1);
				m_mesh.set_color(f3, { 0., 0., 0., 1. });
				m_mesh.set_color(f4, { 0., 0., 0., 1. });
			}
		}
	}
	std::cerr << __FUNCTION__ << ": created " << m_mesh.n_faces()-num << "  new faces\n";
}

void VoronoiRemesh::assignInnerVertices()
{
	int num = 1;
	std::deque<VH> q;
	std::unordered_set<VH> inner;

	auto assigned = OpenMesh::makeTemporaryProperty<FH, bool>(m_ctrl, "visited");

	using Reg = std::pair<ID, size_t>;
	std::deque<Reg> regions;

	std::cerr << "assigning inner vertices\n";

	const auto getRegion = [&](const ID id) {
		return std::find_if(regions.begin(), regions.end(), [&](const Reg &r) {
			return r.first == id;
		});
	};

	while (num > 0) {
		num = 0;
		regions.clear();

		for (VH vh : m_mesh.vertices()) {
			if (!vtt(vh).isBorder() && !vtt(vh).face.is_valid()) {
				num++;

				q.push_back(vh);
				inner.insert(vh);

				std::cerr << "found free vertex\n";
				// find all vertices of this patch not assigned yet
				while(!q.empty()) {
					VH front = q.front();
					q.pop_front();


					for (auto v_it = m_mesh.cvv_begin(front); v_it != m_mesh.cvv_end(front); ++v_it) {
						auto help = vtt(*v_it);

						if (!help.isBorder() && inner.find(*v_it) == inner.end()) {
							inner.insert(*v_it);
							q.push_back(*v_it);
						} else if (help.isBorder() && !isSeedVertex(*v_it)) {
							auto r = getRegion(help.id1);
							if (r != regions.end()) {
								r->second++;
							} else {
								std::cerr << "\t\tadding region" << help.id1 << "\n";
								regions.push_back({ help.id1, 1 });
							}

							r = getRegion(help.id2);
							if (r != regions.end()) {
								r->second++;
							} else {
								std::cerr << "\t\tadding region" << help.id2 << "\n";
								regions.push_back({ help.id2, 1 });
							}
						}
					}
				}

				if (regions.size() < 3) break;

				std::sort(regions.begin(), regions.end(), [&](const Reg lhs, const Reg rhs) {
					return lhs.second > rhs.second;
				});

				FH ctrlFace;

				const auto comb = [&]()
				{
					std::string bitmask(3, 1); // K leading 1's
					bitmask.resize(regions.size(), 0); // N-K trailing 0's

					std::array<ID,3> test;
					// test and permute bitmask
					do {
						for (size_t i = 0, j = 0; i < regions.size(); ++i) // [0..N-1] integers
						{
							if (bitmask[i]) {
								test[j++] = regions[i].first;
							}
						}
						ctrlFace = findDelaunayFace(test[0], test[1], test[2]);
					} while ((!ctrlFace.is_valid() || assigned[ctrlFace]) &&
						std::prev_permutation(bitmask.begin(), bitmask.end()));

					std::cerr << "\tassigning to face " << ctrlFace << " for regions ";
					std::cerr << test[0] << " + " << test[1] << " + " << test[2] << "\n";

					return ctrlFace;
				};

				comb();
				assert(ctrlFace.is_valid());

				assigned[ctrlFace] = true;
				auto color = m_colGen.generateNextColor();
				// mark vertices
				for (VH v : inner) {
					vtt(v).setFace(ctrlFace);
					m_mesh.set_color(v, color);
				}
				// connect to face
				ttv(ctrlFace).inner.insert(
					ttv(ctrlFace).inner.begin(),
					inner.begin(),
					inner.end()
				);

				inner.clear();
				regions.clear();
			}
		}

	}
}

void VoronoiRemesh::splitClosedPaths()
{
	std::cerr << __FUNCTION__ << " START\n";

	size_t nedges = m_mesh.n_edges();

	for (size_t i = 0; i < nedges; ++i) {
		EH edge = m_mesh.edge_handle(i);

		HH he = m_mesh.halfedge_handle(edge, 0);

		const FH f1 = m_mesh.face_handle(he), f2 = m_mesh.opposite_face_handle(he);
		assert(m_mesh.adjToFace(f1, f2));

		if (!isCrossed(edge) && !isSeed(f1) && !isSeed(f2)) {
			VH to = m_mesh.to_vertex_handle(he);
			VH from = m_mesh.from_vertex_handle(he);

			// if both adj vertices are border vertices, split this edge
			if (vtt(from).isBorder() && vtt(to).isBorder()) {
				std::cerr << "splitting faces " << f1 << " and " << f2 << '\n';
				fixCrossing(f1, f2);

				/*FH f3 = m_mesh.face_handle(m_mesh.n_faces() - 2);
				FH f4 = m_mesh.face_handle(m_mesh.n_faces() - 1);
				m_mesh.set_color(f3, { 1.f, 0.5f, 0.f, 1.f });
				m_mesh.set_color(f4, { 1.f, 0.5f, 0.f, 1.f });*/
			}
		}
	}
	std::cerr << __FUNCTION__ << ": added " << m_mesh.n_edges() - nedges << " new edges\n";
}

void VoronoiRemesh::fixPredecessor(const FH fh, const bool rewrite)
{
	std::cerr << "fixing predecessor for " << fh << "(" << pred(fh) << ")\n";
	// nothing do to
	if (isSeed(fh) || !pred(fh).is_valid()) return;

	const auto calcDist = [&](FH from, FH to) {
		P p1 = m_mesh.calc_face_centroid(from);
		P p2 = m_mesh.calc_face_centroid(to);
		return dist(from) + (p1 - p2).norm();
	};

	FH target = pred(fh);
	if (!target.is_valid()) {
		std::cerr << "\tid " << id(fh) << "\n";
		target = m_seeds[id(fh)];
	}

	if (m_mesh.adjToFace(fh, target)) {
		std::cerr << "\tface already adj to predecessor\n";
		dist(fh) = calcDist(target, fh);
	} else {
		std::cerr << "\tcalculating new predecessor\n";

		// look at all neighbor faces and see which one fits
		for (auto h_it = m_mesh.cfh_begin(fh); h_it != m_mesh.cfh_end(fh); ++h_it) {
			const FH f = m_mesh.opposite_face_handle(*h_it);
			// must always belong to same region
			// must not be my own predecessor
			// must be adj to target face
			if (id(fh) != id(f) && isCrossed(*h_it) && pred(f) == fh) continue;

			if (rewrite && m_mesh.adjToFace(f, target)) {
				std::cerr << "\t\trewrite face " << f << '\n';
				// update new predecessor first
				dist(f) = calcDist(target, f);
				pred(f) = target;
				// update this face
				dist(fh) = calcDist(f, fh);
				pred(fh) = f;
				// tag as processed
				m_mesh.status(f).set_tagged(true);
				break;
			} else if (pred(f) == target) {
				std::cerr << "\t\tupdate " << f << '\n';
				// update this face
				dist(fh) = calcDist(f, fh);
				pred(fh) = f;
				break;
			}
		}

	}

	m_mesh.status(fh).set_tagged(true);
	std::cerr << "\t\tpred is now " << pred(fh) << "\n";

	// make sure everything is in order
	assert(m_mesh.adjToFace(fh, pred(fh)));
	assert(fh != pred(pred(fh)));
}

HalfedgeHandle VoronoiRemesh::fixCrossing(
	const FH f0,
	const FH f1,
	const VH prevNode,
	const VH notNode
) {
	std::deque<FH> fixOverwrite, fixOther;
	for (auto f_it = m_mesh.cff_begin(f0); f_it != m_mesh.cff_end(f0); ++f_it) {
		if (*f_it != f1 && pred(*f_it) == f0) fixOther.push_back(*f_it);
	}
	for (auto f_it = m_mesh.cff_begin(f1); f_it != m_mesh.cff_end(f1); ++f_it) {
		if (*f_it != f0 && pred(*f_it) == f1) fixOther.push_back(*f_it);
	}

	size_t nedge = m_mesh.n_edges();
	EH commonEdge;
	for (auto f_it = m_mesh.cfh_begin(f0); f_it != m_mesh.cfh_end(f0); ++f_it) {
		if (m_mesh.opposite_face_handle(*f_it) == f1) {
			commonEdge = m_mesh.edge_handle(*f_it); break;
		}
	}

	VH newNode = m_mesh.splitFacesRivara(f0, f1, true);

	Scalar minDist = std::numeric_limits<Scalar>::max();
	VH newPred;
	// choose id/distance/predecessor for the new vertex (which neighbor is closest)
	for (auto vh = m_mesh.cvoh_begin(newNode); vh != m_mesh.cvoh_end(newNode); ++vh) {
		Scalar len = m_mesh.calc_edge_length(*vh);
		if (len < minDist) {
			newPred = m_mesh.to_vertex_handle(*vh);
			minDist = len;
		}
	}
	assert(newPred.is_valid());
	dist(newNode) = minDist;
	pred(newNode) = newPred;
	id(newNode) = id(newPred);

	// TODO: is this the correct edge
	m_mesh.copy_all_properties(commonEdge, m_mesh.edge_handle(nedge), false);
	m_mesh.set_color(m_mesh.edge_handle(nedge), m_mesh.color(commonEdge));
	for (nedge+=1; nedge < m_mesh.n_edges(); ++nedge) {
		crossed(m_mesh.edge_handle(nedge)) = -1;
	}

	FH f2 = m_mesh.face_handle(m_mesh.n_faces() - 2);
	FH f3 = m_mesh.face_handle(m_mesh.n_faces() - 1);

	fixOverwrite.push_back(f0);
	fixOverwrite.push_back(f1);

	fixOther.push_front(f3);
	fixOther.push_front(f2);

	fixAllFaces(fixOverwrite, fixOther);

	if (prevNode.is_valid() && notNode.is_valid()) {
		HH way = m_mesh.find_halfedge(newNode, prevNode);
		assert(way.is_valid());
		if (!m_mesh.adjToVertex(m_mesh.face_handle(way), notNode)) {
			way = m_mesh.opposite_halfedge_handle(way);
		}
		assert(m_mesh.adjToVertex(m_mesh.face_handle(way), notNode));
		return way;
	}

	return HH();
}

void VoronoiRemesh::shortestPath(
	const VH v,
	const ID id_1,
	const ID id_2,
	const FH ctrlFace,
	const ShortestPath &path
) {

	std::cerr << "\tcalculating shortest path for regions " << id_1 << " (" << m_seeds[id_1];
	std::cerr << ") and " << id_2 << " (" << m_seeds[id_2] << ")\n";

	std::unordered_set<VH> visited;

	const auto borderOf = [&](const VH vh) {
		bool r1 = false, r2 = false;
		for (auto vv = m_mesh.cvv_begin(vh); vv != m_mesh.cvv_end(vh); ++vv) {
			if (id(*vv) == id_1) r1 = true;
			else if (id(*vv) == id_2) r2 = true;
		}
		return r1 && r2;
	};

	// find next edge of a region border
	// (always return the halfedge that is adjacent to face with id f1)
	const auto findNextVertex = [&](const VH from) {
		VH next;
		Scalar minDist = std::numeric_limits<Scalar>::max();

		for (auto h = m_mesh.voh_begin(from); h != m_mesh.voh_end(from); ++h) {
			//const ID id1 = id(m_mesh.face_handle(*h));
			//const ID id2 = id(m_mesh.opposite_face_handle(*h));
			const VH vh = m_mesh.to_vertex_handle(*h);
			// if edge was not crossed and is adjacent to the given region
			if (isBorder(vh) && dist(vh) < minDist &&
				//(id1 == id_1 && id2 == id_2 || id1 == id_2 && id2 == id_1) &&
				visited.find(vh) == visited.end() && borderOf(vh)
			) {
				minDist = dist(vh);
				next = vh;
			}
		}
		return next;
	};

	const auto minConnection = [&](const VH vh) {
		VH partner;
		const ID other = id(vh) == id_1 ? id_2 : id_1;
		Scalar minDist = std::numeric_limits<Scalar>::max();

		std::cerr << "\tmin connect: looking for id " << other << '\n';
		for (auto he = m_mesh.cvoh_begin(vh); he != m_mesh.cvoh_end(vh); ++he) {
			const VH vv = m_mesh.to_vertex_handle(*he);
			std::cerr << "\t\t" << id(vv) << ", " << dist(vv) << ", " << vtt(vv).isBorder() << "\n";
			if (id(vv) == other && dist(vv) < minDist && !vtt(vv).isBorder()) {
				minDist = dist(vv);
				partner = vv;
			}
		}
		return partner;
	};

	VH current = v, start1;

	double minDist = std::numeric_limits<Scalar>::max();

	// find adj faces with shortest path to their respective seed points
	while (current.is_valid()) {

		const Scalar best = dist(current);
		if (best <= minDist) {
			minDist = best;
			start1 = current;
			std::cerr << "\tfound better start vertex " << start1 << ", dist " << best << "\n";
		}

		visited.insert(current);
		current = findNextVertex(current);
		// two regions should not have a circular boundary
		assert(current != v);
	}

	assert(start1.is_valid());
	VH start2 = minConnection(start1);
	if (!start2.is_valid()) {
		m_mesh.set_color(start1, { 1.f, 0.5f, 1.f, 1.f });
		m_debugCancel = true;
		return;
	}

	std::deque<VH> subpath;

	size_t middle = 0;
	// make a queue containing all vertices of the path in correct order
	VH working = start1;
	while (working.is_valid()) {
		subpath.push_front(working);
		std::cerr << "\t\tvertex " << working << " has pred " << pred(working) << "\n";
		working = pred(working);
		middle++;
	}
	assert(isSeedVertex(subpath.front()));

	working = start2;
	while (working.is_valid()) {
		subpath.push_back(working);
		std::cerr << "\t\tvertex " << working << " has pred " << pred(working) << "\n";
		working = pred(working);
	}
	assert(isSeedVertex(subpath.back()));
	assert(subpath.size() >= 2);

	const Color black = { 0.f, 0.f, 0.f, 1.f };

	for (size_t i=0; i < subpath.size(); ++i) {
		const VH node = subpath[i];

		if (i < subpath.size() - 1) {
			if (i > 0) assert(!vtt(node).isBorder());

			const HH he = m_mesh.find_halfedge(node, subpath[i + 1]);
			assert(he.is_valid());
			const EH edge = m_mesh.edge_handle(he);
			assert(!isCrossed(edge));
			crossed(edge) = ctrlFace.idx();
			m_mesh.set_color(edge, black);
		}
		// set color and mark this as a boundary vertex
		path.push(node);
		m_mesh.set_color(node, black);
		vtt(node).setBorder(id_1, id_2);
	}
}

 //////////////////////////////////////////////////////////
 // create voronoi partition
 //////////////////////////////////////////////////////////
void VoronoiRemesh::partition()
{
	// special "priority-queue" for dijkstra
	FaceDijkstra q;

	const double INF = std::numeric_limits<double>::max();
	// initialize face properties
	for (auto &face : m_mesh.faces()) {
		pred(face) = FH();
		id(face) = -1;
		dist(face) = INF;
		q.insert({ INF, face });
	}

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<size_t> dis(0u, m_mesh.n_faces()-1);
	addSeed(q, m_mesh.face_handle(dis(gen)));

	auto marked = OpenMesh::makeTemporaryProperty<EH, short>(m_mesh);

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
				FH f = m_mesh.opposite_face_handle(*he);
				EH edge = m_mesh.edge_handle(*he);

				const P p2 = m_mesh.calc_face_centroid(f);
				// distance to the next face
				const double update = dist(face) + (p1 - p2).norm();
				// update neighbor face distance if the value can be improved
				if (update < dist(f)) {
					auto opp = m_mesh.opposite_halfedge_handle(*he);
					auto next = m_mesh.next_halfedge_handle(opp);
					auto v = m_mesh.to_vertex_handle(next);
					// COND 1: if not homeomorphic to disk (new face is adj to boundary across
					// which lies a face of the same tile) add face as new seed
					if (!homeomorphicDisk(f, v, id(face))) {
						addSeed(q, f);
						stop = true;
						break;
					}
					grow(q, f, face, update);
				}
			}
		}

		// COND 2: if more than one shared boundary per pair of tiles exists, add one of the
		// faces adj to the cut as a new seed face
		reduceCuts(q);

		// COND 3: if one vertex is adj to more than 3 regions, add one adj face as a new seed face
		reduceAdjRegions(q);

	} while (!q.empty());

	if (m_useColors) {
		BezierTMesh::Color boundaryColor(0.f, 0.f, 0.f, 1.f);
		for (auto &edge : m_mesh.edges()) {
			const auto he = m_mesh.halfedge_handle(edge, 0);
			const auto f1 = m_mesh.face_handle(he);
			const auto f2 = m_mesh.opposite_face_handle(he);
			if (id(f1) == id(f2)) {
				m_mesh.set_color(edge, m_colors[id(f1)]);
			} else {
				m_mesh.set_color(edge, boundaryColor);
			}
		}
	}

	// do preventive edge splitting wherever there are two regions only touching at 1 edge
	preventiveEdgeSplits();

	// mark all edges as not crossed
	for (auto e : m_mesh.edges()) {
		crossed(e) = -1;
	}

	m_nvertices = m_mesh.n_vertices();
	m_nedges = m_mesh.n_edges();

	std::cerr << "used " << m_seeds.size() << '/' << m_mesh.n_faces();
	std::cerr << " faces as seeds" << std::endl;

	m_ctrlVerts.resize(m_seeds.size());
	// add vertices of seed faces
	for (FH face : m_seeds) {
		m_ctrlVerts[id(face)] = m_ctrl.add_vertex(m_mesh.calc_face_centroid(face));
	}

	// calculate shortest distances for all vertices
	vertexDijkstra();
}

void VoronoiRemesh::vertexDijkstra(const ID id0, const ID id1)
{
	VertexDijkstra q;

	const Scalar INF = std::numeric_limits<Scalar>::max();

	if (id0 >= 0 && id1 >= 0) {
		const VH start0 = m_seedVerts[id0], start1 = m_seedVerts[id1];
		// called after having found a path (avoid crossing paths)
		for (VH vh : m_mesh.vertices()) {
			if (vh != start0 && vh != start1 && (id(vh) == id0 || id(vh) == id1)) {
				dist(vh) = INF;
				pred(vh) = vtt(vh).isBorder() ? pred(vh) : VH();
			}
		}
		Point p = m_mesh.point(start0);
		// add all neighbors of the start vertex to the queue (if they are not on a border)
		for (auto vv = m_mesh.cvv_begin(start0); vv != m_mesh.cvv_end(start0); ++vv) {
			if (id(*vv) == id0 && !vtt(*vv).isBorder()) {
				dist(*vv) = (p - m_mesh.point(*vv)).norm();
				pred(*vv) = start0;
				q.insert({ dist(*vv), *vv });
			}
		}
		p = m_mesh.point(start1);
		// add all neighbors of the start vertex to the queue (if they are not on a border)
		for (auto vv = m_mesh.cvv_begin(start1); vv != m_mesh.cvv_end(start1); ++vv) {
			if (id(*vv) == id1 && !vtt(*vv).isBorder()) {
				dist(*vv) = (p - m_mesh.point(*vv)).norm();
				pred(*vv) = start1;
				q.insert({ dist(*vv), *vv });
			}
		}
	} else {
		m_seedVerts.clear();
		m_seedVerts.reserve(m_seeds.size());
		// first time calling this
		for (VH vh : m_mesh.vertices()) {
			id(vh) = -1;
			dist(vh) = INF;
			pred(vh) = VH();
		}

		for (FH face : m_seeds) {
			const VH vh = *m_mesh.cfv_begin(face);
			id(vh) = m_seedVerts.size();
			dist(vh) = 0.0;
			m_seedVerts.push_back(vh);
			q.insert({ 0.0, vh });
		}
	}

	const auto adjToRegionFace = [&](const VH vh, const ID id0) {
		for (auto f = m_mesh.cvf_begin(vh); f != m_mesh.cvf_end(vh); ++f) {
			if (id(*f) == id0) return true;
		}
		return false;
	};

	assert(!q.empty());

	do {

		const VH vh = q.begin()->second;
		q.erase(q.begin());

		Point p0 = m_mesh.point(vh);

		auto end = m_mesh.cvoh_end(vh);
		for (auto he = m_mesh.cvoh_begin(vh); he != end; ++he) {
			const VH vv = m_mesh.to_vertex_handle(*he);

			Scalar updateDist = dist(vh) + (p0 - m_mesh.point(vv)).norm();
			// vertex must ...
			// - not be a border vertex
			// - have a larger distance than the new one we calculatd
			// - be adj to at least 1 face with the same id
			if (!vtt(vv).isBorder() && updateDist < dist(vv) && adjToRegionFace(vv, id(vh))) {
				assert(!isCrossed(*he)); // edge must not be crossed, debug
				dist(vv) = updateDist;
				id(vv) = id(vh);
				pred(vv) = vh;
				q.insert({ updateDist, vv });
			}
		}

	} while (!q.empty());

	std::set<ID> adj;
	for (VH vh : m_mesh.vertices()) {
		assert(id(vh) >= 0);
		for (auto vf = m_mesh.cvf_begin(vh); vf != m_mesh.cvf_end(vh); ++vf) {
			adj.insert(id(*vf));
		}
		setBorder(vh, adj.size() > 1);
		if (!vtt(vh).isBorder()) m_mesh.set_color(vh, m_colors[id(vh)]);
		adj.clear();
	}
}

void VoronoiRemesh::prepareFromBaseMesh()
{
	assert(m_useBaseMesh);

	m_colors.reserve(m_mesh.n_faces());
	m_colGen.generateNextNColors(m_mesh.n_faces(), std::back_inserter(m_colors));

	copyMesh(m_mesh, m_ctrl);

	for (FH face : m_mesh.faces()) {
		id(face) = face.idx();
		dist(face) = 0.0;
		pred(face) = FH();

		m_mesh.set_color(face, m_colors[face.idx()]);
	}

	for (EH edge : m_mesh.edges()) {
		crossed(edge) = -1;
	}
}

//////////////////////////////////////////////////////////
// create base mesh
//////////////////////////////////////////////////////////
bool VoronoiRemesh::dualize(bool steps)
{
	std::unordered_set<ID> check;
	std::vector<ID> seedIDs; // need a vector to keep correct iterator order
	std::vector<VH> points;

	VH v;

	bool found = false;

	// for each vertex in the m_mesh
	for (m_vertexIdx; m_vertexIdx < m_mesh.n_vertices(); ++m_vertexIdx) {
		v = m_mesh.vertex_handle(m_vertexIdx);

		std::cerr << "checking vertex " << v << "\n";

		// check how many different regions are around that vertex
		for (auto vf = m_mesh.vf_begin(v); vf != m_mesh.vf_end(v); ++vf) {
			if (check.insert(id(*vf)).second) {
				assert(id(*vf) >= 0 && "no id for face next to vertex");
				seedIDs.push_back(id(*vf));
				points.push_back(m_ctrlVerts[id(*vf)]);
			}
		}

		// only do sth if we found 3 regions
		if (seedIDs.size() > 2) {
			// pick up here again in the next step
			if (found && steps) break;

			std::cerr << "\tVoronoi Vertex\n";

			assert(seedIDs.size() == 3);

			// add the face
			const auto fh = m_ctrl.add_face(points, true);

			for (size_t j = 0; j < seedIDs.size(); ++j) {
				const ID id1 = seedIDs[j];
				const ID id2 = seedIDs[j < 2 ? j + 1 : 0];
				// tell face which regions it was generated from (to later find boundary)
				ttv(fh)[j] = id1;

				ShortestPath bp;
				// collect border halfedges between these two regions
				if (!ShortestPath::has(id1, id2)) {
					bp = ShortestPath(id1, id2);
					shortestPath(v, id1, id2, fh, bp);
					// add path to path collection
					ShortestPath::path(bp);

					if (m_debugCancel) return false;

					// make sure there are no impassable edges
					// (i.e. edge connected to 2 vertices adj to border edges)
					splitClosedPaths();

					if (m_debugCancel) return false;

					// recalculate shortest paths for these two regions
					vertexDijkstra(id1, id2);

					if (m_debugCancel) return false;
				}
			}

			found = true;
		}
		points.clear();
		seedIDs.clear();
		check.clear();
	}

	// if we finished, assign interior vertices
	if (m_vertexIdx == m_mesh.n_vertices()) {
		// TODO: make sure this doesnt happen in partition
		if (m_ctrl.n_faces() < 3) return true;

		// perform assignment
		assignInnerVertices();
	}

	return m_vertexIdx == m_mesh.n_vertices();
}

void VoronoiRemesh::makeShortestPaths()
{
	assert(m_useBaseMesh);

	// create a path for each edge, ids are given by the vertices
	for (EH edge : m_mesh.edges()) {
		HH he = m_mesh.halfedge_handle(edge, 0);
		VH v0 = m_mesh.from_vertex_handle(he), v1 = m_mesh.to_vertex_handle(he);

		ShortestPath path(v0.idx(), v1.idx());
		path.push(v0); path.push(v1);

		if (!vtt(v0).isBorder()) {
			vtt(v0).setBorder(path.first(), path.second());
		}
		if (!vtt(v1).isBorder()) {
			vtt(v1).setBorder(path.first(), path.second());
		}
		ShortestPath::path(path);
	}

	// split faces so we end up with 1 extra vertex per face (and 3 extra faces)
	for (FH face : m_ctrl.faces()) {
		size_t i = 0;
		for (auto vb = m_mesh.cfv_begin(face), ve = m_mesh.cfv_end(face); vb != ve; ++vb) {
			ttv(face)[i++] = vb->idx();
		}

		VH vh = m_mesh.splitFaceBarycentric(face, true);
		vtt(vh).setFace(m_ctrl.face_handle(face.idx()));
		ttv(face).inner.push_back(vh);

		/*size_t end = m_mesh.n_faces();
		for (size_t i = end - 3; i < end; ++i) {
			ttv(face).inner.push_back(m_mesh.splitFaceBarycentric(m_mesh.face_handle(i), true));
		}*/
	}
}

//////////////////////////////////////////////////////////
// parameterization (harmonic map) + surface fitting
//////////////////////////////////////////////////////////
void VoronoiRemesh::fitting()
{
	Parametrization param(m_mesh, m_ctrl, m_vtt, m_ttv, m_pred);
	Fitting fit(m_mesh, m_ctrl, m_ttv, m_vtt);

	param.calcWeights();
	fit.degree(m_mesh.degree());

	for (FH face : m_ctrl.faces()) {
		if (!param.solveLocal(face)) {
			std::cerr << "parametrization for face " << face << " failed\n";
			m_debugCancel = true;
			return;
		}
		if (!fit.solveLocal(face)) {
			std::cerr << "fitting for face " << face << " failed\n";
			m_debugCancel = true;
			return;
		}
	}

}

void VoronoiRemesh::remesh()
{
	if (!m_useBaseMesh) partition();
	else prepareFromBaseMesh();

	// stop if cancel was requested
	if (m_debugCancel) return;

	if (!m_useBaseMesh) dualize();
	else makeShortestPaths();

	// stop if cancel was requested
	if (m_debugCancel) return;

	fitting();

	// replace original mesh
	if (m_copy) copyMesh(m_ctrl, m_mesh);

	std::cerr << "----------- DONE -----------" << std::endl;
}

void VoronoiRemesh::resetPath(FaceDijkstra &q, const FH face)
{
	const FH prevPred = pred(face);
	pred(face) = FH();
	dist(face) = std::numeric_limits<double>::max();

	for (auto fh = m_mesh.cfh_begin(face); fh != m_mesh.cfh_end(face); ++fh) {
		const FH f = m_mesh.opposite_face_handle(*fh);
		if (pred(f) == face) {

		} else if (f != prevPred) {
			q.insert({ dist(f), f });
		}
	}
}

void VoronoiRemesh::repartition(const ID id1)
{
	// special "priority-queue" for dijkstra
	FaceDijkstra q;

	size_t before = m_seeds.size();

	std::cerr << __FUNCTION__ << "(" << id1 << ")\n";

	FH f1 = m_seeds[id1];

	const double INF = std::numeric_limits<double>::max();

	const auto addAdj = [&](const VH vh) {
		for (auto f_it = m_mesh.cvf_begin(vh); f_it != m_mesh.cvf_end(vh); ++f_it) {
			if (id(*f_it) == id(f1) && *f_it != f1) {
				grow(q, *f_it, pred(*f_it), dist(*f_it));
			}
		}
	};

	for (auto v_it = m_mesh.cfv_begin(f1); v_it != m_mesh.cfv_end(f1); ++v_it) {
		addAdj(*v_it);
	}

	size_t count = 0;
	for (FH face : m_mesh.faces()) {
		if (id(face) == id1 && !isSeed(face) && q.find({ dist(face), face }) == q.end()
			//!isSeed(face) && isCrossed(commonEdge(face, pred(face)))
		) {
			/*for (auto ff = m_mesh.cff_begin(face); ff != m_mesh.cff_end(face); ++ff) {
				if (*ff != pred(face)) {
					if (pred(*ff) == face) {
					}
					q.insert({ dist(*ff), *ff });
				}
			}*/
			id(face) = -1;
			pred(face) = FH();
			dist(face) = INF;
			m_mesh.set_color(face, { 0.0, 0.0, 0.0, 1.0 });
			count++;
		}
	}
	std::cerr << "\treset " << count << " faces\n";

	assert(q.size() > 0);

	count = 0;

	do {
		count++;
		size_t beforeSize = q.size()-1, beforeSeeds = m_seeds.size();

		while (!q.empty()) {

			auto it = q.begin();
			FH face = (*it).second;
			q.erase(it);

			P p1 = m_mesh.calc_face_centroid(face);

			bool stop = false;
			// iterate over all incident halfedges of this face
			for (auto he = m_mesh.fh_begin(face); he != m_mesh.fh_end(face) && !stop; ++he) {
				FH f = m_mesh.opposite_face_handle(*he);
				EH edge = m_mesh.edge_handle(*he);

				// dont consider paths where we cannot to cross a boundary edge
				if (isCrossed(edge)) {
					//std::cerr << "\tavoided crossed edge\n";
					continue;
				}

				const P p2 = m_mesh.calc_face_centroid(f);
				// distance to the next face
				const double updateDist = dist(face) + (p1 - p2).norm();
				// update neighbor face distance if the value can be improved
				if (updateDist < dist(f)) {
					//std::cerr << "\tupdating dist for face " << f;
					//std::cerr << '(' << dist(f) << ',' << updateDist << ")\n";

					auto opp = m_mesh.opposite_halfedge_handle(*he);
					auto next = m_mesh.next_halfedge_handle(opp);
					auto v = m_mesh.to_vertex_handle(next);

					// COND 1: if not homeomorphic to disk (new face is adj to boundary across
					// which lies a face of the same tile) add face as new seed
					if (!homeomorphicDisk(f, v, id(face))) {
						addSeed(q, f);
						stop = true;
						std::cerr << "\tadded 1 (c1) seed\n";

						break;
					}
					grow(q, f, face, updateDist);
				}
			}
		}

		//// COND 2: if more than one shared boundary per pair of tiles exists, add one of the
		//// faces adj to the cut as a new seed face
		//reduceCuts(q);
		//std::cerr << "\tq size after c2 " << q.size() << " faces\n";

		//// COND 3: if one vertex is adj to more than 3 regions, add one adj face as a new seed face
		//reduceAdjRegions(q);
		//std::cerr << "\tq size after c3 " << q.size() << " faces\n";

		if (m_seeds.size() > beforeSeeds) {
			std::cerr << "\tadded " << (m_seeds.size() - beforeSeeds) << " seeds\n";
		}

		for (size_t i = beforeSeeds; i < m_seeds.size(); ++i) {
			m_ctrlVerts.push_back(m_ctrl.add_vertex(m_mesh.calc_face_centroid(m_seeds[i])));
		}

	} while (!q.empty());

	std::cerr << "\tused " << count << " iteration(s) for repartition\n";

	for (FH f : m_mesh.faces()) {
		if (id(f) < 0 || (pred(f).is_valid() && f == pred(pred(f)) && !isSeed(f))) {
			std::cerr << "\tERROR 1: face " << f << ", id " << id(f) << ", pred " << pred(f) << "\n";
		}
		assert((isSeed(f) && !pred(f).is_valid()) || m_mesh.adjToFace(f, pred(f)));
	}

	// make sure regions touch at more than 1 edge
	preventiveEdgeSplits();

	std::cerr << __FUNCTION__ << " DONE!\n";
}

void VoronoiRemesh::reduceCuts(FaceDijkstra &q)
{
	std::vector<std::vector<short>> cuts;
	cuts.reserve(m_seeds.size());
	for (int i = 0; i < m_seeds.size(); ++i) {
		cuts.push_back(std::vector<short>(m_seeds.size() - i, 0));
	}

	const auto marked = [&](const EH eh) {
		return m_mesh.status(eh).tagged();
	};

	const auto mark = [&](const EH eh, const bool value) {
		return m_mesh.status(eh).set_tagged(value);
	};

	const auto nextCutEdge = [&](const HH &h, ID id1, ID id2) {
		VH v = m_mesh.to_vertex_handle(h);
		for (auto voh = m_mesh.voh_begin(v); voh != m_mesh.voh_end(v); ++voh) {
			auto e = m_mesh.edge_handle(*voh);
			auto id11 = id(m_mesh.face_handle(*voh));
			auto id22 = id(m_mesh.opposite_face_handle(*voh));
			if (!marked(e) && id11 == id1 && id22 == id2) {
				return *voh;
			}
		}
		return HH();
	};

	for (EH edge : m_mesh.edges()) {
		HH forward = m_mesh.halfedge_handle(edge, 0);
		HH backward = m_mesh.halfedge_handle(edge, 1);

		FH face = m_mesh.face_handle(forward);
		ID id1 = id(face);
		ID id2 = id(m_mesh.face_handle(backward));

		if (marked(edge)) {
			// check id
			if (id1 >= cuts.size() || id2 >= cuts.size() ||
				id1 == id2 || id1 == -1 || id2 == -1) continue;

			while (forward.is_valid() || backward.is_valid()) {
				if (forward.is_valid()) {
					if (isSeed(face)) {
						face = m_mesh.face_handle(forward);
					}
					mark(m_mesh.edge_handle(forward), true);
					forward = nextCutEdge(forward, id1, id2);
				}
				if (backward.is_valid()) {
					if (isSeed(face)) {
						face = m_mesh.face_handle(backward);
					}
					mark(m_mesh.edge_handle(backward), true);
					backward = nextCutEdge(backward, id2, id1);
				}
			};
			// deal with cut if necessary
			if (id1 < id2) {
				cuts[id1][id2 - id1]++;
				if (cuts[id1][id2 - id1] > 1) {
					addSeed(q, face);
				}
			} else {
				cuts[id2][id1 - id2]++;
				if (cuts[id2][id1 - id2] > 1) {
					addSeed(q, face);
				}
			}
		}
	}

	// remove marks from all edges
	std::for_each(m_mesh.edges_begin(), m_mesh.edges_end(), [&](const EH eh) {
		mark(eh, false);
	});
}

void VoronoiRemesh::reduceAdjRegions(FaceDijkstra & q)
{
	const auto adjTiles = [&](const VH v) {
		std::set<ID> tiles;
		for (auto f = m_mesh.cvf_begin(v); f != m_mesh.cvf_end(v); ++f) {
			if (id(*f) >= 0) tiles.insert(id(*f));
		}
		return tiles.size();
	};

	for (VH v : m_mesh.vertices()) {
		if (adjTiles(v) > 3) {
			int before = m_seeds.size();
			for (auto f = m_mesh.cvf_begin(v); f != m_mesh.cvf_end(v); ++f) {
				if (!isSeed(*f)) {
					addSeed(q, *f);
					break;
				}
			}
			if (m_seeds.size() <= before) {
				std::cerr << "ERROR: vertex adj to more than 3 seeds\n";
				// TODO: return an use original mesh
			}
		}
	}
}

void VoronoiRemesh::copyMesh(BezierTMesh & src, BezierTMesh & dest)
{
	dest.clean_keep_reservation();

	auto vs = OpenMesh::makeTemporaryProperty<VH, VH>(dest);
	// works but seems really stupid
	for (const auto &v : src.vertices()) {
		vs[v] = dest.add_vertex_dirty(src.point(v));
	}
	for (const auto &f : src.faces()) {
		std::vector<VH> verts;
		for (auto fv = src.cfv_begin(f); fv != src.cfv_end(f); ++fv) {
			verts.push_back(vs[*fv]);
		}
		const FaceHandle fh = dest.add_face(verts);
		dest.data(fh).points(src.data(f).points());
		dest.set_color(fh, src.color(f));
	}
}

}
