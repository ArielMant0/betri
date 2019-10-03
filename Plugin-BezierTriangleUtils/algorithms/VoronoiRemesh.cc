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

	if (!m_ctrl.get_property_handle(m_ttv, Props::TRITOVERTEX)) {
		m_ctrl.add_property(m_ttv, Props::TRITOVERTEX);
	}

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

	if (m_mesh.get_property_handle(m_pred, Props::PREDECESSOR))
		m_mesh.remove_property(m_pred);

	if (m_mesh.get_property_handle(m_distance, Props::DISTANCE))
		m_mesh.remove_property(m_distance);

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
	size_t num = m_mesh.n_faces();

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

	for (int i = 0; i < counts.size(); ++i) {
		for (int j = 0; j < counts[i].size(); ++j) {
			if (i != j && counts[i][j].first == 1) {
				auto e = counts[i][j].second;

				assert(e != INVALID_E);

				HH h1 = m_mesh.halfedge_handle(e, 0);
				HH h2 = m_mesh.halfedge_handle(e, 1);

				FH f1 = m_mesh.face_handle(h1);
				FH f2 = m_mesh.face_handle(h2);

				fixCrossing(f1, f2);
			}
		}
	}
	std::cerr << __FUNCTION__ << ": created new faces " << num << " to " << m_mesh.n_faces() << "\n";
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

	const FH dummFace = m_mesh.face_handle(0);

	ACG::HaltonColors colGen;

	const auto getRegion = [&](const ID id) {
		return std::find_if(regions.begin(), regions.end(), [&](const Reg &r) {
			return r.first == id;
		});
	};

	const auto borderVertex = [&](const VH v) {
		for (auto f_it = m_mesh.cvf_begin(v); f_it != m_mesh.cvf_end(v); ++f_it) {
			if (!pred(*f_it).is_valid()) return true;
		}
		return false;
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
						} else if (help.isBorder() && !borderVertex(*v_it)) {
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
				auto color = colGen.generateNextColor();
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
	size_t nedges = m_mesh.n_edges();

	for (size_t i = 0; i < nedges; ++i) {
		EH edge = m_mesh.edge_handle(i);

		if (!isCrossed(edge)) {
			HH he = m_mesh.halfedge_handle(edge, 0);
			VH to = m_mesh.to_vertex_handle(he);
			VH from = m_mesh.from_vertex_handle(he);

			// if both adj vertices are border vertices, split this edge
			if (vtt(from).isBorder() && vtt(to).isBorder()) {
				fixCrossing(m_mesh.face_handle(he), m_mesh.opposite_face_handle(he));

				/*FH f3 = m_mesh.face_handle(m_mesh.n_faces() - 2);
				FH f4 = m_mesh.face_handle(m_mesh.n_faces() - 1);
				m_mesh.set_color(f3, { 1.f, 0.5f, 0.f, 1.f });
				m_mesh.set_color(f4, { 1.f, 0.5f, 0.f, 1.f });*/
			}
		}
	}
	std::cerr << __FUNCTION__ << ": added " << m_mesh.n_edges() - nedges << " new edges\n";
}

void VoronoiRemesh::fixPredecessor(const FH fh)
{
	if (!pred(fh).is_valid()) return;

	const auto calcDist = [&](FH from, FH to) {
		P p1 = m_mesh.calc_face_centroid(from);
		P p2 = m_mesh.calc_face_centroid(to);
		return (pred(from).is_valid() ? dist(pred(from)) :  0.0) + (p1 - p2).norm();
	};

	// nothing do to
	if (m_mesh.adjToFace(fh, pred(fh))) {
		dist(fh) = calcDist(fh, pred(fh));
	} else {
		// look at all neighbor faces and see which one is adj to prev pred face
		for (auto f_it = m_mesh.cff_begin(fh); f_it != m_mesh.cff_end(fh); ++f_it) {
			// must always belong to same region
			if (id(fh) == id(f_it) && m_mesh.adjToFace(*f_it, pred(fh))) {
				// make this face our new pred and update distances for both
				dist(*f_it) = calcDist(*f_it, pred(fh));
				pred(*f_it) = pred(fh);
				assert(pred(*f_it) != pred(pred(*f_it)));

				dist(fh) = calcDist(fh, *f_it);
				pred(fh) = *f_it;
				break;
			}
		}
	}
	// make sure everything is in order
	assert(m_mesh.adjToFace(fh, pred(fh)));
	assert(pred(fh) != pred(pred(fh)));
}

HalfedgeHandle VoronoiRemesh::fixCrossing(
	const FH f0,
	const FH f1,
	const VH prevNode,
	const VH notNode
) {
	std::vector<FH> oldNeighbors;
	for (auto f_it = m_mesh.cff_begin(f0); f_it != m_mesh.cff_end(f0); ++f_it) {
		if (*f_it != f1) oldNeighbors.push_back(*f_it);
	}
	for (auto f_it = m_mesh.cff_begin(f1); f_it != m_mesh.cff_end(f1); ++f_it) {
		if (*f_it != f0) oldNeighbors.push_back(*f_it);
	}
	assert(oldNeighbors.size() == 4);

	size_t nedge = m_mesh.n_edges();
	EH commonEdge;
	for (auto f_it = m_mesh.cfh_begin(f0); f_it != m_mesh.cfh_end(f0); ++f_it) {
		if (m_mesh.opposite_face_handle(*f_it) == f1) {
			commonEdge = m_mesh.edge_handle(*f_it); break;
		}
	}

	VH newNode = m_mesh.splitFacesRivara(f0, f1, true);

	// TODO: is this the correct edge
	m_mesh.copy_all_properties(commonEdge, m_mesh.edge_handle(nedge), false);
	m_mesh.set_color(m_mesh.edge_handle(nedge), m_mesh.color(commonEdge));
	for (nedge += 1; nedge < m_mesh.n_edges(); ++nedge) {
		crossed(m_mesh.edge_handle(nedge)) = -1;
	}

	FH f2 = m_mesh.face_handle(m_mesh.n_faces() - 2);
	FH f3 = m_mesh.face_handle(m_mesh.n_faces() - 1);

	fixPredecessor(f0);
	fixPredecessor(f1);
	fixPredecessor(f2);
	fixPredecessor(f3);

	for (FH face : oldNeighbors) {
		fixPredecessor(face);
	}

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
	const ShortestPath & path
) {

	std::cerr << "\calculating shortest path for regions " << id_1 << " and " << id_2 << "\n";

	// find next edge of a region border
	// (always return the halfedge that is adjacent to face with id f1)
	const auto findStartBorder = [&](const VH &v) {
		for (auto h = m_mesh.voh_begin(v); h != m_mesh.voh_end(v); ++h) {
			const ID id1 = id(m_mesh.face_handle(*h));
			const ID id2 = id(m_mesh.opposite_face_handle(*h));
			// if edge was not crossed and is adjacent to the given region
			if (id1 == id_1 && id2 == id_2) {
				return *h;
			} else  if (id1 == id_2 && id2 == id_1) {
				return m_mesh.opposite_halfedge_handle(*h);
			}
		}
		return INVALID_H;
	};

	// find next edge of a region border
	// (always return the halfedge that is adjacent to face with id f1)
	const auto findNextBorder = [&](const VH &v, EH forbidden) {
		for (auto h = m_mesh.voh_begin(v); h != m_mesh.voh_end(v); ++h) {
			const ID id1 = id(m_mesh.face_handle(*h));
			const ID id2 = id(m_mesh.opposite_face_handle(*h));
			const EH edge = m_mesh.edge_handle(*h);
			// if edge was not crossed and is adjacent to the given region
			if (edge != forbidden && (id1 == id_1 && id2 == id_2 ||
				id1 == id_2 && id2 == id_1)) {
				return *h;
			}
		}
		return INVALID_H;
	};

	const auto connectingHalfedge = [&](
		const FH from,
		const FH to,
		const VH adj,
		const FH prev,
		const bool border
	) {
		HH halfedge;
		for (auto he = m_mesh.cfh_begin(from); he != m_mesh.cfh_end(from); ++he) {
			if (m_mesh.to_vertex_handle(*he) == adj || m_mesh.from_vertex_handle(*he) == adj) {
				VH v = m_mesh.to_vertex_handle(*he) == adj ?
					m_mesh.from_vertex_handle(*he) :
					m_mesh.to_vertex_handle(*he);

				// TODO: IMPORTANT: it must be allowed to be adj to the prev face so that we can
				// use both sides of a joined path (which use the same faces)
				// maybe even prefer those
				if (m_mesh.opposite_face_handle(*he) != to && m_mesh.adjToFace(v, to)) {
					if (!isCrossed(*he) && (border || !vtt(v).isBorder()) &&
						(!prev.is_valid() || m_mesh.adjToFace(*he, prev))) {
						return *he;
					} else {
						halfedge = *he;
					}
				}
			}
		}
		return halfedge;
	};

	const auto commonVertex = [&](const FH fh1, const FH fh2) {
		VH vh, vb;
		for (auto v_it = m_mesh.cfv_begin(fh1); v_it != m_mesh.cfv_end(fh1); ++v_it) {
			if (m_mesh.adjToFace(*v_it, fh2)) {
				if (vtt(*v_it).isBorder()) {
					vb = *v_it;
					if (!vh.is_valid()) vh = *v_it;
				} else {
					vh = *v_it;
				}
			}
		}
		/*if (vb.is_valid() && vb != vh) {
			EH edge = m_mesh.edge_handle(m_mesh.find_halfedge(vh, vb));
			m_mesh.set_color(edge, { 0.f, 0.f, 0.f, 1.f });
			crossed(edge) = ctrlFace.idx();
		}*/
		return vb.is_valid() ? vb : vh;
	};

	HH start = findStartBorder(v), he = start;

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
		}

		he = findNextBorder(m_mesh.to_vertex_handle(he), m_mesh.edge_handle(he));

		// two regions should not have a circular boundary
		assert(he != start);
	}

	std::cerr << "\t\tfound faces " << f11 << " and " << f22 << "\n";

	he = start;

	std::deque<FH> subpath;

	size_t middle = 0;
	// make a queue containing all faces of the path in correct order
	FH working = f11;
	while (working.is_valid()) {
		subpath.push_front(working);
		std::cerr << "\t\tface " << working << " has pred " << pred(working) << "\n";
		working = pred(working);
		middle++;
	}
	working = f22;
	while (working.is_valid()) {
		subpath.push_back(working);
		std::cerr << "\t\tface " << working << " has pred " << pred(working) << "\n";
		working = pred(working);
	}

	assert(subpath.size() >= 2);

	VH node = commonVertex(subpath[0], subpath[1]), prevNode;
	assert(node.is_valid());
	// set color and mark this as a boundary vertex
	m_mesh.set_color(node, { 0.f, 0.f, 0.f, 1.f });
	vtt(node).setBorder(id_1, id_2);
	path.push(node);

	HH way; EH edge;
	FH f_it, next;

	for (size_t i=1, j=2; i < subpath.size()-1; ++i, ++j) {
		f_it = subpath[i];
		next = subpath[j];

		// mark border face
		Color oldFaceColor = m_mesh.color(f_it);
		m_mesh.set_color(f_it, { 1.f, 1.f, 1.f, 1.f });

		// find halfedge to next vertex in the path
		way = connectingHalfedge(f_it, next, node, subpath[i-1], i == subpath.size()-2);
		edge = m_mesh.edge_handle(way);
		prevNode = node;

		if (way.is_valid()) {
			node = m_mesh.to_vertex_handle(way) != node ?
				m_mesh.to_vertex_handle(way) :
				m_mesh.from_vertex_handle(way);

			if (j < subpath.size()-1 && vtt(node).isBorder()) {
				std::cerr << "--------------> CROSSING PATHS (at face " << f_it << ") !!!\n";

				// mark edge in question for visual purposes (if only that worked)
				m_mesh.set_color(edge, { 1.f, 1.f, 0.f, 1.f });

				if (!nextEdgeCrossed(edge, f_it, next)) {
					std::cerr << "\tsplitting current + next\n";
					way = fixCrossing(f_it, next, prevNode, node);

					FH f3 = m_mesh.face_handle(m_mesh.n_faces() - 2);
					FH f4 = m_mesh.face_handle(m_mesh.n_faces() - 1);
					m_mesh.set_color(f3, { 0.5f, 0.5f, 0.5f, 1.f });
					m_mesh.set_color(f4, { 0.5f, 0.5f, 0.5f, 1.f });
					m_mesh.set_color(f_it, oldFaceColor);

					edge = m_mesh.edge_handle(way);
					node = m_mesh.to_vertex_handle(way) != prevNode ?
						m_mesh.to_vertex_handle(way) :
						m_mesh.from_vertex_handle(way);
				} else {
					// TODO: can this happen and is this useful if it does?
					std::cerr << "\tweird crossing (splitting next + nextNext)\n";

					//fixCrossing(next, subpath[j+1]);
					//// TODO: correct pred for new faces?
					//// replace edges in other path
					//ShortestPath::replace(m_mesh, node,
					//	m_mesh.vertex_handle(m_mesh.n_vertices()-1), id_1, id_2
					//);

					//FH f3 = m_mesh.face_handle(m_mesh.n_faces() - 2);
					//FH f4 = m_mesh.face_handle(m_mesh.n_faces() - 1);
					//m_mesh.set_color(f3, { 0.0f, 0.9f, 1.f, 1.f });
					//m_mesh.set_color(f4, { 0.0f, 0.9f, 1.f, 1.f });
				}
				// update face variables
				subpath[i] = m_mesh.face_handle(way);
				subpath[i < middle ? i - 1 : j] = pred(subpath[i]);
				f_it = subpath[i];
				next = subpath[j];
				m_mesh.set_color(f_it, { 1.f, 1.f, 1.f, 1.f });

				// TODO: repair pred (so next faces in q) if necessary
				if (false && i >= middle) {
					size_t k = j;
					while (pred(subpath[k]).is_valid()) {
						if (k == subpath.size() - 1) {
							subpath.push_back(pred(subpath[k]));
						} else {
							subpath[k + 1] = pred(subpath[k]);
						}
						k++;
					}
				}
				assert(m_mesh.adjToFace(edge, f_it));
			}
			// set color and mark this as a boundary vertex
			m_mesh.set_color(node, { 0.f, 0.f, 0.f, 1.f });
			vtt(node).setBorder(id_1, id_2);
			// add the edge to the path and color it
			path.push(node);
			crossed(edge) = ctrlFace.idx();
			m_mesh.set_color(edge, { 0.f, 0.f, 0.f, 1.f });
		}
	}
}

void VoronoiRemesh::connectPaths(const ShortestPath & p0, const ShortestPath & p1, const FH f)
{
	if (p0.end() == p1.start() && p0.back() != p1.front()) {
		HH h = m_mesh.find_halfedge(p0.back(), p1.front());
		assert(h.is_valid());
		crossed(h) = f.idx();
		p0.push(p1.front());
	} else if (p0.end() == p1.end() && p0.back() != p1.back()) {
		HH h = m_mesh.find_halfedge(p0.back(), p1.back());
		assert(h.is_valid());
		crossed(h) = f.idx();
		p0.push(p1.back());
	}
}

 //////////////////////////////////////////////////////////
 // create voronoi partition
 //////////////////////////////////////////////////////////
void VoronoiRemesh::partition()
{
	using QElem = std::pair<double, FH>;

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
				grow(ff, face, distance);
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
					auto v = m_mesh.to_vertex_handle(next);
					// COND 1: if not homeomorphic to disk (new face is adj to boundary across
					// which lies a face of the same tile) add face as new seed
					if (!homeomorphicDisk(f, v, id(face))) {
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
				m_mesh.set_color(edge, m_colors[id(f1)]);
			} else {
				m_mesh.set_color(edge, boundaryColor);
			}
		}

		for (auto &edge : m_boundary) {
			m_mesh.set_color(edge, { 1.f, 0.f, 0.f, 1.f });
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

	// TODO: only debug purposes (test if pred relation is correct for all faces)
	for (FH fh : m_mesh.faces()) {
		if (pred(fh).is_valid()) {
			assert(m_mesh.adjToFace(fh, pred(fh)));
			assert(pred(fh) != pred(pred(fh)));
		}
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
	for (m_vertexIdx; m_vertexIdx < m_mesh.n_vertices() && (!found || !steps); ++m_vertexIdx) {
		v = m_mesh.vertex_handle(m_vertexIdx);

		std::cerr << "checking vertex " << v << "\n";

		// check how many different regions are around that vertex
		for (auto vf = m_mesh.vf_begin(v); vf != m_mesh.vf_end(v); ++vf) {
			if (check.insert(id(*vf)).second) {
				seedIDs.push_back(id(*vf));
				points.push_back(m_ctrlVerts[id(*vf)]);
			}
		}

		// only do sth if we found 3 regions
		if (seedIDs.size() > 2) {
			std::cerr << "\tVoronoi Vertex\n";

			assert(seedIDs.size() == 3);

			// add the face
			const auto fh = m_ctrl.add_face(points);

			std::vector<std::pair<ID, ID>> revisit;

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
					revisit.push_back({ id1, id2 });
				}
			}
			// make sure there are no impassable edges
			// (i.e. edge connected to 2 vertices adj to border edges)
			// TODO: does not work well
			// splitClosedPaths();

			auto ab = ShortestPath::path(ttv(fh)[0], ttv(fh)[1]);
			auto bc = ShortestPath::path(ttv(fh)[1], ttv(fh)[2]);
			auto ca = ShortestPath::path(ttv(fh)[2], ttv(fh)[0]);
			connectPaths(ab, bc, fh);
			connectPaths(bc, ca, fh);
			connectPaths(ca, ab, fh);

			found = true;
		}
		points.clear();
		seedIDs.clear();
		check.clear();
	}

	// if we finished, assign interior vertices
	if (m_vertexIdx == m_mesh.n_vertices()) {
		// TODO: make sure this doesnt happen in partition
		if (m_ctrl.n_faces() < 4) return true;

		// perform assignment
		assignInnerVertices();
	}

	return m_vertexIdx == m_mesh.n_vertices();
}

//////////////////////////////////////////////////////////
// parameterization (harmonic map) + surface fitting
//////////////////////////////////////////////////////////
void VoronoiRemesh::fitting()
{
	Parametrization param(m_mesh, m_ctrl, m_ttv, m_vtt, m_pred);

	param.solve();

	Fitting fit(m_mesh, m_ctrl, m_ttv, m_vtt);

	fit.solve();
}

void VoronoiRemesh::remesh()
{
	partition();

	dualize();

	fitting();

	// replace original mesh
	if (m_copy) copyMesh(m_ctrl, m_mesh);

	std::cerr << "----------- DONE -----------" << std::endl;
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