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
				auto color = m_colGen.generateNextColor();
				// mark vertices
				for (VH v : inner) {
					vtt(v).setFace(ctrlFace, ctrlFace.idx());
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
	const ShortestPath & path
) {

	std::cerr << "\tcalculating shortest path for regions " << id_1 << '(' << m_seeds[id_1] << ')';
	std::cerr << " and " << id_2 << '(' << m_seeds[id_2] << ")\n";

	// find next edge of a region border
	// (always return the halfedge that is adjacent to face with id f1)
	const auto findStartBorder = [&](const VH v) {
		for (auto h = m_mesh.voh_begin(v); h != m_mesh.voh_end(v); ++h) {
			const ID id1 = id(m_mesh.face_handle(*h));
			const ID id2 = id(m_mesh.opposite_face_handle(*h));
			// if edge is adjacent to the given regions
			if (id1 == id_1 && id2 == id_2) {
				return *h;
			} else  if (id1 == id_2 && id2 == id_1) {
				return m_mesh.opposite_halfedge_handle(*h);
			}
		}
		return HH();
	};

	// find next edge of a region border
	// (always return the halfedge that is adjacent to face with id f1)
	const auto findNextBorder = [&](const VH v, EH forbidden) {
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
		return HH();
	};

	const auto adjToSeed = [&](const VH vh) {
		for (auto vf = m_mesh.cvf_begin(vh); vf != m_mesh.cvf_end(vh); ++vf) {
			if (isSeed(*vf)) return true;
		}
		return false;
	};

	const auto connectingHalfedge = [&](
		const FH from,
		const FH to,
		const FH before,
		const VH adj,
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
				if (m_mesh.opposite_face_handle(*he) != to && m_mesh.adjToFace(v, to) &&
					m_mesh.opposite_face_handle(*he) != before) {
					if (!isCrossed(*he) && (border || !vtt(v).isBorder() || adjToSeed(v))) {
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
		VH vh, vb0, vb1;
		for (auto v_it = m_mesh.cfv_begin(fh1); v_it != m_mesh.cfv_end(fh1); ++v_it) {
			if (vtt(*v_it).isBorder()) {
				vb0 = *v_it;
				if (m_mesh.adjToFace(*v_it, fh2)) {
					vb1 = *v_it;
				}
			} else if (m_mesh.adjToFace(*v_it, fh2)) {
				vh = *v_it;
			}
		}

		if (vb1.is_valid()) return vb1;

		if (vb0.is_valid() && vh.is_valid() && vb0 != vh) {
			EH edge = m_mesh.edge_handle(m_mesh.find_halfedge(vh, vb0));
			m_mesh.set_color(edge, { 0.f, 0.f, 0.f, 1.f });
			crossed(edge) = ctrlFace.idx();
		}

		assert(vh.is_valid());

		return vh;
	};

	HH start = findStartBorder(v), he = start;

	assert(start.is_valid());

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
			std::cerr << "\tfound better start faces " << f11 << " and " << f22 << "\n";
		}

		he = findNextBorder(m_mesh.to_vertex_handle(he), m_mesh.edge_handle(he));

		// two regions should not have a circular boundary
		assert(he != start);
	}

	assert(f11.is_valid() && f22.is_valid());

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
	assert(isSeed(subpath.front()));

	working = f22;
	while (working.is_valid()) {
		subpath.push_back(working);
		std::cerr << "\t\tface " << working << " has pred " << pred(working) << "\n";
		working = pred(working);
	}
	assert(isSeed(subpath.back()));
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

		FH prev = subpath[i - 1];
		// find halfedge to next vertex in the path
		way = connectingHalfedge(f_it, next, prev, node, i == subpath.size()-2);
		edge = m_mesh.edge_handle(way);
		prevNode = node;

		if (way.is_valid()) {
			node = m_mesh.to_vertex_handle(way) != node ?
				m_mesh.to_vertex_handle(way) :
				m_mesh.from_vertex_handle(way);

			if (j < subpath.size()-1 && vtt(node).isBorder() && !adjToSeed(node)) {
				std::cerr << "--------------> CROSSING PATHS (at face " << f_it << ") !!!\n";

				// mark edge in question for visual purposes (if only that worked)
				m_mesh.set_color(edge, { 1.f, 1.f, 0.f, 1.f });

				if (!commonEdgeCrossed(f_it, next)) {
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
					std::cerr << "\nWARNING: THIS SHOULD NOT HAPPEN (crossing paths)\n\n";
					//m_mesh.set_color(node, { 1.f, 0.f, 0.f, 1.f });
					m_mesh.set_color(f_it, { 1.f, 0.f, 1.f, 1.f });
					m_mesh.set_color(next, { 1.f, 0.f, 1.f, 1.f });
					std::cerr << node << ", " << f_it << " (p: " << pred(f_it) << ", id: ";
					std::cerr << id(f_it) << "), " << next << " (p: " << pred(next) << ", id: ";
					std::cerr << id(next) << ")\n";
					m_debugCancel = true;
					return;
				}
				// update face variables
				subpath[i] = m_mesh.face_handle(way);
				subpath[i < middle ? i - 1 : j] = pred(subpath[i]);
				f_it = subpath[i];
				next = subpath[j];
				m_mesh.set_color(f_it, { 1.f, 1.f, 1.f, 1.f });

				// TODO: repair pred (so next faces in q) if necessary
				if (i >= middle) {
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
			m_mesh.set_color(node, m_colors[id_1]);
			vtt(node).setBorder(id_1, id_2);
			// add the edge to the path and color it
			path.push(node);
			crossed(edge) = ctrlFace.idx();
			m_mesh.set_color(edge, { 0.f, 0.f, 0.f, 1.f });

			// stop loop as soon as we reached the seed face
			if (adjToSeed(node)) break;
		}
	}

	// connect last vertex to other (border) vertex of the same seed face
	//FH lastSeed = subpath.back();
	//for (auto vv = m_mesh.cfv_begin(lastSeed); vv != m_mesh.cfv_end(lastSeed); ++vv) {
	//	if (vtt(*vv).isBorder() && path.back() != *vv) {
	//		HH con = m_mesh.find_halfedge(path.back(), *vv);
	//		crossed(con) = ctrlFace.idx();
	//		m_mesh.set_color(m_mesh.edge_handle(con), { 0.f, 0.f, 0.f, 1.f });
	//		// add vertex to this path
	//		path.push(*vv);
	//		break;
	//	}
	//}
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
	// special "priority-queue" for dijkstra
	Dijkstra q;

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
	for (m_vertexIdx; m_vertexIdx < m_mesh.n_vertices(); ++m_vertexIdx) {
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
			// pick up here again in the next step
			if (found && steps) break;

			std::cerr << "\tVoronoi Vertex\n";

			assert(seedIDs.size() == 3);

			// add the face
			const auto fh = m_ctrl.add_face(points);

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

					// recalculate shortest paths for these two regions
					repartition(id1);
					for (FH faaace : m_mesh.faces()) {
						if (id(faaace) < 0) {
							return false;
						}
					}

					repartition(id2);
					for (FH faaace : m_mesh.faces()) {
						if (id(faaace) < 0) {
							return false;
						}
					}
				}
			}

			/*auto ab = ShortestPath::path(ttv(fh)[0], ttv(fh)[1]);
			auto bc = ShortestPath::path(ttv(fh)[1], ttv(fh)[2]);
			auto ca = ShortestPath::path(ttv(fh)[2], ttv(fh)[0]);
			connectPaths(ab, bc, fh);
			connectPaths(bc, ca, fh);
			connectPaths(ca, ab, fh);*/

			// make sure there are no impassable edges
			// (i.e. edge connected to 2 vertices adj to border edges)
			// TODO: does not work well
			splitClosedPaths();

			for (FH f : m_mesh.faces()) {
				if (id(f) < 0 || (pred(f).is_valid() && f == pred(pred(f)) && !isSeed(f))) {
					std::cerr << "\tERROR 2: face " << f << ", id " << id(f) << ", pred " << pred(f) << "\n";
				}
				assert((isSeed(f) && !pred(f).is_valid()) || m_mesh.adjToFace(f, pred(f)));
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

//////////////////////////////////////////////////////////
// parameterization (harmonic map) + surface fitting
//////////////////////////////////////////////////////////
void VoronoiRemesh::fitting()
{
	Parametrization param(m_mesh, m_ctrl, m_vtt, m_ttv, m_pred);

	param.solve();

	Fitting fit(m_mesh, m_ctrl, m_ttv, m_vtt);

	fit.solve();
}

void VoronoiRemesh::remesh()
{
	partition();

	if (m_debugCancel) return;

	dualize();

	if (m_debugCancel) return;

	fitting();

	// replace original mesh
	if (m_copy) copyMesh(m_ctrl, m_mesh);

	std::cerr << "----------- DONE -----------" << std::endl;
}

void VoronoiRemesh::resetPath(Dijkstra &q, const FH face)
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
	Dijkstra q;

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

void VoronoiRemesh::reduceCuts(Dijkstra &q)
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

void VoronoiRemesh::reduceAdjRegions(Dijkstra & q)
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
