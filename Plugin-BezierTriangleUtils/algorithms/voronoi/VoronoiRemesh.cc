#include "VoronoiRemesh.hh"

#include "VoronoiParametrization.hh"
#include "VoronoiFitting.hh"

#include <queue>
#include <unordered_set>
#include <map>
#include <random>
#include <fstream>

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
	for (int i = 0, h = m_seeds.size()-1; i < m_seeds.size()-1; ++i, --h) {
		std::vector<std::pair<int, EH>> arr;
		arr.reserve(h);
		for (int j = 0; j < h; ++j) {
			arr.push_back({ 0, EH() });
		}
		counts.push_back(arr);
	}

	for (EH edge : m_mesh.edges()) {
		HH h = m_mesh.halfedge_handle(edge, 0);
		FH f1 = m_mesh.face_handle(h), f2 = m_mesh.opposite_face_handle(h);
		assert(id(f1) >= 0 && id(f2) >= 0);
		if (id(f1) != id(f2)) {
			const int i1 = std::min(id(f1), id(f2));
			const int i2 = std::max(id(f2), id(f1)) - 1 - i1;
			// only store count once!
			counts[i1][i2].first++;
			counts[i1][i2].second = edge;
		}
	}

	for (int i = 0; i < counts.size(); ++i) {
		for (int j = 0; j < counts[i].size(); ++j) {
			if (counts[i][j].first == 1) {
				EH e = counts[i][j].second;

				assert(e.is_valid());

				HH he = m_mesh.halfedge_handle(e, 0);

				FH f1 = m_mesh.face_handle(he);
				FH f2 = m_mesh.opposite_face_handle(he);

				fixCrossing(f1, f2);

				// make sure the vertices along this split always belong
				// to those two regions, so we can actually find a path trough
				VH node = m_mesh.vertex_handle(m_mesh.n_vertices() - 1);
				const ID id0 = id(f1), id1 = id(f2);
				assert(id(node) == id0 || id(node) == id1);
				const ID other = id(node) == id0 ? id1 : id0;
				setColor(node, m_colors[id(node)]);

				VH v0 = m_mesh.to_vertex_handle(m_mesh.next_halfedge_handle(he));
				VH v1 = m_mesh.to_vertex_handle(
					m_mesh.next_halfedge_handle(m_mesh.opposite_halfedge_handle(he))
				);

				if (id(v0) != other && id(v1) != other) {
					VH which = adjToRegion(v0, other) ? v0 : v1;
					id(which) = other;
					findShortestPath(which, other);
					pred(node) = which;
					assert(m_mesh.find_halfedge(which, node).is_valid());
					dist(node) = dist(which) + (m_mesh.point(which) - m_mesh.point(node)).norm();
					setColor(which, m_colors[id(which)]);
				}
			}
		}
	}
	std::cerr << __FUNCTION__ << ": created " << m_mesh.n_faces()-num << "  new faces\n";
}

void VoronoiRemesh::assignInnerVertices()
{
	std::deque<VH> q;
	std::unordered_set<VH> inner;

	auto assigned = OpenMesh::makeTemporaryProperty<FH, bool>(m_ctrl, "visited");

	using Reg = std::pair<ID, size_t>;
	std::vector<Reg> regions;

	std::cerr << "assigning inner vertices\n";

	const auto getRegion = [&](const ID id) {
		return std::find_if(regions.begin(), regions.end(), [&](const Reg &r) {
			return r.first == id;
		});
	};

	for (VH vh : m_mesh.vertices()) {

		if (!vtt(vh).isBorder() && !vtt(vh).face.is_valid()) {

			q.push_back(vh);
			inner.insert(vh);

			// find all vertices of this patch
			while (!q.empty()) {

				VH front = q.front();
				q.pop_front();

				// check neighbors
				for (auto v_it = m_mesh.cvv_begin(front), v_e = m_mesh.cvv_end(front);
					v_it != v_e; ++v_it
				) {
					auto &help = vtt(*v_it);

					if (!help.isBorder() && inner.find(*v_it) == inner.end()) {
						inner.insert(*v_it);
						q.push_back(*v_it);
					} else if (help.isBorder() && !isSeedVertex(*v_it)) {
						auto r = getRegion(help.id1);
						if (r != regions.end()) {
							r->second++;
						} else {
							//std::cerr << "\t\tadding region" << help.id1 << "\n";
							regions.push_back({ help.id1, 1 });
						}

						r = getRegion(help.id2);
						if (r != regions.end()) {
							r->second++;
						} else {
							//std::cerr << "\t\tadding region" << help.id2 << "\n";
							regions.push_back({ help.id2, 1 });
						}
					}
				}
			}

			assert(regions.size() > 2);

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
			};

			if (regions.size() == 3) {
				ctrlFace = findDelaunayFace(
					regions[0].first,
					regions[1].first,
					regions[2].first
				);
				std::cerr << "\tassigning to face " << ctrlFace << " for regions ";
				std::cerr << regions[0].first << " + " << regions[1].first << " + " << regions[2].first << "\n";
			} else {
				comb();
			}

			assert(ctrlFace.is_valid());
			assert(!assigned[ctrlFace]);

			assigned[ctrlFace] = true;
			Color color = m_colGen.generateNextColor();
			// mark vertices
			for (VH v : inner) {
				vtt(v).setFace(ctrlFace);
				setColor(v, color);
			}
			// connect to face
			ttv(ctrlFace).inner.insert(
				ttv(ctrlFace).inner.end(),
				inner.begin(),
				inner.end()
			);

			assert(ttv(ctrlFace).inner.size() == inner.size());

			inner.clear();
			regions.clear();
		}
	}

#ifdef BEZIER_DEBUG
	if (debugCancel()) return;

	for (FH fh : m_ctrl.faces()) {
		if (!assigned[fh]) {
			m_mesh.set_color(fh, { 1.f, 1.f, 1.f, 1.f });
			debugCancel("face not assigned");
		}
	}

	for (VH vh : m_mesh.vertices()) {
		if (!vtt(vh).isBorder() && !vtt(vh).face.is_valid()) {
			m_mesh.set_color(vh, { 1.f, 1.f, 1.f, 1.f });
			debugCancel("vertex not assigned");
		}
	}
#endif

	debugCancel();
}

void VoronoiRemesh::splitClosedPaths()
{
	std::cerr << __FUNCTION__ << " START\n";

	size_t nedges = m_mesh.n_edges();
	size_t count = 0, count2 = 0, count3 = 0;

	for (size_t i = 0; i < nedges; ++i) {
		EH edge = m_mesh.edge_handle(i);

		HH he = m_mesh.halfedge_handle(edge, 0);

		const FH f1 = m_mesh.face_handle(he), f2 = m_mesh.opposite_face_handle(he);

		if (!isCrossed(edge)) {// && !isSeed(f1) && !isSeed(f2)) {
			VH to = m_mesh.to_vertex_handle(he);
			VH from = m_mesh.from_vertex_handle(he);

			// if both adj vertices are border vertices, split this edge
			if (vtt(from).isBorder() && vtt(to).isBorder()) {
				fixCrossing(f1, f2);

				FH f3 = m_mesh.face_handle(m_mesh.n_faces() - 2);
				FH f4 = m_mesh.face_handle(m_mesh.n_faces() - 1);

				if (isSeed(f1)) {
					// make sure seed face is adj to seed vertex
					if (id(f3) == id(f1) && m_mesh.adjToVertex(f3, m_seedVerts[id(f1)])) {
						m_seeds[id(f1)] = f3;
					} else if (id(f4) == id(f1) && m_mesh.adjToVertex(f4, m_seedVerts[id(f1)])) {
						m_seeds[id(f1)] = f4;
					}
				}
				if (isSeed(f2) && !m_mesh.adjToVertex(f2, m_seedVerts[id(f2)])) {
					// make sure seed face is adj to seed vertex
					if (id(f3) == id(f2) && m_mesh.adjToVertex(f3, m_seedVerts[id(f2)])) {
						m_seeds[id(f2)] = f3;
					} else if (id(f4) == id(f2) && m_mesh.adjToVertex(f4, m_seedVerts[id(f2)])) {
						m_seeds[id(f2)] = f4;
					}
				}

				count++;
			} /*else if (id(from) == id(to) && id(f1) == id(f2) &&
				(vtt(from).isBorder() && !vtt(to).isBorder() && onRegionBorder(to) ||
				!vtt(from).isBorder() && vtt(to).isBorder() && onRegionBorder(from))
				//id(from) == id(to) && onRegionBorder(to) && id(f1) == id(f2)
			) {
				// when a vertex is adj to a path across the mesh
				fixCrossing(f1, f2);
				count2++;
			}*/ else if (!vtt(to).isBorder() && !vtt(from).isBorder() &&
				onRegionBorder(to) && onRegionBorder(from) && id(f1) == id(f2)
			) {
				// when we have 2 vertices, connected by and edge, both of which
				// lie on a region border and both adj faces have the same region id
				fixCrossing(f1, f2);
				count3++;

				VH newNode = m_mesh.vertex_handle(m_mesh.n_vertices() - 1);
				id(newNode) = id(to);
			} else if (vtt(to).isBorder() && !vtt(from).isBorder() &&
				onRegionBorder(from) && id(f1) == id(f2)
			) {
				fixCrossing(f1, f2);
				count3++;

				VH newNode = m_mesh.vertex_handle(m_mesh.n_vertices() - 1);
				id(newNode) = id(to);
			} else if (!vtt(to).isBorder() && vtt(from).isBorder() &&
				onRegionBorder(to) && id(f1) == id(f2)
			) {
				fixCrossing(f1, f2);
				count3++;

				VH newNode = m_mesh.vertex_handle(m_mesh.n_vertices() - 1);
				id(newNode) = id(from);
			}
		}
	}

	std::cerr << __FUNCTION__ << ": split " << count << " edges (" << count2;
	std::cerr << ", " << count3 << ")\n";
}

void VoronoiRemesh::fixPredecessor(const FH fh, const bool rewrite)
{
	//std::cerr << "fixing predecessor for " << fh << "(" << pred(fh) << ")\n";

	// we acutally dont have to care about face predecessors
	// since all shortest path are foudn using vertices
	return;

	if (isSeed(fh) || !pred(fh).is_valid()) {
		return;
	}

	const auto calcDist = [&](FH from, FH to) {
		P p1 = m_mesh.calc_face_centroid(from);
		P p2 = m_mesh.calc_face_centroid(to);
		return dist(from) + (p1 - p2).norm();
	};

	FH target = pred(fh);
	if (!target.is_valid()) {
		std::cerr << __FUNCTION__ << " id " << id(fh) << "\n";
		target = m_seeds[id(fh)];
	}

	if (m_mesh.adjToFace(fh, target)) {
		//std::cerr << "\tface already adj to predecessor\n";
		dist(fh) = calcDist(target, fh);
	} else {
		//std::cerr << "\tcalculating new predecessor\n";

		// look at all neighbor faces and see which one fits
		for (auto h_it = m_mesh.cfh_begin(fh); h_it != m_mesh.cfh_end(fh); ++h_it) {
			const FH f = m_mesh.opposite_face_handle(*h_it);
			// must always belong to same region
			// must not be my own predecessor
			// must be adj to target face
			if (id(fh) != id(f) && isCrossed(*h_it) && pred(f) == fh) continue;

			if (rewrite && m_mesh.adjToFace(f, target)) {
				//std::cerr << "\t\trewrite face " << f << '\n';
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
				//std::cerr << "\t\tupdate " << f << '\n';
				// update this face
				dist(fh) = calcDist(f, fh);
				pred(fh) = f;
				break;
			}
		}

	}

	m_mesh.status(fh).set_tagged(true);
	//std::cerr << "\t\tpred is now " << pred(fh) << "\n";

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

	EH commonEdge;
	VH vFrom, vTo;
	size_t nedge = m_mesh.n_edges();

	for (auto f_it = m_mesh.cfh_begin(f0); f_it != m_mesh.cfh_end(f0); ++f_it) {
		if (m_mesh.opposite_face_handle(*f_it) == f1) {
			vFrom = m_mesh.from_vertex_handle(*f_it);
			vTo = m_mesh.to_vertex_handle(*f_it);
			commonEdge = m_mesh.edge_handle(*f_it); break;
		}
	}

	VH newNode = m_mesh.splitFacesRivara(f0, f1, true);

	if (m_seedVerts.size() > 0) {

		if (pred(vFrom) == vTo) {
			pred(newNode) = vTo;
			id(newNode) = id(vTo);
			dist(newNode) = dist(vTo) + (m_mesh.point(vTo) - m_mesh.point(newNode)).norm();

			pred(vFrom) = newNode;
			dist(vFrom) = dist(newNode) + (m_mesh.point(vFrom) - m_mesh.point(newNode)).norm();
		} else if (pred(vTo) == vFrom) {
			pred(newNode) = vFrom;
			id(newNode) = id(vFrom);
			dist(newNode) = dist(vFrom) + (m_mesh.point(vFrom) - m_mesh.point(newNode)).norm();

			pred(vTo) = newNode;
			dist(vTo) = dist(newNode) + (m_mesh.point(vTo) - m_mesh.point(newNode)).norm();
		} else  {
			Scalar minDist = std::numeric_limits<Scalar>::max();
			VH newPred;
			// choose id/distance/predecessor for the new vertex (which neighbor is closest)
			for (auto vh = m_mesh.cvoh_begin(newNode); vh != m_mesh.cvoh_end(newNode); ++vh) {
				VH np = m_mesh.to_vertex_handle(*vh);
				Scalar len = dist(np) + m_mesh.calc_edge_length(*vh);
				if (!vtt(np).isBorder() && len <= minDist && adjToRegion(newNode, id(np))) {
					newPred = np;
					minDist = len;
					if (isSeedVertex(np)) break;
				}
			}
			if (newPred.is_valid()) {
				dist(newNode) = minDist;
				pred(newNode) = newPred;
				id(newNode) = id(newPred);
			} else {
				id(newNode) = id(m_mesh.face_handle(m_mesh.halfedge_handle(newNode)));
				dist(newNode) = minDist;
			}
		}
	}

	// TODO: is this the correct edge
	m_mesh.copy_all_properties(commonEdge, m_mesh.edge_handle(nedge), false);
	setColor(m_mesh.edge_handle(nedge), m_mesh.color(commonEdge));
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
	ShortestPath &path
) {

	std::cerr << "\tcalculating shortest path for regions " << id_1 << " (" << m_seedVerts[id_1];
	std::cerr << ") and " << id_2 << " (" << m_seedVerts[id_2] << ")\n";

	std::unordered_set<VH> visited;

	const auto minConnection = [&](const VH vh, const ID other) {
		VH partner;

		//std::cerr << "\tmin connect: has id " << id(vh);
		//std::cerr << ", looking for id " << other << '\n';

		Scalar minDist = std::numeric_limits<Scalar>::max();

		for (auto he = m_mesh.cvoh_begin(vh); he != m_mesh.cvoh_end(vh); ++he) {
			const VH vv = m_mesh.to_vertex_handle(*he);
			//std::cerr << "\t\t" << id(vv) << ", " << dist(vv) << ", ";
			//std::cerr << vtt(vv).isBorder() << "\n";
			if (id(vv) == other && !vtt(vv).isBorder() &&
				dist(vv) + (m_mesh.point(vv)-m_mesh.point(vh)).norm() < minDist
			) {
				minDist = dist(vv);
				partner = vv;
			}
		}
		return partner;
	};

	// find next edge of a region border
	// (always return the halfedge that is adjacent to face with id f1)
	const auto findNextVertex = [&](const VH from) {
		std::tuple<VH,VH,VH> next;

		//std::cerr << "PASS\n";
		for (auto h = m_mesh.voh_begin(from); h != m_mesh.voh_end(from); ++h) {

			const ID id1 = id(m_mesh.face_handle(*h));
			const ID id2 = id(m_mesh.opposite_face_handle(*h));
			const VH vh = m_mesh.to_vertex_handle(*h);

			//std::cerr << '\t' << id(vh) << " " << border(vh) << " " << id1 << " " << id2 << '\n';
			// if adj faces are border faces and the vertex was not already visited
			if ((id1 == id_1 && id2 == id_2 || id1 == id_2 && id2 == id_1) &&
				visited.find(vh) == visited.end()
			) {
				std::get<0>(next) = vh;

				VH p1 = minConnection(vh, id_1);
				VH p2 = minConnection(vh, id_2);
				if (p1.is_valid() && p2.is_valid()) {
					std::get<1>(next) = p1;
					std::get<2>(next) = p2;
					return next;
				}
			}
		}
		return next;
	};

	const auto findStartPair = [&]() {
		Scalar minDist = std::numeric_limits<Scalar>::max();
		std::tuple<VH, VH, VH> current = { v, minConnection(v, id_1), minConnection(v, id_2) };
		std::tuple<VH, VH, VH> start;


		// find adj faces with shortest path to their respective seed points
		while (std::get<0>(current).is_valid()) {

			// on border
			VH v00 = std::get<0>(current);
			// min connection for region 1
			VH v01 = std::get<1>(current);
			// min connection for region 2
			VH v02 = std::get<2>(current);

			if (!vtt(v00).isBorder() && v01.is_valid() && v02.is_valid()) {

				const Scalar best = dist(v01) + dist(v02) +
					(m_mesh.point(v00) - m_mesh.point(v01)).norm() +
					(m_mesh.point(v00) - m_mesh.point(v02)).norm();

				if ((id(v01) == id_1 || id(v01) == id_2) &&
					(id(v02) == id_1 || id(v02) == id_2) &&
					best <= minDist
				) {
					minDist = best;
					std::get<0>(start) = v00;
					std::get<1>(start) = v01;
					std::get<2>(start) = v02;

					assert(id(v01) != id(v02));

					std::cerr << "\tfound better start vertices " << v01 << " and ";
					std::cerr << v02 << " with dist " << best << "\n";
				}
			}

			visited.insert(v00);

			current = findNextVertex(v00);

			// two regions should not have a circular boundary
			assert(std::get<0>(current) != v);
		}

		return start;
	};

	auto startPair = findStartPair();
	VH bridge = std::get<0>(startPair);
	VH start1 = std::get<1>(startPair);
	VH start2 = std::get<2>(startPair);

	// if we didnt find a starting point, look into other direction (if possible)
	if (!start1.is_valid() || !start2.is_valid()) {
		startPair = findStartPair();
		start1 = std::get<1>(startPair);
		start2 = std::get<2>(startPair);
	}

#ifdef BEZIER_DEBUG
	if (!start1.is_valid()) {
		m_mesh.set_color(v, { 0.6f, 0.1f, 0.45f, 1.f });
		debugCancel("invalid FIRST shortest path start vertex");
		return;
	}
	if (!start2.is_valid()) {
		m_mesh.set_color(start1, { 0.6f, 0.1f, 0.45f, 1.f });
		debugCancel("invalid SECOND shortest path start vertex");
		return;
	}
#endif
	assert(start1.is_valid());
	assert(start2.is_valid());

	std::deque<VH> subpath;

	size_t middle = 0;
	// make a queue containing all vertices of the path in correct order
	VH working = start1;
	while (working.is_valid()) {
		subpath.push_front(working);
		//std::cerr << "\t\tvertex " << working << " has pred " << pred(working) << "\n";
		working = pred(working);
		middle++;
	}
	// add middle vertex on the border
	subpath.push_back(bridge);
	middle++;

	working = start2;
	while (working.is_valid()) {
		subpath.push_back(working);
		//std::cerr << "\t\tvertex " << working << " has pred " << pred(working) << "\n";
		working = pred(working);
	}
	assert(subpath.size() >= 2);

	const Color black = { 0.f, 0.f, 0.f, 1.f };

	for (size_t i=0; i < subpath.size(); ++i) {

		const VH node = subpath[i];
		assert(i == 0 || i == subpath.size() - 1 || !vtt(node).isBorder());

		if (i > 0) {
			HH he = m_mesh.find_halfedge(path.back(), node);
			if (!he.is_valid()) he = m_mesh.find_halfedge(node, path.back());

#ifdef BEZIER_DEBUG
			if (!he.is_valid()) {
				m_mesh.set_color(node, { 0.6f, 0.1f, 0.45f, 1.f });
				m_mesh.set_color(path.back(), { 0.6f, 0.45f, 0.1f, 1.f });
				debugCancel("shortestPath: no connecting edge");
				return;
			}
#endif
			const EH edge = m_mesh.edge_handle(he);
			crossed(edge) = ctrlFace.idx();
			setColor(edge, black);
		}

		// set color and mark this as a boundary vertex
		path.push(node);
		setColor(node, black);
		vtt(node).setBorder(id_1, id_2);
	}
	path.start(id(path.front()));
	std::cerr << "\tfound path using " << path.size() << " vertices\n";
}

void VoronoiRemesh::setShortestPath(const VH vh)
{
	std::vector<Scalar> dists;
	Scalar distSum = 0., INF = std::numeric_limits<Scalar>::max();

	ID target = id(vh);
	VH node = vh, targetVertex = m_seedVerts[target], vertex;

	while (vertex != targetVertex) {

		vertex.invalidate();

		Point p = m_mesh.point(node);
		Scalar minDist = INF;

		for (auto v_it = m_mesh.cvv_begin(node); v_it != m_mesh.cvv_end(node); ++v_it) {
			Scalar d = (m_mesh.point(*v_it) - p).norm();
			if (!vtt(*v_it).isBorder() && adjToRegion(*v_it, target)) {
				if (dist(*v_it) < INF && id(*v_it) == target) {
					minDist = d;
					vertex = *v_it;
					break;
				} else if (d < minDist) {
					minDist = d;
					vertex = *v_it;
				}
			}
		}

		if (vertex.is_valid()) {
			dists.push_back(minDist);
			distSum += minDist;

			id(vertex) = target;
			pred(node) = vertex;

			node = vertex;
		} else {
			debugCancel("vertex not valid");
			return;
		}
	}

	node = vh;
	Color col = m_colors[target];
	for (int i = dists.size() - 1; i >= 0; --i) {
		dist(node) = distSum - dists[i];
		setColor(node, col);
		node = pred(node);
	}

	std::cerr << __FUNCTION__ << " found path\n";
}

bool VoronoiRemesh::addExtraSeed(FaceDijkstra &q)
{
	FH add;
	Scalar maxDist = -1.0;

	//for (FH face : m_mesh.faces()) {

	//	if (dist(face) > maxDist && !adjToSeedFace(face)) {
	//		add = face;
	//		maxDist = dist(face);
	//	}
	//}

	for (FH face : m_seeds) {

		FH f = face, next;

		Scalar maxLocal = -1.0;
		bool update;

		// find the longest path to the border in each seed
		do {
			update = false;

			for (auto f_it = m_mesh.cff_begin(f), f_e = m_mesh.cff_end(f); f_it != f_e; ++f_it) {
				if (id(*f_it) == id(face) && dist(*f_it) > maxLocal) {
					maxLocal = dist(*f_it);
					next = *f_it;
					update = true;
				}
			}
			if (update) {
				f = next;
			}

		} while (update);

		// if longer than prev longest dist and not next to a seed
		if (dist(f) > maxDist && !isSeed(f) && !adjToSeedFace(f)) {
			add = f;
			maxDist = dist(f);
		}
	}

	if (add.is_valid() && !isSeed(add) && !adjToSeedFace(add)) {
		addSeed(q, add);
		setColor(add, { 0.9f, 0.9f, 0.9f, 0.75f });
		return true;
	}
	return false;
}

void VoronoiRemesh::faceSP(FaceDijkstra & q)
{
	const auto growTiles = [&]() {
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
				const P edgeM = m_mesh.calc_edge_midpoint(edge);

				// distance to the next face
				const double update = dist(face) + (p1 - edgeM).norm() + (p2 - edgeM).norm();

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
	};

	do {
		growTiles();

		// COND 2: if more than one shared boundary per pair of tiles exists, add one of the
		// faces adj to the cut as a new seed face
		reduceCuts(q);

		if (debugCancel()) return;

		growTiles();

		// COND 3: if one vertex is adj to more than 3 regions, add one adj face as a new seed face
		reduceAdjRegions(q);

		if (debugCancel()) return;

	} while (!q.empty());
}

void VoronoiRemesh::ensureReachable(const ID id0)
{
	// ensure that each vertex in a region can actually be reached from the seed vertex
	// check all paths that cross this face
	// if a path goes from border to border and does not contain the seed vertex
	//  - then the region without the seed vertex must become its own region

	//const VH src = m_seedVerts[id0];

	//VH start;

	//bool lastWasCrossed = false;
	//for (auto h_it = m_mesh.cvoh_begin(src); h_it != m_mesh.cvoh_end(src); ++h_it) {
	//	if (lastWasCrossed && isCrossed(*h_it)) {
	//		start = m_mesh.to_vertex_handle(*h_it);
	//	}
	//	lastWasCrossed = isCrossed(*h_it);
	//}

	//if (start.is_valid()) {

	//}
}

void VoronoiRemesh::vertexSP(VertexDijkstra & q)
{
	std::set<VH> fixed;

	const auto isOkay = [&](VH from, VH to) {
		if (fixed.find(to) != fixed.end())
			return -1; // already fixed, so not okay to change

		std::set<ID> ids;

		// make sure that vertices adj to paths have the same id as the path vertex
		for (auto v_it = m_mesh.cvv_begin(to), v_e = m_mesh.cvv_end(to); v_it != v_e; ++v_it) {
			if (vtt(*v_it).isBorder() && adjToRegion(to, id(*v_it))) {
				ids.insert(id(*v_it));
			}
		}
		if (!ids.empty()) {
			if (ids.size() == 1) return *ids.begin() == id(from) ? 1 : -1;

			ID maxID;
			int maxAdj = 0;

			// if 'to' is adj to 2 regions and well as 2 path vertices
			// then we simply assign it to the region with the most faces
			// around this vertex
			for (ID id0 : ids) {
				int adj = countRegionFaces(to, id0);
				if (adj > maxAdj) {
					maxAdj = adj;
					maxID = id0;
				}
			}

			return maxID == id(from) ? 1 : -1;
		}


		/*
		 * Look at all vertices around the destination vertex and check whether this is
		 * a siuation where only 1 edge is between the two borders of a region, then the
		 * vertex is question may not be put into another region, so that we can guarantee
		 * that all vertices of the region are reached/passable.
		 */
		if (onRegionBorder(to)) {
			for (auto h_it = m_mesh.cvoh_begin(to), h_e = m_mesh.cvoh_end(to); h_it != h_e; ++h_it) {
				ID id0 = id(m_mesh.face_handle(*h_it));
				// inner edge
				if (id0 == id(m_mesh.opposite_face_handle(*h_it))) {
					VH next = m_mesh.to_vertex_handle(*h_it);
					if (onRegionBorder(next)) {
						// if ids are the same its okay but must be fixed, otherwise not okay
						return id0 == id(from) || (id(next) >= 0 && id(next) == id0) ? 1 : -1;
					}
				}
			}
		}

		return 0; // okay, must not be fixed
	};

	do {

		const VH vh = q.begin()->second;
		q.erase(q.begin());

		Point p0 = m_mesh.point(vh);

		auto end = m_mesh.cvoh_end(vh);
		for (auto he = m_mesh.cvoh_begin(vh); he != end; ++he) {
			VH vv = m_mesh.to_vertex_handle(*he);

			Scalar updateDist = dist(vh) + (p0 - m_mesh.point(vv)).norm();

			const bool adjacent = adjToRegion(vv, id(vh));
			// vertex must ...
			// - not be a border vertex
			// - have a larger distance than the new one we calculatd
			// - be adj to at least 1 face with the same id
			if (!vtt(vv).isBorder() && updateDist < dist(vv) && adjacent) {
				// edge must not be crossed, debug
				assert(he->is_valid());
				assert(!isCrossed(*he));

				dist(vv) = updateDist;
				id(vv) = id(vh);
				pred(vv) = vh;

				if (!onRegionBorder(vv)) {
					q.insert({ updateDist, vv });
				}
			}
		}

	} while (!q.empty());
}

void VoronoiRemesh::findShortestPath(const VH vh, const ID id0)
{
	VH node = vh;
	VH seed = m_seedVerts[id0];
	size_t count = 0;

	while (pred(node).is_valid() && id(pred(node)) != id0) {
		count++;
		Scalar minDist = std::numeric_limits<Scalar>::max();
		VH newPred;
		for (auto v_it = m_mesh.cvv_begin(node), v_e = m_mesh.cvv_end(node); v_it != v_e; ++v_it) {
			if (id(*v_it) == id0 && dist(*v_it) < minDist && !vtt(*v_it).isBorder()) {
				minDist = dist(*v_it);
				newPred = *v_it;
			}
		}
		assert(newPred.is_valid());
		pred(node) = newPred;
		dist(node) = dist(newPred) + (m_mesh.point(newPred) - m_mesh.point(node)).norm();
		node = newPred;
	}

	std::cerr << __FUNCTION__ << " needed " << count << " iterations" << std::endl;

}

 //////////////////////////////////////////////////////////
 // create voronoi partition
 //////////////////////////////////////////////////////////
void VoronoiRemesh::partition()
{
	// prepare
	m_seeds.clear();
	m_seedVerts.clear();
	m_ctrlVerts.clear();

	m_nvertices = 0;
	m_nedges = 0;
	m_vertexIdx = 0;

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

	faceSP(q);

#ifdef BEZIER_DEBUG
	if (debugCancel()) return;
#endif

	int iter = 2;
	// add a new seed if we still need for partitions
	while (m_seeds.size() < m_minPartition && addExtraSeed(q)) {
		faceSP(q);
		std::cerr << "partition iteration " << iter++ << std::endl;
		if (debugCancel()) return;
	}

	if (m_useColors) {
		BezierTMesh::Color boundaryColor(0.f, 0.f, 0.f, 1.f);
		for (EH edge : m_mesh.edges()) {
			const auto he = m_mesh.halfedge_handle(edge, 0);
			const auto f1 = m_mesh.face_handle(he);
			const auto f2 = m_mesh.opposite_face_handle(he);
			if (id(f1) == id(f2)) {
				setColor(edge, m_colors[id(f1)]);
			} else {
				setColor(edge, boundaryColor);
			}
		}
	}

	m_nvertices = m_mesh.n_vertices();
	m_nedges = m_mesh.n_edges();

	std::cerr << "used " << m_seeds.size() << '/' << m_mesh.n_faces();
	std::cerr << " faces as seeds" << std::endl;

	m_seedVerts.clear();
	m_seedVerts.reserve(m_seeds.size());
	for (FH face : m_seeds) {
		VH vh;
		if (onRegionBorder(face)) {
			vh = m_mesh.splitFaceBarycentric(face, true);
		} else {
			for (auto it = m_mesh.cfv_begin(face); it != m_mesh.cfv_end(face); ++it) {
				if (!onRegionBorder(*it) &&
					std::find(m_seedVerts.begin(), m_seedVerts.end(), *it) == m_seedVerts.end()
				) {
					vh = *it;
					break;
				}
			}
		}
		if (!vh.is_valid()) {
			vh = m_mesh.splitFaceBarycentric(face, true);
		}

		id(vh) = m_seedVerts.size();
		dist(vh) = 0.0;
		m_seedVerts.push_back(vh);
	}

	m_ctrlVerts.clear();
	m_ctrlVerts.resize(m_seeds.size());
	// add vertices of seed faces
	for (VH vertex : m_seedVerts) {
		m_ctrlVerts[id(vertex)] = m_ctrl.add_vertex(m_mesh.point(vertex));
	}

	// do preventive edge splitting wherever there are two regions only touching at 1 edge
	preventiveEdgeSplits();

	// mark all edges as not crossed
	for (auto e : m_mesh.edges()) {
		crossed(e) = -1;
	}

	splitClosedPaths();

#ifdef BEZIER_DEBUG
	if (debugCancel()) return;
#endif

	// calculate shortest distances for all vertices
	vertexDijkstra();

	debugCancel();
}

void VoronoiRemesh::vertexDijkstra(const ID id0, const ID id1)
{
	std::cerr << __FUNCTION__ << " (" << id0 << ", " << id1 << ") \n";
	VertexDijkstra q;

	const bool update = id0 >= 0 && id1 >= 0;
	const Scalar INF = std::numeric_limits<Scalar>::max();

	if (update) {

		const VH start0 = m_seedVerts[id0], start1 = m_seedVerts[id1];
		// called after having found a path (avoid crossing paths)
		for (VH vh : m_mesh.vertices()) {
			if (!isSeedVertex(vh) && (adjToRegion(vh, id0) || adjToRegion(vh, id1))) {
				dist(vh) = INF;
				pred(vh) = VH();
			}
		}
		Point p = m_mesh.point(start0);
		// add all neighbors of the start vertex to the queue (if they are not on a border)
		for (auto he = m_mesh.cvoh_begin(start0); he != m_mesh.cvoh_end(start0); ++he) {
			VH vv = m_mesh.to_vertex_handle(*he);

			if (adjToRegion(vv, id0) && !vtt(vv).isBorder() && !isSeedVertex(vv)) {
				dist(vv) = (p - m_mesh.point(vv)).norm();
				pred(vv) = start0;
				q.insert({ dist(vv), vv });
			}
		}
		p = m_mesh.point(start1);
		// add all neighbors of the start vertex to the queue (if they are not on a border)
		for (auto he = m_mesh.cvoh_begin(start1); he != m_mesh.cvoh_end(start1); ++he) {
			VH vv = m_mesh.to_vertex_handle(*he);

			if (adjToRegion(vv, id1) && !vtt(vv).isBorder() && !isSeedVertex(vv)) {
				dist(vv) = (p - m_mesh.point(vv)).norm();
				pred(vv) = start1;
				q.insert({ dist(vv), vv });
			}
		}
	} else {
		// first time calling this
		for (VH vh : m_mesh.vertices()) {
			id(vh) = -1;
			dist(vh) = INF;
			pred(vh) = VH();
		}

		for (size_t i = 0; i < m_seedVerts.size(); ++i) {
			dist(m_seedVerts[i]) = 0.0;
			id(m_seedVerts[i]) = i;
			q.insert({ 0.0, m_seedVerts[i] });
		}
	}

	assert(!q.empty());

	vertexSP(q);

	int count = 0;

	for (VH vh : m_mesh.vertices()) {

		if (id(vh) < 0) {
			VH minPred = minPredecessor(vh);
			if (minPred.is_valid()) {
				id(vh) = id(minPred);
				dist(vh) = dist(minPred) + (m_mesh.point(minPred) - m_mesh.point(vh)).norm();
				pred(vh) = minPred;
				assert(!vtt(minPred).isBorder());
				assert(dist(minPred) < INF);
#ifdef BEZIER_DEBUG
			} else {
				m_mesh.set_color(vh, { 0.f, 0.f, 0.f, 0.f });
				debugCancel("had no id, cannot find pred");
				continue;
#endif
			}
		}
		assert(id(vh) >= 0);

#ifdef BEZIER_DEBUG
		if (pred(vh).is_valid() && !m_mesh.find_halfedge(pred(vh), vh).is_valid()) {
			m_mesh.set_color(vh, { 1.f, 1.f, 1.f, 1.f });
			m_mesh.set_color(pred(vh), { 1.f, 1.f, 1.f, 1.f });
			debugCancel("no edge between vertex and pred");
			return;
		}
#endif
		assert(adjToRegion(vh, id(vh)));

		bool noNeighbors = countAdjRegions(vh) == 1;
		if (noNeighbors) count++;
		border(vh, noNeighbors);

		assert(dist(vh) < INF || vtt(vh).isBorder());

		if (!vtt(vh).isBorder()) {
			setColor(vh, m_colors[id(vh)]);
		}
	}
	std::cerr << "inner: " << count << ", outer: " << m_mesh.n_vertices() - count << std::endl;
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

		setColor(face, m_colors[face.idx()]);
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
		for (auto vf = m_mesh.cvf_ccwbegin(v); vf != m_mesh.cvf_ccwend(v); ++vf) {
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

			std::cerr << "\n\tVoronoi Vertex (" << seedIDs[0] << ", " << seedIDs[1];
			std::cerr << ", " << seedIDs[2] << ")\n";

			assert(seedIDs.size() == 3);

			// add the face
			const FH fh = m_ctrl.add_face(points[0], points[1], points[2], true);

#ifdef BEZIER_DEBUG
			if (!fh.is_valid()) {
				m_mesh.set_color(m_seedVerts[seedIDs[0]], { 0.5f, 1.f, 0.5f, 0.f });
				m_mesh.set_color(m_seedVerts[seedIDs[1]], { 0.5f, 1.f, 0.5f, 0.f });
				m_mesh.set_color(m_seedVerts[seedIDs[2]], { 0.5f, 1.f, 0.5f, 0.f });
				m_mesh.set_color(v, { 1.f, 1.f, 1.f, 0.f });
				debugCancel("face not valid");
				return false;
			}
#endif

			for (size_t j = 0; j < 3; ++j) {
				const ID id1 = seedIDs[j];
				const ID id2 = seedIDs[j < 2 ? j + 1 : 0];
				// tell face which regions it was generated from (to later find boundary)
				ttv(fh)[j] = id1;

				// collect border halfedges between these two regions
				if (!ShortestPath::has(id1, id2)) {
					ShortestPath bp = ShortestPath(id1, id2);
					shortestPath(v, id1, id2, fh, bp);
					// add path to path collection
					ShortestPath::path(bp);

					if (debugCancel())
						return false;

					ttv(fh).addBoundarySize(bp.size());
					// make sure there are no impassable edges
					// (i.e. edge connected to 2 vertices adj to border edges)
					splitClosedPaths();

					if (debugCancel())
						return false;

					//ensureReachable(id1);
					//ensureReachable(id2);

					// recalculate shortest paths for these two regions
					vertexDijkstra(id1, id2);
					//vertexDijkstra();

					if (debugCancel())
						return false;

				} else {
					const ShortestPath &bp = ShortestPath::path(id1, id2);
					ttv(fh).addBoundarySize(bp.size());
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

		for (FH face : m_ctrl.faces()) {
			m_ctrl.recalculateCPs(face);
		}
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
	}
}

//////////////////////////////////////////////////////////
// parameterization (harmonic map) + surface fitting
//////////////////////////////////////////////////////////
void VoronoiRemesh::fitting()
{
	VoronoiParametrization param(m_mesh, m_ctrl, m_vtt, m_ttv, m_pred);
	VoronoiFitting fit(m_mesh, m_ctrl, m_ttv, m_vtt);

	fit.degree(m_mesh.degree());

	for (FH face : m_ctrl.faces()) {
		if (!param.solveLocal(face)) {
			std::cerr << "parametrization for face " << face << " failed\n";
			debugCancel("parametrization failed");
			return;
		}
		std::cerr << "\nfinished parametrization for face " << face << "\n";

		if (!fit.solveLocal(face)) {
			std::cerr << "fitting for face " << face << " failed\n";
			debugCancel("fitting failed");
			return;
		}
		std::cerr << "\nfinished fitting for face " << face << "\n";
	}

	m_ctrl.update_normals();
}

void VoronoiRemesh::smooth()
{
	// average edge control points so we don't have any holes
	for (EH edge : m_ctrl.edges()) {
		m_ctrl.interpolateEdgeControlPoints(edge, true);
	}

	if (m_untwist) {
		for (FH face : m_ctrl.faces()) {
			m_ctrl.untwistControlPoints(face);
		}
	}
}

void VoronoiRemesh::remesh()
{
	if (!m_useBaseMesh) partition();
	else prepareFromBaseMesh();

	// stop if cancel was requested
	if (debugCancel())
		return;

	if (!m_useBaseMesh) dualize();
	else makeShortestPaths();

	// stop if cancel was requested
	if (debugCancel())
		return;

	fitting();

	// stop if cancel was requested
	if (debugCancel())
		return;

	// replace original mesh
	if (m_copy)
		copyMesh(m_ctrl, m_mesh);

	m_mesh.garbage_collection();

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

void VoronoiRemesh::reduceCuts(FaceDijkstra &q)
{
	std::cerr << "----------------------------------------------\nreducing cuts\n";

	std::vector<std::vector<std::vector<FH>>> cuts;
	cuts.reserve(m_seeds.size());
	for (int i = 0; i < m_seeds.size(); ++i) {
		std::vector<std::vector<FH>> vec;
		vec.reserve(m_seeds.size() - i - 1);
		for (int j = 0; j < m_seeds.size()-i-1; ++j) {
			vec.push_back(std::vector<FH>());
		}
		cuts.push_back(vec);
	}

	const auto marked = [&](const EH eh) {
		return m_mesh.status(eh).tagged();
	};

	const auto mark = [&](const EH eh, const bool value) {
		return m_mesh.status(eh).set_tagged(value);
	};

	const auto nextCutEdge = [&](const HH h, ID id1, ID id2) {
		VH v = m_mesh.to_vertex_handle(h);
		for (auto voh = m_mesh.voh_begin(v); voh != m_mesh.voh_end(v); ++voh) {
			auto e = m_mesh.edge_handle(*voh);
			auto id11 = id(m_mesh.face_handle(*voh));
			auto id22 = id(m_mesh.opposite_face_handle(*voh));
			if (!marked(e) && (id11 == id1 && id22 == id2 ||
				id11 == id2 && id22 == id1)) {
				return *voh;
			}
		}
		return HH();
	};

	const auto stuff = [&](const FH fh) {
		for (auto v_it = m_mesh.cfv_begin(fh), v_e = m_mesh.cfv_end(fh); v_it != v_e; ++v_it) {
			for (auto f_it = m_mesh.cvf_begin(*v_it), f_e = m_mesh.cvf_end(*v_it); f_it != f_e; ++f_it) {
				if (isSeed(*f_it)) return true;
			}
		}
		return false;
	};

	for (EH edge : m_mesh.edges()) {
		HH forward = m_mesh.halfedge_handle(edge, 0);
		HH backward = m_mesh.halfedge_handle(edge, 1);

		FH face, backup;
		ID id1 = id(m_mesh.face_handle(forward));
		ID id2 = id(m_mesh.face_handle(backward));

		if (!marked(edge)) {
			// check id
			if (id1 >= cuts.size() || id2 >= cuts.size() ||
				id1 == id2 || id1 == -1 || id2 == -1) continue;

			mark(edge, true);

			Scalar maxDist = -1.0, maxDistBackup = -1.0;

			while (forward.is_valid() || backward.is_valid()) {

				if (forward.is_valid()) {

					//VH to = m_mesh.to_vertex_handle(forward);
					std::array<FH, 2> faces{ {
							m_mesh.face_handle(forward),
							m_mesh.opposite_face_handle(forward)
					} };
					//for (auto f_it = m_mesh.cvf_begin(to), f_e = m_mesh.cvf_end(to);
					//	f_it != f_e; ++f_it
					//) {
					for (FH f_it : faces) {
						if (dist(f_it) > maxDistBackup && !isSeed(f_it)) {
							backup = f_it;
							maxDistBackup = dist(backup);
							// better case
							if (maxDistBackup > maxDist && !stuff(f_it)) {
								face = f_it;
								maxDist = maxDistBackup;
							}
						}
					}
					forward = nextCutEdge(forward, id1, id2);
					if (forward.is_valid()) {
						mark(m_mesh.edge_handle(forward), true);
					}
				}

				if (backward.is_valid()) {

					//VH to = m_mesh.to_vertex_handle(backward);
					std::array<FH, 2> faces{ {
							m_mesh.face_handle(backward),
							m_mesh.opposite_face_handle(backward)
					} };
					//for (auto f_it = m_mesh.cvf_begin(to), f_e = m_mesh.cvf_end(to);
					//	f_it != f_e; ++f_it
					//) {
					for (FH f_it : faces) {
						if (dist(f_it) > maxDistBackup && !isSeed(f_it)) {
							backup = f_it;
							maxDistBackup = dist(backup);
							// better case
							if (maxDistBackup > maxDist && !stuff(f_it)) {
								face = f_it;
								maxDist = maxDistBackup;
							}
						}
					}
					backward = nextCutEdge(backward, id2, id1);
					if (backward.is_valid()) {
						mark(m_mesh.edge_handle(backward), true);
					}
				}
			};

			// deal with cut if necessary
			if (id1 < id2) {
				cuts[id1][id2 - id1 - 1].push_back(face.is_valid() ? face : backup);
			} else {
				cuts[id2][id1 - id2 - 1].push_back(face.is_valid() ? face : backup);
			}
		}
	}

	size_t bef = m_seeds.size();

	for (size_t i = 0; i < cuts.size(); ++i) {
		for (size_t j = 0; j < cuts[i].size(); ++j) {

			if (cuts[i][j].size() <= 1) continue;

			FH last;
			int k = (int)cuts[i][j].size() - 1;
			int count = 0;

			std::cerr << "checking cuts for regions " << i << " and " << i + j + 1 << '\n';
			// look at all cut faces
			while (k >= 0) {

				last = cuts[i][j][k];
				std::cerr << "\tvalid ? " << last.is_valid() << '\n';
				std::cerr << "\tseed ? " << isSeed(last) << '\n';
				if (count < cuts[i][j].size() - 1 && !isSeed(last)) {
					count++;
					addSeed(q, last);
				}
				k--;
			}
#ifdef BEZIER_DEBUG
			if (count != cuts[i][j].size() - 1) {
				for (FH f : cuts[i][j]) {
					m_mesh.set_color(f, { 1., 0.5, 0.5, 1.f });
				}
				debugCancel("did not add enough new seeds for cut");
				return;
			}
#endif
			//std::cerr << "regions (" << i << "," << j+i << ") had " << cuts[i][j].size();
			//std::cerr << " shared cuts ... added " << count << " new seeds\n";
		}
	}

	std::cerr << "added " << m_seeds.size()-bef << " new seeds\n";
	std::cerr << "----------------------------------------------\n";

	// remove marks from all edges
	std::for_each(m_mesh.edges_begin(), m_mesh.edges_end(), [&](const EH eh) {
		mark(eh, false);
	});
}

void VoronoiRemesh::reduceAdjRegions(FaceDijkstra & q)
{
	std::cerr << "\nreducing adj regions\n";
	const auto adjRegions = [&](VH vh) {
		std::set<ID> r;
		for (auto f = m_mesh.cvf_begin(vh); f != m_mesh.cvf_end(vh); ++f) {
			if (id(*f) >= 0) r.insert(id(*f));
		}
		return r;
	};

	for (VH v : m_mesh.vertices()) {
		if (countAdjRegions(v) > 3) {

			Scalar maxDist = 0.0;
			FH target;
			int before = m_seeds.size();

			for (auto f = m_mesh.cvf_begin(v); f != m_mesh.cvf_end(v); ++f) {
				if (!isSeed(*f) && dist(*f) > maxDist) {
					target = *f;
					maxDist = dist(*f);
					break;
				}
			}
#ifdef BEZIER_DEBUG
			if (!target.is_valid()) {
				m_mesh.set_color(v, { 1.f, 0.5f, 0.5f, 1.f });
				debugCancel("too many seeds around vertex");
				return;
			} else {
#endif
				addSeed(q, target);
				assert(m_seeds.size() > before);
#ifdef BEZIER_DEBUG
			}
#endif
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
