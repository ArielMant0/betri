#include "VoronoiRemesh.hh"

#include "VoronoiParametrization.hh"

#include <queue>
#include <unordered_set>
#include <map>
#include <random>
#include <fstream>

#include <OpenFlipper/BasePlugin/PluginFunctions.hh>

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

	ShortestPath::clear();
}

void VoronoiRemesh::splitLongEdges()
{
	Scalar min = std::numeric_limits<Scalar>::max(), avg = 0.0, max = 0.0;

	for (EH eh : m_mesh.edges()) {
		Scalar len = m_mesh.calc_edge_length(eh);
		if (len < min) min = len;
		if (len > max) max = len;
		avg += len;
	}
	avg /= m_mesh.n_edges();

	int count = 0;
	const Scalar threshold = (4.0 / 3.0) * avg;

	std::array<FH, 4> update;

	for (auto eh = m_mesh.edges_begin(), e_e = m_mesh.edges_end(); eh != e_e; ++eh) {

		Scalar len = m_mesh.calc_edge_length(*eh);
		if (len > threshold) {

			count++;
			HH hh = m_mesh.halfedge_handle(*eh, 0);
			update[0] = m_mesh.face_handle(hh);
			update[1] = m_mesh.opposite_face_handle(hh);

			size_t before = m_mesh.n_faces();
			m_mesh.split_edge(*eh, m_mesh.add_vertex(m_mesh.calc_edge_midpoint(*eh)));
			update[2] = m_mesh.face_handle(before);
			update[3] = m_mesh.face_handle(before+1);

			for (FH face : update) {
				m_mesh.recalculateCPs(face);
			}
		}
	}

	std::cerr << "split " << count << " edges\n";
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
	for (FH fh : m_ctrl.faces()) {
		if (!assigned[fh]) {
			setColor(fh, { 1.f, 1.f, 1.f, 1.f });
			debugCancel("face not assigned");
		}
	}

	for (VH vh : m_mesh.vertices()) {
		if (!vtt(vh).isBorder() && !vtt(vh).face.is_valid()) {
			setColor(vh, { 1.f, 1.f, 1.f, 1.f });
			debugCancel("vertex not assigned");
		}
	}
#endif
}

void VoronoiRemesh::splitClosedPaths(std::set<ID> only)
{
	std::cerr << __FUNCTION__ << " START\n";

	size_t nedges = m_mesh.n_edges();
	size_t count = 0, count2 = 0, count3 = 0;

	const auto valid = [&](EH edge) {

		if (only.empty()) return true;

		HH hh = m_mesh.halfedge_handle(edge, 0);
		VH v0 = m_mesh.from_vertex_handle(hh);
		VH v1 = m_mesh.to_vertex_handle(hh);

		return only.find(id(v0)) != only.end() &&
			only.find(id(v1)) != only.end();
	};

	for (size_t i = 0; i < m_mesh.n_edges(); ++i) {
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
			}else if (!vtt(to).isBorder() && !vtt(from).isBorder() &&
				onRegionBorder(to) && onRegionBorder(from) &&
				id(f1) == id(f2) && valid(edge)
			) {
				// when we have 2 vertices, connected by and edge, both of which
				// lie on a region border and both adj faces have the same region id
				fixCrossing(f1, f2);
				count3++;

				VH newNode = m_mesh.vertex_handle(m_mesh.n_vertices() - 1);
				id(newNode) = id(f1);

			} else if (vtt(to).isBorder() && !vtt(from).isBorder() &&
				onRegionBorder(from) && id(f1) == id(f2)
			) {
				fixCrossing(f1, f2);
				count3++;

				VH newNode = m_mesh.vertex_handle(m_mesh.n_vertices() - 1);
				id(newNode) = id(f1);

			} else if (!vtt(to).isBorder() && vtt(from).isBorder() &&
				onRegionBorder(to) && id(f1) == id(f2)
			) {
				fixCrossing(f1, f2);
				count3++;

				VH newNode = m_mesh.vertex_handle(m_mesh.n_vertices() - 1);
				id(newNode) = id(f1);
			}
		}
	}

	std::cerr << __FUNCTION__ << ": split " << count << " edges (" << count2;
	std::cerr << ", " << count3 << ")\n";
}

void VoronoiRemesh::fixPredecessor(const FH fh, const bool rewrite)
{
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
	if (useColors()) {
		setColor(m_mesh.edge_handle(nedge), m_mesh.color(commonEdge));
	}
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
		setColor(v, { 0.6f, 0.1f, 0.45f, 1.f });
		debugCancel("invalid FIRST shortest path start vertex");
		return;
	}
	if (!start2.is_valid()) {
		setColor(start1, { 0.6f, 0.1f, 0.45f, 1.f });
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
				setColor(node, { 0.6f, 0.1f, 0.45f, 1.f });
				setColor(path.back(), { 0.6f, 0.45f, 0.1f, 1.f });
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

bool VoronoiRemesh::addExtraSeed(FaceDijkstra &q)
{
	FH add;
	Scalar maxDist = -1.0;

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

void VoronoiRemesh::growTiles(FaceDijkstra &q)
{
	int count = 0, added = 0;
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

				// COND 1: if not homeomorphic to disk add face as new seed
				if (!homeomorphicDisk(f, m_mesh.opposite_halfedge_handle(*he), id(face))) {

					bool okay = true;
					for (auto v_it = m_mesh.cfv_begin(f), v_e = m_mesh.cfv_end(f);
						v_it != v_e; ++v_it
					) {
						if (countAdjRegions(*v_it) > 2) {
							okay = false;
						}
					}

					if (okay) {
						addSeed(q, f);
						stop = true;
						count++;
					}
					break;
				} else {
					grow(q, f, face, update);
					added++;
				}
			}
		}
	}
	std::cerr << "disk topology added " << count << " seeds\n";
	std::cerr << "face dijkstra updated " << added << " faces\n";
}


bool VoronoiRemesh::faceSP(FaceDijkstra & q, const bool stepwise)
{
	do {
		// grow regions until whole mesh is covered
		growTiles(q);

		// COND 2: if more than one shared boundary per pair of tiles exists, add one of the
		// faces adj to the cut as a new seed face
		reduceCuts(q);

		if (debugCancel()) return false;

		growTiles(q);

		// COND 3: if one vertex is adj to more than 3 regions, add one adj face as a new seed face
		reduceAdjRegions(q);

		if (debugCancel()) return false;

	} while (!stepwise && !q.empty());

	return q.empty();
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

bool VoronoiRemesh::partitionIsValid()
{
	int count1 = 0, count2 = 0;

	for (VH vh : m_mesh.vertices()) {
		if (countAdjRegions(vh) > 3) {
			count1++;
		}
	}

	std::vector<std::vector<std::vector<FH>>> cuts;
	findRegionCuts(cuts);

	size_t bef = m_seeds.size();

	for (size_t i = 0; i < cuts.size(); ++i) {
		for (size_t j = 0; j < cuts[i].size(); ++j) {
			if (cuts[i][j].size() > 1) {
				count2++;
			}
		}
	}

	if (count1 > 0) {
		std::cerr << count1 << " vertices have more than 3 adj regions" << std::endl;
	}
	if (count2 > 0) {
		std::cerr << count2 << " pairs of regions share more than 1 cut" << std::endl;
	}

	return count1 == 0 && count2 == 0;
}

 //////////////////////////////////////////////////////////
 // create voronoi partition
 //////////////////////////////////////////////////////////
bool VoronoiRemesh::partition(const bool stepwise, bool &done)
{
	if (m_splits && m_q.empty()) {
		splitLongEdges();
	}

	int i = 0;
	bool success = false;
	constexpr int MAX_ATTEMPTS = 3;

	// attempt max MAX_ATTEMPTS times before we give up #hacky
	do {
		if (!stepwise || m_q.empty()) {
			reset();

			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_int_distribution<size_t> dis(0u, m_mesh.n_faces() - 1);
			// find two random seeds
			while (m_seeds.size() < 2) {
				FH next = m_mesh.face_handle(dis(gen));
				if (!isSeed(next) && !adjToSeedFace(next)) {
					addSeed(m_q, next);
				}
			}
		}
		assert(!m_q.empty());

		std::cerr << "\nPartition: Iteration " << i << '\n' << std::endl;
		std::cerr << std::endl;

		if (calcPartition(stepwise, done)) {
			// partition was successfull
			success = true;
			break;
		}

	} while (!stepwise && ++i < MAX_ATTEMPTS);

	return success;
}
void VoronoiRemesh::reset()
{
	// clear shortest paths
	ShortestPath::clear();

	// make sure we have the color properties we need
	if (useColors()) {
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

	// prepare (reset)
	m_seeds.clear();
	m_colors.clear();
	m_seedVerts.clear();
	m_ctrlVerts.clear();

	m_nvertices = 0;
	m_nedges = 0;
	m_vertexIdx = 0;

	m_errorMsg = "";
	m_debugCancel = false;

	const double INF = std::numeric_limits<double>::max();
	// initialize face properties
	for (FH face : m_mesh.faces()) {
		pred(face) = FH();
		id(face) = -1;
		dist(face) = INF;
	}
}

bool VoronoiRemesh::calcPartition(const bool stepwise, bool &done)
{
	done = faceSP(m_q, stepwise);

	if (debugCancel()) {
		return false;
	}

	if ((!stepwise || done) && !partitionIsValid()) {
		std::cerr << "INVALID PARTITION\n";
		return false;
	}

	if (stepwise && !done) {
		return true;
	}

	int iter = 0;
	// add a new seed if we still need more partitions
	while (!stepwise && m_seeds.size() < minPartition() && addExtraSeed(m_q)) {
		faceSP(m_q, false);
		if (debugCancel()) return false;
	}

	std::cerr << "used " << m_seeds.size() << '/' << m_mesh.n_faces();
	std::cerr << " faces as seeds" << std::endl;

	if (useColors()) {
		BezierTMesh::Color boundaryColor(0.f, 0.f, 0.f, 1.f);
		for (EH edge : m_mesh.edges()) {
			const auto he = m_mesh.halfedge_handle(edge, 0);
			const auto f1 = m_mesh.face_handle(he);
			const auto f2 = m_mesh.opposite_face_handle(he);
			if (id(f1) == id(f2)) {
				setColor(edge, getRegionColor(f1));
			} else {
				setColor(edge, boundaryColor);
			}
		}
	}

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

	splitClosedPaths(std::set<ID>());

	m_nvertices = m_mesh.n_vertices();
	m_nedges = m_mesh.n_edges();

	// calculate shortest distances for all vertices
	vertexDijkstra();

	return true;
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
			VH minPred = minPredecessor(vh, true);
			if (minPred.is_valid()) {
				id(vh) = id(minPred);
				dist(vh) = dist(minPred) + (m_mesh.point(minPred) - m_mesh.point(vh)).norm();
				pred(vh) = minPred;
				assert(!vtt(minPred).isBorder());
				assert(dist(minPred) < INF);
#ifdef BEZIER_DEBUG
			} else {
				setColor(vh, { 0.f, 0.f, 0.f, 1.f });
				debugCancel("had no id, cannot find pred");
				continue;
#endif
			}
		}
		assert(id(vh) >= 0);

#ifdef BEZIER_DEBUG
		if (pred(vh).is_valid() && !m_mesh.find_halfedge(pred(vh), vh).is_valid()) {
			setColor(vh, { 1.f, 1.f, 1.f, 1.f });
			setColor(pred(vh), { 1.f, 1.f, 1.f, 1.f });
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
			setColor(vh, getRegionColor(vh));
		}
	}
	std::cerr << "inner: " << count << ", outer: " << m_mesh.n_vertices() - count << std::endl;
}

//////////////////////////////////////////////////////////
// create base mesh
//////////////////////////////////////////////////////////
bool VoronoiRemesh::dualize(bool &done, bool steps)
{
	bool success = dualize(steps);
	done = m_vertexIdx == m_mesh.n_vertices();

	return success;
}

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

			if (!fh.is_valid()) {
				setColor(m_seedVerts[seedIDs[0]], { 0.5f, 1.f, 0.5f, 0.f });
				setColor(m_seedVerts[seedIDs[1]], { 0.5f, 1.f, 0.5f, 0.f });
				setColor(m_seedVerts[seedIDs[2]], { 0.5f, 1.f, 0.5f, 0.f });
				setColor(v, { 1.f, 1.f, 1.f, 0.f });
				debugCancel("face not valid");
				return false;
			}

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
					splitClosedPaths({ { id1, id2 } });

					if (debugCancel())
						return false;

					// recalculate shortest paths for these two regions
					vertexDijkstra(id1, id2);

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
		// perform assignment
		assignInnerVertices();

		if (debugCancel()) return false;
	}

	return true;
}

//////////////////////////////////////////////////////////
// parameterization (harmonic map) + surface fitting
//////////////////////////////////////////////////////////
bool VoronoiRemesh::fitting()
{
	size_t degree = m_mesh.degree();
	size_t cpNums = pointsFromDegree(degree);

	VoronoiParametrization param(m_mesh, m_ctrl, m_paramWeights, m_vtt, m_ttv, m_pred);
	VoronoiFitting fit(m_mesh, m_ctrl, m_ttv, m_vtt, fittingSamples());
	fit.solver(fittingSolver());
	fit.degree(degree);

	for (FH face : m_ctrl.faces()) {
		m_ctrl.setControlPointsFromCorners(face, cpNums);
	}

	for (FH face : m_ctrl.faces()) {

		if (!param.solveLocal(face)) {
			std::cerr << "parametrization for face " << face << " failed\n";
			debugCancel("parametrization failed");
			return false;
		}

		if (!fit.solveLocal(face, m_interpolate)) {
			std::cerr << "fitting for face " << face << " failed\n";
			debugCancel("fitting failed");
			return false;
		}
	}

	if (m_interpolate) {
		// average edge control points so we don't have any holes
		for (EH edge : m_ctrl.edges()) {
			m_ctrl.interpolateEdgeControlPoints(edge, true);
		}
	}

	// replace original mesh
	if (m_overwrite) {
		copyMesh(m_ctrl, m_mesh);
		// update normals
		m_mesh.garbage_collection();
		m_mesh.update_normals();
	} else {
		// update normals
		m_ctrl.update_normals();
	}

	return true;
}

bool VoronoiRemesh::remesh()
{
	bool done;

	// create partition
	if (!partition(false, done))
		return false;

	// dualize partition
	if (!dualize(false))
		return false;

	// do parametrization and fittin
	if(!fitting())
		return false;

	std::cerr << "----------- DONE -----------" << std::endl;

	return true;
}

void VoronoiRemesh::findRegionCuts(std::vector<std::vector<std::vector<FH>>>& cuts)
{
	cuts.clear();
	cuts.reserve(m_seeds.size());
	for (int i = 0; i < m_seeds.size(); ++i) {
		std::vector<std::vector<FH>> vec;
		vec.reserve(m_seeds.size() - i - 1);
		for (int j = 0; j < m_seeds.size() - i - 1; ++j) {
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

	const auto hasSeedNeighbor = [&](const FH fh) {
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

					std::array<FH, 2> faces{ {
							m_mesh.face_handle(forward),
							m_mesh.opposite_face_handle(forward)
					} };
					for (FH f_it : faces) {
						if (dist(f_it) > maxDistBackup && !isSeed(f_it)) {
							backup = f_it;
							maxDistBackup = dist(backup);
							// better case
							if (maxDistBackup > maxDist && !hasSeedNeighbor(f_it)) {
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

					std::array<FH, 2> faces{ {
							m_mesh.face_handle(backward),
							m_mesh.opposite_face_handle(backward)
					} };
					for (FH f_it : faces) {
						if (dist(f_it) > maxDistBackup && !isSeed(f_it)) {
							backup = f_it;
							maxDistBackup = dist(backup);
							// better case
							if (maxDistBackup > maxDist && !hasSeedNeighbor(f_it)) {
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

	// remove marks from all edges
	std::for_each(m_mesh.edges_begin(), m_mesh.edges_end(), [&](const EH eh) {
		mark(eh, false);
	});
}

void VoronoiRemesh::reduceCuts(FaceDijkstra &q)
{
	std::cerr << "----------------------------------------------\nreducing cuts\n";

	std::vector<std::vector<std::vector<FH>>> cuts;
	findRegionCuts(cuts);

	size_t bef = m_seeds.size();

	for (size_t i = 0; i < cuts.size(); ++i) {
		for (size_t j = 0; j < cuts[i].size(); ++j) {

			if (cuts[i][j].size() <= 1) continue;

			FH last;
			int k = (int)cuts[i][j].size() - 1;
			int count = 0;

			// look at 1 cut face
			while (k >= 0 && count == 0) {

				last = cuts[i][j][k];

				if (last.is_valid() && count < cuts[i][j].size() - 1 &&
					!isSeed(last)) {
					count++;
					addSeed(q, last);
					//return; // TODO: remove?
				}
				k--;
			}
		}
	}

	std::cerr << "added " << m_seeds.size()-bef << " new seeds\n";
	std::cerr << "----------------------------------------------\n";
}

void VoronoiRemesh::reduceAdjRegions(FaceDijkstra & q)
{
	std::cerr << "\nreducing adj regions\n";

	constexpr Scalar INF = std::numeric_limits<Scalar>::max();

	const auto resetRegion = [&](ID &region) {

		FH newSeed;
		Scalar max = 0.0;

		for (FH face : m_mesh.faces()) {
			if (id(face) == region) {

				if (dist(face) > max) {
					max = dist(face);
					newSeed = face;
				}

				id(face) = -1;
				dist(face) = INF;
				pred(face) = FH();
			}
		}

		if (newSeed.is_valid()) {
			id(newSeed) = region;
			dist(newSeed) = 0.0;
			m_seeds[region] = newSeed;

			q.insert({ 0.0, newSeed });
		} else {

			newSeed = m_seeds[region];

			m_seeds.erase(m_seeds.begin() + region);
			m_colors.erase(m_colors.begin() + region);

			for (FH face : m_mesh.faces()) {
				if (id(face) > region) {
					id(face) = id(face) - 1;
				}
			}

			FH newPred;
			Scalar min = INF;
			Point p = m_mesh.calc_face_centroid(newSeed);

			for (auto h = m_mesh.cfh_begin(newSeed); h != m_mesh.cfh_end(newSeed); ++h) {

				FH ff = m_mesh.opposite_face_handle(*h);
				Point mid = m_mesh.calc_edge_midpoint(*h);

				Scalar d = (p-mid).norm() + (mid-m_mesh.calc_face_centroid(ff)).norm();
				if (d < min) {
					min = d;
					newPred = ff;
				}
			}

			assert(newPred.is_valid());

			id(newSeed) = id(newPred);
			dist(newSeed) = min;
			pred(newSeed) = newPred;
		}
	};


	int count = 0;

	for (int i = 0; i < m_mesh.n_vertices(); ++i) {

		VH v = m_mesh.vertex_handle(i);

		int regionCount = countAdjRegions(v);

		if (regionCount > 3) {

			FH target;

			for (auto f = m_mesh.cvf_begin(v); f != m_mesh.cvf_end(v); ++f) {
				if (!isSeed(*f)) {
					target = *f;
					break;
				}
			}

			if (!target.is_valid()) {
				setColor(v, { 1.f, 0.5f, 0.5f, 1.f });
				std::cerr << " - already added " << count << " seeds -" << std::endl;
				debugCancel("too many seeds around vertex");
				return;
			} else {
				addSeed(q, target);
				count++;
			}
		}
	}

	std::cerr << "\tadded " << count << " seeds\n";
}

void VoronoiRemesh::copyMesh(BezierTMesh & src, BezierTMesh & dest)
{
	dest.clean_keep_reservation();

	auto vs = OpenMesh::makeTemporaryProperty<VH, VH>(dest);

	bool hasVColors = src.has_vertex_colors();
	if (hasVColors && !dest.has_vertex_colors()) {
		dest.request_vertex_colors();
	}
	bool hasFColors = src.has_face_colors();
	if (hasFColors && !dest.has_face_colors()) {
		dest.request_face_colors();
	}

	// works but seems really stupid
	for (const VH v : src.vertices()) {
		vs[v] = dest.add_vertex_dirty(src.point(v));
		// set color
		if (hasVColors) {
			dest.set_color(vs[v], src.color(v));
		}
	}

	for (const FH f : src.faces()) {
		std::vector<VH> verts;
		for (auto fv = src.cfv_begin(f); fv != src.cfv_end(f); ++fv) {
			verts.push_back(vs[*fv]);
		}
		const FaceHandle fh = dest.add_face(verts);
		dest.data(fh).points(src.data(f).points());
		// set color
		if (hasVColors) {
			dest.set_color(fh, src.color(f));
		}
	}
}

}
