#include "BezierTMesh.hh"

#include <iostream>

using FaceHandle = BezierTMesh::FaceHandle;

// TODO:: make it work for variable degree
void BezierTMesh::addCPsToFace(FaceHandle &f, unsigned int degree)
{
	auto v_it = cfv_begin(f);
	auto p1 = point(*v_it); v_it++;
	auto p2 = point(*v_it); v_it++;
	auto p3 = point(*v_it);

	auto bt = data(f);
	bt.degree(degree);
	bt.addPoint(p1);
	bt.addPoint(p1 * 0.5f + p2 * 0.5f);
	bt.addPoint(p2);
	bt.addPoint(p2 * 0.5f + p3 * 0.5f);
	bt.addPoint(p3);
	bt.addPoint(p3 * 0.5f + p1 * 0.5f);
}

void BezierTMesh::addCPsToFace(const FaceHandle &f, unsigned int degree)
{
	auto v_it = cfv_begin(f);
	auto p1 = point(*v_it); v_it++;
	auto p2 = point(*v_it); v_it++;
	auto p3 = point(*v_it);

	auto bt = data(f);
	bt.degree(degree);
	bt.addPoint(p1);
	bt.addPoint(p1 * 0.5f + p2 * 0.5f);
	bt.addPoint(p2);
	bt.addPoint(p2 * 0.5f + p3 * 0.5f);
	bt.addPoint(p3);
	bt.addPoint(p3 * 0.5f + p1 * 0.5f);
}

std::array<FaceHandle,3> BezierTMesh::splitFaceDyadical(
	FaceHandle fh,
	std::function<bool(FaceHandle)> mark,
	bool copy
) {
	std::array<FaceHandle, 3> neighbors;
	// new vertices (along old edges)
	std::array<VertexHandle, 3> verts;
	// new edges
	std::array<HalfedgeHandle, 3> edges;
	std::array<HalfedgeHandle, 3> oldEdges;

	size_t i = 0;

	if (status(fh).tagged()) {
		std::cerr << __FUNCTION__ << ": " << "helper face (" << fh << ")\n";

		for (auto he = cfh_begin(fh); he != cfh_end(fh); ++he, ++i) {
			VertexHandle v = to_vertex_handle(*he);
			if (status(v).tagged()) {
				std::cerr << "\t\tfound tagged vertex (" << i << ")\n";
				edges[i] = *he;
				verts[i] = v;
				he++;
				assert(!status(to_vertex_handle(*he)).tagged());
			} else {
				std::cerr << "\t\tfound normal vertex (" << i << ")\n";
			}
			neighbors[i] = opposite_face_handle(*he);
			oldEdges[i] = *he;
		}
		status(fh).set_tagged(false);
	} else {
		std::cerr << __FUNCTION__ << ": " << "normal face (" << fh << ")\n";

		for (auto he = cfh_begin(fh); he != cfh_end(fh); ++he, ++i) {
			neighbors[i] = opposite_face_handle(*he);
			oldEdges[i] = *he;
			if (status(to_vertex_handle(*he)).tagged()) {
				std::cerr << "\t\tfound tagged vertex (" << i << ")\n";
			} else {
				std::cerr << "\t\tfound normal vertex (" << i << ")\n";
			}
		}
	}
	assert(i == 3);

	for (i = 0; i < edges.size(); ++i) {
		auto edge = oldEdges[i];
		if (!edges[i].is_valid()) {
			EdgeHandle e = edge_handle(edge);
			verts[i] = add_vertex(calc_edge_midpoint(e));
			status(verts[i]).set_tagged(true);
			auto he = splitEdgeSimple(e, verts[i]);
			// correct halfedges after split (varies depending on whether edge is h0 or h1)
			if (to_vertex_handle(edge) == verts[i]) {
				edges[i] = edge;
				oldEdges[i] = next_halfedge_handle(edge);
			} else {
				edges[i] = he;
			}
			set_halfedge_handle(neighbors[i], edges[i]);
			// copy properties (not base!)
			if (copy) copy_all_properties(e, edge_handle(edges[i]), false);
			assert(to_vertex_handle(edges[i]) == verts[i]);
			assert(opposite_halfedge_handle(next_halfedge_handle(opposite_halfedge_handle(oldEdges[i]))) == edges[i]);
		}
		assert(is_valid_handle(edges[i]));
		assert(is_valid_handle(oldEdges[i]));
	}
	assert(i == 3);

	int before = n_faces();
	std::array<HalfedgeHandle, 3> inner;
	// add new faces
	for (i = 0; i < edges.size(); ++i) {
		const auto previous = oldEdges[(i-1+edges.size())%edges.size()];
		// add face manually
		const auto face = new_face();
		const auto edge = new_edge(to_vertex_handle(edges[i]), from_vertex_handle(previous));
		inner[i] = opposite_halfedge_handle(edge);
		// set next halfedges
		set_next_halfedge_handle(edges[i], edge);
		set_next_halfedge_handle(edge, previous);
		// set correct face
		set_face_handle(edges[i], face);
		set_face_handle(previous, face);
		set_face_handle(edge, face);
		set_face_handle(inner[i], fh);
		// set prev halfedge handles (does nothing if no prev handle is present)
		set_prev_halfedge_handle(edges[i], previous);
		set_prev_halfedge_handle(previous, edge);
		set_prev_halfedge_handle(edge, edges[i]);
		// set halfedge handle of the new face
		set_halfedge_handle(face, edges[i]);

		if (copy) {
			copy_all_properties(fh, face, false);
		}
	}
	assert(n_faces() == before + 3);

	for (i = 0; i < inner.size(); ++i) {
		set_next_halfedge_handle(inner[i], inner[(i + 1) % edges.size()]);
		set_prev_halfedge_handle(inner[i], inner[(i - 1 + edges.size()) % edges.size()]);
	}
	set_halfedge_handle(fh, inner[0]);

	// invert tags so faces that need to be split again can be identified
	for (auto f : neighbors) {
		if (mark(f)) {
			status(f).set_tagged(true);
		}
	}

	return neighbors;
}

void BezierTMesh::correctSplits(bool copy)
{
	const auto nfaces = n_faces();
	for (size_t i = 0; i < nfaces; ++i) {

		FaceHandle fh = face_handle(i);
		if (status(fh).tagged()) {
			status(fh).set_tagged(false);
			// collect tagged vertices
			std::vector<VertexHandle> verts;
			HalfedgeHandle h;
			for (auto he = fh_begin(fh); he != fh_end(fh); he++) {
				VertexHandle v = to_vertex_handle(*he);
				if (status(v).tagged()) {
					verts.push_back(v);
					h = *he;
					status(v).set_tagged(false);
				}
			}

			// we cannot have a face with more than one split edge
			assert(verts.size() == 0 || verts.size() == 1);

			if (verts.size() == 1) {
				// perform rivara split
				const auto nn = next_halfedge_handle(next_halfedge_handle(h));
				const auto face = new_face();
				const auto edge = new_edge(verts[0], to_vertex_handle(nn));
				const auto opp = opposite_halfedge_handle(edge);
				// set next handles
				set_next_halfedge_handle(h, edge);
				set_next_halfedge_handle(edge, next_halfedge_handle(nn));
				set_next_halfedge_handle(opp, next_halfedge_handle(h));
				set_next_halfedge_handle(nn, opp);
				// set prev halfedge handles (does nothing if no prev handle is present)
				set_prev_halfedge_handle(next_halfedge_handle(h), opp);
				set_prev_halfedge_handle(opp, nn);
				set_prev_halfedge_handle(edge, h);
				set_prev_halfedge_handle(next_halfedge_handle(nn), edge);
				// set face handles
				set_face_handle(h, face);
				set_face_handle(edge, face);
				set_face_handle(next_halfedge_handle(nn), face);
				set_face_handle(opp, fh);
				// set halfedge handles of the faces
				set_halfedge_handle(face, h);
				set_halfedge_handle(fh, next_halfedge_handle(h));

				if (copy) {
					copy_all_properties(fh, face, false);
				}
			}
		}
	}
}