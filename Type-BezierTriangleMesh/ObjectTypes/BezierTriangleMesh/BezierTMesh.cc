#include "BezierTMesh.hh"

#include "BezierMathUtil.hh"

#include <iostream>

void BezierTMesh::addCPsToFace(const FaceHandle f)
{
	auto vertexHandle = fv_begin(f);
	auto vh0 = *(vertexHandle++);
	auto vh1 = *(vertexHandle++);
	auto vh2 = *(vertexHandle);

	std::vector<Point> cp_vec;

	auto p0 = point(vh0);
	auto p1 = point(vh1);
	auto p2 = point(vh2);

	//const float STEPSIZE = round((1.0 / GRAD) * 100) / 100;
	// TODO 1.01 ...
	const float CP_STEPSIZE = 1.0 / m_degree;
	int i = 0;
	for (double u = 0.0; u <= 1.01; u += CP_STEPSIZE) {
		for (double v = 0.0; u + v <= 1.01; v += CP_STEPSIZE) {
			double w = 1 - u - v;
			i++;
			cp_vec.push_back(p0 * u + p1 * v + p2 * w);
		}
	}

	data(f).points(cp_vec);
}

void BezierTMesh::recalculateCPs(const FaceHandle f)
{
	auto vertexHandle = fv_begin(f);
	auto vh0 = *(vertexHandle++);
	auto vh1 = *(vertexHandle++);
	auto vh2 = *(vertexHandle);

	auto p0 = point(vh0);
	auto p1 = point(vh1);
	auto p2 = point(vh2);

	auto bezier = data(f);

	int i = 0;
	const float CP_STEPSIZE = 1.0 / m_degree;

	for (double u = 0.0; u <= 1.01; u += CP_STEPSIZE) {
		for (double v = 0.0; u + v <= 1.01; v += CP_STEPSIZE) {
			double w = 1 - u - v;
			bezier.controlPoint(i++, p0 * u + p1 * v + p2 * w);
		}
	}
}

void BezierTMesh::splitFaceDyadical(
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
			set_halfedge_handle(neighbors[i], opposite_halfedge_handle(oldEdges[i]));
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
		// set halfedge handle of the new face
		set_halfedge_handle(face, edges[i]);
		assert(is_valid_handle(face));
		assert(is_valid_handle(fh));

		if (copy) {
			copy_all_properties(fh, face, false);
		}
	}
	assert(n_faces() == before + 3);

	for (i = 0; i < inner.size(); ++i) {
		set_next_halfedge_handle(inner[i], inner[(i + 1) % edges.size()]);
	}
	set_halfedge_handle(fh, inner[0]);

	// call maark function for each previous neighbor face
	for (auto f : neighbors) {
		if (mark(f)) {
			status(f).set_tagged(true);
			// DEBUG: count number of total and tagged vertices of the face
			//int count = 0, total = 0;
			//for (auto vh = cfv_begin(f); vh != cfv_end(f); ++vh) {
			//	if (status(*vh).tagged()) count++;
			//	total++;
			//}
			//assert((count == 1 && total == 4) || (count == 2 && total == 5));
		}
	}
}

BezierTMesh::VertexHandle BezierTMesh::splitFaceBarycentric(FaceHandle fh, bool copy)
{
	// add new vertex
	VertexHandle v = new_vertex(calc_face_centroid(fh));

	size_t i = 0;
	std::array<HalfedgeHandle, 6> hes;
	for (auto h_it = cfh_begin(fh); h_it != cfh_end(fh); ++h_it, ++i) {
		hes[i] = *h_it;
		// add new edge
		hes[i+3] = new_edge(to_vertex_handle(*h_it), v);
		set_next_halfedge_handle(opposite_halfedge_handle(hes[i+3]), next_halfedge_handle(*h_it));
	}

	FaceHandle f;
	HalfedgeHandle prev;
	for (i = 0; i < 3; ++i) {
		prev = opposite_halfedge_handle(i == 0 ? hes[5] : hes[i+2]);
		set_next_halfedge_handle(hes[i], hes[i+3]);
		set_next_halfedge_handle(hes[i+3], prev);
		if (i == 0) {
			f = fh;
		} else {
			f = new_face();
		}
		// add face and set handles correctly
		set_halfedge_handle(f, hes[i]);
		set_face_handle(hes[i], f);
		set_face_handle(hes[i+3], f);
		set_face_handle(prev, f);

		if (copy) {
			copy_all_properties(fh, f, false);
		}
	}
	set_halfedge_handle(v, opposite_halfedge_handle(hes[3]));

	return v;
}

void BezierTMesh::splitFacesRivara(FaceHandle f1, FaceHandle f2, bool copy)
{
	assert(adjToFace(f1, f2));

	HalfedgeHandle connect;

	for (auto he = cfh_begin(f1); he != cfh_end(f1); ++he) {
		if (adjToFace(*he, f2)) connect = *he;
	}

	VertexHandle v = new_vertex(calc_edge_midpoint(connect));
	HalfedgeHandle nh = halfedge_handle(edge_handle(connect), 0);
	connect = splitEdgeSimple(edge_handle(connect), v, false);

	HalfedgeHandle o0 = next_halfedge_handle(nh);
	HalfedgeHandle o1 = next_halfedge_handle(opposite_halfedge_handle(connect));
	HalfedgeHandle on0 = next_halfedge_handle(o0);
	HalfedgeHandle on1 = next_halfedge_handle(o1);

	HalfedgeHandle i0 = new_edge(v, to_vertex_handle(o0));
	HalfedgeHandle i1 = new_edge(v, to_vertex_handle(o1));

	set_next_halfedge_handle(o0, opposite_halfedge_handle(i0));
	set_next_halfedge_handle(opposite_halfedge_handle(i0), nh);
	set_next_halfedge_handle(i0, on0);
	set_next_halfedge_handle(connect, i0);

	set_next_halfedge_handle(o1, i1);
	set_next_halfedge_handle(i1, opposite_halfedge_handle(connect));
	set_next_halfedge_handle(opposite_halfedge_handle(i1), on1);
	set_next_halfedge_handle(opposite_halfedge_handle(nh), opposite_halfedge_handle(i1));

	FaceHandle f11 = new_face(), f22 = new_face();

	set_face_handle(on0, f1);
	set_face_handle(connect, f1);
	set_face_handle(i0, f1);
	set_halfedge_handle(f1, connect);

	set_face_handle(o1, f2);
	set_face_handle(opposite_halfedge_handle(connect), f2);
	set_face_handle(i1, f2);
	set_halfedge_handle(f2, opposite_halfedge_handle(connect));

	set_face_handle(nh, f11);
	set_face_handle(o0, f11);
	set_face_handle(opposite_halfedge_handle(i0), f11);
	set_halfedge_handle(f11, nh);

	set_face_handle(opposite_halfedge_handle(nh), f22);
	set_face_handle(on1, f22);
	set_face_handle(opposite_halfedge_handle(i1), f22);
	set_halfedge_handle(f22, opposite_halfedge_handle(nh));

	if (copy) {
		copy_all_properties(f1, f11);
		copy_all_properties(f2, f22);
	}
}

void BezierTMesh::correctSplits(bool copy)
{
	const auto nfaces = n_faces();
	for (size_t i = 0; i < nfaces; ++i) {

		FaceHandle fh = face_handle(i);
		if (status(fh).tagged()) {
			std::cerr << "found tagged face (needs rivara split): " << fh << "\n";
			// tagged vertex
			VertexHandle vertex;
			HalfedgeHandle h;

			int i = 0;
			for (auto he = cfh_begin(fh); he != cfh_end(fh); ++he, ++i) {
				VertexHandle v = to_vertex_handle(*he);
				if (status(v).tagged()) {
					vertex = v;
					h = *he;
					status(v).set_tagged(false);
				}
			}

			// we must have visited exactly 4 vertices
			assert(i == 4);
			// we cannot have a face with more than one split edge
			assert(is_valid_handle(vertex));

			std::cerr << "\t splitting edge for face " << fh << " and ";
			std::cerr << opposite_face_handle(h) << "\n";

			FaceHandle oppFace = opposite_face_handle(h);
			HalfedgeHandle oppHe = opposite_halfedge_handle(next_halfedge_handle(h));

			status(fh).set_tagged(false);
			splitRivara(h, vertex, copy);

			if (status(oppFace).tagged()) {
				status(oppFace).set_tagged(false);
				splitRivara(oppHe, vertex, copy);
			}
		}
	}
}

void BezierTMesh::splitRivara(const HalfedgeHandle he, const VertexHandle vh, bool copy)
{
	const auto fh = face_handle(he);
	// perform rivara split
	const auto nn = next_halfedge_handle(next_halfedge_handle(he));
	const auto face = new_face();
	const auto edge = new_edge(vh, to_vertex_handle(nn));
	const auto opp = opposite_halfedge_handle(edge);

	HalfedgeHandle hen = next_halfedge_handle(he);
	HalfedgeHandle hep = next_halfedge_handle(nn);

	// set next handles
	set_next_halfedge_handle(he, edge);
	set_next_halfedge_handle(edge, next_halfedge_handle(nn));
	set_next_halfedge_handle(opp, hen);
	set_next_halfedge_handle(nn, opp);
	// set face handles
	set_face_handle(he, face);
	set_face_handle(edge, face);
	set_face_handle(hep, face);
	set_face_handle(opp, fh);
	// set halfedge handles of the faces
	set_halfedge_handle(face, he);
	set_halfedge_handle(fh, hen);

	if (copy) {
		copy_all_properties(fh, face, false);
		set_color(face, color(fh));
	}
}
