#include "BezierTMesh.hh"

#include "BezierMathUtil.hh"

#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <iostream>

void BezierTMesh::addCPsToFace(const FaceHandle f)
{
	auto vertexHandle = cfv_begin(f);
	VertexHandle vh0 = *(vertexHandle++);
	VertexHandle vh1 = *(vertexHandle++);
	VertexHandle vh2 = *(vertexHandle);

	std::vector<Point> cp_vec;

	Point p0 = point(vh0);
	Point p1 = point(vh1);
	Point p2 = point(vh2);

	update_normal(f);
	Point n0 = calc_vertex_normal(vh0);
	Point n1 = calc_vertex_normal(vh1);
	Point n2 = calc_vertex_normal(vh2);

	const int stuff = betri::gaussSum(m_degree + 1) - 1;
	cp_vec.reserve(stuff);

	const float CP_STEPSIZE = 1.0 / m_degree;
	int i = 0;
	// TODO: kann man vllt besser machen
	for (double u = 0.0; u <= 1.01; u += CP_STEPSIZE) {
		for (double v = 0.0; u + v <= 1.01; v += CP_STEPSIZE, ++i) {
			double w = 1 - u - v;
			Point p = p0 * u + p1 * v + p2 * w;
			cp_vec.push_back(p);
		}
	}

	//assert(cp_vec[0] == point(vh0));
	//assert(cp_vec[m_degree] == point(vh1));
	//assert(cp_vec.back() == point(vh2));

	data(f).points(cp_vec);
}

void BezierTMesh::tessellate(size_t amount)
{
	BezierTMesh copy;
	copy.degree(1);
	copy.request_face_normals();
	copy.request_vertex_normals();

	applyTessellation(&copy, amount);

	///////////////
	// copy mesh //
	///////////////
	clean_keep_reservation();

	auto vs = OpenMesh::makeTemporaryProperty<VertexHandle, VertexHandle>(copy);
	// TODO: works but seems really stupid
	for (const VertexHandle v : copy.vertices()) {
		vs[v] = add_vertex_dirty(copy.point(v));
	}
	for (const FaceHandle f : copy.faces()) {
		std::vector<VertexHandle> verts;
		for (auto fv = copy.cfv_begin(f); fv != copy.cfv_end(f); ++fv) {
			verts.push_back(vs[*fv]);
		}
		add_face(verts, true);
	}

	garbage_collection();
}

void BezierTMesh::tessellateToTrimesh(TriMesh &mesh, size_t amount)
{
	applyTessellation(&mesh, amount);
}

template <typename MeshT>
void BezierTMesh::applyTessellation(MeshT *mesh, size_t amount)
{
	const float newVertices = betri::mersennePrime(amount);
	const float vertexSum = betri::gaussSum(newVertices + 2);
	const float stepsize = 1.0 / (double(newVertices) + 1.0);

	const auto getHandle = [&](Point p) {
		auto viter = std::find_if(mesh->vertices_begin(), mesh->vertices_end(), [&](VertexHandle vh) {
			return (mesh->point(vh) - p).norm() < 0.0001;
		});

		if (viter != mesh->vertices_end()) {
			return *viter;
		}

		return mesh->add_vertex(p);
	};

	Point pos;
	// Iterate over all faces
	for (FaceHandle face : faces()) {

		std::vector<BezierTMesh::Point> newHandleVector(vertexSum);

		// Iterate in two directions (u,v) which can use to determine the
		// point at which the bezier triangle should be evaluated
		int handleIt = 0;
		for (double u = 0.0; u <= 1.0; u += stepsize) {
			for (double v = 0.0; u + v <= 1.0; v += stepsize) {

				Point toEval = betri::getBaryCoords(u, v);
				pos = betri::evalSurface(data(face).points(), toEval, m_degree);

				// Add Point
				// TODO dont add the Points that are already in there (3 starting points)
				//VertexHandle newPointHandle = copy.add_vertex(pos);
				newHandleVector[handleIt++] = pos;//newPointHandle;
			}
		}

		// Example - first half of the triangles
		// 0 1 5 b=5
		// 1 2 6 b=5
		// 2 3 7 b=5
		// 3 4 8 b=5
		// pos1+2 pos2+2 pos3+1 b=5+4
		// 5 6 9 b=9
		// 6 7 10 b=9
		// 7 8 11 b=9
		// pos1+2 pos2+2 pos3+1 b=5+4+3
		// 9 10 12 b=12
		// 10 11 13 b=12
		// pos1+2 pos2+2 pos3+1 b=5+4+3+2
		// 12 13 14 b=14

		int pos1 = 0;
		int pos2 = 1;
		int pos3 = newVertices + 2;
		int border = newVertices + 2;
		int boderAdd = border - 1;

		// Iterate all added Points and add pairs of three as a new face
		for (; pos3 < newHandleVector.size(); ) {
			// bottom triangle
			FaceHandle fh = mesh->add_face(
				getHandle(newHandleVector[pos1]),
				getHandle(newHandleVector[pos2]),
				getHandle(newHandleVector[pos3])
			);
			// Add the controllPoints to the face
			//data(fh).points(data(face).points());

			if (pos2 + 1 < border) {
				// top triangle
				fh = mesh->add_face(
					getHandle(newHandleVector[pos2]),
					getHandle(newHandleVector[pos3 + 1]),
					getHandle(newHandleVector[pos3])
				);
				// Add the controllPoints to the face
				//data(fh).points(data(face).points());
			}

			if (pos2 + 1 == border) {
				border += boderAdd--;
				pos1++;
				pos2++;
			}
			pos1++;
			pos2++;
			pos3++;
		}
	}
}

void BezierTMesh::recalculateCPs(const FaceHandle f)
{
	addCPsToFace(f);
}

void BezierTMesh::interpolateEdgeControlPoints(const EdgeHandle eh, const bool between)
{
	HalfedgeHandle h0 = halfedge_handle(eh, 0), h1 = halfedge_handle(eh, 1);
	FaceHandle f0 = face_handle(h0), f1 = face_handle(h1);

	auto &cp0 = data(f0), &cp1 = data(f1);

	VertexHandle v00 = from_vertex_handle(h0);
	VertexHandle v01 = to_vertex_handle(h0);

	Point p00 = point(v00);
	Point p01 = point(v01);

	int fi0 = 2 - cpCornerIndex(f0, v00);
	int ti0 = 2 - cpCornerIndex(f0, v01);
	std::vector<Point> p0 = cp0.edgePoints(fi0, ti0, m_degree);

	int fi1 = 2 - cpCornerIndex(f1, v00);
	int ti1 = 2 - cpCornerIndex(f1, v01);
	std::vector<Point> p1 = cp1.edgePoints(fi1, ti1, m_degree);

	std::cerr.precision(2);
	std::cerr.setf(std::ios::fixed, std::ios::floatfield);

	p0[0] = p00;
	p0[m_degree] = p01;

	p1[0] = p00;
	p1[m_degree] = p01;

	if (between) {
		for (int i = 1; i < p0.size()-1; ++i) {
			p0[i] = 0.5 * p0[i] + 0.5 * p1[i];
			p1[i] = p0[i];
		}
	}

	cp0.edgePoints(fi0, ti0, m_degree, p0);
	cp1.edgePoints(fi1, ti1, m_degree, p1);
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
			set_color(f, color(fh));
		}
	}
	set_halfedge_handle(v, opposite_halfedge_handle(hes[3]));

	return v;
}

BezierTMesh::VertexHandle BezierTMesh::splitFacesRivara(FaceHandle of1, FaceHandle of2, bool copy)
{

	HalfedgeHandle connect;

	for (auto he = cfh_begin(of1); he != cfh_end(of1); ++he) {
		if (opposite_face_handle(*he) == of2) connect = *he;
	}

	assert(connect.is_valid());

	VertexHandle vh = new_vertex(calc_edge_midpoint(connect));
	EdgeHandle eh = edge_handle(connect);

	HalfedgeHandle h0 = halfedge_handle(eh, 0);
	HalfedgeHandle o0 = halfedge_handle(eh, 1);

	VertexHandle   v2 = to_vertex_handle(o0);

	HalfedgeHandle e1 = new_edge(vh, v2);
	HalfedgeHandle t1 = opposite_halfedge_handle(e1);

	FaceHandle     f0 = face_handle(h0);
	FaceHandle     f3 = face_handle(o0);

	set_halfedge_handle(vh, h0);
	set_vertex_handle(o0, vh);

	if (!is_boundary(h0)) {
		HalfedgeHandle h1 = next_halfedge_handle(h0);
		HalfedgeHandle h2 = next_halfedge_handle(h1);

		VertexHandle v1 = to_vertex_handle(h1);

		HalfedgeHandle e0 = new_edge(vh, v1);
		HalfedgeHandle t0 = opposite_halfedge_handle(e0);

		FaceHandle f1 = new_face();
		if (copy) {
			copy_all_properties(f0, f1, false);
			set_color(f1, color(f0));
		}
		set_halfedge_handle(f0, h0);
		set_halfedge_handle(f1, h2);

		set_face_handle(h1, f0);
		set_face_handle(t0, f0);
		set_face_handle(h0, f0);

		set_face_handle(h2, f1);
		set_face_handle(t1, f1);
		set_face_handle(e0, f1);

		set_next_halfedge_handle(h0, h1);
		set_next_halfedge_handle(h1, t0);
		set_next_halfedge_handle(t0, h0);

		set_next_halfedge_handle(e0, h2);
		set_next_halfedge_handle(h2, t1);
		set_next_halfedge_handle(t1, e0);
	} else {
		set_next_halfedge_handle(prev_halfedge_handle(h0), t1);
		set_next_halfedge_handle(t1, h0);
		// halfedge handle of _vh already is h0
	}


	if (!is_boundary(o0)) {
		HalfedgeHandle o1 = next_halfedge_handle(o0);
		HalfedgeHandle o2 = next_halfedge_handle(o1);

		VertexHandle v3 = to_vertex_handle(o1);

		HalfedgeHandle e2 = new_edge(vh, v3);
		HalfedgeHandle t2 = opposite_halfedge_handle(e2);

		FaceHandle f2 = new_face();
		if (copy) {
			copy_all_properties(f3, f2, false);
			set_color(f2, color(f3));
		}
		set_halfedge_handle(f2, o1);
		set_halfedge_handle(f3, o0);

		set_face_handle(o1, f2);
		set_face_handle(t2, f2);
		set_face_handle(e1, f2);

		set_face_handle(o2, f3);
		set_face_handle(o0, f3);
		set_face_handle(e2, f3);

		set_next_halfedge_handle(e1, o1);
		set_next_halfedge_handle(o1, t2);
		set_next_halfedge_handle(t2, e1);

		set_next_halfedge_handle(o0, e2);
		set_next_halfedge_handle(e2, o2);
		set_next_halfedge_handle(o2, o0);
	} else {
		set_next_halfedge_handle(e1, next_halfedge_handle(o0));
		set_next_halfedge_handle(o0, e1);
		set_halfedge_handle(vh, e1);
	}

	if (halfedge_handle(v2) == h0) set_halfedge_handle(v2, t1);

	return vh;
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
