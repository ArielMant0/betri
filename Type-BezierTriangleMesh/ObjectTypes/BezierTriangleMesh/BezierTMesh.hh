#pragma once

#include "BezierTriangleMeshTypes.hh"

#include <OpenFlipper/common/ObjectTypeDLLDefines.hh>

class OBJECTTYPEDLLEXPORTONLY BezierTMesh : public OpenMesh::TriMesh_ArrayKernelT<BezierTriangleTraits>
{
public:

	BezierTMesh() :
		OpenMesh::TriMesh_ArrayKernelT<BezierTriangleTraits>(),
		m_render(false),
		m_degree(2) {}

	BezierTMesh(const BezierTMesh &other) :
		OpenMesh::TriMesh_ArrayKernelT<BezierTriangleTraits>(other),
		m_render(other.m_render),
		m_degree(other.m_degree) {}

	void degreeElevation(FaceHandle fh);

	void degreeReduction(FaceHandle fh);

	void recalculateCPs(const FaceHandle f);

	FaceHandle add_face(const std::vector<VertexHandle> &vhs, bool cps=false)
	{
		FaceHandle fh = TriConnectivity::add_face(vhs);
		if (cps && fh.is_valid()) addCPsToFace(fh);
		return fh;
	}

	FaceHandle add_face(const VertexHandle v1, const VertexHandle v2, const VertexHandle v3, bool cps=false)
	{
		FaceHandle fh = TriConnectivity::add_face(v1, v2, v3);
		if (cps && fh.is_valid()) addCPsToFace(fh);
		return fh;
	}

	FaceHandle add_face(const VertexHandle *vhs, size_t vhsize, bool cps = false)
	{
		FaceHandle fh = TriConnectivity::add_face(vhs, vhsize);
		if (cps && fh.is_valid()) addCPsToFace(fh);
		return fh;
	}

	/// control whether the mesh can be rendered
	bool isRenderable() const { return m_render; }
	void setRenderable() { m_render = true; }

	/// control bezier triangle degree
	void degree(size_t degree) { m_degree = degree; }

	size_t degree() { return m_degree; }

	HalfedgeHandle splitEdgeSimple(EdgeHandle _eh, VertexHandle _vh, bool copy=false)
	{
		HalfedgeHandle h0 = halfedge_handle(_eh, 0);
		HalfedgeHandle h1 = halfedge_handle(_eh, 1);

		VertexHandle vfrom = from_vertex_handle(h0);

		HalfedgeHandle ph0 = prev_halfedge_handle(h0);
		HalfedgeHandle nh1 = next_halfedge_handle(h1);

		bool boundary0 = is_boundary(h0);
		bool boundary1 = is_boundary(h1);

		//add the new edge
		HalfedgeHandle new_e = new_edge(from_vertex_handle(h0), _vh);

		//fix the vertex of the opposite halfedge
		set_vertex_handle(h1, _vh);

		//fix the halfedge connectivity
		set_next_halfedge_handle(new_e, h0);
		set_next_halfedge_handle(h1, opposite_halfedge_handle(new_e));

		set_next_halfedge_handle(ph0, new_e);
		set_next_halfedge_handle(opposite_halfedge_handle(new_e), nh1);

		if (!boundary0) {
			set_face_handle(new_e, face_handle(h0));
		} else {
			set_boundary(new_e);
		}

		if (!boundary1) {
			set_face_handle(opposite_halfedge_handle(new_e), face_handle(h1));
		} else {
			set_boundary(opposite_halfedge_handle(new_e));
		}

		set_halfedge_handle(_vh, h0);
		adjust_outgoing_halfedge(_vh);

		if (halfedge_handle(vfrom) == h0) {
			set_halfedge_handle(vfrom, new_e);
			adjust_outgoing_halfedge(vfrom);
		}

		if (copy) {
			copy_all_properties(_eh, edge_handle(new_e), false);
		}

		return new_e;
	}

	void splitFaceDyadical(
		FaceHandle fh,
		std::function<bool(FaceHandle)> mark,
		bool copy=false
	);

	VertexHandle splitFaceBarycentric(FaceHandle fh, bool copy=false);

	VertexHandle splitFacesRivara(FaceHandle f1, FaceHandle f2, bool copy=false);

	void correctSplits(bool copy=false);

	bool adjToFace(const VertexHandle v, const FaceHandle f) const
	{
		for (auto fit = cvf_begin(v); fit != cvf_end(v); ++fit) {
			if (*fit == f) return true;
		}
		return false;
	}

	bool adjToFace(const HalfedgeHandle he, const FaceHandle f) const
	{
		return face_handle(he) == f || opposite_face_handle(he) == f;
	}

	bool adjToFace(const EdgeHandle e, const FaceHandle f) const
	{
		return face_handle(halfedge_handle(e,0)) == f || face_handle(halfedge_handle(e, 1)) == f;
	}

	bool adjToFace(const FaceHandle f1, const FaceHandle f2) const
	{
		for (auto fit = cff_begin(f1); fit != cff_end(f1); ++fit) {
			if (*fit == f2) return true;
		}
		return false;
	}

private:

	void addCPsToFace(const FaceHandle f);

	void splitRivara(const HalfedgeHandle he, const VertexHandle vh, bool copy=false);

	bool m_render;
	size_t m_degree;
};