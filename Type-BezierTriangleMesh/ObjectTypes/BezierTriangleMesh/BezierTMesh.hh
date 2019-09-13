#include "BezierTriangleMeshTypes.hh"

#include <OpenFlipper/common/ObjectTypeDLLDefines.hh>

class OBJECTTYPEDLLEXPORTONLY BezierTMesh : public OpenMesh::TriMesh_ArrayKernelT<BezierTriangleTraits>
{
public:

	void degreeElevation(FaceHandle &fh);

	void degreeReduction(FaceHandle &fh);

	void addCPsToFace(FaceHandle &f, unsigned int degree=2);

	void addCPsToFace(const FaceHandle &f, unsigned int degree=2);

	HalfedgeHandle splitEdgeSimple(EdgeHandle _eh, VertexHandle _vh)
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

		return new_e;
	}

	HalfedgeHandle splitEdgeCopySimple(EdgeHandle eh, VertexHandle vh)
	{
		HalfedgeHandle he = splitEdgeSimple(eh, vh);
		copy_all_properties(eh, edge_handle(he), false);
		return he;
	}

	std::array<FaceHandle,3> splitFaceDyadical(
		FaceHandle fh,
		std::function<bool(FaceHandle)> mark,
		bool copy=false
	);

	void correctSplits(bool copy=false);

private:

	void splitRivara(const HalfedgeHandle he, const VertexHandle vh, bool copy=false);
};