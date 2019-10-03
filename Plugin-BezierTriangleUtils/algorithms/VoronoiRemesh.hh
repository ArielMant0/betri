#include "Common.hh"
#include "ShortestPath.hh"

namespace betri
{

class VoronoiRemesh
{
public:

	using ID = ShortestPath::ID;
	using VH = ShortestPath::VH;
	using EH = ShortestPath::EH;
	using HH = ShortestPath::HH;
	using FH = ShortestPath::FH;

	using P = BezierTMesh::Point;

	struct Props
	{
		static constexpr char *REGION = "region";
		static constexpr char *PREDECESSOR = "pred";
		static constexpr char *DISTANCE = "dist";
		static constexpr char *CROSSED = "crossed";

		static constexpr char *VERTEXTOTRI = "vtt";
		static constexpr char *TRITOVERTEX = "ttv";
		static constexpr char *BORDER = "border";
	};

	VoronoiRemesh(BezierTMesh &mesh, BezierTMesh &ctrl, bool colors=true, bool copy=false) :
		m_mesh(mesh),
		m_ctrl(ctrl),
		m_useColors(colors),
		m_copy(copy),
		m_steps(0u),
		m_vertexIdx(0u),
		m_colors(),
		m_seeds(),
		m_boundary(),
		m_ctrlVerts()
	{
		prepare();
	}

	~VoronoiRemesh()
	{
		cleanup();
	}

	void prepare();
	void cleanup();

	// performs all steps
	void remesh();
	// partitions mesh into voronoi regions
	void partition();
	// performs "dualizing" by finding paths on the mesh, can be performed stepwise
	bool dualize(bool steps=false);
	// parametrize and fit to surface
	void fitting();

	void useColors(bool use) { m_useColors = use; }
	bool useColors() const { return m_useColors; }

	void increaseStep() { m_steps++; }

	ID& id(FH fh) { return m_mesh.property(m_region, fh); }
	ID id(const FH fh) const { return m_mesh.property(m_region, fh); }

	FH& pred(FH fh) { return m_mesh.property(m_pred, fh); }
	FH pred(const FH fh) const { return m_mesh.property(m_pred, fh); }
	double& dist(FH fh) { return m_mesh.property(m_distance, fh); }

	TriToVertex& ttv(FH fh) { return m_ctrl.property(m_ttv, fh); }
	VertexToTri& vtt(VH vh) { return m_mesh.property(m_vtt, vh); }

	//FaceSplit& split(FH fh) { return m_mesh.property(m_split, fh); }

	ID& crossed(EH eh) { return m_mesh.property(m_crossed, eh); }
	ID& crossed(HH he) { return crossed(m_mesh.edge_handle(he)); }

	ID crossed(const EH eh) const { return m_mesh.property(m_crossed, eh); }
	ID crossed(const HH he) const { return crossed(m_mesh.edge_handle(he)); }

	bool isCrossed(const EH eh) const { return crossed(eh) >= 0; }
	bool isCrossed(const HH he) const { return isCrossed(m_mesh.edge_handle(he)); }

	std::pair<ID,ID>& faceBorder(FH fh) { return m_mesh.property(m_fborder, fh); }

	bool isBorder(const FH fh)
	{
		auto b = faceBorder(fh);
		return b.first != -1 && b.second != -1;
	}

	void setFaceBorder(const FH fh, ID id1=-1, ID id2=-1)
	{
		auto b = faceBorder(fh);
		b.first = id1;
		b.second = id2;
	}

	bool nextEdgeCrossed(EH eh, FH fh, FH next) const
	{
		HH he = m_mesh.halfedge_handle(eh, 0);
		if (m_mesh.face_handle(he) != fh) {
			he = m_mesh.halfedge_handle(eh, 1);
		}
		assert(m_mesh.face_handle(he) == fh);

		HH hn = m_mesh.next_halfedge_handle(he);
		HH hp = m_mesh.prev_halfedge_handle(he);

		return (m_mesh.opposite_face_handle(hn) != next || !isCrossed(hn))  &&
			(m_mesh.opposite_face_handle(hp) != next || !isCrossed(hp)) ;
	}

	bool isRegionBorderEdge(EH e) const
	{
		HH h = m_mesh.halfedge_handle(e,0);
		return id(m_mesh.face_handle(h)) != id(m_mesh.opposite_face_handle(h));
	}

private:

	static void copyMesh(BezierTMesh &src, BezierTMesh &dest);

	void preventiveEdgeSplits();

	void assignInnerVertices();

	void splitClosedPaths();

	void fixPredecessor(const FH fh);

	HalfedgeHandle fixCrossing(
		const FH f0,
		const FH f1,
		const VH prevNode=VH(),
		const VH notNode=VH()
	);

	void shortestPath(
		const VH v,
		const ID id_1,
		const ID id_2,
		const FH ctrlFace,
		const ShortestPath &path
	);

	void connectPaths(const ShortestPath & p0, const ShortestPath & p1, const FH f);

	ID seedVertex(const VH vh) const
	{
		for (auto f_it = m_mesh.cvf_begin(vh); f_it != m_mesh.cvf_end(vh); ++f_it) {
			if (!pred(*f_it).is_valid()) return id(*f_it);
		}
		return -1;
	}

	FH findDelaunayFace(ID r0, ID r1, ID r2)
	{
		for (FH f : m_ctrl.faces()) {
			if (ttv(f).isRegion(r0, r1, r2)) {
				return f;
			}
		}
		return FH();
	}

	// -------------------------------------------------------------- //
	// member variables
	// -------------------------------------------------------------- //

	BezierTMesh &m_mesh, &m_ctrl;

	bool m_useColors, m_copy;
	size_t m_steps;
	size_t m_nvertices, m_nedges, m_vertexIdx;

	std::vector<Color> m_colors;
	std::set<FH> m_seeds;
	std::vector<EH> m_boundary;
	std::vector<VH> m_ctrlVerts;

	// property handles
	OpenMesh::FPropHandleT<ID>			  m_region;
	OpenMesh::FPropHandleT<FH>			  m_pred;
	OpenMesh::FPropHandleT<double>		  m_distance;
	// must be something other than bool because vector<bool> is handled uniquely in C++
	OpenMesh::EPropHandleT<ID>			  m_crossed;
	//OpenMesh::FPropHandleT<FaceSplit>	  m_split;

	OpenMesh::FPropHandleT<TriToVertex>   m_ttv;
	OpenMesh::VPropHandleT<VertexToTri>	  m_vtt;

	OpenMesh::FPropHandleT<std::pair<ID,ID>>   m_fborder;
};

}