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

	enum class FaceSplit : char
	{
		NONE = 0,
		MAKE_FOUR = 1,
		TWO = 2,
		FOUR = 4
	};

	VoronoiRemesh(BezierTMesh &mesh, BezierTMesh &ctrl, bool colors=true) :
		m_mesh(mesh),
		m_ctrl(ctrl),
		m_useColors(colors),
		m_colors(),
		m_seeds(),
		m_boundary()
	{
		prepare();
	}

	~VoronoiRemesh()
	{
		cleanup();
	}

	void prepare();
	void cleanup();

	void remesh();

	static void copyMesh(BezierTMesh &src, BezierTMesh &dest);

	ID& id(FH fh) { return m_mesh.property(m_region, fh); }
	ID id(FH fh) const { return m_mesh.property(m_region, fh); }

	FH& pred(FH fh) { return m_mesh.property(m_pred, fh); }
	double& dist(FH fh) { return m_mesh.property(m_distance, fh); }

	TriToVertex& ttv(FH fh) { return m_ctrl.property(m_ttv, fh); }
	VertexToTri& vtt(VH vh) { return m_mesh.property(m_vtt, vh); }

	//FaceSplit& split(FH fh) { return m_mesh.property(m_split, fh); }

	ID& crossed(EH eh) { return m_mesh.property(m_crossed, eh); }
	ID& crossed(HH he) { return crossed(m_mesh.edge_handle(he)); }

	ID crossed(EH eh) const { return m_mesh.property(m_crossed, eh); }
	ID crossed(HH he) const { return crossed(m_mesh.edge_handle(he)); }

	bool isCrossed(EH eh) const { return crossed(eh) >= 0; }
	bool isCrossed(HH he) const { return isCrossed(m_mesh.edge_handle(he)); }

	bool isCrossed(VH vh) const
	{
		int adj = 0;
		for (auto e_it = m_mesh.cve_begin(vh); e_it != m_mesh.cve_end(vh); ++e_it) {
			if (isCrossed(e_it)) adj++;
		}
		return adj >= 2;
	}

	bool nextEdgeCrossed(EH eh, FH fh) const
	{
		for (auto e_it = m_mesh.cfe_begin(fh); e_it != m_mesh.cfe_end(fh); ++e_it) {
			auto next = std::next(e_it) != m_mesh.cfe_end(fh) ?
				std::next(e_it) :
				m_mesh.cfe_begin(fh);

			if (*e_it == eh && isCrossed(next)) return true;
		}
		return false;
	}

	bool isRegionBorderEdge(EH e) const
	{
		HH h = m_mesh.halfedge_handle(e,0);
		return id(m_mesh.face_handle(h)) != id(m_mesh.opposite_face_handle(h));
	}

private:

	using QElem = std::pair<double, FH>;

	void partition();

	void preventiveEdgeSplits();

	void assignInnerVertices();

	void splitClosedPaths(ShortestPath &sp);

	void fixPredecessor(const FH fh, const bool forceId=true);

	void shortestPath(
		const VH v,
		const FH f1,
		const FH f2,
		const FH ctrlFace,
		ShortestPath &path,
		const std::vector<VH> &sv
	);

	// -------------------------------------------------------------- //

	// member variables
	BezierTMesh &m_mesh, &m_ctrl;

	bool m_useColors;
	std::vector<Color> m_colors;
	std::set<FH> m_seeds;
	std::vector<EH> m_boundary;

	// property handles
	OpenMesh::FPropHandleT<ID>			  m_region;
	OpenMesh::FPropHandleT<FH>			  m_pred;
	OpenMesh::FPropHandleT<double>		  m_distance;
	// must be something other than bool because vector<bool> is handled uniquely in C++
	OpenMesh::EPropHandleT<ID>			  m_crossed;
	//OpenMesh::FPropHandleT<FaceSplit>	  m_split;

	OpenMesh::FPropHandleT<TriToVertex>   m_ttv;
	OpenMesh::VPropHandleT<VertexToTri>	  m_vtt;
};

}