#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

#include "Parametrization.hh"

namespace betri
{

class VoronoiRemesh
{
public:

	using ID = int;
	using VH = BezierTMesh::VertexHandle;
	using EH = BezierTMesh::EdgeHandle;
	using HH = BezierTMesh::HalfedgeHandle;
	using FH = BezierTMesh::FaceHandle;
	using P = BezierTMesh::Point;
	using Color = BezierTMesh::Color;

	using Scalar = BezierTMesh::Scalar;
	using Vec2 = ACG::VectorT<Scalar, 2>;

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

	void remesh(unsigned int size);

	static void copyMesh(BezierTMesh &src, BezierTMesh &dest);

	ID& id(FH fh) { return m_mesh.property(m_region, fh); }
	FH& pred(FH fh) { return m_mesh.property(m_pred, fh); }
	double& dist(FH fh) { return m_mesh.property(m_distance, fh); }

	TriToVertex& ttv(FH fh) { return m_ctrl.property(m_ttv, fh); }
	VertexToTri& vtt(VH vh) { return m_mesh.property(m_vtt, vh); }

	int& border(HH hh) { return m_mesh.property(m_border, hh); }

	ID& crossed(EH eh) { return m_mesh.property(m_crossed, eh); }
	ID& crossed(HH he) { return crossed(m_mesh.edge_handle(he)); }

	bool isCrossed(EH eh) { return crossed(eh) >= 0; }
	bool isCrossed(HH he) { return isCrossed(m_mesh.edge_handle(he)); }

private:

	using QElem = std::pair<double, FH>;

	void partition();

	void preventiveEdgeSplits();

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
	// TODO: add property to each halfedge(!) that shows which delaunay triangle
	// it is a part of (uses face index)
	OpenMesh::HPropHandleT<int>			  m_border;

	OpenMesh::FPropHandleT<TriToVertex>   m_ttv;
	OpenMesh::VPropHandleT<VertexToTri>	  m_vtt;
};

}