#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

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

	struct Props
	{
		static constexpr char *REGION = "region";
		static constexpr char *PREDECESSOR = "pred";
		static constexpr char *DISTANCE = "dist";
		static constexpr char *CROSSED = "crossed";

		static constexpr char *FACETOTRI = "ftt";
		static constexpr char *TRITOFACE = "ttf";
		static constexpr char *TRIBOUND = "tribound";

	};

	struct TriToVertex
	{
		std::set<VH> inner; // inner vertices
		std::set<VH> outer; // outer boundary vertices (found using shortest paths)

		void set(std::set<VH> &in, std::set<VH> &out)
		{
			inner = in;
			outer = out;
		}
	};

	struct VertexToTri
	{
		FH face; // delaunay triangle in control mesh (invalid if border)
		double u, v; // parameterization

		void set(double uVal, double vVal)
		{
			u = uVal;
			v = vVal;
		}
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

	OpenMesh::FPropHandleT<TriToVertex>   m_ttv;
	OpenMesh::VPropHandleT<VertexToTri>	  m_vtt;
};

}