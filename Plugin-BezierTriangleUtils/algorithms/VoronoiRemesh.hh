#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

#include "Parametrization.hh"
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

	void remesh(unsigned int size);

	static void copyMesh(BezierTMesh &src, BezierTMesh &dest);

	ID& id(FH fh) { return m_mesh.property(m_region, fh); }
	FH& pred(FH fh) { return m_mesh.property(m_pred, fh); }
	double& dist(FH fh) { return m_mesh.property(m_distance, fh); }

	TriToVertex& ttv(FH fh) { return m_ctrl.property(m_ttv, fh); }
	VertexToTri& vtt(VH vh) { return m_mesh.property(m_vtt, vh); }

	int& border(HH hh) { return m_mesh.property(m_border, hh); }
	FaceSplit& split(FH fh) { return m_mesh.property(m_split, fh); }

	ID& crossed(EH eh) { return m_mesh.property(m_crossed, eh); }
	ID& crossed(HH he) { return crossed(m_mesh.edge_handle(he)); }

	bool isCrossed(EH eh) { return crossed(eh) >= 0; }
	bool isCrossed(HH he) { return isCrossed(m_mesh.edge_handle(he)); }

private:

	using QElem = std::pair<double, FH>;

	void partition();

	void preventiveEdgeSplits();

	bool hasShortestPath(const ID id1, const ID id2) const;
	ShortestPath getShortestPath(ID id1, ID id2);

	// face splitting (to add boundary edges to the mesh)
	void splitFace(FH face);

	// -------------------------------------------------------------- //

	// member variables
	BezierTMesh &m_mesh, &m_ctrl;

	bool m_useColors;
	std::vector<Color> m_colors;
	std::set<FH> m_seeds;
	std::vector<EH> m_boundary;
	// stores the shortest path for each pair of tiles that is needed to
	// construct the delaunay triangulation
	std::unordered_set<ShortestPath> m_paths;


	// property handles
	OpenMesh::FPropHandleT<ID>			  m_region;
	OpenMesh::FPropHandleT<FH>			  m_pred;
	OpenMesh::FPropHandleT<double>		  m_distance;
	// must be something other than bool because vector<bool> is handled uniquely in C++
	OpenMesh::EPropHandleT<ID>			  m_crossed;
	// TODO: add property to each halfedge(!) that shows which delaunay triangle
	// it is a part of (uses face index)
	OpenMesh::HPropHandleT<int>			  m_border;
	OpenMesh::FPropHandleT<FaceSplit>	  m_split;

	OpenMesh::FPropHandleT<TriToVertex>   m_ttv;
	OpenMesh::VPropHandleT<VertexToTri>	  m_vtt;
};

}