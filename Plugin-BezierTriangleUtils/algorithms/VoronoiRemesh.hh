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

	struct FaceToTri
	{
		std::vector<VH> verts;

		std::vector<P> points(BezierTMesh &mesh)
		{
			std::vector<P> result;
			std::transform(verts.begin(), verts.end(), std::back_inserter(result),
				[mesh](BezierTMesh::VertexHandle &v) { return mesh.point(v); }
			);
			return result;
		}
	};

	struct TriToFace
	{
		FH triangle;
		double hmap;

		void set(FH &face, double hValue)
		{
			triangle = face;
			hmap = hValue;
		}
	};

	VoronoiRemesh(BezierTMesh &mesh, BezierTMesh &ctrl) : m_mesh(mesh), m_ctrl(ctrl)
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

	TriToFace& ftt(FH fh) { return m_mesh.property(m_ttf, fh); }
	FaceToTri& ttf(FH fh) { return m_mesh.property(m_ftt, fh); }

	short& crossed(EH eh) { return m_mesh.property(m_crossed, eh); }
	short& crossed(HH he) { return crossed(m_mesh.edge_handle(he)); }

	bool isCrossed(EH eh) { return crossed(eh) == 1; }
	bool isCrossed(HH he) { return isCrossed(m_mesh.edge_handle(he)); }

private:

	void partition(std::vector<FH> &seeds, bool useColors=true);

	void dijkstra(std::vector<FH> &seeds, bool useColors=true);

	void preventiveEdgeSplits(unsigned int size);

	// member variables
	BezierTMesh &m_mesh, &m_ctrl;

	// property handles
	OpenMesh::FPropHandleT<ID>        m_region;
	OpenMesh::FPropHandleT<FH>        m_pred;
	OpenMesh::FPropHandleT<double>    m_distance;
	// must be something other than bool because vector<bool> is handled uniquely in C++
	OpenMesh::EPropHandleT<short>      m_crossed;

	OpenMesh::FPropHandleT<TriToFace> m_ttf;
	OpenMesh::FPropHandleT<FaceToTri> m_ftt;
	OpenMesh::VPropHandleT<short>     m_tribound;
};

}