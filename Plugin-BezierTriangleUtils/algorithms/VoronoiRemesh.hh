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

		static constexpr char *TRITOFACE = "ttf";
		static constexpr char *FACETOTRI = "ftt";
	};

	struct TriangleToFace
	{
		std::vector<FH> faces;

		void set(std::vector<FH> &faces_)
		{
			faces = faces_;
		}

		std::vector<P> points(BezierTMesh &mesh)
		{
			std::vector<P> result;
			std::transform(faces.begin(), faces.end(), std::back_inserter(result),
				[mesh](BezierTMesh::FaceHandle &f) { return mesh.calc_face_centroid(f); }
			);
		}
	};

	struct FaceToTriangle
	{
		FH triangle;

		void set(FH &face)
		{
			triangle = face;
		}

		BezierTMesh::Point point(BezierTMesh &mesh)
		{
			return mesh.calc_face_centroid(triangle);
		}
	};

	VoronoiRemesh(BezierTMesh &mesh) : m_mesh(mesh)
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

	ID& id(FH fh) { return m_mesh.property(m_region, fh); }
	FH& pred(FH fh) { return m_mesh.property(m_pred, fh); }
	double& dist(FH fh) { return m_mesh.property(m_distance, fh); }

	FaceToTriangle& ftt(FH fh) { return m_mesh.property(m_ftt, fh); }
	TriangleToFace& ttf(FH fh) { return m_mesh.property(m_ttf, fh); }

	char& crossed(EH eh) { return m_mesh.property(m_crossed, eh); }
	char& crossed(HH he) { return crossed(m_mesh.edge_handle(he)); }

	bool isCrossed(EH eh) { return (int)crossed(eh) == 1; }
	bool isCrossed(HH he) { return isCrossed(m_mesh.edge_handle(he)); }

private:

	void partition(std::vector<FH> &seeds, bool useColors=true);

	void dijkstra(std::vector<FH> &seeds, bool useColors=true);

	// member variables
	BezierTMesh &m_mesh;

	// property handles
	OpenMesh::FPropHandleT<ID>       m_region;
	OpenMesh::FPropHandleT<FH>       m_pred;
	OpenMesh::FPropHandleT<double>   m_distance;
	// must be something other than bool because vector<bool> is handled uniquely in C++
	OpenMesh::EPropHandleT<char>     m_crossed;

	OpenMesh::FPropHandleT<TriangleToFace> m_ttf;
	OpenMesh::FPropHandleT<FaceToTriangle> m_ftt;
};

}