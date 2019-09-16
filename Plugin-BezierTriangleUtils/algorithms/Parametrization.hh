#pragma once


#include <ACG/Math/VectorT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <string>

#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

namespace betri
{

struct TriToVertex
{
	std::set<BezierTMesh::VertexHandle> inner; // inner vertices
	std::set<BezierTMesh::VertexHandle> outer; // outer boundary vertices (found using shortest paths)

	void set(std::set<BezierTMesh::VertexHandle> &in, std::set<BezierTMesh::VertexHandle> &out)
	{
		inner = in;
		outer = out;
	}

	bool isOut(const BezierTMesh::VertexHandle v)
	{
		return outer.find(v) != outer.end();
	}

	bool isIn(const BezierTMesh::VertexHandle v)
	{
		return inner.find(v) != inner.end();
	}
};

struct VertexToTri
{
	BezierTMesh::FaceHandle face; // delaunay triangle in control mesh (invalid if border)
	ACG::VectorT<BezierTMesh::Scalar, 2> uv; // parameterization

	void set(double u, double v)
	{
		uv[0] = u;
		uv[1] = v;
	}
};

/**
 * @brief Computes a Laplace-based iterative parametrization of a surface
 *
 */
class Parametrization
{

public:
	// Useful typedefs
	typedef BezierTMesh::VertexHandle      VertexHandle;
	typedef BezierTMesh::EdgeHandle        EdgeHandle;
	typedef BezierTMesh::FaceHandle        FaceHandle;

	typedef BezierTMesh::HalfedgeHandle    HalfedgeHandle;
	typedef BezierTMesh::VertexIter        VertexIter;
	typedef BezierTMesh::VertexVertexIter  VertexVertexIter;
	typedef BezierTMesh::VertexFaceIter    VertexFaceIter;
	typedef BezierTMesh::FaceVertexIter    FaceVertexIter;
	typedef BezierTMesh::EdgeIter          EdgeIter;
	typedef BezierTMesh::Scalar            Scalar;
	typedef BezierTMesh::Point             Point;
	typedef ACG::VectorT<Scalar,2>         Vec2;
	typedef ACG::VectorT<Scalar,3>         Vec3;

	typedef Eigen::SparseMatrix<Scalar> EigenSpMatT;
	typedef Eigen::Triplet<Scalar>      EigenTripletT;
	typedef Eigen::VectorXd             EigenVectorT;

	using Vertices = std::set<VertexHandle>;

	enum WeightType
	{
		Cotangent, Uniform
	};

	Parametrization(
		BezierTMesh &mesh,
		BezierTMesh &ctrl,
		OpenMesh::FPropHandleT<TriToVertex> &ttv,
		OpenMesh::VPropHandleT<VertexToTri> &vtt,
		OpenMesh::FPropHandleT<FaceHandle> &pred
	) :
		m_mesh(mesh),
		m_ctrl(ctrl),
		m_ttv(ttv),
		m_vtt(vtt),
		m_pred(pred),
		m_inner(nullptr),
		m_outer(nullptr),
		m_weightType(Uniform)
	{
		prepare();
	}

	/** Useful helper functions!
	 * use these for getting and setting the:
	 *  edge weights
	 *  vertex weights
	 *  texcoords
	 *  and for getting the equation system index of inner mesh vertices
	 */
	Scalar& weight (EdgeHandle _eh) { return m_mesh.property(m_eweight, _eh); }
	Scalar& weight (VertexHandle _vh) { return m_mesh.property(m_vweight, _vh); }
	Vec2& hmap (VertexHandle _vh) { return vtt(_vh).uv; }
	int& sysid (VertexHandle _vh) { return m_mesh.property(m_sysid, _vh); }

	int& id (FaceHandle _fh) { return m_mesh.property(m_id, _fh); }

	TriToVertex& ttv(FaceHandle fh) { return m_ctrl.property(m_ttv, fh); }
	VertexToTri& vtt(VertexHandle vh) { return m_mesh.property(m_vtt, vh); }

	// directly solve parametrization
	void solve();

	static constexpr char *vweightName = "vWeightProp";
	static constexpr char *eweightName = "eWeightProp";
	static constexpr char *sysidName = "sysidProp";

private:

	// add/remove needed properties for weights, texture coords etc.
	void prepare();
	void cleanup();

	bool isCorner(const VertexHandle v, const FaceHandle face)
	{
		for (auto he = m_mesh.cvoh_begin(v); he != m_mesh.cvoh_end(v); ++he) {
			if (isSeedFace(m_mesh.face_handle(*he)) && ttv(face).isOut(v)) {
				return true;
			}
		}

		return false;
	}

	bool isSeed(const VertexHandle v)
	{
		for (auto f = m_mesh.cvf_begin(v); f != m_mesh.cvf_end(v); ++f) {
			if (isSeedFace(*f)) {
				return true;
			}
		}

		return false;
	}

	bool isSeedFace(const FaceHandle f)
	{
		return !m_mesh.property(m_pred, f).is_valid();
	}

	// computes weights
	void calcWeights();

	// initialize texture coordinates
	void initCoords(const FaceHandle face);

	void solveLocal(Vertices &inner, Vertices &outer, const FaceHandle face);

	// Function for adding the entries of one row in the equation system
	void add_row_to_system(
		std::vector<EigenTripletT>& _triplets,
		EigenVectorT& _rhsu,
		EigenVectorT& _rhsv,
		VertexHandle _origvh
	);


private:

    BezierTMesh &m_mesh, &m_ctrl;
	Vertices *m_inner;
	Vertices *m_outer;

    // helper variables
    size_t nv_total_;
    size_t nv_inner_;
    size_t nv_bdry_;

    // OpenMesh mesh properties holding the texture coordinates and weights
	OpenMesh::VPropHandleT<Scalar>			m_vweight;
	OpenMesh::EPropHandleT<Scalar>			m_eweight;
	OpenMesh::VPropHandleT<int>				m_sysid;
	OpenMesh::FPropHandleT<int>				m_id;

	OpenMesh::FPropHandleT<TriToVertex>		m_ttv;
	OpenMesh::VPropHandleT<VertexToTri>		m_vtt;
	OpenMesh::FPropHandleT<FaceHandle>		m_pred;

	WeightType m_weightType;
};

}

