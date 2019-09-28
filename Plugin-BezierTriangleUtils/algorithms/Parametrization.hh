#pragma once

#include "Common.hh"
#include "ShortestPath.hh"

#include <ACG/Math/VectorT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
//#include <string>

#include <Eigen/Sparse>
#include <Eigen/SparseCore>

namespace betri
{

using EigenSpMatT = Eigen::SparseMatrix<Scalar>;
using EigenTripletT = Eigen::Triplet<Scalar>;
using EigenVectorT = Eigen::VectorXd;

/**
 * @brief Computes a Laplace-based iterative parametrization of a surface
 *
 */
class Parametrization
{

public:

	using Vertices = std::vector<VertexHandle>;

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

	~Parametrization()
	{
		cleanup();
		if (m_outer) delete m_outer;
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
		auto a = ttv(face)[0];
		auto b = ttv(face)[1];
		auto c = ttv(face)[2];

		auto ab = ShortestPath::path(a, b);
		auto bc = ShortestPath::path(b, c);
		auto ca = ShortestPath::path(c, a);

		for (auto he = m_mesh.cvih_begin(v); he != m_mesh.cvih_end(v); ++he) {
			if (ab.list()[0] == *he || bc.list()[0] == *he || ca.list()[0] == *he) {
				return true;
			}
		}

		return false;
	}

	bool isInner(const VertexHandle v, const FaceHandle face)
	{
		return std::find(ttv(face).inner.begin(), ttv(face).inner.end(), v) != ttv(face).inner.end();
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

	void solveLocal(const FaceHandle face);

	// Function for adding the entries of one row in the equation system
	void addRow(
		std::vector<EigenTripletT>& _triplets,
		EigenVectorT& _rhsu,
		EigenVectorT& _rhsv,
		VertexHandle _origvh,
		FaceHandle face
	);

	///////////////////////////////////////////////////////////
	// member variables
	///////////////////////////////////////////////////////////

    BezierTMesh &m_mesh, &m_ctrl;
	Vertices *m_inner;
	std::vector<VertexHandle> *m_outer;

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

