#pragma once

#include "Common.hh"
#include "../voronoi/ShortestPath.hh"
#include "NGonMapper.hh"

#include <ACG/Math/VectorT.hh>
#include <OpenMesh/Core/Utils/Property.hh>

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

	explicit Parametrization(
		BezierTMesh &mesh,
		BezierTMesh &ctrl,
		OpenMesh::VPropHandleT<VertexToTri> &vtt,
		OpenMesh::FPropHandleT<TriToVertex> &ttv,
		OpenMesh::FPropHandleT<FaceHandle> &pred
	) :
		m_mesh(mesh),
		m_ctrl(ctrl),
		m_ttv(ttv),
		m_vtt(vtt),
		m_pred(pred),
		m_inner(nullptr),
		m_weightType(Uniform),
		m_mapper(mesh, vtt)
	{
		prepare();
	}

	~Parametrization()
	{
		cleanup();
	}

	// directly solve parametrization
	bool solve();

	bool solveLocal(const FaceHandle face);
	bool solveLocal(const VertexHandle vh);

	// computes weights (for complete mesh)
	static void calcWeights(
		BezierTMesh &mesh,
		WeightType weightType,
		OpenMesh::VPropHandleT<Scalar> &vweight,
		OpenMesh::EPropHandleT<Scalar> &eweight
	);

	void calcWeights(const VertexHandle vh);
	void calcWeights();

	static bool test(BezierTMesh *mesh=nullptr);

	static constexpr char *vweightName = "vWeightProp";
	static constexpr char *eweightName = "eWeightProp";
	static constexpr char *sysidName = "sysidProp";

private:

	// add/remove needed properties for weights, texture coords etc.
	void prepare();
	void cleanup();

	Scalar& weight (EdgeHandle _eh) { return m_mesh.property(m_eweight, _eh); }
	Scalar& weight (VertexHandle _vh) { return m_mesh.property(m_vweight, _vh); }
	Vec2& hmap (VertexHandle _vh) { return vtt(_vh).uv; }
	int& sysid (VertexHandle _vh) { return m_mesh.property(m_sysid, _vh); }

	TriToVertex& ttv(FaceHandle fh) { return m_ctrl.property(m_ttv, fh); }
	VertexToTri& vtt(VertexHandle vh) { return m_mesh.property(m_vtt, vh); }

	// initialize uv coordinates
	void initCoords(const FaceHandle face);
	void initCoords(const VertexHandle vh);

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

    // helper variable
    size_t nv_inner_;

    // OpenMesh mesh properties holding the texture coordinates and weights
	OpenMesh::VPropHandleT<Scalar>			m_vweight;
	OpenMesh::EPropHandleT<Scalar>			m_eweight;
	OpenMesh::VPropHandleT<int>				m_sysid;

	OpenMesh::FPropHandleT<TriToVertex>		m_ttv;
	OpenMesh::VPropHandleT<VertexToTri>		m_vtt;
	OpenMesh::FPropHandleT<FaceHandle>		m_pred;

	WeightType m_weightType;

	/// maps boundary vertices (to some convex polygon dictated by the mapper class)
	NGonMapper m_mapper;
};

}

