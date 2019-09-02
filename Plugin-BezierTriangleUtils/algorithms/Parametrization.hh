#pragma once


#include <ACG/Math/VectorT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <string>

#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

namespace betri
{

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

	typedef Eigen::SparseMatrix<Scalar> EigenSpMatT;
	typedef Eigen::Triplet<Scalar>      EigenTripletT;
	typedef Eigen::VectorXd             EigenVectorT;

	using Vertices = std::vector<VertexHandle>;

	enum WeightType
	{
		Cotangent, Uniform
	};

	Parametrization(BezierTMesh &mesh) : m_mesh(mesh)
	{
		prepare();
	}

	//	std::vector<BezierTMesh::Point> &vertices,
	//	std::vector<std::array<int, 2>> &edges,
	//	WeightType weights=Cotangent
	//) : m_weightType(weights)
	//{
	//	prepare();
	//}

	/** Useful helper functions!
	 * use these for getting and setting the:
	 *  edge weights
	 *  vertex weights
	 *  texcoords
	 *  and for getting the equation system index of inner mesh vertices
	 */
	Scalar& weight (EdgeHandle _eh) { return m_mesh.property(m_eweight, _eh); }
	Scalar& weight (VertexHandle _vh) { return m_mesh.property(m_vweight, _vh); }
	Vec2& hmap (VertexHandle _vh) { return m_mesh.property(m_hmap, _vh); }
	int& sysid (VertexHandle _vh) { return m_mesh.property(m_sysid, _vh); }

	int& id (FaceHandle _fh) { return m_mesh.property(m_id, _fh); }


	// directly solve parametrization
	void solve(const std::array<VertexHandle, 3> boundary, const int rId, const char *idName);

	static constexpr char *vweightName = "vWeightProp";
	static constexpr char *eweightName = "eWeightProp";
	static constexpr char *hmapName = "hmapName";
	static constexpr char *sysidName = "sysidProp";


private:

	// add/remove needed properties for weights, texture coords etc.
	void prepare();
	void cleanup();

	// computes weights
	void calcWeights();

	// initialize texture coordinates
	void initCoords(int id);

	// Function for adding the entries of one row in the equation system
	void add_row_to_system(
		std::vector<EigenTripletT>& _triplets,
		EigenVectorT& _rhsu,
		EigenVectorT& _rhsv,
		VertexHandle _origvh
	);


private:

    BezierTMesh &m_mesh;
	Vertices m_inner;
	Vertices m_outer;

    // helper variables
    size_t nv_total_;
    size_t nv_inner_;
    size_t nv_bdry_;

    // OpenMesh mesh properties holding the texture coordinates and weights
    OpenMesh::VPropHandleT<Vec2>         m_hmap;
	OpenMesh::VPropHandleT<Scalar>       m_vweight;
	OpenMesh::EPropHandleT<Scalar>       m_eweight;
	OpenMesh::VPropHandleT<int>          m_sysid;
	OpenMesh::FPropHandleT<int>          m_id;


	WeightType m_weightType;
};

}

