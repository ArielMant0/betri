#pragma once

#include "../common/Parametrization.hh"
#include "../common/NGonMapper.hh"
#include "ShortestPath.hh"

#include <ACG/Math/VectorT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

namespace betri
{

/**
 * @brief Computes a Laplace-based iterative parametrization of a surface
 *
 */
class VoronoiParametrization : public Parametrization
{
public:

	enum WeightMode
	{
		uniform = 0,
		cotangent
	};

    VoronoiParametrization() = delete;

	explicit VoronoiParametrization(
		BezierTMesh &mesh,
		BezierTMesh &ctrl,
		int weightMode,
		OpenMesh::VPropHandleT<VertexToTri> &vtt,
		OpenMesh::FPropHandleT<TriToVertex> &ttv,
		OpenMesh::FPropHandleT<FaceHandle> &pred
	) :
		Parametrization(mesh),
		m_ctrl(ctrl),
		m_ttv(ttv),
		m_vtt(vtt),
		m_pred(pred),
		m_inner(nullptr),
		m_mapper(mesh, vtt)
	{
		switch (weightMode) {
			case 1: m_mode = cotangent; break;
			default: m_mode = uniform;
		}
		prepare();
	}

	~VoronoiParametrization()
	{
		cleanup();
	}

	// directly solve parametrization
	bool solve() override;
	// add/remove needed properties for weights, texture coords etc.
	void prepare() override;
	void cleanup() override;

	bool solveLocal(const FaceHandle face);

	void calcWeights(const FaceHandle face);

    // computes weights (for complete mesh)
	static void calcWeights(
		BezierTMesh &mesh,
		WeightMode &mode,
		OpenMesh::VPropHandleT<Scalar> &vweight,
		OpenMesh::PropertyManager<OpenMesh::VPropHandleT<bool>, BezierTMesh> &inFace
	);

	static constexpr char *vweightName = "vWeightProp";
	static constexpr char *eweightName = "eWeightProp";
	static constexpr char *sysidName = "sysidProp";

private:

	Scalar& weight(VertexHandle _vh) { return m_mesh.property(m_vweight, _vh); }
	Scalar& weight(EdgeHandle eh) { return m_mesh.property(m_eweight, eh); }

	Vec2& hmap(VertexHandle _vh) { return vtt(_vh).uv; }
	int& sysid(VertexHandle _vh) { return m_mesh.property(m_sysid, _vh); }

	TriToVertex& ttv(FaceHandle fh) { return m_ctrl.property(m_ttv, fh); }
	VertexToTri& vtt(VertexHandle vh) { return m_mesh.property(m_vtt, vh); }

	// initialize uv coordinates
	void initCoords(const FaceHandle face);

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

    BezierTMesh &m_ctrl;
	Vertices *m_inner;

    // helper variable
    size_t nv_inner_;

	WeightMode m_mode;

    // OpenMesh mesh properties holding the texture coordinates and weights
	OpenMesh::VPropHandleT<Scalar>			m_vweight;
	OpenMesh::EPropHandleT<Scalar>			m_eweight;
	OpenMesh::VPropHandleT<int>				m_sysid;

	OpenMesh::FPropHandleT<TriToVertex>		m_ttv;
	OpenMesh::VPropHandleT<VertexToTri>		m_vtt;
	OpenMesh::FPropHandleT<FaceHandle>		m_pred;

	/// maps boundary vertices (to some convex polygon dictated by the mapper class)
	NGonMapper<VertexToTri> m_mapper;
};

}

