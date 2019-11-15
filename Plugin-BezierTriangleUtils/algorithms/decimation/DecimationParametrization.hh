#pragma once

#include "../common/Parametrization.hh"
#include "../common/NGonMapper.hh"

#include <ACG/Math/VectorT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

namespace betri
{

class DecimationParametrization : public Parametrization
{
public:

	explicit DecimationParametrization(BezierTMesh &mesh, OpenMesh::VPropHandleT<Vec2> &uvProp) :
		Parametrization(mesh), m_mapper(mesh, uvProp)
	{
		prepare();
	}

	~DecimationParametrization()
	{
		cleanup();
	}

	// directly solve parametrization
	bool solve() override;

    // add/remove needed properties for weights, texture coords etc.
	void prepare() override;
	void cleanup() override;

	bool solveLocal(const FaceHandle face);

	// computes weights (for complete mesh)
	static void calcWeights(
		BezierTMesh &mesh,
		OpenMesh::VPropHandleT<Scalar> &vweight,
		OpenMesh::PropertyManager<OpenMesh::VPropHandleT<bool>, BezierTMesh> &inFace
	);

	void calcWeights(const FaceHandle face);

	static bool test(BezierTMesh *mesh=nullptr);

private:

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

    // helper variable
    size_t nv_inner_;

	/// maps boundary vertices (to some convex polygon dictated by the mapper class)
	NGonMapper<Vec2> m_mapper;
};

}

