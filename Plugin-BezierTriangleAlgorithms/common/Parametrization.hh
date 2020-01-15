#pragma once

#include "Common.hh"

#include <Eigen/Sparse>
#include <Eigen/SparseCore>

namespace betri
{

using EigenSpMatT = Eigen::SparseMatrix<Scalar>;
using EigenTripletT = Eigen::Triplet<Scalar>;
using EigenVectorT = Eigen::VectorXd;

/**
 * @brief Computes a Laplace-based iterative parametrization of a given surface
 *
 */
class Parametrization
{
public:

	using Vertices = std::vector<VertexHandle>;

	explicit Parametrization(BezierTMesh &mesh) : m_mesh(mesh) {}

	// directly solve parametrization
	virtual bool solve() = 0;

	// add/remove needed properties for weights, texture coords etc.
	virtual void prepare() = 0;
	virtual void cleanup() = 0;

protected:

	///////////////////////////////////////////////////////////
	// member variables
	///////////////////////////////////////////////////////////

	BezierTMesh &m_mesh;
};

}

