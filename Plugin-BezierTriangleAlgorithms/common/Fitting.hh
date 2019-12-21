#pragma once

#include "Common.hh"

#include <ACG/Math/VectorT.hh>

#include <Eigen/Dense>

namespace betri
{

class Fitting
{
public:

	using EigenMatT = Eigen::MatrixXd;
	using EigenVectorT = Eigen::VectorXd;

	using Vertices = std::vector<VertexHandle>;


	explicit Fitting(BezierTMesh &mesh) :
		m_mesh(mesh), m_degree(1) {}

	void degree(size_t degree) { m_degree = degree; }
	size_t degree() const { return m_degree; }

	virtual bool solve() = 0;


	virtual void prepare() = 0;
	virtual void cleanup() = 0;

protected:

	static Scalar calcCoeffs(Vec2 uv, int i, int j, size_t degree)
	{
		assert(std::islessequal(uv[0], 1.0));
		assert(std::islessequal(uv[1], 1.0));
		return eval(i, j, uv, degree);
	}

	static bool solveSystem(EigenMatT &A, EigenVectorT &rhs, EigenVectorT &result)
	{
		auto solver = (A.transpose() * A).ldlt();
		result = solver.solve(A.transpose() * rhs);
		if (solver.info() != Eigen::Success) {
			std::cerr << __FUNCTION__ << ": solver failed! (" << solver.info() << ")\n";
			return false;
		}

		return true;
	}

	///////////////////////////////////////////////////////////
	// member variables
	///////////////////////////////////////////////////////////

	BezierTMesh &m_mesh;
	size_t m_degree;
};

} // namespace betri
