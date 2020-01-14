#pragma once

#include "Common.hh"

#include <ACG/Math/VectorT.hh>

#include <Eigen/Dense>

namespace betri
{

class Fitting
{
public:

	enum Solver
	{
		normal_equation,
		qr_decomposition,
		adaptive
	};

	using EigenMatT = Eigen::MatrixXd;
	using EigenVectorT = Eigen::VectorXd;

	using Vertices = std::vector<VertexHandle>;


	explicit Fitting(BezierTMesh &mesh) :
		m_mesh(mesh), m_degree(1), m_solver(Solver::normal_equation) {}

	void degree(size_t degree) { m_degree = degree; }
	size_t degree() const { return m_degree; }

	void solver(Solver solver) { m_solver = solver; }
	Solver solver() const { return m_solver; }

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

	static bool solveSystem(
		EigenMatT &A,
		EigenVectorT &rhs,
		EigenVectorT &result,
		Solver solver=normal_equation
	) {
		Eigen::ComputationInfo info;

		if (solver == adaptive) {
			solver = A.rows() > 100 ? qr_decomposition : normal_equation;
		}

		// solve system with specified solver
		switch (solver) {
			case normal_equation:
			{
				auto solver = (A.transpose() * A).ldlt();
				result = solver.solve(A.transpose() * rhs);
				info = solver.info();
				break;
			}
			case qr_decomposition:
			{
				auto solver = A.colPivHouseholderQr();
				result = solver.solve(rhs);
				info = solver.info();
				break;
			}
		}

		if (info != Eigen::Success) {
			std::cerr << __FUNCTION__ << ": solver failed! (" << info << ")\n";
			return false;
		}

		return true;
	}

	///////////////////////////////////////////////////////////
	// member variables
	///////////////////////////////////////////////////////////

	BezierTMesh &m_mesh;
	size_t m_degree;
	Solver m_solver;
};

} // namespace betri
