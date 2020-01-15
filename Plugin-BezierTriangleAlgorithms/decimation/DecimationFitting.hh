#pragma once

#include "../common/Fitting.hh"

namespace betri
{

class DecimationFitting : public Fitting
{
public:

	DecimationFitting() = delete;

	DecimationFitting(BezierTMesh &mesh, Solver solver=Solver::normal_equation) : Fitting(mesh) {
		m_solver = solver;
	}

	bool solve() override;

	void prepare() override;
	void cleanup() override;

	void setBarycentricCoords(std::vector<Vec2> &bary);

	bool solveLocal(
		FitCollection &fitColl,
		Scalar &error,
		const bool apply,
		const bool interpolate
	);

private:

	std::vector<Vec2> m_samples;
};

} // namespace betri