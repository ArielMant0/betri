#pragma once

#include "../common/Fitting.hh"

namespace betri
{

class DecimationFitting : public Fitting
{
public:

	DecimationFitting() = delete;

	DecimationFitting(BezierTMesh &mesh) : Fitting(mesh) {}

	bool solve() override;

	void prepare() override;
	void cleanup() override;

	void setBarycentricCoords(std::vector<Vec2> &bary);

	bool solveLocal(FitCollection &fitColl, Scalar &error, const bool apply);

	static bool test(BezierTMesh *mesh);

private:

	std::vector<Vec2> m_samples;
};

} // namespace betri