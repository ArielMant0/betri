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

	bool solveLocal(FitCollection &fitColl, Scalar &error, const bool apply);

private:

};

} // namespace betri