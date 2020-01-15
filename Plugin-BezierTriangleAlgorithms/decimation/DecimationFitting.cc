#include "DecimationFitting.hh"

namespace betri
{

void DecimationFitting::prepare()
{}

void DecimationFitting::cleanup()
{}

void DecimationFitting::setBarycentricCoords(std::vector<Vec2> &bary)
{
	m_samples = bary;
}

bool DecimationFitting::solve()
{
	return true;
}

bool DecimationFitting::solveLocal(
	FitCollection &fitColl,
	Scalar &error,
	const bool apply,
	const bool interpolate
) {
	FaceHandle face = fitColl.face;

	size_t degree = m_mesh.degree();
	size_t cpNum = pointsFromDegree(degree);

	size_t uvSize = fitColl.size();
	assert(uvSize == m_samples.size());
	size_t matSize = uvSize;

	// sample surface at given (u,v) coordinates -> rhs
	// system matrix
	EigenMatT A(matSize, cpNum);
	A.setZero();

	// right hand sides for x,y,z coordinates
	EigenVectorT rhsx(matSize);
	rhsx.setZero();
	EigenVectorT rhsy(matSize);
	rhsy.setZero();
	EigenVectorT rhsz(matSize);
	rhsz.setZero();

	// resulting coordinates for x,y,z
	EigenVectorT resultX(cpNum);
	resultX.setZero();
	EigenVectorT resultY(cpNum);
	resultY.setZero();
	EigenVectorT resultZ(cpNum);
	resultZ.setZero();

	// --------------------------------------------
	// setup matrix and right-hand side
	// --------------------------------------------

	for (size_t i = 0; i < matSize; ++i) {
		rhsx[i] = fitColl[i][0];
		rhsy[i] = fitColl[i][1];
		rhsz[i] = fitColl[i][2];
	}

	if (apply && !interpolate) {
		auto &cp = m_mesh.data(face);
		for (size_t i = 0; i < cpNum; ++i) {
			Point p = cp.controlPoint(i);
			resultX[i] = p[0];
			resultY[i] = p[1];
			resultZ[i] = p[2];
		}
	}

	// fix edge control points for the edge
	// that lies on the 1-ring boundary

	for (size_t row = 0; row < matSize; ++row) {

		Vec2 uv = m_samples[row];

		for (size_t i = 0, column = 0; i <= degree; ++i) {
			for (size_t j = 0; j + i <= degree; ++j, ++column) {
				A(row, column) = calcCoeffs(uv, i, j, degree);
			}
		}
	}

	// --------------------------------------------
	// solve
	// --------------------------------------------

	bool success = solveSystem(A, rhsx, resultX, m_solver);
	success = solveSystem(A, rhsy, resultY, m_solver) && success;
	success = solveSystem(A, rhsz, resultZ, m_solver) && success;

	// --------------------------------------------
	// write back data
	// --------------------------------------------

	if (success) {

		error = 0.;

		if (apply) {

			Point zero(0.0);

			auto &cp = m_mesh.data(face);
			// set control points
			for (size_t i = 0; i < cpNum; ++i) {
				Point p(resultX[i], resultY[i], resultZ[i]);
				if (interpolate || cp.controlPoint(i) == zero) {
					cp.controlPoint(i, p);
				}
			}

			// set control points for adj faces (only those for the incident edge)
			for (auto h_it = m_mesh.cfh_begin(face), h_e = m_mesh.cfh_end(face);
				h_it != h_e; ++h_it
			) {
				m_mesh.copyEdgeControlPoints(
					face,
					m_mesh.opposite_face_handle(*h_it),
					*h_it
				);
			}

			// calculate errors
			for (size_t i = 0; i < matSize; ++i) {
				Point p(rhsx[i], rhsy[i], rhsz[i]);
				error = std::max(
					error,
					(p - evalSurface(cp.points(), m_samples[i], degree)).norm()
				);
			}

		} else {
			std::vector<Point> cps;
			cps.reserve(cpNum);

			// only calculate max error
			for (size_t i = 0; i < cpNum; ++i) {
				cps.push_back(Point(resultX[i], resultY[i], resultZ[i]));
			}

			// calculate errors
			for (size_t i = 0; i < matSize; ++i) {
				Point p(rhsx[i], rhsy[i], rhsz[i]);
				error = std::max(
					error,
					(p - evalSurface(cps, m_samples[i], degree)).norm()
				);
			}
		}
	}

	return success;
}

} // namespace betri
