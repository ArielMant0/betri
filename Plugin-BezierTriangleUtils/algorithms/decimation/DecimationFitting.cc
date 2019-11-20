#include "DecimationFitting.hh"

namespace betri
{

void DecimationFitting::prepare()
{}

void DecimationFitting::cleanup()
{}

bool DecimationFitting::solve()
{
	return true;
}

bool DecimationFitting::solveLocal(FitCollection &fitColl, Scalar &error, const bool apply)
{
	FaceHandle face = fitColl.face;

	fitColl.sort();

	size_t degree = m_mesh.degree();
	size_t cpNum = pointsFromDegree(degree);

	size_t uvSize = fitColl.uvs.size();
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

	/*if (apply) {
		std::cerr << "uvs and positions:\n";
		for (size_t i = 0; i < uvSize; ++i) {
			std::cerr << '\t' << fitColl.uv(i) << " - " << fitColl.point(i) << '\n';
		}
	}*/

	for (size_t i = 0; i < matSize; ++i) {
		Point p = fitColl.point(i);
		rhsx[i] = p[0];
		rhsy[i] = p[1];
		rhsz[i] = p[2];
	}

	for (size_t row = 0; row < matSize; ++row) {

		Vec2 uv = fitColl.uv(row);

		for (size_t i = 0, column = 0; i <= degree; ++i) {
			for (size_t j = 0; j + i <= degree; ++j, ++column) {
				A(row, column) = calcCoeffs(uv, i, j, degree);
			}
		}
	}

	// --------------------------------------------
	// solve
	// --------------------------------------------

	bool success = solveSystem(A, rhsx, resultX);
	success = solveSystem(A, rhsy, resultY) && success;
	success = solveSystem(A, rhsz, resultZ) && success;

	// --------------------------------------------
	// write back data
	// --------------------------------------------

	if (success) {
		error = 0.;
		if (apply) {
			// set face control points to result points and calculate max error

			auto &cp = m_mesh.data(face);
			// resize control point vector
			cp.prepare(cpNum);

			std::cerr << "\nface " << face << " has control points:\n";
			// write control point positions back
			for (size_t i = 0; i < cpNum; ++i) {
				Point p(resultX[i], resultY[i], resultZ[i]);
				error = std::max(error, (p - cp.controlPoint(i)).norm());
				cp.controlPoint(i, p);
				std::cerr << "\t(" << i << ") = " << p << "\n";
			}
		} else {
			// only calculate max error
			auto &cp = m_mesh.data(face);

			for (size_t i = 0; i < cpNum; ++i) {
				Point p(resultX[i], resultY[i], resultZ[i]);
				error = std::max(error, (p - cp.controlPoint(i)).norm());
			}
		}
	}

	return success;
}

} // namespace betri
