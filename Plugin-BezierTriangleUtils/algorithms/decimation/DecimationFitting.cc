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

bool DecimationFitting::solveLocal(FitCollection &fitColl, Scalar &error, const bool apply)
{
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

	/*if (apply) {
		std::cerr << "uvs and positions:\n";
		for (size_t i = 0; i < uvSize; ++i) {
			std::cerr << '\t' << fitColl.uv(i) << " - " << fitColl.point(i) << '\n';
		}
	}*/

	for (size_t i = 0; i < matSize; ++i) {
		Point p = fitColl[i];
		rhsx[i] = p[0];
		rhsy[i] = p[1];
		rhsz[i] = p[2];
	}

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

	bool success = solveSystem(A, rhsx, resultX);
	success = solveSystem(A, rhsy, resultY) && success;
	success = solveSystem(A, rhsz, resultZ) && success;

	// --------------------------------------------
	// write back data
	// --------------------------------------------

	if (success) {

		error = 0.;
		auto &cp = m_mesh.data(face);

		//if (apply) {
		//	std::cerr << "control points BEFORE align:\n";
		//	for (auto it = cp.cpBegin(); it != cp.cpEnd(); ++it) {
		//		std::cerr << '\t' << *it << '\n';
		//	}
		//	std::cerr << "calculated control points:\n";
		//	for (size_t i = 0; i < cpNum; ++i) {
		//		std::cerr << '\t' << Point(resultX[i], resultY[i], resultZ[i]) << '\n';
		//	}
		//}

		//cp.align(degree,
		//	Point(resultX[0], resultY[0], resultZ[0]),
		//	Point(resultX[degree], resultY[degree], resultZ[degree]),
		//	Point(resultX[cpNum - 1], resultY[cpNum - 1], resultZ[cpNum - 1])
		//);

		//if (apply) {
		//	std::cerr << "control points AFTER align:\n";
		//	for (auto it = cp.cpBegin(); it != cp.cpEnd(); ++it) {
		//		std::cerr << '\t' << *it << '\n';
		//	}
		//}

		if (apply) {
			// set face control points to result points and calculate max error
			for (size_t i = 0; i < cpNum; ++i) {
				Point p(resultX[i], resultY[i], resultZ[i]);
				cp.controlPoint(i, p);
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

bool DecimationFitting::test(BezierTMesh *mesh)
{
	assert(mesh != nullptr);



	return true;
}

} // namespace betri
