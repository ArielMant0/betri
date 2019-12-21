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

	for (size_t i = 0; i < matSize; ++i) {
		rhsx[i] = fitColl[i][0];
		rhsy[i] = fitColl[i][1];
		rhsz[i] = fitColl[i][2];
	}

	if (apply) {
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

	bool success = solveSystem(A, rhsx, resultX);
	success = solveSystem(A, rhsy, resultY) && success;
	success = solveSystem(A, rhsz, resultZ) && success;

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
				if (cp.controlPoint(i) == zero) {
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

bool DecimationFitting::test(BezierTMesh *mesh)
{
	assert(mesh != nullptr);

	VertexHandle a = mesh->add_vertex(Point(1., 0.5, 0.)); // A
	VertexHandle b = mesh->add_vertex(Point(0.654508, 0.975528, 0.)); // B - to collapse into
	VertexHandle c = mesh->add_vertex(Point(0.0954915, 0.792893, 0.)); // C
	VertexHandle d = mesh->add_vertex(Point(0.0954915, 0.206107, 0.)); // D
	VertexHandle e = mesh->add_vertex(Point(0.654508, 0.0244717, 0.)); // E
	VertexHandle f = mesh->add_vertex(Point(0.5, 0.5, 0.)); // F - middle

	FaceHandle abf = mesh->add_face(a, b, f, true);
	FaceHandle bcf = mesh->add_face(b, c, f, true);
	FaceHandle cdf = mesh->add_face(c, d, f, true);
	FaceHandle def = mesh->add_face(d, e, f, true);
	FaceHandle eaf = mesh->add_face(e, a, f, true);

	mesh->update_normals();

	assert(mesh->n_vertices() == 6);
	assert(mesh->n_edges() == 10);
	assert(mesh->n_faces() == 5);

	return true;

	std::array<FitCollection, 3> remain;
	remain[0].face = cdf;
	remain[0].add(mesh->point(b));
	remain[0].add(mesh->point(d));
	remain[0].add(mesh->point(c));
	remain[0].add(mesh->point(c) * 0.5 + mesh->point(b) * 0.5);
	remain[0].add(mesh->point(d) * 0.5 + mesh->point(b) * 0.5);
	remain[0].add(mesh->point(c) * 0.5 + mesh->point(d) * 0.5);

	remain[1].face = def;
	remain[1].add(mesh->point(b));
	remain[1].add(mesh->point(e));
	remain[1].add(mesh->point(d));
	remain[1].add(mesh->point(d) * 0.5 + mesh->point(b) * 0.5);
	remain[1].add(mesh->point(e) * 0.5 + mesh->point(b) * 0.5);
	remain[1].add(mesh->point(d) * 0.5 + mesh->point(e) * 0.5);

	remain[2].face = eaf;
	remain[2].add(mesh->point(b));
	remain[2].add(mesh->point(a));
	remain[2].add(mesh->point(e));
	remain[2].add(mesh->point(e) * 0.5 + mesh->point(b) * 0.5);
	remain[2].add(mesh->point(a) * 0.5 + mesh->point(b) * 0.5);
	remain[2].add(mesh->point(e) * 0.5 + mesh->point(a) * 0.5);

	std::vector<Vec2> samples = { {
		Vec2(0., 0.),
		Vec2(0., 1.),
		Vec2(1., 0.),
		Vec2(0.5, 0.),
		Vec2(0., 0.5),
		Vec2(0.5, 0.5)
	} };

	size_t degree = 2;
	size_t cpNum = pointsFromDegree(degree);

	size_t uvSize = samples.size();
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
	EigenVectorT resultY(cpNum);
	EigenVectorT resultZ(cpNum);

	bool success = true;

	for (FitCollection &fitColl : remain) {

		resultX.setZero();
		resultY.setZero();
		resultZ.setZero();

		for (size_t i = 0; i < matSize; ++i) {
			rhsx[i] = fitColl[i][0];
			rhsy[i] = fitColl[i][1];
			rhsz[i] = fitColl[i][2];
		}

		for (size_t row = 0; row < matSize; ++row) {

			Vec2 uv = samples[row];

			for (size_t i = 0, column = 0; i <= degree; ++i) {
				for (size_t j = 0; j + i <= degree; ++j, ++column) {
					A(row, column) = calcCoeffs(uv, i, j, degree);
				}
			}
		}

		// --------------------------------------------
		// solve
		// --------------------------------------------

		success = success && solveSystem(A, rhsx, resultX);
		success = solveSystem(A, rhsy, resultY) && success;
		success = solveSystem(A, rhsz, resultZ) && success;

		if (success) {
			auto &cp = mesh->data(fitColl.face);

			std::cerr << "new control points for face " << fitColl.face << '\n';
			// set face control points to result points and calculate max error
			for (size_t i = 0; i < cpNum; ++i) {
				Point p(resultX[i], resultY[i], resultZ[i]);
				std::cerr << "\t(" << i << ") " << p << '\n';
				cp.controlPoint(i, p);
			}
		}
	}

	mesh->collapse(mesh->find_halfedge(f, b));

	mesh->garbage_collection();
	mesh->update_normals();

	return success;
}

} // namespace betri
