#include "Fitting.hh"

#include "../voronoi/ShortestPath.hh"

#include <algorithm>

namespace betri
{

bool Fitting::solve()
{
	m_degree = m_mesh.degree();
	assert(m_degree >= 2);

	std::cerr << "fitting control points for degree " << m_degree << "\n";

	// TODO: make sure edge control points are the same for adj faces
	for (const FaceHandle face : m_ctrl.faces()) {
		if (!solveLocal(face)) return false;
	}
	return true;
}

void Fitting::prepare()
{
	if (!m_mesh.get_property_handle(m_sysid, "fit-sysid"))
		m_mesh.add_property(m_sysid, "fit-sysid");
}

void Fitting::cleanup()
{
	if (m_mesh.get_property_handle(m_sysid, "fit-sysid"))
		m_mesh.remove_property(m_sysid);
}

bool Fitting::solveSystem(EigenMatT &A, EigenVectorT & rhs, EigenVectorT &result)
{
	auto solver = (A.transpose() * A).ldlt();
	result = solver.solve(A.transpose() * rhs);
	if (solver.info() != Eigen::Success) {
		std::cerr << __FUNCTION__ << ": test failed for x! (" << solver.info() << ")\n";
		return false;
	}

	return true;
}

bool Fitting::solveLocal(const FaceHandle face)
{
	size_t nv_inner_ = pointsFromDegree(m_degree);

	Vertices &inner = ttv(face).inner;
	Vertices outVerts;

	size_t inSize = std::min(inner.size(), m_samples);
	size_t outSize = 0;

	if (inSize < m_samples) {
		auto ab = ShortestPath::path(ttv(face)[0], ttv(face)[1]).list();
		auto bc = ShortestPath::path(ttv(face)[1], ttv(face)[2]).list();
		auto ca = ShortestPath::path(ttv(face)[2], ttv(face)[0]).list();

		outSize = std::min(nv_inner_, ab.size() + bc.size() + ca.size());
		size_t perPath = outSize / 3;

		outVerts.reserve(outSize);
		for (size_t i = 0; i < ab.size() && outVerts.size() < perPath; ++i) {
			outVerts.push_back(ab[i]);
		}
		bool reverse = ab.back() == bc.back();
		for (size_t i = 0; i < bc.size() && outVerts.size() < 2 * perPath; ++i) {
			outVerts.push_back(bc[reverse ? bc.size() - i - 1 : i]);
		}
		reverse = reverse ? bc.front() == ca.back() : bc.back() == ca.back();
		for (size_t i = 0; i < ca.size() && outVerts.size() < outSize; ++i) {
			outVerts.push_back(ca[reverse ? ca.size() - i - 1 : i]);
		}
	}

	size_t matSize = inSize + outSize;
	assert(matSize > 0);

	// system matrix
	EigenMatT A(matSize, nv_inner_);
	A.setZero();

	// right hand sides for x,y,z coordinates
	EigenVectorT rhsx(matSize);
	rhsx.setZero();
	EigenVectorT rhsy(matSize);
	rhsy.setZero();
	EigenVectorT rhsz(matSize);
	rhsz.setZero();

	// resulting coordinates for x,y,z
	EigenVectorT resultX(nv_inner_);
	resultX.setZero();
	EigenVectorT resultY(nv_inner_);
	resultY.setZero();
	EigenVectorT resultZ(nv_inner_);
	resultZ.setZero();

	// --------------------------------------------
	// setup matrix and right-hand side
	// --------------------------------------------

	for (size_t i = 0; i < matSize; ++i) {
		VertexHandle vh = i < inSize ? inner[i] : outVerts[i - inSize];
		Point p = m_mesh.point(vh);
		//std::cerr << vh << " has uv " << hmap(vh) << std::endl;
		rhsx[i] = p[0];
		rhsy[i] = p[1];
		rhsz[i] = p[2];
	}

	for (size_t row = 0; row < matSize; ++row) {
		VertexHandle vh = row < inSize ? inner[row] : outVerts[row - inSize];
		for (size_t i = 0, column=0; i <= m_degree; ++i) {
			for (size_t j = 0; j + i <= m_degree; ++j, ++column) {
				A(row, column) = calcCoeffs(hmap(vh), i, j, m_degree);
			}
		}
	}

	/*
	std::cerr << "\nmatrix is\n" << A << "\n";
	std::cerr << "-> rhs x\n" << rhsx << "\n";
	std::cerr << "-> rhs y\n" << rhsy << "\n";
	std::cerr << "-> rhs z\n" << rhsz << "\n";*/

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
		// resize control point vector
		m_ctrl.data(face).prepare(nv_inner_);

		size_t mod = m_degree + 1;
		// write control point positions back
		for (size_t i = 0, count = 0; i <= m_degree; ++i) {
			for (size_t j = 0; i + j < nv_inner_; count++) {
				size_t index = i + j;

				Point p(resultX[index], resultY[index], resultZ[index]);
				m_ctrl.data(face).controlPoint(count, p);

				if (i == m_degree) break;
				j += mod;
				mod--;
			}
			mod = m_degree + 1;
		}

		std::cerr << "\nface " << face << " has control points:\n";
		size_t i(0u);
		for (Point p : m_ctrl.data(face).points()) {
			std::cerr << "\t(" << i++ << ") = " << p << "\n";
		}
	}
	return success;
}

bool Fitting::test(BezierTMesh *mesh)
{
	constexpr size_t degree = 2;
	constexpr size_t matSize = 10;

	size_t nv_inner_ = pointsFromDegree(degree);

	// system matrix
	EigenMatT A(matSize, nv_inner_);
	A.setZero();

	// right hand sides for x,y,z coordinates
	EigenVectorT rhsx(matSize);
	rhsx.setZero();
	EigenVectorT rhsy(matSize);
	rhsy.setZero();
	EigenVectorT rhsz(matSize);
	rhsz.setZero();

	// resulting coordinates for x,y,z
	EigenVectorT resultX(nv_inner_);
	resultX.setZero();
	EigenVectorT resultY(nv_inner_);
	resultY.setZero();
	EigenVectorT resultZ(nv_inner_);
	resultZ.setZero();

	// --------------------------------------------
	// data to test with
	// --------------------------------------------

	// simple triangle (flat)
	//std::array<Point, matSize> points = {{
	//	{   0.,   0.,  0. },
	//	{  .25,   0.,  0. },
	//	{  .75,   0.,  0. },
	//	{   1.,   0.,  0. },
	//	{   0.,  .25,  0. },
	//	{ .375,  .25,  0. },
	//	{  .75,  .25,  0. },
	//	{   0.,  .75,  0. },
	//	{  .25,  .75,  0. },
	//	{   0.,   1.,  0. }
	//}};

	// simple triangle (curved)
	std::array<Point, matSize> points = {{
		{   0.,   0.,   0. },
		{  .25,   0.,  .25 },
		{  .75,   0.,  .25 },
		{   1.,   0.,   0. },
		{   0.,  .25,  .25 },
		{ .375,  .25,   .5 },
		{  .75,  .25,  .25 },
		{   0.,  .75,  .25 },
		{  .25,  .75,  .25 },
		{   0.,   1.,   0. }
	}};

	std::array<Vec2, matSize> uv = {{
		{   0.,   0. },
		{  .25,   0. },
		{  .75,   0. },
		{   1.,   0. },
		{   0.,  .25 },
		{ .375,  .25 },
		{  .75,  .25 },
		{   0.,  .75 },
		{  .25,  .75 },
		{   0.,   1. }
	}};

	// --------------------------------------------
	// setup matrix and right-hand side
	// --------------------------------------------
	for (size_t i = 0; i < matSize; ++i) {
		rhsx[i] = points[i][0];
		rhsy[i] = points[i][1];
		rhsz[i] = points[i][2];
	}

	for (size_t row = 0; row < matSize; ++row) {
		for (size_t i = 0, column = 0; i <= degree; ++i) {
			for (size_t j = 0; j + i <= degree; ++j, ++column) {
				A(row, column) = calcCoeffs(uv[row], i, j, degree);
			}
		}
	}

	// --------------------------------------------
	// solve
	// --------------------------------------------

	bool result = solveSystem(A, rhsx, resultX);
	result = solveSystem(A, rhsy, resultY) && result;
	result = solveSystem(A, rhsz, resultZ) && result;

	// --------------------------------------------
	// write back data
	// --------------------------------------------

	FaceHandle face;

	const bool useMesh = mesh != nullptr;
	if (useMesh) {
		VertexHandle v0 = mesh->add_vertex(points[0]);
		VertexHandle v1 = mesh->add_vertex(points[3]);
		VertexHandle v2 = mesh->add_vertex(points[9]);
		face = mesh->add_face(v0, v1, v2);
		mesh->data(face).prepare(nv_inner_);
	}

	size_t mod = degree + 1;
	std::cerr << "control points:\n";

	for (size_t i = 0, count=0; i <= degree; ++i) {
		for (size_t j = 0; i+j < nv_inner_; count++) {
			size_t index = i + j;

			Point p(resultX[index], resultY[index], resultZ[index]);
			if (useMesh) mesh->data(face).controlPoint(count, p);
			std::cerr << '\t' << index << " : " << p << '\n';

			if (i == degree) break;
			j += mod;
			mod--;
		}
		mod = degree + 1;
	}

	if (useMesh) {
		std::cerr << "check:\n";
		for (auto b = mesh->data(face).cpBegin(); b != mesh->data(face).cpEnd(); ++b) {
			std::cerr << '\t' << *b << '\n';
		}
	}

	return result;
}

}