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

bool Fitting::solveLocal(const FaceHandle face)
{
	size_t nv_inner_ = (m_degree + 1)*(m_degree + 2) / 2;
	Vertices &inner = ttv(face).inner;

	auto ab = ShortestPath::path(ttv(face)[0], ttv(face)[1]).list();
	auto bc = ShortestPath::path(ttv(face)[1], ttv(face)[2]).list();
	auto ca = ShortestPath::path(ttv(face)[2], ttv(face)[0]).list();

	Vertices outVerts;
	size_t outer = 3;//std::min(nv_inner_, ab.size() + bc.size() + ca.size());
	size_t perPath = outer / 3;

	for (size_t i = 0; i < ab.size() && outVerts.size() < perPath; ++i) {
		outVerts.push_back(ab[i]);
	}
	bool reverse = ab.back() == bc.back();
	for (size_t i = 0; i < bc.size() && outVerts.size() < 2*perPath; ++i) {
		outVerts.push_back(bc[reverse ? bc.size()-i-1 : i]);
	}
	reverse = reverse ? bc.front() == ca.back() : bc.back() == ca.back();
	for (size_t i = 0; i < ca.size() && outVerts.size() < outer; ++i) {
		outVerts.push_back(ca[reverse ? ca.size() - i - 1 : i]);
	}

	// TODO: add min for very large meshes
	size_t matSize = inner.size() + outVerts.size();
	//size_t matSize = inner.size();
	//size_t matSize = std::min(nv_inner_, std::max(inner.size() / 2, nv_inner_));
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

	// TODO: does vertex order matter?
	for (size_t i = 0; i < matSize; ++i) {
		VertexHandle vh = i < inner.size() ? inner[i] : outVerts[i - inner.size()];
		Point p = m_mesh.point(vh);
		std::cerr << vh << " has uv " << hmap(vh) << std::endl;
		rhsx[i] = p[0];
		rhsy[i] = p[1];
		rhsz[i] = p[2];
	}

	for (size_t row = 0; row < matSize; ++row) {
		VertexHandle vh = row < inner.size() ? inner[row] : outVerts[row - inner.size()];
		for (size_t i = 0, column=0; i <= m_degree; ++i) {
			for (size_t j = 0; j + i <= m_degree; ++j, ++column) {
				A(row, column) = calcCoeffs(vh, i, j);
			}
		}
	}

	std::cerr << "\nmatrix is\n" << A << "\n";
	std::cerr << "-> rhs x\n" << rhsx << "\n";
	std::cerr << "-> rhs y\n" << rhsy << "\n";
	std::cerr << "-> rhs z\n" << rhsz << "\n";

	auto solver = (A.transpose() * A).ldlt();

	bool error = false;
	resultX = solver.solve(A.transpose() * rhsx);
	if (solver.info() != Eigen::Success) {
		error = true;
		std::cerr << __FUNCTION__ << ": solve failed for x! (" << solver.info() << ")\n";
	}

	resultY = solver.solve(A.transpose() * rhsy);
	if (solver.info() != Eigen::Success) {
		error = true;
		std::cerr << __FUNCTION__ << ": solve failed for y! (" << solver.info() << ")\n";
	}

	resultZ = solver.solve(A.transpose() * rhsz);
	if (solver.info() != Eigen::Success) {
		error = true;
		std::cerr << __FUNCTION__ << ": solve failed for z! (" << solver.info() << ")\n";
	}

	if (!error) {
		// write control point positions back
		m_ctrl.data(face).prepare(nv_inner_);
		for (size_t i = 0; i < nv_inner_; ++i) {
			m_ctrl.data(face).controlPoint(i, Point(resultX(i), resultY(i), resultZ(i)));
		}

		std::cerr << "\nface " << face << " has control points:\n";
		size_t i(0u);
		for (Point p : m_ctrl.data(face).points()) {
			std::cerr << "\t(" << i++ << ") = " << p << "\n";
		}
	} else {
		m_ctrl.recalculateCPs(face);
	}
	return !error;
}

}