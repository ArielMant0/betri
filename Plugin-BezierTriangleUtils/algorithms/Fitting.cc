#include "Fitting.hh"

#include <algorithm>

namespace betri
{

void Fitting::solve(unsigned int degree)
{
	m_degree = std::max(2u, degree);

	std::cerr << "fitting control points for degree " << m_degree << "\n";

	// TODO: make sure edge control points are the same for adj faces
	for (const FaceHandle face : m_ctrl.faces()) {
		solveLocal(ttv(face).inner, ttv(face).outer, face);
	}
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

size_t Fitting::calcCPCount(unsigned int degree)
{
	return (degree*degree + 3 * degree + 2) / 2;
}

void Fitting::solveLocal(Vertices &inner, Vertices &outer, const FaceHandle face)
{
	size_t nv_inner_ = calcCPCount(m_degree);

	assert(inner.size() >= nv_inner_);

	// system matrix
	EigenMatT A(nv_inner_, nv_inner_);

	// right hand sides for x,y,z coordinates
	EigenVectorT rhsx(nv_inner_);
	rhsx.setZero();
	EigenVectorT rhsy(nv_inner_);
	rhsy.setZero();
	EigenVectorT rhsz(nv_inner_);
	rhsz.setZero();

	// resulting coordinates for x,y,z
	EigenVectorT resultX(nv_inner_);
	resultX.setZero();
	EigenVectorT resultY(nv_inner_);
	resultY.setZero();
	EigenVectorT resultZ(nv_inner_);
	resultZ.setZero();

	// TODO: does vertex order matter?
	auto v = inner.begin();
	for (size_t i = 0; i < nv_inner_; ++i, ++v) {
		Point p = m_mesh.point(*v);
		rhsx[sysid(*v)] = p[0];
		rhsy[sysid(*v)] = p[1];
		rhsz[sysid(*v)] = p[2];
	}

	v = inner.begin();
	for (size_t i = 0, column = 0; i <= m_degree; ++i) {
		for (size_t j = 0; j + i <= m_degree; ++j, ++column) {
			auto coeff = calcCoeffs(*(v++), i, j);
			for (size_t row = 0; row < nv_inner_; ++row) {
				A(row, column) = coeff;
			}
		}
		assert(column <= nv_inner_);
	}

	const auto QRsolver = A.colPivHouseholderQr();
	resultX = QRsolver.solve(rhsx);
	if (QRsolver.info() != Eigen::Success)
		std::cerr << __FUNCTION__ << ": solve failed for x!" << std::endl;

	resultY = QRsolver.solve(rhsy);
	if (QRsolver.info() != Eigen::Success)
		std::cerr << __FUNCTION__ << ": solve failed for y!" << std::endl;

	resultZ = QRsolver.solve(rhsz);
	if (QRsolver.info() != Eigen::Success)
		std::cerr << __FUNCTION__ << ": solve failed for z!" << std::endl;

	v = inner.begin();
	// write control point positions back
	for (size_t i = 0; i <= nv_inner_; ++i, ++v) {
		auto vid = sysid(*v);
		m_ctrl.data(face).setPoint(i, Point(resultX(vid), resultY(vid), resultZ(vid)));
	}
}

}