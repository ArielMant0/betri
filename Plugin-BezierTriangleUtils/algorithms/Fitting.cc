#include "Fitting.hh"

#include <algorithm>

namespace betri
{

void Fitting::solve()
{
	m_degree = m_mesh.degree();
	assert(m_degree >= 2);

	std::cerr << "fitting control points for degree " << m_degree << "\n";

	// TODO: make sure edge control points are the same for adj faces
	for (const FaceHandle face : m_ctrl.faces()) {
		solveLocal(ttv(face).inner, face);
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

void Fitting::solveLocal(Vertices &inner, const FaceHandle face)
{
	size_t nv_inner_ = calcCPCount(m_degree);

	assert(inner.size() >= nv_inner_);

	size_t matSize = std::min(nv_inner_, std::max(inner.size()/2, nv_inner_));

	// system matrix
	EigenMatT A(matSize, nv_inner_);

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
		Point p = m_mesh.point(inner[i]);
		std::cerr << "\t" << p << " (u,v) = " << hmap(inner[i]) << "\n";
		rhsx[i] = p[0];
		rhsy[i] = p[1];
		rhsz[i] = p[2];
	}

	for (size_t row = 0; row < matSize; ++row) {
		for (size_t i = 0, column=0; i <= m_degree; ++i) {
			for (size_t j = 0; j + i <= m_degree; ++j, ++column) {
				A(row, column) = calcCoeffs(inner[row], i, j);
			}
		}
	}

	rhsx = rhsx;//A.transpose() * rhsx;
	rhsy = rhsy;//A.transpose() * rhsy;
	rhsz = rhsz; //A.transpose() * rhsz;

	std::cerr << "\nmatrix is\n" << A << "\n";
	std::cerr << "-> rhs x\n" << rhsx << "\n";
	std::cerr << "-> rhs y\n" << rhsy << "\n";
	std::cerr << "-> rhs z\n" << rhsz << "\n";

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

	// write control point positions back
	m_ctrl.data(face).prepare(nv_inner_);
	for (size_t i = 0; i < nv_inner_; ++i) {
		Point p = { resultX(i), resultY(i), resultZ(i) };
		std::cerr << "\tcontrol point " << i << " is " << p << "\n";
		m_ctrl.data(face).controlPoint(i, p);
	}
}

}