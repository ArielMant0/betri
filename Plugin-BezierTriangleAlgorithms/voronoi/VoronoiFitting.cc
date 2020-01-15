#include "VoronoiFitting.hh"

#include "ShortestPath.hh"

#include <algorithm>

namespace betri
{

bool VoronoiFitting::solve()
{
	m_degree = m_mesh.degree();
	assert(m_degree >= 2);

	std::cerr << "fitting control points for degree " << m_degree << "\n";

	for (const FaceHandle face : m_ctrl.faces()) {
		if (!solveLocal(face, false)) return false;
	}
	return true;
}

void VoronoiFitting::prepare()
{
	if (!m_mesh.get_property_handle(m_sysid, "fit-sysid"))
		m_mesh.add_property(m_sysid, "fit-sysid");
}

void VoronoiFitting::cleanup()
{
	if (m_mesh.get_property_handle(m_sysid, "fit-sysid"))
		m_mesh.remove_property(m_sysid);
}

void VoronoiFitting::sortInner(const FaceHandle face)
{
	std::sort(ttv(face).inner.begin(), ttv(face).inner.end(),
		[&](const VertexHandle v0, const VertexHandle v1) {
			return hmap(v0)[0] < hmap(v1)[0];
		}
	);
}

bool VoronoiFitting::solveLocal(const FaceHandle face, const bool interpolate)
{
	size_t nv_inner_ = pointsFromDegree(m_degree);
	size_t degree = m_mesh.degree();

	Vertices inner, outVerts;
	Vertices *orig = &ttv(face).inner;

	const ShortestPath &ab = ShortestPath::path(ttv(face)[0], ttv(face)[1]);
	const ShortestPath &bc = ShortestPath::path(ttv(face)[1], ttv(face)[2]);
	const ShortestPath &ca = ShortestPath::path(ttv(face)[2], ttv(face)[0]);

	auto &listA = ab.list(bc);
	auto &listB = bc.list();
	auto &listC = ca.list(bc.end());
	size_t sizeA = listA.size() - 1;
	size_t sizeB = listB.size() - 1;
	size_t sizeC = listC.size() - 1;

	// use max number of samples
	if (m_samples == 0) {
		m_samples = orig->size() + ttv(face).boundarySize - 3;
	}

	size_t outSize = sizeA + sizeB + sizeC;
	size_t inSize = orig->size();
	// if there are more points than allowed, scale back
	if (inSize + outSize > m_samples) {
		outSize = m_samples*0.25 < outSize ? m_samples*0.25 : outSize;
		inSize = m_samples-outSize;
		// in case we don't have enough inner vertices
		if (inSize > orig->size()) {
			inSize = orig->size();
			outSize = std::min(m_samples-inSize, sizeA + sizeB + sizeC);
		}
	}
	assert(inSize <= orig->size());
	assert(outSize <= ttv(face).boundarySize-3);

	inner.reserve(inSize);
	outVerts.reserve(outSize);

	sortInner(face);
	// samples that lie inside the face
	size_t mod = std::max((size_t)1, orig->size() / inSize);
	for (size_t i = 0; i < orig->size() && inner.size() < inSize; i += mod) {
		inner.push_back(orig->at(i));
	}

	// samples that lie on the border
	if (outSize > 0) {
		// sample in regular intervals (easy here because vectors are already sorted after uv)
		mod = std::max((size_t)1, outSize / (sizeA + sizeB + sizeC));

		for (size_t i = 0; i < outSize; i+=mod) {
			if (i < sizeA) {
				outVerts.push_back(listA[i]);
			} else if (i - sizeA < sizeB) {
				outVerts.push_back(listB[i-sizeA]);
			} else if (i - sizeA - sizeB < sizeC) {
				outVerts.push_back(listC[i-sizeA-sizeB]);
			}
		}
	}
	assert(outVerts.size() == outSize);
	assert(inner.size() == inSize);

	size_t matSize = inSize + outSize;
	// in case we dont have enough points, make the face linear
	if (matSize < nv_inner_) {
		m_mesh.recalculateCPs(face);
		std::cerr << "\n---- not enough points to fit against (" << matSize << ")\n";
		return true;
	}

	std::cerr << "\tsolving fitting system of size " << matSize << " x " << nv_inner_ << std::endl;

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
		rhsx[i] = p[0];
		rhsy[i] = p[1];
		rhsz[i] = p[2];
	}

	auto &cp = m_ctrl.data(face);
	for (size_t i = 0; i < nv_inner_; ++i) {
		Point p = cp.controlPoint(i);
		resultX[i] = p[0];
		resultY[i] = p[1];
		resultZ[i] = p[2];
	}

	for (size_t row = 0; row < matSize; ++row) {
		VertexHandle vh = row < inSize ? inner[row] : outVerts[row - inSize];
		for (size_t i = 0, column=0; i <= m_degree; ++i) {
			for (size_t j = 0; j + i <= m_degree; ++j, ++column) {
				A(row, column) = calcCoeffs(hmap(vh), i, j, m_degree);
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

		const Point zero(0.0);
		// write control point positions back
		for (size_t i = 0; i < nv_inner_; ++i) {

			Point p(resultX[i], resultY[i], resultZ[i]);
			// only update control points that are (0, 0, 0)
			if (cp.controlPoint(i) == zero) {
				cp.controlPoint(i, p);
			}
		}

		if (!interpolate) {
			// set control points for adj faces (only those for the incident edge)
			for (auto h_it = m_ctrl.cfh_begin(face), h_e = m_ctrl.cfh_end(face);
				h_it != h_e; ++h_it
			) {
				m_ctrl.copyEdgeControlPoints(
					face,
					m_ctrl.opposite_face_handle(*h_it),
					*h_it
				);
			}
		}
	}
	return success;
}

}