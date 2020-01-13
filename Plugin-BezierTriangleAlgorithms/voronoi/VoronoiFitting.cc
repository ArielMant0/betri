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

	// TODO: make sure edge control points are the same for adj faces
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

	if (m_samples == 0) {
		m_samples = nv_inner_ + ttv(face).boundarySize - 3;
	}

	Vertices *orig = &ttv(face).inner;
	// use boundary vertices (minus 3 because we store corners mutliple twice)
	size_t outSize = std::min(ttv(face).boundarySize-3, m_samples/3);
	size_t inSize = std::max(nv_inner_, std::min(orig->size(), m_samples-outSize));

	inner.reserve(inSize);
	outVerts.reserve(outSize);

	sortInner(face);
	// samples that lie inside the face
	size_t mod = orig->size() / inSize;
	for (size_t i = 0; i < orig->size() && inner.size() < inSize; i += mod) {
		inner.push_back(orig->at(i));
	}

	const ShortestPath &ab = ShortestPath::path(ttv(face)[0], ttv(face)[1]);
	const ShortestPath &bc = ShortestPath::path(ttv(face)[1], ttv(face)[2]);
	const ShortestPath &ca = ShortestPath::path(ttv(face)[2], ttv(face)[0]);

	auto &listA = ab.list(bc);
	auto &listB = bc.list();
	auto &listC = ca.list(bc.end());

	// samples that lie on the border
	if (outSize > 0) {

		size_t perPath = outSize / 3;
		// sample in regular intervals (easy here because vectors are already sorted after uv)
		mod = std::max((size_t)1, (listA.size()-1) / perPath);
		for (size_t i = 0; i < listA.size() && outVerts.size() <= perPath; i+=mod) {
			outVerts.push_back(listA[i]);
		}
		mod = std::max((size_t)1, (listB.size()-1) / perPath);
		for (size_t i = 0; i < listB.size() && outVerts.size() <= 2 * perPath; i+=mod) {
			outVerts.push_back(listB[i]);
		}
		mod = std::max((size_t)1, (listC.size()-1) / perPath);
		for (size_t i = 0; i < listC.size() && outVerts.size() < outSize; i+=mod) {
			outVerts.push_back(listC[i]);
		}
	}
	assert(outVerts.size() == outSize);
	assert(inner.size() == inSize);

	size_t matSize = inSize + outSize;
	assert(matSize >= nv_inner_ && "matrix must have at least # control point entries");

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

	bool success = solveSystem(A, rhsx, resultX);
	success = solveSystem(A, rhsy, resultY) && success;
	success = solveSystem(A, rhsz, resultZ) && success;

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

bool VoronoiFitting::test(BezierTMesh *mesh)
{
	constexpr size_t degree = 2;
	constexpr size_t matSize = 10;

	mesh->degree(degree);

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
	std::cerr.precision(2);
	std::cerr.setf(std::ios::fixed, std::ios::floatfield);

	for (size_t i = 0; i < nv_inner_; ++i) {
		Point p(resultX[i], resultY[i], resultZ[i]);
		if (useMesh) mesh->data(face).controlPoint(i, p);
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