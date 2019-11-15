#include "VoronoiParametrization.hh"

#include <cmath>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <Eigen/Dense>

namespace betri
{

void VoronoiParametrization::prepare()
{
	if(!m_mesh.get_property_handle(m_vweight, vweightName))
		m_mesh.add_property(m_vweight, vweightName);

	if(!m_mesh.get_property_handle(m_sysid, sysidName))
		m_mesh.add_property(m_sysid, sysidName);
}

//-----------------------------------------------------------------------------

void VoronoiParametrization::cleanup()
{
	if (m_mesh.get_property_handle(m_vweight,  vweightName))
		m_mesh.remove_property(m_vweight);

	if (m_mesh.get_property_handle(m_sysid, sysidName))
		m_mesh.remove_property(m_sysid);
}

//-----------------------------------------------------------------------------

void VoronoiParametrization::calcWeights(
	BezierTMesh &mesh,
	OpenMesh::VPropHandleT<Scalar> &vweight,
	OpenMesh::PropertyManager<OpenMesh::VPropHandleT<bool>, BezierTMesh> &inFace
) {
	VertexIter        v_it, v_end(mesh.vertices_end());
	EdgeIter          e_it, e_end(mesh.edges_end());
	VertexFaceIter    vf_it;
	FaceVertexIter    fv_it;
	HalfedgeHandle    h0, h1, h2;
	VertexHandle      v0, v1;
	Point             p0, p1, p2, d0, d1;
	Scalar            w, area;

	const auto valence = [&](const VertexHandle vh) {
		int val = 0;
		for (auto vv = mesh.cvv_begin(vh); vv != mesh.cvv_end(vh); ++vv) {
			if (inFace[*vv]) val++;
		}
		return val;
	};

	for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
		if (inFace[*v_it])
			mesh.property(vweight, *v_it) = 1.0 / valence(*v_it);
	}
}

//-----------------------------------------------------------------------------

void VoronoiParametrization::calcWeights(const FaceHandle face)
{
	auto inFace = OpenMesh::makeTemporaryProperty<VertexHandle, bool>(m_mesh);
	// mark all inner vertices as belonging to this face
	for (VertexHandle vh : *m_inner) {
		inFace[vh] = true;
	}
	// mark all border vertices as belonging to this face
	const ShortestPath &ab = ShortestPath::path(ttv(face)[0], ttv(face)[1]);
	for (VertexHandle vh : ab.list()) {
		inFace[vh] = true;
	}
	const ShortestPath &bc = ShortestPath::path(ttv(face)[1], ttv(face)[2]);
	for (VertexHandle vh : bc.list()) {
		inFace[vh] = true;
	}
	const ShortestPath &ca = ShortestPath::path(ttv(face)[2], ttv(face)[0]);
	for (VertexHandle vh : ca.list()) {
		inFace[vh] = true;
	}

	calcWeights(m_mesh, m_vweight, inFace);
}

//-----------------------------------------------------------------------------

bool VoronoiParametrization::test(BezierTMesh *mesh)
{
	assert(mesh != nullptr);
	using Container = std::vector<VertexHandle>;

	OpenMesh::VPropHandleT<VertexToTri> vtt;
	mesh->add_property(vtt);

	const auto addPointsOnLine = [&](Container &vec, Point from, Point to, size_t count, size_t id) {
		const Point dir = (to - from).normalize();
		const Scalar add = (to - from).norm() / count;

		// adds count-1 points (not last one)
		for (size_t i = 0; i < count; ++i) {
			vec.push_back(mesh->add_vertex(from + i * add * dir));
			mesh->property(vtt, vec.back()).setBorder(id, id < 2 ? id + 1 : 0);
		}
	};

	// -----------------------------------------------------
	// construct mesh
	// -----------------------------------------------------

	// three sides of the triangle
	Container ab, bc, ca;
	// inner vertices
	Container inner;

	mesh->clean();
	// add all inner vertices
	inner.push_back(mesh->add_vertex({ 0.25, 0.25, 0. }));
	inner.push_back(mesh->add_vertex({ 0.5, 0.25, 0. }));
	inner.push_back(mesh->add_vertex({ 0.25, 0.5, 0. }));
	for (VertexHandle vh : inner) {
		mesh->property(vtt, vh).id1 = 0;
	}
	// add all outer vertices
	addPointsOnLine(ab, { 0., 0., 0. }, { 0., 1., 0. }, 3, 0);
	addPointsOnLine(bc, { 0., 1., 0. }, { 1., 0., 0. }, 3, 1);
	addPointsOnLine(ca, { 1., 0., 0. }, { 0., 0., 0. }, 3, 2);
	ab.push_back(bc.front());
	bc.push_back(ca.front());
	ca.push_back(ab.front());
	// construct the faces
	mesh->add_face(ab[0], inner[0], ab[1], true);
	mesh->add_face(ab[0], ca[2], inner[0], true);

	mesh->add_face(ab[1], inner[2], ab[2], true);
	mesh->add_face(ab[1], inner[0], inner[2], true);

	mesh->add_face(ab[2], inner[2], bc[0], true);
	mesh->add_face(inner[2], bc[1], bc[0], true);

	mesh->add_face(bc[1], inner[2], inner[1], true);
	mesh->add_face(bc[1], inner[1], bc[2], true);

	mesh->add_face(bc[2], inner[1], ca[0], true);
	mesh->add_face(ca[0], inner[1], ca[1], true);

	mesh->add_face(ca[1], inner[1], ca[2], true);
	mesh->add_face(ca[2], inner[1], inner[0], true);

	mesh->add_face(inner[0], inner[1], inner[2], true);

	// -----------------------------------------------------
	// calc weights
	// -----------------------------------------------------
	OpenMesh::VPropHandleT<Scalar> vweight;
	mesh->add_property(vweight);

	auto inFace = OpenMesh::makeTemporaryProperty<VertexHandle, bool>(*mesh);
	for (VertexHandle vh : mesh->vertices()) {
		inFace[vh] = true;
	}

	calcWeights(*mesh, vweight, inFace);

	// -----------------------------------------------------
	// init coords
	// -----------------------------------------------------
	NGonMapper<VertexToTri> mapper(*mesh, vtt);
	for (VertexHandle vh : inner) {
		mesh->property(vtt, vh).uv = mapper.middle();
		std::cerr << vh << " inner uv = " << mesh->property(vtt, vh).uv << '\n';
	}

	// map boundary
	std::vector<BoundaryMapper<VertexToTri>::Path*> p;
	p.push_back(&ab);
	p.push_back(&bc);
	p.push_back(&ca);

	mapper.map(p);

	// -----------------------------------------------------
	// fill matrix and rhs
	// -----------------------------------------------------

	size_t nv_inner_ = inner.size();
	// system matrix
	EigenSpMatT A(nv_inner_, nv_inner_);

	// right hand sides for u and v coordinates
	EigenVectorT rhsu(nv_inner_);
	rhsu.setZero();
	EigenVectorT rhsv(nv_inner_);
	rhsv.setZero();

	std::vector<EigenTripletT> triplets;
	for (size_t i = 0; i < nv_inner_; ++i) {

		const VertexHandle vh = inner[i];
		assert(i == vh.idx());
		const Scalar w(mesh->property(vweight, vh));
		Scalar weightsum(0.);

		for (auto vv_it = mesh->vv_begin(vh); vv_it != mesh->vv_end(vh); ++vv_it) {
			weightsum += w;

			if (vv_it->idx() >= nv_inner_) {
				// TODO: why do i need to switch this up?
				rhsu[i] -= w*mesh->property(vtt, *vv_it).uv[1];
				rhsv[i] -= w*mesh->property(vtt, *vv_it).uv[0];
			} else {
				triplets.push_back(EigenTripletT(i, vv_it->idx(), w));
			}
		}
		triplets.push_back(EigenTripletT(i, i, -weightsum));
	}

	A.setFromTriplets(triplets.begin(), triplets.end());
	std::cerr << "matrix\n" << A << std::endl;
	std::cerr << "rhs u\n" << rhsu << '\n';
	std::cerr << "rhs v\n" << rhsv << '\n';

	// -----------------------------------------------------
	// solve
	// -----------------------------------------------------
	bool okay = true;
	Eigen::BiCGSTAB<EigenSpMatT> bicg(A);

	auto resultU = bicg.solve(rhsu);
	if (bicg.info() != Eigen::Success) {
		okay = false;
		std::cerr << "solve failed for u!" << std::endl;
	}

	auto resultV = bicg.solve(rhsv);
	if (bicg.info() != Eigen::Success) {
		okay = false;
		std::cerr << "solve failed for v!" << std::endl;
	}

	std::cerr << "result values:\n";
	for (size_t i = 0; i < nv_inner_; ++i) {
		std::cerr << '\t' << inner[i] << " pos: " << mesh->point(inner[i]) << ", uv = (";
		std::cerr << resultU[i] << ", " << resultV[i] << ")\n";
		mesh->set_point(inner[i], Point(resultU[i], resultV[i], 0.));
	}

	return okay;
}

//-----------------------------------------------------------------------------

void VoronoiParametrization::initCoords(const FaceHandle face)
{
	// map boundary vertices onto triangle in texture space
	// (preserve edge length ratio)

	size_t innerIdx = 0;
	// reset all (interior) coordinates to triangle midpoint (also circle midpoint)
	for (auto vh : *m_inner) {
		hmap(vh) = m_mapper.middle();
		sysid(vh) = innerIdx++;
	}

	const ShortestPath &ab = ShortestPath::path(ttv(face)[0], ttv(face)[1]);
	const ShortestPath &bc = ShortestPath::path(ttv(face)[1], ttv(face)[2]);
	const ShortestPath &ca = ShortestPath::path(ttv(face)[2], ttv(face)[0]);

	std::vector<BoundaryMapper<VertexToTri>::Path*> p;
	// this makes sure every path is in the right order
	// -> it always starts with the id the previous path ended with
	p.push_back(&ab.list(bc));
	p.push_back(&bc.list());
	p.push_back(&ca.list(bc.end()));

	assert(bc.front() == ab.back());
	assert(ca.front() == bc.back());
	assert(ab.front() == ca.back());

	// map boundary
	m_mapper.map(p);

	//for (VertexHandle vh : ab.list()) {
	//	m_mesh.set_color(vh, { 0., hmap(vh)[0], hmap(vh)[1], 1. });
	//}
	//for (VertexHandle vh : bc.list()) {
	//	m_mesh.set_color(vh, { 0., hmap(vh)[0], hmap(vh)[1], 1. });
	//}
	//for (VertexHandle vh : ca.list()) {
	//	m_mesh.set_color(vh, { 0., hmap(vh)[0], hmap(vh)[1], 1. });
	//}
}

//-----------------------------------------------------------------------------

bool VoronoiParametrization::solveLocal(const FaceHandle face)
{
	m_inner = &ttv(face).inner;
	nv_inner_ = m_inner->size();
	assert(nv_inner_ > 0);

	calcWeights(face);

	// calculate coordinates
	initCoords(face);

	// system matrix
	EigenSpMatT A(nv_inner_, nv_inner_);

	// right hand sides for u and v coordinates
	EigenVectorT rhsu(nv_inner_);
	rhsu.setZero();
	EigenVectorT rhsv(nv_inner_);
	rhsv.setZero();

	std::vector<EigenTripletT> triplets;

	// for all inner vertices, setup the corresponding row of the linear systems (u and v)
	// call add_row_to_system for all inner vertices
	// smooth parameterization using Laplacian relaxation.
	for (VertexHandle v : *m_inner) {
		addRow(triplets, rhsu, rhsv, v, face);
	}

	// now we have all triplets to setup the matrix A
	A.setFromTriplets(triplets.begin(), triplets.end());

	/**
	 * vertexweight the matrix is not symmetric!  Hence, we cannot use an ordinary conjugate
	 * gradient or cholesky decomposition. For solving this non-symmetric system we use a
	 * Biconjugate gradient stabilized method. You can experiment with different solvers
	 * (see http://eigen.tuxfamily.org/dox/TutorialSparse.html#TutorialSparseDirectSolvers) but
	 * first you need to make the system SPD (symmetric positive definite) try e.g. setting
	 * the vertexweight in add_row_to_system to 1. (this should NOT change the result)
	 */
	Eigen::BiCGSTAB<EigenSpMatT> bicg(A); // performs a Biconjugate gradient stabilized method

	bool error = false;

	auto resultU = bicg.solve(rhsu);
	if (bicg.info() != Eigen::Success) {
		error = true;
		std::cerr << "solve failed!" << std::endl;
	}

	auto resultV = bicg.solve(rhsv);
	if (bicg.info() != Eigen::Success) {
		error = true;
		std::cerr << "solve failed!" << std::endl;
	}

	if (!error) {
		std::cerr << "\tcalculated uv for " << m_inner->size() << " vertices\n";
		// write back to hmap
		for (const VertexHandle v : *m_inner) {
			hmap(v) = Vec2(resultU[sysid(v)], resultV[sysid(v)]);
			m_mesh.set_color(v, { 0., hmap(v)[0], hmap(v)[1], 1. });
			//std::cerr << "vertex " << v << " has uv " << hmap(v) << std::endl;
			assert(std::isgreaterequal(hmap(v)[0], 0.0));
			assert(std::isgreaterequal(hmap(v)[1], 0.0));
			assert(std::islessequal(hmap(v)[0], 1.0));
			assert(std::islessequal(hmap(v)[1], 1.0));
		}
	}

	return !error;
}

//-----------------------------------------------------------------------------

bool VoronoiParametrization::solve()
{
	for (FaceHandle face : m_ctrl.faces()) {
		if (!solveLocal(face)) return false;
	}
	return true;
}

//-----------------------------------------------------------------------------

void VoronoiParametrization::addRow(
	std::vector<EigenTripletT>& _triplets,
	EigenVectorT& _rhsu,
	EigenVectorT& _rhsv,
	VertexHandle _origvh,
	FaceHandle face
) {
	// if vertex is boundary do nothing
	if (m_mesh.is_boundary(_origvh))
		return;

	Scalar weightsum(0.);
	Scalar w(weight(_origvh));

	for (auto vv_it = m_mesh.vv_begin(_origvh); vv_it != m_mesh.vv_end(_origvh); ++vv_it) {
		if (vtt(*vv_it).isBorderOf(ttv(face))) {
			weightsum += w;
			// update rhs (u,v)
			// TODO: why v,u and not u,v ?
			_rhsu[sysid(_origvh)] -= w*hmap(*vv_it)[0];
			_rhsv[sysid(_origvh)] -= w*hmap(*vv_it)[1];
		} else if (vtt(*vv_it).face == face) { //vtt(*vv_it).isInnerOf(ttv(face))) {
			weightsum += w;
			// update matrix (only vertices that are part of the local face)
			_triplets.push_back(EigenTripletT(sysid(_origvh), sysid(*vv_it), w));
		}
	}
	_triplets.push_back(EigenTripletT(sysid(_origvh), sysid(_origvh), -weightsum));
}


} // namespace betri
