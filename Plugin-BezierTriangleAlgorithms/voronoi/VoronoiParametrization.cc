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

	if (!m_mesh.get_property_handle(m_eweight, eweightName))
		m_mesh.add_property(m_eweight, eweightName);

	if(!m_mesh.get_property_handle(m_sysid, sysidName))
		m_mesh.add_property(m_sysid, sysidName);

	// calculate cotangent edge weights (only needs to be done once)

	Scalar w;
	HalfedgeHandle    h0, h1, h2;
	VertexHandle      v0, v1;
	Point             p0, p1, p2, d0, d1;

	for (auto e_it = m_mesh.edges_begin(), e_e = m_mesh.edges_end();
		e_it != e_e; ++e_it
	) {
		w = 0.0;

		// Compute cotangent edge weights

		h0 = m_mesh.halfedge_handle(*e_it, 0);
		v0 = m_mesh.to_vertex_handle(h0);
		p0 = m_mesh.point(v0);

		h1 = m_mesh.halfedge_handle(*e_it, 1);
		v1 = m_mesh.to_vertex_handle(h1);
		p1 = m_mesh.point(v1);

		h2 = m_mesh.next_halfedge_handle(h0);
		p2 = m_mesh.point(m_mesh.to_vertex_handle(h2));
		d0 = (p0 - p2).normalize();
		d1 = (p1 - p2).normalize();
		w += 1.0 / tan(acos(d0 | d1));

		h2 = m_mesh.next_halfedge_handle(h1);
		p2 = m_mesh.point(m_mesh.to_vertex_handle(h2));
		d0 = (p0 - p2).normalize();
		d1 = (p1 - p2).normalize();
		w += 1.0 / tan(acos(d0 | d1));

		weight(*e_it) = w;
	}
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
	Scalar area;

	for (auto v_it = mesh.vertices_begin(), v_e = mesh.vertices_end();
		v_it != v_e; ++v_it
	) {
		area = 0.0;

		// Compute vertex weights:
		//   1.0 / sum(1/3 of area of incident triangles)

		auto vf_end = mesh.vf_end(*v_it);
		for (auto vf_it = mesh.vf_begin(*v_it); vf_it != vf_end; ++vf_it) {

			auto fv_it = mesh.fv_begin(*vf_it);

			bool relevant = inFace[*fv_it];
			
			const Point& P = mesh.point(*fv_it);  ++fv_it;
			relevant = relevant || inFace[*fv_it];
			const Point& Q = mesh.point(*fv_it);  ++fv_it;
			relevant = relevant || inFace[*fv_it];
			const Point& R = mesh.point(*fv_it);

			if (relevant) {
				area += ((Q - P) % (R - P)).norm() * 0.5 * 0.3333;
			}
		}

		mesh.property(vweight, *v_it) = 1.0 / (4.0 * area);
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

	std::vector<BoundaryMapper<VertexToTri>::Path*> paths;

	auto &listA = ab.list(ttv(face)[0]);
	auto &listB = bc.list(ttv(face)[1]);
	auto &listC = ca.list(ttv(face)[2]);

	paths.push_back(&listA);
	paths.push_back(&listB);
	paths.push_back(&listC);

	// map boundary
	m_mapper.map(paths);
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
		for (VertexHandle v : *m_inner) {
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
	Scalar vertexweight(weight(_origvh));

	for (auto vv_it = m_mesh.vv_begin(_origvh); vv_it != m_mesh.vv_end(_origvh); ++vv_it) {

		EdgeHandle eh = m_mesh.edge_handle(vv_it.current_halfedge_handle());
		Scalar w = weight(eh);
		w *= vertexweight;

		if (vtt(*vv_it).isBorderOf(ttv(face))) {
			
			weightsum += w;

			// update rhs (u,v)
			_rhsu[sysid(_origvh)] -= w*hmap(*vv_it)[0];
			_rhsv[sysid(_origvh)] -= w*hmap(*vv_it)[1];
		} else if (vtt(*vv_it).face == face) {
			weightsum += w;
			// update matrix (only vertices that are part of the local face)
			_triplets.push_back(EigenTripletT(sysid(_origvh), sysid(*vv_it), w));
		}
	}
	_triplets.push_back(EigenTripletT(sysid(_origvh), sysid(_origvh), -weightsum));
}


} // namespace betri
