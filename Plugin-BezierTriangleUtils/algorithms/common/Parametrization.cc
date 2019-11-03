#include "Parametrization.hh"

#include <cmath>

namespace betri
{

void Parametrization::prepare()
{
	if(!m_mesh.get_property_handle(m_vweight, vweightName))
		m_mesh.add_property(m_vweight, vweightName);

	if(!m_mesh.get_property_handle(m_eweight, eweightName))
		m_mesh.add_property(m_eweight, eweightName);

	if(!m_mesh.get_property_handle(m_sysid, sysidName))
		m_mesh.add_property(m_sysid, sysidName);
}

//-----------------------------------------------------------------------------

void Parametrization::cleanup()
{
	if (m_mesh.get_property_handle(m_vweight,  vweightName))
		m_mesh.remove_property(m_vweight);

	if (m_mesh.get_property_handle(m_eweight, eweightName))
		m_mesh.remove_property(m_eweight);

	if (m_mesh.get_property_handle(m_sysid, sysidName))
		m_mesh.remove_property(m_sysid);
}

//-----------------------------------------------------------------------------

void Parametrization::calcWeights()
{
	VertexIter        v_it, v_end(m_mesh.vertices_end());
	EdgeIter          e_it, e_end(m_mesh.edges_end());
	VertexFaceIter    vf_it;
	FaceVertexIter    fv_it;
	HalfedgeHandle    h0, h1, h2;
	VertexHandle      v0, v1;
	Point             p0, p1, p2, d0, d1;
	Scalar            w, area;

	// Uniform weighting
	if (m_weightType == Uniform) {
		for (e_it = m_mesh.edges_begin(); e_it != e_end; ++e_it) {
			weight(*e_it) = 1.0;
		}

		for (v_it = m_mesh.vertices_begin(); v_it != v_end; ++v_it) {
			weight(*v_it) = 1.0 / m_mesh.valence(*v_it);
		}
	}
	// Cotangent weighting
	else if (m_weightType == Cotangent) {
		for (e_it = m_mesh.edges_begin(); e_it != e_end; ++e_it) {
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
			w += 1.0 / tan(acos(d0|d1));

			h2 = m_mesh.next_halfedge_handle(h1);
			p2 = m_mesh.point(m_mesh.to_vertex_handle(h2));
			d0 = (p0 - p2).normalize();
			d1 = (p1 - p2).normalize();
			w += 1.0 / tan(acos(d0|d1));

			weight(*e_it) = w;
		}


		for (v_it = m_mesh.vertices_begin(); v_it != v_end; ++v_it) {
			area = 0.0;

			// Compute vertex weights:
			//   1.0 / sum(1/3 of area of incident triangles)

			auto vf_end = m_mesh.vf_end(*v_it);
			for (vf_it = m_mesh.vf_begin(*v_it); vf_it != vf_end; ++vf_it)
			{
				fv_it = m_mesh.fv_begin(*vf_it);

				const Point& P = m_mesh.point(*fv_it);  ++fv_it;
				const Point& Q = m_mesh.point(*fv_it);  ++fv_it;
				const Point& R = m_mesh.point(*fv_it);

				area += ((Q-P)%(R-P)).norm() * 0.5 * 0.3333;
			}

			weight(*v_it) = 1.0 / (4.0 * area);
		}
	}
}

//-----------------------------------------------------------------------------

void Parametrization::calcWeights(const VertexHandle vh)
{
	if (m_weightType == Uniform) {
		// vertex weight
		weight(vh) = 1.0 / m_mesh.valence(vh);
		// edge weights
		for (auto e_it = m_mesh.cve_begin(vh); e_it != m_mesh.cve_end(vh); ++e_it) {
			weight(*e_it) = 1.0;
		}
	} else if (m_weightType == Cotangent) {
		// Compute vertex weights:
		//   1.0 / sum(1/3 of area of incident triangles)

		Scalar area = 0.0;
		auto vf_end = m_mesh.vf_end(vh);
		for (auto vf_it = m_mesh.vf_begin(vh); vf_it != vf_end; ++vf_it) {
			auto fv_it = m_mesh.fv_begin(*vf_it);

			const Point& P = m_mesh.point(*fv_it);  ++fv_it;
			const Point& Q = m_mesh.point(*fv_it);  ++fv_it;
			const Point& R = m_mesh.point(*fv_it);

			area += ((Q - P) % (R - P)).norm() * 0.5 * 0.3333;
		}

		weight(vh) = 1.0 / (4.0 * area);


		Scalar w;
		// Compute cotangent edge weights
		for (auto e_it = m_mesh.cve_begin(vh); e_it != m_mesh.cve_end(vh); ++e_it) {
			w = 0.0;

			HalfedgeHandle h0 = m_mesh.halfedge_handle(*e_it, 0);
			VertexHandle v0 = m_mesh.to_vertex_handle(h0);
			Point p0 = m_mesh.point(v0);

			HalfedgeHandle h1 = m_mesh.halfedge_handle(*e_it, 1);
			VertexHandle v1 = m_mesh.to_vertex_handle(h1);
			Point p1 = m_mesh.point(v1);

			HalfedgeHandle h2 = m_mesh.next_halfedge_handle(h0);
			Point p2 = m_mesh.point(m_mesh.to_vertex_handle(h2));
			Point d0 = (p0 - p2).normalize();
			Point d1 = (p1 - p2).normalize();
			w += 1.0 / tan(acos(d0 | d1));

			h2 = m_mesh.next_halfedge_handle(h1);
			p2 = m_mesh.point(m_mesh.to_vertex_handle(h2));
			d0 = (p0 - p2).normalize();
			d1 = (p1 - p2).normalize();
			w += 1.0 / tan(acos(d0 | d1));

			weight(*e_it) = w;
		}
	}

}

//-----------------------------------------------------------------------------

void Parametrization::initCoords(const VertexHandle vh)
{
	// map boundary vertices onto n-gon in texture space
	// (preserve edge length ratio)

	size_t innerIdx = 0;
	// reset all (interior) coordinates to ngon midpoint (also circle midpoint)
	for (auto vh : *m_inner) {
		hmap(vh) = Vec2(0.5f, 0.5f);
		sysid(vh) = innerIdx++;
	}

	// TODO: map to boundary
	std::vector<BoundaryMapper::Path*> p;

	// map boundary
	m_mapper.map(p);
}

//-----------------------------------------------------------------------------

void Parametrization::initCoords(const FaceHandle face)
{
	// map boundary vertices onto triangle in texture space
	// (preserve edge length ratio)

	size_t innerIdx = 0;
	// reset all (interior) coordinates to triangle midpoint (also circle midpoint)
	for (auto vh : *m_inner) {
		hmap(vh) = Vec2(0.5f, 0.5f);
		sysid(vh) = innerIdx++;
	}

	const ShortestPath &ab = ShortestPath::path(ttv(face)[0], ttv(face)[1]);
	const ShortestPath &bc = ShortestPath::path(ttv(face)[1], ttv(face)[2]);
	const ShortestPath &ca = ShortestPath::path(ttv(face)[2], ttv(face)[0]);

	std::vector<BoundaryMapper::Path*> p;
	p.push_back(&ab.list());
	p.push_back(&bc.list());
	p.push_back(&ca.list());

	// map boundary
	m_mapper.map(p);
}

//-----------------------------------------------------------------------------

bool Parametrization::solveLocal(const VertexHandle vh)
{
	nv_inner_ = m_mesh.valence(vh);
	assert(nv_inner_ > 0);

	// calculate coordinates
	initCoords(vh);

	// system matrix
	EigenSpMatT A(nv_inner_, nv_inner_);

	// right hand sides for u and v coordinates
	EigenVectorT rhsu(nv_inner_);
	rhsu.setZero();
	EigenVectorT rhsv(nv_inner_);
	rhsv.setZero();

	// resulting texture coordinates for u and v
	EigenVectorT resultU(nv_inner_);
	resultU.setZero();
	EigenVectorT resultV(nv_inner_);
	resultV.setZero();

	std::vector<EigenTripletT> triplets;

	// TODO: add stuff for other

	// now we have all triplets to setup the matrix A
	A.setFromTriplets(triplets.begin(), triplets.end());

	// now we can solve for u and v

	/**
	 * vertexweight the matrix is not symmetric!  Hence, we cannot use an ordinary conjugate
	 * gradient or cholesky decomposition. For solving this non-symmetric system we use a
	 * Biconjugate gradient stabilized method. You can experiment with different solvers
	 * (see http://eigen.tuxfamily.org/dox/TutorialSparse.html#TutorialSparseDirectSolvers) but
	 * first you need to make the system SPD (symmetric positive definite) try e.g. setting
	 * the vertexweight in add_row_to_system to 1. (this should NOT change the result)
	 */
	Eigen::BiCGSTAB<EigenSpMatT> bicg(A); // performs a Biconjugate gradient stabilized method
	resultU = bicg.solve(rhsu);
	if (bicg.info() != Eigen::Success)
		std::cerr << "solve failed!" << std::endl;

	resultV = bicg.solve(rhsv);
	if (bicg.info() != Eigen::Success)
		std::cerr << "solve failed!" << std::endl;

	// TODO: write back to hmap
	return false;
}

bool Parametrization::solveLocal(const FaceHandle face)
{
	m_inner = &ttv(face).inner;
	nv_inner_ = m_inner->size();
	assert(nv_inner_ > 0);

	// calculate coordinates
	initCoords(face);

	// system matrix
	EigenSpMatT A(nv_inner_, nv_inner_);

	// right hand sides for u and v coordinates
	EigenVectorT rhsu(nv_inner_);
	rhsu.setZero();
	EigenVectorT rhsv(nv_inner_);
	rhsv.setZero();

	// resulting texture coordinates for u and v
	EigenVectorT resultU(nv_inner_);
	resultU.setZero();
	EigenVectorT resultV(nv_inner_);
	resultV.setZero();

	std::vector<EigenTripletT> triplets;

	// for all inner vertices, setup the corresponding row of the linear systems (u and v)
	// call add_row_to_system for all inner vertices
	// smooth parameterization using Laplacian relaxation.
	for (const auto &v : *m_inner) {
		addRow(triplets, rhsu, rhsv, v, face);
	}
	std::cerr << " number of triplets (i.e. number of non-zeros) " << triplets.size();
	std::cerr << ", per row " << (triplets.size() / nv_inner_) << std::endl;

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

	resultU = bicg.solve(rhsu);
	if (bicg.info() != Eigen::Success) {
		error = true;
		std::cerr << "solve failed!" << std::endl;
	}

	resultV = bicg.solve(rhsv);
	if (bicg.info() != Eigen::Success) {
		error = true;
		std::cerr << "solve failed!" << std::endl;
	}

	if (!error) {
		// write back to hmap
		for (const auto v : *m_inner) {
			hmap(v) = Vec2(resultU[sysid(v)], resultV[sysid(v)]);
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

bool Parametrization::solve()
{
	// TODO: only needs to be done once?
	calcWeights();

	for (FaceHandle face : m_ctrl.faces()) {
		if (!solveLocal(face)) return false;
	}
	return true;
}

//-----------------------------------------------------------------------------

void Parametrization::addRow(
	std::vector<EigenTripletT>& _triplets,
	EigenVectorT& _rhsu,
	EigenVectorT& _rhsv,
	VertexHandle _origvh,
	FaceHandle face
) {
	// if vertex is boundary do nothing
	if (m_mesh.is_boundary(_origvh))
		return;

	// else setup one row of the matrix, need local indices

	// INSERT CODE:
	// setup one row of the equation system by pushing back (_triplets.push_back(...))
	// the triplets (of non-zero entries) of the Laplacian for vertex _origvh
	// For constrained (boundary) neighbors also add the corresponding right hand side entries
	// to _rhsu and _rhsv
	// use sysid(vh) to get the corresponding row and columns indices in the system matrix

	Scalar weightsum(0.);
	Scalar vertexweight(weight(_origvh));

	for (auto vv_it = m_mesh.vv_begin(_origvh); vv_it != m_mesh.vv_end(_origvh); ++vv_it) {
		EdgeHandle eh = m_mesh.edge_handle(vv_it.current_halfedge_handle());
		Scalar w = weight(eh);
		w *= vertexweight;
		weightsum += w;

		if (vtt(*vv_it).isBorder()) {
			// update rhs (u,v)
			_rhsu[sysid(_origvh)] -= w*hmap(*vv_it)[0];
			_rhsv[sysid(_origvh)] -= w*hmap(*vv_it)[1];
		} else {
			// update matrix (only vertices that are part of the local face)
			_triplets.push_back(EigenTripletT(sysid(_origvh), sysid(*vv_it), w));
		}
	}
	_triplets.push_back(EigenTripletT(sysid(_origvh), sysid(_origvh), -weightsum));
}


} // namespace betri