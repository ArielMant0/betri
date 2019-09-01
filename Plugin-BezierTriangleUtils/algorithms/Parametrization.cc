#include "Parametrization.hh"

namespace betri
{

void Parametrization::prepare()
{
	if(!m_mesh.get_property_handle(m_hmap, hmapName))
		m_mesh.add_property(m_hmap, hmapName);

	if(!m_mesh.get_property_handle(m_vweight, vweightName))
		m_mesh.add_property(m_vweight, vweightName);

	if(!m_mesh.get_property_handle(m_eweight, eweightName))
		m_mesh.add_property(m_eweight, eweightName);

	if(!m_mesh.get_property_handle(m_sysid, sysidName))
		m_mesh.add_property(m_sysid, sysidName);

	nv_total_ = m_mesh.n_vertices();
	// should always be total-3
	nv_inner_ = 0;
	// should always be 3
	nv_bdry_  = 0;

	// also map the indices of inner vertices to equation system vertices [0, nv_inner_-1]
	for (auto v_it = m_mesh.vertices_begin(); v_it != m_mesh.vertices_end(); ++v_it) {
		if (m_mesh.is_boundary(*v_it)) {
			nv_bdry_++;
		} else {
			sysid(*v_it) = nv_inner_;
			nv_inner_++;
		}
	}

	std::cerr << "INFO: this mesh has " << nv_bdry_ << " boundary vertices and " << nv_inner_;
	std::cerr << " inner vertices, the total number is " << nv_total_  << std::endl;
}

//-----------------------------------------------------------------------------

void Parametrization::cleanup()
{
	if (m_mesh.get_property_handle(m_hmap, hmapName))
		m_mesh.remove_property(m_hmap);

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

			h0 = m_mesh.halfedge_handle(e_it.handle(), 0);
			v0 = m_mesh.to_vertex_handle(h0);
			p0 = m_mesh.point(v0);

			h1 = m_mesh.halfedge_handle(e_it.handle(), 1);
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

void Parametrization::initCoords(std::array<BezierTMesh::Point, 3> &triangle)
{
	// INSERT CODE:
	// Map boundary vertices onto triangle in texture space
	// (preserve edge length ratio)
	// Map interior vertices to triangle center

	VertexIter      v_it, v_end(m_mesh.vertices_end());
	VertexHandle    vh;
	HalfedgeHandle  hh;

	// find 1st boundary vertex
	for (v_it = m_mesh.vertices_begin(); v_it != v_end; ++v_it) {
		if (m_mesh.is_boundary(*v_it))
			break;
	}

	// boundary found ?
	if (v_it == v_end) {
		std::cerr << "No boundary found\n";
		return;
	}

	// reset all coordinates to triangle midpoint
	for (v_it = m_mesh.vertices_begin(); v_it != v_end; ++v_it) {
		hmap(*v_it) = Vec2(0.33f, 0.33f);
	}

	// get boundary loop
	vh = v_it.handle();
	hh = m_mesh.halfedge_handle(vh);
	int i = 0;
	assert(m_mesh.is_boundary(hh));
	do
	{
		hmap(m_mesh.to_vertex_handle(hh)) = { triangle[i][0], triangle[i++][1] };
		assert(i <= 3);
		hh = m_mesh.next_halfedge_handle(hh);
	} while (hh != m_mesh.halfedge_handle(vh));
}

//-----------------------------------------------------------------------------

void Parametrization::solve(std::array<BezierTMesh::Point, 3> &triangle)
{
	// make sure the boundary has been mapped to a circle (we need these texcoords in add_row_to_system_matrix)
	initCoords(triangle);

	// also make sure the weights have been computed
	calcWeights();

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

	// the matrix is build/initialized from a set of triplets (i.e. (rowid, colid, value))

	/**
	 * Info: add_row_to_system_matrix should add entries to the vector of triplets, the matrix
	 *
	 * is build from these triplets below
	 * see http://eigen.tuxfamily.org/dox/TutorialSparse.html#TutorialSparseFilling for more
	 * details on triplets and setting up sparse matrices)
	 */
	std::vector<EigenTripletT> triplets;

	// INSERT CODE:
	// for all inner vertices, setup the corresponding row of the linear systems (u and v)
	// TODO: Call add_row_to_system for all inner vertices
	// Smooth parameterization using Laplacian relaxation.
	//--- start strip ---

	for (auto v_it = m_mesh.vertices_begin(); v_it != m_mesh.vertices_end(); ++v_it) {
		// skip boundary vertices
		if (!m_mesh.is_boundary(*v_it)) {
			add_row_to_system(triplets, rhsu, rhsv, *v_it);
		}
	}
	//--- end strip ---
	std::cerr << " number of triplets (i.e. number of non-zeros) " << triplets.size();
	std::cerr << ", per row " << (triplets.size()/nv_inner_) << std::endl;

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

	// write back to hmap
	for (VertexIter v_it = m_mesh.vertices_begin(); v_it != m_mesh.vertices_end(); ++v_it) {
		// skip boundary vertices
		if (!m_mesh.is_boundary(*v_it)) {
			hmap(*v_it) = Vec2(resultU[sysid(*v_it)], resultV[sysid(*v_it)]);
		}
	}
}

//-----------------------------------------------------------------------------

void Parametrization::add_row_to_system(
	std::vector<EigenTripletT>& _triplets,
	EigenVectorT& _rhsu,
	EigenVectorT& _rhsv,
	VertexHandle _origvh
) {
	// if vertex is boundary do nothing
	if (m_mesh.is_boundary(_origvh))
		return;

	// else setup one row of the matrix, need local indices

	// INSERT CODE:
	// todo: setup one row of the equation system by pushing back (_triplets.push_back(...))
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

		// update rhs (u,v)
		if (m_mesh.is_boundary(*vv_it)) {
			_rhsu[sysid(_origvh)] -= w*hmap(*vv_it)[0];
			_rhsv[sysid(_origvh)] -= w*hmap(*vv_it)[1];
		}
		// update matrix
		else {
			_triplets.push_back(EigenTripletT(sysid(_origvh), sysid(*vv_it), w));
		}
	}
	_triplets.push_back(EigenTripletT(sysid(_origvh), sysid(_origvh), -weightsum));
}


} // namespace betri
