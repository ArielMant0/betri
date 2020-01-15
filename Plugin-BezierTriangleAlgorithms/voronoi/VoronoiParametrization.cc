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

	if (m_mode == WeightMode::cotangent) {

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
	} else {
		for (auto e_it = m_mesh.edges_begin(), e_e = m_mesh.edges_end();
			e_it != e_e; ++e_it
		) {
			weight(*e_it) = 1.0;
		}
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
	WeightMode &mode,
	OpenMesh::VPropHandleT<Scalar> &vweight,
	OpenMesh::PropertyManager<OpenMesh::VPropHandleT<bool>, BezierTMesh> &inFace
) {
	Scalar area;

	if (mode == WeightMode::cotangent) {

		for (auto v_it = mesh.vertices_begin(), v_e = mesh.vertices_end();
			v_it != v_e; ++v_it
		) {
			area = 0.0;

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

			mesh.property(vweight, *v_it) = 1.0 / (2.0 * area);
		}
	} else {

		const auto valence = [&](VertexHandle vh) {

			int val = 0;
			for (auto vv_it = mesh.cvv_begin(vh), v_e = mesh.cvv_end(vh); vv_it != v_e; ++vv_it) {
				if (inFace[*vv_it]) {
					val++;
				}
			}
			return val;
		};

		for (auto v_it = mesh.vertices_begin(), v_e = mesh.vertices_end();
			v_it != v_e; ++v_it
		) {
			mesh.property(vweight, *v_it) = 1.0 / valence(*v_it);
		}
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

	calcWeights(m_mesh, m_mode, m_vweight, inFace);
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
			//assert(std::isgreaterequal(resultU[sysid(v)], 0.0));
			//assert(std::isgreaterequal(resultV[sysid(v)], 0.0));
			//assert(std::islessequal(resultU[sysid(v)], 1.0));
			//assert(std::islessequal(resultV[sysid(v)], 1.0));
			hmap(v) = Vec2(
				std::min(1.0, std::max(resultU[sysid(v)], 0.0)),
				std::min(1.0, std::max(resultV[sysid(v)], 0.0))
			);
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

	for (auto h_it = m_mesh.cvoh_begin(_origvh); h_it != m_mesh.cvoh_end(_origvh); ++h_it) {

		EdgeHandle eh = m_mesh.edge_handle(*h_it);
		VertexHandle vert = m_mesh.to_vertex_handle(*h_it);
		Scalar w = weight(eh) * vertexweight;

		if (vtt(vert).isBorderOf(ttv(face))) {

			weightsum += w;

			// update rhs (u,v)
			_rhsu[sysid(_origvh)] -= w*hmap(vert)[0];
			_rhsv[sysid(_origvh)] -= w*hmap(vert)[1];
		} else if (vtt(vert).face == face) {
			weightsum += w;
			// update matrix (only vertices that are part of the local face)
			_triplets.push_back(EigenTripletT(sysid(_origvh), sysid(vert), w));
		}
	}
	_triplets.push_back(EigenTripletT(sysid(_origvh), sysid(_origvh), -weightsum));
}


} // namespace betri
