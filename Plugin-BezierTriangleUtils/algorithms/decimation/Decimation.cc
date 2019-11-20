#include "Decimation.hh"

namespace betri
{

void Decimation::prepare()
{
	// quadric property of each vertex: normal equations of one-ring faces
	if (!m_mesh.get_property_handle(m_hprio, "halfedgeprioprop"))
		m_mesh.add_property(m_hprio, "halfedgeprioprop");

	// the best halfedge handle only is needed during decimation
	if (!m_mesh.get_property_handle(m_target, "vertextargetprop"))
		m_mesh.add_property(m_target, "vertextargetprop");

	if (!m_mesh.get_property_handle(m_uv, "vuv"))
		m_mesh.add_property(m_uv, "vuv");
}

void Decimation::cleanup()
{
	if (m_mesh.get_property_handle(m_hprio, "halfedgeprioprop"))
		m_mesh.remove_property(m_hprio);

	if (m_mesh.get_property_handle(m_target, "vertextargetprop"))
		m_mesh.remove_property(m_target);

	if (m_mesh.get_property_handle(m_uv, "vuv"))
		m_mesh.remove_property(m_uv);
}

void Decimation::calculateErrors()
{
	//std::cerr << "calculating errors:\n";

	m_mesh.update_face_normals();
	for (HalfedgeHandle hh : m_mesh.halfedges()) {
		calculateError(hh);

		//std::cerr << hh << " has error " << priority(hh) << '\n';

	}
}

void Decimation::calculateError(const HalfedgeHandle hh)
{
	// halfedges that result in an illegal collapse have a priority of -1
	priority(hh) = -1.0;

	if (isCollapseLegal(hh)) {
		// "simulate collapse" -> fit all 1-Ring faces -> use max error
		Scalar error = fit(hh, false);
		assert(std::isgreaterequal(error, 0.));
		priority(hh) = error;
	}
}

void Decimation::enqueueVertex(const VertexHandle vh)
{
	Scalar prio, min_prio(std::numeric_limits<Scalar>::max());
	HalfedgeHandle min_hh;

	// find best out-going halfedge
	for (auto voh_it = m_mesh.voh_begin(vh), voh_end = m_mesh.voh_end(vh);
		voh_it != voh_end; ++voh_it) {

		if (isCollapseLegal(*voh_it)) {
			prio = priority(*voh_it);

			if (prio != -1.0 && prio < min_prio) {
				min_prio = prio;
				min_hh = *voh_it;
			}
		}
	}

	// remove from queue
	if (priority(vh) != -1.0)
		m_q->erase(vh);

	// update queue
	if (min_hh.is_valid()) {
		target(vh) = min_hh;
		if (min_prio >= 0.0)
			m_q->insert(vh);
	}
}

bool Decimation::isCollapseLegal(const HalfedgeHandle hh)
{
	// collect vertices
	VertexHandle v0 = m_mesh.from_vertex_handle(hh);
	VertexHandle v1 = m_mesh.to_vertex_handle(hh);

	// collect faces
	FaceHandle fl = m_mesh.face_handle(hh);
	FaceHandle fr = m_mesh.opposite_face_handle(hh);

	// backup point positions
	const Point p0 = m_mesh.point(v0);
	const Point p1 = m_mesh.point(v1);

	// topological test
	if (!m_mesh.is_collapse_ok(hh)) {
		return false;
	}
	// test boundary stuff
	if (m_mesh.is_boundary(v0) && !m_mesh.is_boundary(v1)) {
		return false;
	}

	bool collapseOK = true;

	 // simulate collapse
	m_mesh.set_point(v0, p1);

	// check for flipping normals
	double c(1.0);
	const double min_cos(cos(0.25 * M_PI));

	for (auto vf_it(m_mesh.cvf_begin(v0)), vf_end(m_mesh.cvf_end(v0)); vf_it != vf_end; ++vf_it) {
		// dont check triangles that would be deleted anyway
		if ((*vf_it != fl) && (*vf_it != fr)) {

			Point n0 = m_mesh.normal(*vf_it);
			Point n1 = m_mesh.calc_face_normal(*vf_it);
			// check wether normal flips due to collapse
			if ((c = (n0 | n1)) < min_cos) {
				break;
			}
		}
	}

	// undo simulation changes
	m_mesh.set_point(v0, p0);

	if (c < min_cos) {
		collapseOK = false;
	}

	// return the result of the collapse simulation
	return collapseOK;
}

bool Decimation::decimate(size_t complexity, bool stepwise)
{
	// only set first time this is called
	m_complexity = complexity == 0 ? m_complexity : complexity;

	bool done = m_nverts <= m_complexity;

	// build priority queue...
	if (!stepwise || (m_q->empty() && !done)) {
		for (VertexHandle vh : m_mesh.vertices()) {
			enqueueVertex(vh);
		}
	}

	// do the actual decimation...
	while (m_nverts > m_complexity && !m_q->empty()) {
		step();
		// in case we only want to do a single step
		if (stepwise) break;
	}

	m_mesh.garbage_collection();

	done = m_nverts <= m_complexity || m_q->empty();
	// only update when we're done
	if (done) {
		m_q->clear();
		// reset parameters to avoid buggy behaviour on next round
		m_nverts = m_mesh.n_vertices();
		m_complexity = m_nverts;
	}

	// update normals
	m_mesh.update_normals();

	return done;
}

void Decimation::step()
{
	// store one-ring
	std::vector<HalfedgeHandle> oneRing;

	VertexHandle vh = *m_q->begin();
	m_q->erase(m_q->begin());

	HalfedgeHandle hh = target(vh);
	VertexHandle from = m_mesh.from_vertex_handle(hh);

	// perform collapse
	if (isCollapseLegal(hh)) {
		FaceHandle f0 = m_mesh.face_handle(hh);
		FaceHandle f1 = m_mesh.opposite_face_handle(hh);
		// storing neighbors beforehand is easier
		for (auto f_it = m_mesh.cvf_begin(from), f_end = m_mesh.cvf_end(from);
			f_it != f_end; ++f_it
		) {
			// TODO: this is quite a lot of halfedges
			if (*f_it != f0 && *f_it != f1) {
				for (auto h_it = m_mesh.cfh_begin(*f_it); h_it != m_mesh.cfh_end(*f_it); ++h_it) {
					oneRing.push_back(*h_it);
				}
			}
		}
		std::cerr << "collapsing halfedge " << hh << " with error " << priority(hh) << '\n';
		// fit remaining faces (and apply update)
		fit(hh, true);
		// collapse halfedge
		m_mesh.collapse(hh);
		// now we have one less vertex
		--m_nverts;
	}

	// recalculate the error first, so we know all incident halfedges
	// are updated before updating the queue
	for (HalfedgeHandle he : oneRing) {
		calculateError(he);
	}
	// update queue (reinsert the vertex)
	for (HalfedgeHandle he : oneRing) {
		enqueueVertex(m_mesh.from_vertex_handle(he));
	}
}

Scalar Decimation::fit(const HalfedgeHandle hh, const bool apply)
{
	// source and target vertex
	VertexHandle from = m_mesh.from_vertex_handle(hh), to = m_mesh.to_vertex_handle(hh);
	size_t valence = m_mesh.valence(from);

	// stores all 3D points and corresponding uv's per face
	std::vector<FitCollection> fitColls;

	// parametrize to n-gon
	m_param.solveLocal(from, to, fitColls, apply);

	Scalar maxError = 0., error = 0.;

	assert(fitColls.size() == valence - 2);
	for (size_t i = 0; i < fitColls.size(); ++i) {
		// fit all surrounding faces
		m_fit.solveLocal(fitColls[i], error, apply);
		// store max error
		maxError = std::max(error, maxError);
	}

	// make faces "match" again
	if (apply) {
		std::set<EdgeHandle> visited;
		for (size_t i = 0; i < fitColls.size(); ++i) {
			for (auto e_it = m_mesh.cfe_begin(fitColls[i].face),
				e_end = m_mesh.cfe_end(fitColls[i].face); e_it != e_end; ++e_it
			) {
				if (visited.find(*e_it) == visited.end()) {
					m_mesh.interpolateEdgeControlPoints(*e_it);
					visited.insert(*e_it);
				}
			}
		}
	}

	return maxError;
}

}

