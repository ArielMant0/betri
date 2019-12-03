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

	if (!m_mesh.get_property_handle(m_vprio, "vertexprio"))
		m_mesh.add_property(m_vprio, "vertexprio");
}

void Decimation::cleanup()
{
	if (m_mesh.get_property_handle(m_hprio, "halfedgeprioprop"))
		m_mesh.remove_property(m_hprio);

	if (m_mesh.get_property_handle(m_target, "vertextargetprop"))
		m_mesh.remove_property(m_target);

	if (m_mesh.get_property_handle(m_vprio, "vertexprio"))
		m_mesh.remove_property(m_vprio);
}

void Decimation::calculateErrors()
{
	Scalar minError = std::numeric_limits<Scalar>::max(),
		maxError = 0.,
		avgError = 0.;

	size_t count = 0;

	m_mesh.update_face_normals();

	for (HalfedgeHandle hh : m_mesh.halfedges()) {
		calculateError(hh);
		Scalar error = priority(hh);
		if (error != -1.) {
			maxError = std::max(maxError, error);
			minError = std::min(minError, error);
			avgError += error;
			count++;
		}
	}
	avgError = avgError / count;

	std::cerr << "calcluated errors:\n\tmin: " << minError << "\n\tmax: " << maxError;
	std::cerr << "\n\tavg: " << avgError << std::endl;

	for (VertexHandle vh : m_mesh.vertices()) {
		priority(vh) = -1.0;
	}
}

void Decimation::calculateError(const HalfedgeHandle hh)
{
	if (isCollapseLegal(hh)) {
		// "simulate collapse" -> fit all 1-Ring faces -> use max error
		Scalar error = fit(hh, false);
		assert(std::isgreaterequal(error, 0.));
		priority(hh) = error;
	} else {
		// halfedges that result in an illegal collapse have a priority of -1
		priority(hh) = -1.0;
	}
}

void Decimation::updateError(const HalfedgeHandle hh)
{
	//std::cerr << "update hh " << hh << " prio: " << priority(hh);
	bool legal = isCollapseLegal(hh);
	if (priority(hh) != -1.0 && legal) {
		// "simulate collapse" -> fit all 1-Ring faces -> use max error
		Scalar error = fit(hh, false);
		assert(std::isgreaterequal(error, 0.));
		priority(hh) = priority(hh) + error;
	} else {
		// halfedges that result in an illegal collapse have a priority of -1
		priority(hh) = -1.0;
	}
	//std::cerr << " -> " << priority(hh) << "(legal ? " << legal << ")\n";
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

			if (prio >= 0.0 && prio < min_prio) {
				min_prio = prio;
				min_hh = *voh_it;
			}
		}
	}

	// remove from queue
	if (priority(vh) != -1.0) {
		m_q->erase(vh);
		priority(vh) = -1.0;
	}

	// update queue
	if (min_hh.is_valid()) {
		//std::cerr << "update vertex " << vh << " prio " << min_prio << " halfedge ";
		//std::cerr << min_hh << " with prio " << priority(min_hh) << std::endl;
		target(vh) = min_hh;
		priority(vh) = min_prio;
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
	if (m_q->empty() && !done) {

		// prepare fitting by telling it barycentric coordinates used for surface sampling
		m_fit.setBarycentricCoords(m_param.getBarycentricCoords());
		// calculate fitting errors
		calculateErrors();

		// fill queue
		for (VertexHandle vh : m_mesh.vertices()) {
			enqueueVertex(vh);
		}
	}

	// do the actual decimation...
	while (m_nverts > m_complexity && !m_q->empty()) {
		step();
		if (debugCancel())
			return true;

		// in case we only want to do a single step
		if (stepwise) break;
	}

	done = m_nverts <= m_complexity || m_q->empty();

	// only update when we're done
	if (done) {
		m_q->clear();
		// reset parameters to avoid buggy behaviour on next round
		m_nverts = m_mesh.n_vertices();
		m_complexity = m_nverts;
	}

	m_mesh.garbage_collection();

	//if (false && !done && stepwise) {
	//
	//	m_q->clear();
	//	//delete m_q;

	//	//m_mesh.get_property_handle(m_hprio, "halfedgeprioprop");
	//	//m_mesh.get_property_handle(m_target, "vertextargetprop");

	//	//VertexCmp cmp(m_mesh, m_hprio, m_target);
	//	//m_q = new std::set<VertexHandle, VertexCmp>(cmp);

	//	for (HalfedgeHandle hh : m_mesh.halfedges()) {
	//		enqueueVertex(m_mesh.from_vertex_handle(hh));
	//	}
	//}

	// update normals
	m_mesh.update_normals();

	return done;
}

void Decimation::step()
{
	auto testit = m_q->begin();
	for (size_t i = 0; i < 15 && testit != m_q->end(); ++i, ++testit) {
		std::cerr << i << " " << priority(*testit) << std::endl;
	}

	VertexHandle vh = *m_q->begin();
	m_q->erase(m_q->begin());

	HalfedgeHandle hh = target(vh);

	if (!hh.is_valid()) return;

	VertexHandle from = m_mesh.from_vertex_handle(hh);
	VertexHandle to = m_mesh.to_vertex_handle(hh);

	// perform collapse
	if (isCollapseLegal(hh)) {
		FaceHandle f0 = m_mesh.face_handle(hh);
		FaceHandle f1 = m_mesh.opposite_face_handle(hh);

		// store one-ring
		std::vector<FaceHandle> faces;
		std::set<HalfedgeHandle> oneRing;
		std::set<VertexHandle> oneRingV;

		for (auto v_it = m_mesh.cvv_begin(from); v_it != m_mesh.cvv_end(from); ++v_it) {
			oneRingV.insert(*v_it);
		}

		// storing neighbors beforehand is easier
		for (auto f_it = m_mesh.cvf_begin(from), f_end = m_mesh.cvf_end(from);
			f_it != f_end; ++f_it
		) {
			// TODO: this is quite a lot of halfedges
			if (*f_it != f0 && *f_it != f1) {
				//m_mesh.set_color(*f_it, { 0.f, 0.75f, 0.25f, 1.f });
				faces.push_back(*f_it);
			}
		}
		std::cerr << "collapsing halfedge " << hh << " with error " << priority(hh) << '\n';
		// fit remaining faces (and apply update)
		fit(hh, true);

		if (debugCancel()) {
			return;
		}

		// collapse halfedge
		m_mesh.collapse(hh);
		// remove vertex from q since it counts as deleted now
		m_q->erase(from);


		for (VertexHandle vh : oneRingV) {
			for (auto h_it = m_mesh.cvoh_begin(vh); h_it != m_mesh.cvoh_end(vh); ++h_it) {
				oneRing.insert(*h_it);
			}
		}

		// make faces "match" again
		std::set<EdgeHandle> visited;

		for (size_t i = 0; i < faces.size(); ++i) {
			for (auto e_it = m_mesh.cfe_begin(faces[i]), e_end = m_mesh.cfe_end(faces[i]);
				e_it != e_end; ++e_it
			) {
				if (visited.find(*e_it) == visited.end()) {
					m_mesh.interpolateEdgeControlPoints(*e_it, true);
					visited.insert(*e_it);
				}
			}
		}

		// now we have one less vertex
		--m_nverts;

		// recalculate the error first, so we know all incident halfedges
		// are updated before updating the queue
		for (HalfedgeHandle he : oneRing) {
			updateError(he);
		}
		// update queue (reinsert the vertex)
		for (VertexHandle vh : oneRingV) {
			enqueueVertex(vh);
		}

		// TODO: why is q not sorted?
		//assert(std::is_sorted(m_q->begin(), m_q->end(), [&](const VertexHandle &v0, const VertexHandle &v1) {
		//	Scalar p0 = priority(v0), p1 = priority(v1);
		//	// std::set needs UNIQUE keys -> handle equal priorities
		//	return p0 == p1 ? v0.idx() < v1.idx() : p0 < p1;
		//}));
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
	if (!m_param.solveLocal(from, to, fitColls, apply)) {
		debugCancel("parametrization failed");
		return 0.;
	}

	Scalar maxError = 0., error = 0.;

	assert(fitColls.size() == valence - 2);
	for (size_t i = 0; i < fitColls.size(); ++i) {
		// fit all surrounding faces
		if (!m_fit.solveLocal(fitColls[i], error, apply)) {
			debugCancel("fitting failed");
			return 0.;
		}
		// store max error
		maxError = std::max(error, maxError);
	}

	return maxError;
}

}

