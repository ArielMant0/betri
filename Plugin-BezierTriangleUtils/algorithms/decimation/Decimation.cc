#include "Decimation.hh"

namespace betri
{

void Decimation::prepare()
{
	// vertex property holding the priorities of each vertex
	if (!m_mesh.get_property_handle(m_prio, "vertexprioprop"))
		m_mesh.add_property(m_prio, "vertexprioprop");

	// quadric property of each vertex: normal equations of one-ring faces
	if (!m_mesh.get_property_handle(m_quadric, "vertexquadricprop"))
		m_mesh.add_property(m_quadric, "vertexquadricprop");

	// the best halfedge handle only is needed during decimation
	if (!m_mesh.get_property_handle(m_target, "vertextargetprop"))
		m_mesh.add_property(m_target, "vertextargetprop");
}

void Decimation::cleanup()
{
	if (m_mesh.get_property_handle(m_prio, "vertexprioprop"))
		m_mesh.remove_property(m_prio);

	if (m_mesh.get_property_handle(m_quadric, "vertexquadricprop"))
		m_mesh.remove_property(m_quadric);

	if (m_mesh.get_property_handle(m_target, "vertextargetprop"))
		m_mesh.remove_property(m_target);
}

void Decimation::initQuadrics()
{
	m_mesh.update_face_normals();
	for (VertexHandle vh : m_mesh.vertices()) {

		priority(vh) = -1.0;
		vertexQuadric(vh).clear();

		// iterate over faces incident to vertex
		for (auto vf_it = m_mesh.vf_begin(vh), vf_end = m_mesh.vf_end(vh);
			vf_it != vf_end; ++vf_it) {

			// plane equation
			const BezierTMesh::Normal n = m_mesh.normal(*vf_it);
			const double a = n[0];
			const double b = n[1];
			const double c = n[2];
			const double d = -(n | m_mesh.point(vh));

			// plane -> quadric
			vertexQuadric(vh) += Quadricd(a, b, c, d);
		}
	}
}

void Decimation::enqueueVertex(const VertexHandle vh)
{
	double prio, min_prio(FLT_MAX);
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

	// update queue
	if (priority(vh) != -1.0) {
		m_q->erase(vh);
		priority(vh) = -1.0;
	}

	if (min_hh.is_valid()) {
		priority(vh) = min_prio;
		vertexTarget(vh) = min_hh;
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

	done = m_nverts <= m_complexity || m_q->empty();
	// only update when we're done
	if (done) {
		m_q->clear();
		m_mesh.garbage_collection();
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
	std::vector<VertexHandle> oneRing;

	VertexHandle vh = *m_q->begin();
	m_q->erase(m_q->begin());

	HalfedgeHandle hh = vertexTarget(vh);
	VertexHandle to = m_mesh.to_vertex_handle(hh);
	VertexHandle from = m_mesh.from_vertex_handle(hh);

	// perform collapse
	if (isCollapseLegal(hh)) {
		// storing neighbors beforehand is easier and more efficient
		for (auto vv_it = m_mesh.vv_begin(from), vv_end = m_mesh.vv_end(from);
			vv_it != vv_end; ++vv_it) {
			oneRing.push_back(*vv_it);
		}
		m_mesh.collapse(hh);
		// update quadric the vertex is collapsed into
		vertexQuadric(to) += vertexQuadric(from);
		// now we have one less vertex
		--m_nverts;
	}

	// update queue
	for (auto or_it = oneRing.begin(), or_end = oneRing.end(); or_it != or_end; ++or_it) {
		enqueueVertex(*or_it);
	}
}


}

