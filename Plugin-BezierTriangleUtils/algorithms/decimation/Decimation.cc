#include "Decimation.hh"

#include <OpenFlipper/libs_required/ACG/Utils/ColorCoder.hh>

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
	m_minError = std::numeric_limits<Scalar>::max();
	m_maxError = 0;
	m_avgError = 0.;

	size_t count = 0;

	m_mesh.update_face_normals();

	for (HalfedgeHandle hh : m_mesh.halfedges()) {

		calculateError(hh);
		Scalar error = priority(hh);

		if (error != -1.0) {
			m_maxError = std::max(m_maxError, error);
			m_minError = std::min(m_minError, error);
			m_avgError += error;
			count++;
		}
	}
	m_avgError /= count;

	std::cerr << "calcluated errors:\n\tmin: " << m_minError << "\n\tmax: " << m_maxError;
	std::cerr << "\n\tavg: " << m_avgError << std::endl;

	for (VertexHandle vh : m_mesh.vertices()) {
		priority(vh) = -1.0;
	}
}

void Decimation::calculateError(const HalfedgeHandle hh)
{
	// "simulate collapse" -> fit all 1-Ring faces -> use max error
	priority(hh) = isCollapseLegal(hh) ? fit(hh, false) : -1.0;
}

void Decimation::updateError(const HalfedgeHandle hh, const Scalar add)
{
	bool legal = isCollapseLegal(hh);

	if (priority(hh) != -1.0 && legal) {
		// "simulate collapse" -> fit all 1-Ring faces -> use max error
		Scalar error = fit(hh, false);
		priority(hh) = error != -1.0 ? add + error : -1.0;
	} else {
		// halfedges that result in an illegal collapse have a priority of -1
		priority(hh) = -1.0;
	}
}

void Decimation::enqueueVertex(const VertexHandle vh)
{
	Scalar prio, min_prio(std::numeric_limits<Scalar>::max());
	HalfedgeHandle min_hh;


	// find best out-going halfedge
	for (auto voh_it = m_mesh.voh_begin(vh), voh_end = m_mesh.voh_end(vh);
		voh_it != voh_end; ++voh_it) {

		prio = priority(*voh_it);

		if (prio != -1.0 && prio < min_prio) {
			min_prio = prio;
			min_hh = *voh_it;
		}
	}

	// remove from queue
	if (priority(vh) != -1.0) {
		m_q->erase(vh);
		priority(vh) = -1.0;
	}

	// update queue
	if (min_hh.is_valid()) {
		target(vh) = min_hh;
		priority(vh) = min_prio;
		assert(min_prio != -1.0);
		m_q->insert(vh);
	}
}

bool Decimation::isCollapseLegal(const HalfedgeHandle hh)
{
	// collect vertices
	VertexHandle v0 = m_mesh.from_vertex_handle(hh);
	VertexHandle v1 = m_mesh.to_vertex_handle(hh);

	// topological test
	if (!m_mesh.is_collapse_ok(hh)) {
		return false;
	}
	// test boundary stuff
	if (m_mesh.is_boundary(v0) && !m_mesh.is_boundary(v1)) {
		return false;
	}

	return true;

	//// collect faces
	//FaceHandle fl = m_mesh.face_handle(hh);
	//FaceHandle fr = m_mesh.face_handle(m_mesh.opposite_halfedge_handle(hh));

	//// backup point positions
	//const Point p0 = m_mesh.point(v0);
	//const Point p1 = m_mesh.point(v1);

	//// simulate collapse
	//m_mesh.set_point(v0, p1);

	//bool collapseOK = true;

	//// check for flipping normals
	//double c(1.0);
	//const double min_cos(0.);//cos(0.5 * M_PI));

	//for (auto vf_it(m_mesh.cvf_begin(v0)), vf_end(m_mesh.cvf_end(v0)); vf_it != vf_end; ++vf_it) {
	//	// dont check triangles that would be deleted anyway
	//	if ((*vf_it != fl) && (*vf_it != fr)) {

	//		TriMesh::Point n0 = m_mesh.normal(*vf_it);
	//		TriMesh::Point n1 = m_mesh.calc_face_normal(*vf_it);
	//		// check wether normal flips due to collapse
	//		if ((c = (n0 | n1)) < min_cos) {
	//			collapseOK = false;
	//			break;
	//		}
	//	}
	//}

	//// undo simulation changes
	//m_mesh.set_point(v0, p0);

	//// return the result of the collapse simulation
	//return collapseOK;
}

void Decimation::calcErrorStatistics()
{
	m_minError = std::numeric_limits<Scalar>::max();
	m_maxError = 0.0;
	m_avgError = 0.0;

	for (VertexHandle vh : m_mesh.vertices()) {
		Scalar prio = priority(vh);
		if (prio != -1.0) {
			m_minError = std::min(m_minError, prio);
			m_maxError = std::max(m_maxError, prio);
			m_avgError += prio;
		}
	}
}

void Decimation::setColorsFromError()
{
	ACG::ColorCoder coder(m_minError, m_maxError, false);

	Scalar prio;
	Color black(0.f, 0.f, 0.f, 1.f);

	for (VertexHandle vh : m_mesh.vertices()) {
		prio = priority(vh);
		m_mesh.set_color(vh, prio != -1.0 ? coder.color_float4_raw(prio) : black);
	}
}

void Decimation::initialize(size_t complexity)
{
	// only set one time before decimation is called
	m_complexity = complexity == 0 ? m_complexity : complexity;
	m_nverts = m_mesh.n_vertices();

	// build priority queue...
	m_q->clear();

	// prepare fitting by telling it barycentric coordinates used for surface sampling
	m_fit.setBarycentricCoords(m_param.getBarycentricCoords());
	// calculate fitting errors
	calculateErrors();

	// fill queue
	for (VertexHandle vh : m_mesh.vertices()) {
		enqueueVertex(vh);
	}

	std::cerr << __FUNCTION__ << "\n\ttarget complexity: " << m_complexity;
	std::cerr << "\n\tqueue size: " << m_q->size() << std::endl;

	if (m_useColors) {
		// get vertex colors if we dont have any
		if (!m_mesh.has_vertex_colors()) {
			m_mesh.request_vertex_colors();
		}

		setColorsFromError();
	}
}

bool Decimation::decimate(bool stepwise)
{
	bool done = m_nverts <= m_complexity;

	// do the actual decimation...
	while (m_nverts > m_complexity && !m_q->empty()) {
		step();
		if (debugCancel())
			return true;

		// in case we only want to do a single step
		if (stepwise) break;
	}

	std::cerr << "\treached complexity: " << m_nverts;
	std::cerr << "\n\tqueue size: " << m_q->size();

	done = m_nverts <= m_complexity || m_q->empty();

	m_mesh.garbage_collection();

	if (!done && stepwise) {

		// queue has to be updated since vertex handles change
		// TODO: try using q over pointers to be able to use garbage_collection(...)
		//		 with retaining vertex handles
		m_q->clear();

		for (VertexHandle vh : m_mesh.vertices()) {
			enqueueVertex(vh);
		}
	}

	// update error statistics
	calcErrorStatistics();

	if (m_useColors) {
		setColorsFromError();
	}

	// update normals
	m_mesh.update_normals();

	return done;
}

void Decimation::step()
{
	VertexHandle vh = *m_q->begin();
	m_q->erase(m_q->begin());

	HalfedgeHandle hh = target(vh);

	if (!hh.is_valid()) return;

	VertexHandle from = m_mesh.from_vertex_handle(hh);
	VertexHandle to = m_mesh.to_vertex_handle(hh);

	const Scalar collapseError = priority(hh);

	if (collapseError == -1.0) return;

	// perform collapse
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
		if (*f_it != f0 && *f_it != f1) {
			//m_mesh.set_color(*f_it, { 0.f, 0.75f, 0.25f, 1.f });
			faces.push_back(*f_it);
		}
	}
	std::cerr << "collapsing halfedge " << hh << " with error " << collapseError << '\n';
	// fit remaining faces (and apply update)
	fit(hh, true);

	if (debugCancel()) {
		return;
	}

	// collapse halfedge
	m_mesh.collapse(hh);

	for (VertexHandle vh : oneRingV) {
		for (auto h_it = m_mesh.cvoh_begin(vh); h_it != m_mesh.cvoh_end(vh); ++h_it) {
			oneRing.insert(*h_it);
		}
	}

	// make faces "match" again
	std::set<EdgeHandle> visited;
	std::set<FaceHandle> otherFaces(faces.begin(), faces.end());

	for (size_t i = 0; i < faces.size(); ++i) {
		for (auto h_it = m_mesh.cfh_begin(faces[i]), h_end = m_mesh.cfh_end(faces[i]);
			h_it != h_end; ++h_it
		) {
			EdgeHandle edge = m_mesh.edge_handle(*h_it);
			if (visited.find(edge) == visited.end()) {
				m_mesh.interpolateEdgeControlPoints(edge, true);
				visited.insert(edge);
			}

			if (otherFaces.find(m_mesh.opposite_face_handle(*h_it)) == otherFaces.end()) {
				otherFaces.insert(m_mesh.opposite_face_handle(*h_it));
			}
		}
	}

	if (m_untwist) {
		for (FaceHandle fh : otherFaces) {
			m_mesh.untwistControlPoints(fh);
		}
	}

	// now we have one less vertex
	--m_nverts;

	// recalculate the error first, so we know all incident halfedges
	// are updated before updating the queue
	for (HalfedgeHandle he : oneRing) {
		updateError(he, collapseError);
	}
	// update queue (reinsert the vertex)
	for (VertexHandle vh : oneRingV) {
		enqueueVertex(vh);
	}
}

Scalar Decimation::fit(const HalfedgeHandle hh, const bool apply)
{
	// source and target vertex
	VertexHandle from = m_mesh.from_vertex_handle(hh), to = m_mesh.to_vertex_handle(hh);

	// stores all 3D points and corresponding uv's per face
	std::vector<FitCollection> fitColls;

	// parametrize to n-gon
	if (!m_param.solveLocal(from, to, fitColls, apply)) {
		//debugCancel("parametrization failed");
		return -1.0;
	}

	Scalar maxError = 0., error = 0.;

	assert(fitColls.size() == m_mesh.valence(from) - 2);
	for (size_t i = 0; i < fitColls.size(); ++i) {
		// fit all surrounding faces
		if (!m_fit.solveLocal(fitColls[i], error, apply)) {
			debugCancel("fitting failed");
			return -1.0;
		}
		// store max error
		maxError = std::max(error, maxError);
	}

	return maxError;
}

}

