#pragma once

#include "../common/Common.hh"
#include "DecimationFitting.hh"
#include "DecimationParametrization.hh"

namespace betri
{

class Decimation
{
public:

    Decimation(BezierTMesh &mesh) :
		m_mesh(mesh),
		m_complexity(m_mesh.n_vertices()),
		m_nverts(m_mesh.n_vertices()),
		m_q(nullptr),
		m_param(mesh),
		m_fit(mesh),
		m_cancel(false)
    {
		prepare();
		// create priority q
		//VertexCmp cmp(mesh, m_hprio, m_target);
		VertexCmp cmp(mesh);
		m_q = new std::set<VertexHandle, VertexCmp>(cmp);
    }

    ~Decimation()
    {
        cleanup();
		if (m_q) {
			delete m_q;
			m_q = nullptr;
		}
    }

    bool decimate(size_t complexity=0, bool stepwise=false);


private:

	//-----------------------------------------------//
	// compare functor for priority queue
	//-----------------------------------------------//
	//struct VertexCmp
	//{
	//	BezierTMesh *m_mesh;
	//	OpenMesh::HPropHandleT<Scalar> &m_prio;
	//	OpenMesh::VPropHandleT<HalfedgeHandle> &m_target;

	//	VertexCmp(
	//		BezierTMesh &mesh,
	//		OpenMesh::HPropHandleT<Scalar> &prio,
	//		OpenMesh::VPropHandleT<HalfedgeHandle> &target
	//	) : m_mesh(&mesh), m_prio(prio), m_target(target) {}

	//	Scalar priority(const VertexHandle vh) const
	//	{
	//		HalfedgeHandle h = m_mesh->property(m_target, vh);
	//		return h.is_valid() ? m_mesh->property(m_prio, h) : -1.0;
	//	}

	//	bool operator()(const VertexHandle v0, const VertexHandle v1) const
	//	{
	//		assert(m_mesh != nullptr);
	//		Scalar p0 = priority(v0), p1 = priority(v1);
	//		// std::set needs UNIQUE keys -> handle equal priorities
	//		return p0 == p1 ? v0.idx() < v1.idx() : p0 < p1;
	//	}
	//};

	struct VertexCmp
	{
		BezierTMesh *m_mesh;

		VertexCmp(BezierTMesh &mesh) : m_mesh(&mesh) {}

		bool operator()(const VertexHandle v0, const VertexHandle v1) const
		{
			assert(m_mesh != nullptr);
			OpenMesh::VPropHandleT<double> vprio;
			m_mesh->get_property_handle(vprio, "vertexprio");

			Scalar p0 = m_mesh->property(vprio, v0),
				p1 = m_mesh->property(vprio, v1);
			// std::set needs UNIQUE keys -> handle equal priorities
			return p0 == p1 ? v0.idx() < v1.idx() : p0 < p1;
		}
	};

	//-----------------------------------------------//
	// member functions
	//-----------------------------------------------//
	void step();

    void prepare();
    void cleanup();

	Scalar fit(const HalfedgeHandle hh, const bool apply);

	void calculateErrors();
	void calculateError(const HalfedgeHandle he);
	void updateError(const HalfedgeHandle he, const Scalar add);

	HalfedgeHandle& target(const VertexHandle vh) { return m_mesh.property(m_target, vh); }

	void enqueueVertex(const VertexHandle vh);

	bool isCollapseLegal(const HalfedgeHandle hh);

	Scalar& priority(const HalfedgeHandle hh) { return m_mesh.property(m_hprio, hh); }
	//Scalar priority(const VertexHandle vh)
	//{
	//	HalfedgeHandle h = target(vh);
	//	return h.is_valid() ? m_mesh.property(m_hprio, h) : -1.0;
	//}
	Scalar& priority(const VertexHandle vh) { return m_mesh.property(m_vprio, vh); }

	Scalar calcVertexPriority(const VertexHandle vh)
	{
		Scalar min = std::numeric_limits<Scalar>::max();
		HalfedgeHandle hh;

		for (auto h_it = m_mesh.cvoh_begin(vh); h_it != m_mesh.cvoh_end(vh); ++h_it) {
			if (priority(*h_it) < min) {
				min = priority(*h_it);
				hh = *h_it;
			}
		}

		return min;
	}

	void debugCancel(const char *msg)
	{
		m_cancel = true;
		m_errors += msg;
		m_errors += '\n';
	}
	void debugCancel(const std::string &msg)
	{
		m_cancel = true;
		m_errors += msg;
		m_errors += '\n';
	}
	bool debugCancel()
	{
		if (m_cancel && !m_errors.empty()) {
			std::cerr << "------------------------------\n";
			std::cerr << "Decimation Error:\n" << m_errors << std::endl;
			std::cerr << "------------------------------\n";
			m_errors.clear();
		}
		return m_cancel;
	}

	//-----------------------------------------------//
	// member variables
	//-----------------------------------------------//

	BezierTMesh &m_mesh;

	DecimationFitting m_fit;
	DecimationParametrization m_param;

	std::string m_errors;
	bool m_cancel;

	// desired complexity and current vertex count
	size_t m_complexity, m_nverts;

	// queue
	std::set<VertexHandle, VertexCmp> *m_q;

	// property handles
	OpenMesh::HPropHandleT<Scalar> m_hprio;
	OpenMesh::VPropHandleT<Scalar> m_vprio;
	OpenMesh::VPropHandleT<HalfedgeHandle> m_target;
};

}
