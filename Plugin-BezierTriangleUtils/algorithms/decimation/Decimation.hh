#pragma once

#include "../common/Common.hh"
#include "QuadricT.hh"

namespace betri
{

class Decimation
{
public:

    Decimation(BezierTMesh &mesh) :
		m_mesh(mesh),
		m_complexity(m_mesh.n_vertices()),
		m_nverts(m_mesh.n_vertices())
    {
        prepare();
		// create priority q
		VertexCmp cmp(mesh, m_prio);
		m_q = new std::set<VertexHandle, VertexCmp>(cmp);
		// initialize quadrics
		initQuadrics();
    }

    ~Decimation()
    {
        cleanup();
		if (m_q) delete m_q;
    }

    bool decimate(size_t complexity=0, bool stepwise=false);


private:

	//-----------------------------------------------//
	// compare functor for priority queue
	//-----------------------------------------------//
	struct VertexCmp
	{
		BezierTMesh *m_mesh;
		OpenMesh::VPropHandleT<double> m_prio;

		VertexCmp(BezierTMesh mesh, OpenMesh::VPropHandleT<double> prio) :
			m_mesh(&mesh), m_prio(prio) {}

		bool operator()(VertexHandle v0, VertexHandle v1) const
		{
			assert(m_mesh != nullptr);
			// std::set needs UNIQUE keys -> handle equal priorities
			return ((m_mesh->property(m_prio, v0) == m_mesh->property(m_prio, v1)) ?
				(v0.idx() < v1.idx()) :
				(m_mesh->property(m_prio, v0) < m_mesh->property(m_prio, v1)));
		}
	};

	//-----------------------------------------------//
	// member functions
	//-----------------------------------------------//
	void step();

    void prepare();
    void cleanup();

	void initQuadrics();

	Quadricd& vertexQuadric(const VertexHandle vh) { return m_mesh.property(m_quadric, vh); }
	HalfedgeHandle& vertexTarget(const VertexHandle vh) { return m_mesh.property(m_target, vh); }

	void enqueueVertex(const VertexHandle vh);

	bool isCollapseLegal(const HalfedgeHandle hh);

	double& priority(const VertexHandle vh) { return m_mesh.property(m_prio, vh); }
	double priority(const HalfedgeHandle hh)
	{
		VertexHandle v0 = m_mesh.from_vertex_handle(hh);
		VertexHandle v1 = m_mesh.to_vertex_handle(hh);

		Quadricd q = vertexQuadric(v0);
		q += vertexQuadric(v1);

		return q(m_mesh.point(v1));
	}

	//-----------------------------------------------//
	// member variables
	//-----------------------------------------------//

	BezierTMesh &m_mesh;

	// desired complexity and current vertex count
	size_t m_complexity, m_nverts;

	// queue
	std::set<VertexHandle, VertexCmp> *m_q;

	// property handles
	OpenMesh::VPropHandleT<double> m_prio;
	OpenMesh::VPropHandleT<Quadricd> m_quadric;
	OpenMesh::VPropHandleT<HalfedgeHandle> m_target;
};

}
