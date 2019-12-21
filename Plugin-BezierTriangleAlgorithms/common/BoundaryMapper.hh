#pragma once

#include "Common.hh"

namespace betri
{

template <typename T>
class BoundaryMapper
{
public:

	explicit BoundaryMapper(BezierTMesh &mesh, OpenMesh::VPropHandleT<T> &uvProp) :
		m_mesh(mesh),
		m_uv(uvProp){}

    using Path = std::vector<VertexHandle>;

    virtual void map(std::vector<Path*> &paths) = 0;

	virtual Vec2 middle() = 0;

	T& hmap(VertexHandle vh) { return m_mesh.property(m_uv, vh); }

    Scalar pathLength(Path *path)
    {
        Scalar l = 0.;
        Point p = m_mesh.point(path->front());
        for (VertexHandle vh : *path) {
            l += (p - m_mesh.point(vh)).norm();
            p = m_mesh.point(vh);
        }
        return l;
    }

	static void clamp(Vec2 &uv)
	{
		uv[0] = uv[0] <= 0. ? 0. : (uv[0] >= 1. ? 1. : uv[0]);
		uv[1] = uv[1] <= 0. ? 0. : (uv[1] >= 1. ? 1. : uv[1]);
	}

	static void clamp(Point &p)
	{
		p[0] = p[0] <= 0. ? 0. : (p[0] >= 1. ? 1. : p[0]);
		p[1] = p[1] <= 0. ? 0. : (p[1] >= 1. ? 1. : p[1]);
		p[2] = p[2] <= 0. ? 0. : (p[2] >= 1. ? 1. : p[2]);
	}

protected:

    BezierTMesh &m_mesh;

	OpenMesh::VPropHandleT<T> m_uv;

};

}
