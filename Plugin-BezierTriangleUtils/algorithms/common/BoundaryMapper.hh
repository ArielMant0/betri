#pragma once

#include "Common.hh"

namespace betri
{

class BoundaryMapper
{
public:

	explicit BoundaryMapper(BezierTMesh &mesh, OpenMesh::VPropHandleT<VertexToTri> &vtt) :
		m_mesh(mesh),
		m_vtt(vtt) {}

    using Path = std::vector<VertexHandle>;

    virtual void map(std::vector<Path*> &paths) = 0;

	Vec2& hmap(VertexHandle vh) { return m_mesh.property(m_vtt, vh).uv; }
    ID& id(VertexHandle vh) { return m_mesh.property(m_vtt, vh).id1; }

    Scalar pathLength(Path *path)
    {
        Scalar l = 0.;
        Point p = m_mesh.point(path->at(0));
        for (VertexHandle vh : *path) {
            l += (p - m_mesh.point(vh)).norm();
            p = m_mesh.point(vh);
        }
        return l;
    }

protected:

    BezierTMesh &m_mesh;

	OpenMesh::VPropHandleT<VertexToTri> m_vtt;

};

}
