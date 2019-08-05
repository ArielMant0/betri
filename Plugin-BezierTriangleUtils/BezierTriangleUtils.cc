#include "BezierTriangleUtils.hh"

namespace betri
{

void addBezierTriangles(BezierTMesh &btmesh)
{
	for (auto f_it = btmesh.faces_begin(); f_it != btmesh.faces_end(); ++f_it) {
		auto v_it = btmesh.fv_begin(*f_it);
		auto p1 = btmesh.point(*v_it); v_it++;
		auto p2 = btmesh.point(*v_it); v_it++;
		auto p3 = btmesh.point(*v_it);

		btmesh.data(*f_it).addPoint(p1);
		btmesh.data(*f_it).addPoint(p1 * 0.5f + p2 * 0.5f);
		btmesh.data(*f_it).addPoint(p2);
		btmesh.data(*f_it).addPoint(p2 * 0.5f + p3 * 0.5f);
		btmesh.data(*f_it).addPoint(p3);
		btmesh.data(*f_it).addPoint(p3 * 0.5f + p1 * 0.5f);
	}
}

}
