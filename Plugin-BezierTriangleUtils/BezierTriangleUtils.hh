#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMeshTypes.hh>

namespace betri
{
    void addBezierTriangles(BezierTMesh &mesh);

	void remesh(BezierTMesh &mesh);

	void voronoi(BezierTMesh &mesh, unsigned int size);

}
