#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

namespace betri
{

void voronoiRemesh(BaseObjectData *object, BaseObjectData *ctrl, bool useColors=true);

void decimate(BaseObjectData *object);

}
