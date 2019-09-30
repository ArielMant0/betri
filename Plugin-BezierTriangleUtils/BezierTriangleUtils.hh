#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

namespace betri
{

void voronoiRemesh(BaseObjectData *object, BaseObjectData *ctrl, bool useColors, bool steps);

void voronoiRemeshStep(BaseObjectData *object, BaseObjectData *ctrl, bool useColors);

void decimate(BaseObjectData *object);

}
