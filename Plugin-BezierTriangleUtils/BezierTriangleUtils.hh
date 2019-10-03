#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

namespace betri
{

void voronoiInit(BaseObjectData *object, BaseObjectData *ctrl, bool useColors);

void voronoiRemesh(BaseObjectData *object, BaseObjectData *ctrl);

void voronoiPartition(BaseObjectData *object, BaseObjectData *ctrl);

bool voronoiDual(BaseObjectData *object, BaseObjectData *ctrl, bool steps);

void voronoiFitting(BaseObjectData *object, BaseObjectData *ctrl);

}
