#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

namespace betri
{

enum class TestOptions
{
	fitting,
	parametrization
};

void voronoiInit(BaseObjectData *object, BaseObjectData *ctrl, bool useColors);

void voronoiRemesh(BaseObjectData *object, BaseObjectData *ctrl);

void voronoiPartition(BaseObjectData *object, BaseObjectData *ctrl);

bool voronoiDual(BaseObjectData *object, BaseObjectData *ctrl, bool steps);

void voronoiFitting(BaseObjectData *object, BaseObjectData *ctrl);

void voronoiFittingTest(BaseObjectData *object, BaseObjectData *ctrl);

bool decimation(BaseObjectData *object, size_t complexity, bool steps);

bool test(TestOptions which, BezierTMesh *mesh=nullptr);

}
