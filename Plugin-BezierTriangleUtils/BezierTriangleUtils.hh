#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

namespace betri
{

enum class TestOptions
{
	voronoi_fit,
	voronoi_param,
	decimation_fit,
	decimation_param
};

void randomMeshUV(BezierTMesh &mesh);

void voronoiInit(BaseObjectData *object, BaseObjectData *ctrl, size_t count, bool untwist, bool useColors);

void voronoiRemesh(BaseObjectData *object, BaseObjectData *ctrl);

void voronoiPartition(BaseObjectData *object, BaseObjectData *ctrl);

bool voronoiDual(BaseObjectData *object, BaseObjectData *ctrl, bool steps);

void voronoiFitting(BaseObjectData *object, BaseObjectData *ctrl);

void voronoiSmooth(BaseObjectData *object, BaseObjectData *ctrl);

void voronoiFittingTest(BaseObjectData *object, BaseObjectData *ctrl);

bool decimation(BaseObjectData *object, size_t complexity, bool steps, bool untwist);

bool test(TestOptions which, BezierTMesh *mesh=nullptr);

}
