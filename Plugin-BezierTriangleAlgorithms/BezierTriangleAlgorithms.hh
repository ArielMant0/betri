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

//////////////////////////////////////////
// voronoi
//////////////////////////////////////////

void voronoiInit(
	BaseObjectData *object,
	BaseObjectData *ctrl,
	size_t count,
	const bool useColors,
	const bool interpolate
);

void voronoiRemesh(BaseObjectData *object, BaseObjectData *ctrl);

void voronoiPartition(BaseObjectData *object, BaseObjectData *ctrl);

bool voronoiDual(BaseObjectData *object, BaseObjectData *ctrl, bool steps);

void voronoiFitting(BaseObjectData *object, BaseObjectData *ctrl);

void voronoiFittingTest(BaseObjectData *object, BaseObjectData *ctrl);

//////////////////////////////////////////
// decimation
//////////////////////////////////////////

void decimationInit(BaseObjectData *object, size_t complexity, bool color=true);

bool decimation(BaseObjectData *object, bool steps, bool interpolate);

//////////////////////////////////////////
// tests
//////////////////////////////////////////

bool test(TestOptions which, BezierTMesh *mesh=nullptr);

}
