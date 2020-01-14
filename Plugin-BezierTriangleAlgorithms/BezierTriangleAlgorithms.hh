#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

#include "voronoi/VoronoiRemeshPerObjectData.hh"
#include "decimation/DecimationPerObjectData.hh"

namespace betri
{

enum class TestOptions
{
	voronoi_fit,
	voronoi_param,
	decimation_fit,
	decimation_param
};

struct VoronoiInfo
{
	std::string name;
	std::string vertices;
	std::string edges;
	std::string faces;
	std::string partition;
};

struct DecimationInfo
{
	std::string name;
	std::string vertices;
	std::string edges;
	std::string faces;
	std::string target;
};

//////////////////////////////////////////
// voronoi
//////////////////////////////////////////

VoronoiRemesh* getVoronoiObject(BaseObjectData *object, BaseObjectData *ctrl);

void voronoiInit(
	BaseObjectData *object,
	BaseObjectData *ctrl,
	size_t count,
	const bool useColors,
	const bool interpolate,
	const bool overwrite,
	const bool splits,
	const int paramWeights,
	const size_t fittingSamples,
	const int fittingSolver
);

bool voronoiRemesh(BaseObjectData *object, BaseObjectData *ctrl);

bool voronoiPartition(BaseObjectData *object, BaseObjectData *ctrl, const bool steps, bool &done);

bool voronoiDual(BaseObjectData *object, BaseObjectData *ctrl, bool steps, bool &done);

bool voronoiFitting(BaseObjectData *object, BaseObjectData *ctrl);

VoronoiInfo voronoiInfo(BaseObjectData *object, BaseObjectData *ctrl);

//////////////////////////////////////////
// decimation
//////////////////////////////////////////

Decimation* getDecimationObject(BaseObjectData *object);

void decimationInit(
	BaseObjectData *object,
	const size_t complexity,
	const size_t fittingSamples,
	const int fittingSolver,
	const bool color=true
);

bool decimation(BaseObjectData *object, const bool steps, const bool interpolate);

DecimationInfo decimationInfo(BaseObjectData *object);

//////////////////////////////////////////
// tests
//////////////////////////////////////////

bool test(TestOptions which, BezierTMesh *mesh=nullptr);

}
